from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import warnings
from dataclasses import dataclass, replace
from pathlib import Path

import jsonschema
import numpy as np
from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.ldraw.model import LDrawBuild
from lego_technic_sim.physics.connectors import ConnectorType, classify_connector, is_connector
from lego_technic_sim.physics.model import JointType, PhysicsScene
from lego_technic_sim.physics.motor_detection import is_motor_part
from lego_technic_sim.physics.unit_builder import (
    _connector_shaft, _determine_connection_type, _find_port_connections, _port_on_shaft,
    PORT_POSITION_TOLERANCE, build_units_and_joints,
)

from .schema import MANIFEST_SCHEMA, MAX_FEET

SIMULATOR_REVISION = "1b70ff4b6c567b3c8541d5ef8ff4c56b4be24a5b"
LDU_TO_METERS = 0.0004
MOTION_GRAMMARS = ("technic-motion-v1", "technic-motion-v2", "technic-motion-v3")


class AdapterError(ValueError):
    def __init__(self, message, *, construction_connectivity=None, repair_hints=None, joint_preservation=None):
        self.construction_connectivity = construction_connectivity
        self.joint_preservation = joint_preservation
        self.repair_hints = repair_hints
        super().__init__(message)


class ConstructionConnectivityError(AdapterError):
    def __init__(self, evidence):
        self.evidence = evidence
        parts = set(evidence["disconnectedParts"])
        for bond in evidence["failedBonds"]:
            parts.update([bond["connector"], bond["partA"], bond["partB"]])
        hints = {"kind": "rigid-bond-engagement", "parts": sorted(parts), "bodies": [evidence["chassis"]]}
        super().__init__(f"Construction connectivity failed: {len(evidence['failedBonds'])} rigid bonds; "
                         f"{len(evidence['disconnectedParts'])} parts without a witnessed frame path.",
                         construction_connectivity=evidence, repair_hints=hints)


class JointPreservationError(AdapterError):
    def __init__(self, evidence, connectivity):
        failed = [pivot for pivot in evidence["pivots"] if pivot["status"] == "reject"]
        competition = [pivot for pivot in failed if pivot["unexpectedParts"]]
        focus = competition or failed
        parts = {pivot["connector"] for pivot in focus}
        parts.update(part for pivot in focus for part in (
            pivot["unexpectedParts"] if competition else pivot["mergedParts"] + pivot["splitParts"]))
        bodies = sorted({body for pivot in focus for body in pivot["expectedBodies"] + pivot["mergedBodies"] + pivot["splitBodies"]})
        partition_failure = any(p["mergedBodies"] or p["splitBodies"] for p in focus)
        kind = "joint-port-engagement" if competition or not partition_failure else "rigid-body-partition"
        super().__init__(
            f"Joint preservation failed: {len(failed)} obligations; "
            + "; ".join(f"{pivot['connector']}: {', '.join(pivot['failures'])}" for pivot in failed),
            construction_connectivity=connectivity, joint_preservation=evidence,
            repair_hints={"kind": kind, "parts": sorted(parts), "bodies": bodies})


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _reject_constant(value):
    raise AdapterError(f"Non-finite JSON number: {value}")


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        _reject_constant(value)
    return number


def load_manifest(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"), parse_constant=_reject_constant,
                      parse_float=_finite_float)
    errors = sorted(jsonschema.Draft202012Validator(MANIFEST_SCHEMA).iter_errors(data),
                    key=lambda error: str(list(error.path)))
    if errors:
        raise AdapterError("Invalid manifest: " + "; ".join(
            f"{list(error.path)}: {error.message}" for error in errors[:10]))
    if data["simulator"]["revision"] != SIMULATOR_REVISION:
        raise AdapterError("Manifest targets a different simulator revision.")
    model = (path.parent / data["modelFile"]).resolve()
    if model.parent != path.parent.resolve():
        raise AdapterError("modelFile must name a file next to the manifest.")
    if hashlib.sha256(model.read_bytes()).hexdigest() != data["ldrawSha256"]:
        raise AdapterError("LDraw checksum mismatch; regenerate the candidate.")
    expected = data["expected"]
    bodies = set(expected["bodies"])
    if len(bodies) != len(expected["bodies"]) or data["chassis"] not in bodies:
        raise AdapterError("Body IDs must be unique and include the chassis.")
    part_ids = [part["id"] for part in data["parts"]]
    if len(part_ids) != len(set(part_ids)):
        raise AdapterError("Part instance IDs must be unique.")
    by_part_id = {part["id"]: part for part in data["parts"]}
    motion_grammar = data["grammarVersion"] in MOTION_GRAMMARS
    new_genes = {"remote", "batterySlot", "batteryLevel", "receiverSlot", "receiverSide", "motorDirection"}
    if not motion_grammar and data["grammarVersion"] not in ("technic-fourbar-v3", "technic-fourbar-v4", "technic-fourbar-v5") and (
            "payloads" in data or new_genes.intersection(data["genome"])):
        raise AdapterError("Payload/control extensions require grammar v3.")
    if data["grammarVersion"] in ("technic-fourbar-v3", "technic-fourbar-v4", "technic-fourbar-v5"):
        if not new_genes.issubset(data["genome"]) or "payloads" not in data:
            raise AdapterError("v3 manifests require all payload/control genes and payloads.")
        payloads = data["payloads"]
        products = [payload["product"] for payload in payloads]
        if sorted(products) != (["8881", "8884"] if data["genome"]["remote"] else []):
            raise AdapterError("Remote walkers require exactly one 8881 and one 8884; wired walkers carry neither.")
        if not data["genome"]["remote"] and any(
                data["genome"][key] != value for key, value in
                (("batterySlot", 1), ("batteryLevel", 0), ("receiverSlot", 0), ("receiverSide", 1))):
            raise AdapterError("Wired genomes must use canonical inactive payload genes.")
        if len({payload["partInstance"] for payload in payloads}) != len(payloads):
            raise AdapterError("Payload part instances must be unique.")
        for payload in payloads:
            part = by_part_id.get(payload["partInstance"])
            expected_part = {"8881": "58119.dat", "8884": "58123.dat"}[payload["product"]]
            carrier_body = ("wagon-chassis" if data["grammarVersion"] == "technic-fourbar-v5"
                            and payload.get("carrier") == "wagon" else data["chassis"])
            if part is None or part["part"] != expected_part or part["body"] != carrier_body:
                raise AdapterError("Payload must reference the correct electronics part attached to its carrier chassis.")
    wagon_genes = {"towWagon", "batteryOnWagon", "receiverOnWagon", "wagonAxleHole"}
    if data["grammarVersion"] in ("technic-fourbar-v4", "technic-fourbar-v5"):
        g = data["genome"]
        if not wagon_genes.issubset(g) or "groundWheels" not in data:
            raise AdapterError("v4 manifests require wagon genes and explicit groundWheels.")
        if ((g["batteryOnWagon"] or g["receiverOnWagon"]) and (not g["remote"] or not g["towWagon"])
                or (not g["towWagon"] and g["wagonAxleHole"] != 4)):
            raise AdapterError("Invalid inactive wagon/electronics genes.")
        wheels = data["groundWheels"]
        if len(wheels) != (2 if g["towWagon"] else 0) or len(set(wheels)) != len(wheels):
            raise AdapterError("Towing designs require exactly two designated wagon wheels; solo designs require none.")
        for wheel in wheels:
            if wheel not in by_part_id or by_part_id[wheel]["part"] != "56145c01.dat":
                raise AdapterError("Designated wagon wheels must reference actual supported wheel assemblies.")
        for foot in data["feet"]:
            part = by_part_id.get(foot.get("partInstance"))
            if ("contactRadiusMeters" not in foot or part is None or part["body"] != foot["body"]
                    or part["part"] != "32063.dat"):
                raise AdapterError("v4 feet must identify a bounded contact region on their actual coupler part.")
        for payload in data["payloads"]:
            on_wagon = g["batteryOnWagon"] if payload["product"] == "8881" else g["receiverOnWagon"]
            if payload.get("carrier") != ("wagon" if on_wagon else "walker"):
                raise AdapterError("Payload carrier differs from the placement genes.")
        if data["grammarVersion"] == "technic-fourbar-v5" and g["towWagon"]:
            hitch = {joint["id"]: joint for joint in expected["joints"] if joint["id"].startswith("drawbar-")}
            if set(hitch) != {"drawbar-walker-pitch", "drawbar-wagon-pitch"}:
                raise AdapterError("Towing requires two passive drawbar pitch hinges.")
            for name, body in (("walker", data["chassis"]), ("wagon", "wagon-chassis")):
                joint = hitch[f"drawbar-{name}-pitch"]
                if ({joint["bodyA"], joint["bodyB"]} != {body, "drawbar"}
                        or not np.allclose(joint["axis"], [0, 0, 1])
                        or any(drive["joint"] == joint["id"] for drive in expected["drives"])):
                    raise AdapterError("Drawbar hinges must be passive transverse pitch joints between separate bodies.")
    elif not motion_grammar and (wagon_genes.intersection(data["genome"]) or "groundWheels" in data):
        raise AdapterError("Wagon extensions require grammar v4.")
    if motion_grammar:
        _validate_motion_metadata(data, by_part_id)
    for part in data["parts"]:
        if is_connector(part["part"]) != (part["body"] is None):
            raise AdapterError(f"Inconsistent connector/body assignment: {part['id']}")
    actual_bodies = {part["body"] for part in data["parts"] if part["body"] is not None}
    if bodies != actual_bodies:
        raise AdapterError("The expected bodies do not match the structural part assignments.")
    joint_ids = [joint["id"] for joint in expected["joints"]]
    if len(joint_ids) != len(set(joint_ids)):
        raise AdapterError("Joint IDs must be unique.")
    for joint in expected["joints"]:
        if joint["bodyA"] not in bodies or joint["bodyB"] not in bodies or joint["bodyA"] == joint["bodyB"]:
            raise AdapterError(f"Invalid joint body reference: {joint['id']}")
        source = by_part_id.get(joint["connector"])
        if source is None or source["body"] is not None or not math.isclose(np.linalg.norm(joint["axis"]), 1.0, abs_tol=1e-8):
            raise AdapterError(f"Invalid joint connector/axis: {joint['id']}")
    for gear in expected["gears"]:
        if gear["bodyA"] not in bodies or gear["bodyB"] not in bodies or gear["bodyA"] == gear["bodyB"]:
            raise AdapterError("Invalid gear body reference.")
    for drive in expected["drives"]:
        if drive["joint"] not in joint_ids or drive["motorPart"] not in part_ids:
            raise AdapterError("Invalid drive reference.")
        if not is_motor_part(by_part_id[drive["motorPart"]]["part"]):
            raise AdapterError("A drive must reference an actual supported motor part.")
    if len({drive["joint"] for drive in expected["drives"]}) != len(expected["drives"]):
        raise AdapterError("Duplicate driven joints.")
    for foot in data["feet"]:
        if foot["body"] not in bodies:
            raise AdapterError("Invalid foot body reference.")
    for part in data["parts"]:
        rotation = np.array(part["rotation"])
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or not math.isclose(np.linalg.det(rotation), 1.0, abs_tol=1e-8):
            raise AdapterError(f"Invalid proper rotation: {part['id']}")
    return data


def _validate_motion_metadata(data, by_part_id):
    genome, motion = data["genome"], data["motion"]
    bodies = data["expected"]["bodies"]
    choices = [choice["body"] for choice in genome["bodies"]]
    if len(choices) != len(set(choices)) or set(choices) != set(bodies):
        raise AdapterError("Realization choices must implement every motion body exactly once.")
    if genome["mechanismId"] != motion["mechanismId"] or len(data["expected"]["drives"]) != 1:
        raise AdapterError("A motion realization must identify its mechanism and exactly one motor input.")
    connections = {choice["group"]: choice for choice in genome["connections"]}
    pivots = motion["sharedPivots"]
    if (len(connections) != len(genome["connections"]) or len({p["id"] for p in pivots}) != len(pivots)
            or set(connections) != {pivot["id"] for pivot in pivots}):
        raise AdapterError("Realization connector choices and shared pivot groups differ.")
    abstract = [hinge for pivot in pivots for hinge in pivot["abstractHinges"]]
    if len(abstract) != len(set(abstract)):
        raise AdapterError("Abstract hinges must belong to exactly one shared pivot.")
    declared = set()
    for pivot in pivots:
        connector_id = f"pivot-{pivot['id']}"
        source = by_part_id.get(connector_id)
        choice = connections[pivot["id"]]
        connector_part = (choice["placement"]["part"] if data["grammarVersion"] == "technic-motion-v3"
                          else choice["part"])
        joints = [joint for joint in data["expected"]["joints"] if joint["connector"] == connector_id]
        members = set(pivot["bodies"])
        if (source is None or source["part"] != connector_part or len(members) != len(pivot["bodies"])
                or not 2 <= len(members) <= 3 or not members.issubset(bodies)
                or len(joints) != len(members)-1
                or len(pivot["abstractHinges"]) != len(joints)):
            raise AdapterError("Shared pivot must be one real connector and an equivalent N-1 hinge tree.")
        reached = {next(iter(members))}
        for _ in members:
            for joint in joints:
                if reached.intersection((joint["bodyA"], joint["bodyB"])):
                    reached.update((joint["bodyA"], joint["bodyB"]))
        if reached != members:
            raise AdapterError("Physical shared pivot is not connected over its declared bodies.")
        reference = joints[0]
        axis = np.asarray(reference["axis"])
        if any(np.linalg.norm(np.cross(np.asarray(joint["axis"]), axis)) > 1e-7
               or np.linalg.norm(np.cross(np.asarray(joint["position"])-reference["position"], axis)) > 1e-5
               for joint in joints):
            raise AdapterError("Shared pivot hinge declarations must be coaxial.")
        declared.update(joint["id"] for joint in joints)
    if declared != {joint["id"] for joint in data["expected"]["joints"]}:
        raise AdapterError("Every physical hinge must belong to a declared motion pivot.")
    for foot in data["feet"]:
        source = by_part_id.get(foot.get("partInstance"))
        if (source is None or source["body"] != foot["body"] or "contactRadiusMeters" not in foot
                or source["part"] not in {"3701.dat", "3710.dat", "32063.dat", "32523.dat", "32316.dat",
                                          "32524.dat", "32525.dat", "60483.dat", "32271.dat"}):
            raise AdapterError("Motion feet must name bounded regions on supported physical foot parts.")
    if data["grammarVersion"] in ("technic-motion-v2", "technic-motion-v3"):
        logical = motion["logicalFeet"]
        regions = {foot["id"]: foot for foot in data["feet"]}
        listed = [region for foot in logical for region in foot["regions"]]
        if (len(regions) != len(data["feet"]) or len({foot["id"] for foot in logical}) != len(logical)
                or motion["footCount"] != len(logical) or not 1 <= len(logical) <= MAX_FEET
                or len(set(listed)) != len(listed) or set(listed) != set(regions)):
            raise AdapterError("Logical feet must partition all physical contact regions and match the inferred foot count.")
        for foot in logical:
            if not foot["regions"] or foot["body"] not in bodies or any(
                    regions[region]["body"] != foot["body"] or regions[region]["logicalFoot"] != foot["id"]
                    for region in foot["regions"]):
                raise AdapterError("Each logical foot must reference its own actual body and contact regions.")
    if data["grammarVersion"] == "technic-motion-v3":
        _validate_walker_metadata(data, by_part_id)


def _validate_walker_metadata(data, by_part_id):
    genome = data["genome"]
    declared = []

    def placement(choice, body):
        declared.append(choice["id"])
        actual = by_part_id.get(choice["id"])
        position = [choice["position"][axis] for axis in ("x", "y", "z")]
        rotation = [[choice["rotation"][column][row] for column in ("x", "y", "z")]
                    for row in ("x", "y", "z")]
        if (actual is None or actual["body"] != body or actual["part"] != choice["part"]
                or actual["color"] != choice["color"] or actual["position"] != position
                or actual["rotation"] != rotation):
            raise AdapterError("Walker realization choices differ from the exported physical parts.")

    for body in genome["bodies"]:
        for choice in body["parts"]:
            placement(choice, body["body"])
    for choice in genome["connections"]:
        placement(choice["placement"], None)
        pivot = next(p for p in data["motion"]["sharedPivots"] if p["id"] == choice["group"])
        if sorted(choice["hinges"]) != sorted(pivot["abstractHinges"]):
            raise AdapterError("Walker pivot choices differ from their abstract hinge groups.")
    for choice in genome["mounts"]:
        placement(choice, None)
    if len(declared) != len(set(declared)) or set(declared) != set(by_part_id):
        raise AdapterError("Walker choices must cover every exported part exactly once.")
    payloads = data["payloads"]
    if sorted(p["product"] for p in payloads) != ["8881", "8884"]:
        raise AdapterError("A remote walker must carry one battery and one receiver.")
    for payload in payloads:
        source = by_part_id.get(payload["partInstance"])
        expected = {"8881": "58119.dat", "8884": "58123.dat"}[payload["product"]]
        if (source is None or source["part"] != expected or source["body"] != data["chassis"]
                or payload.get("carrier") != "walker"):
            raise AdapterError("Remote hardware must be physically attached to the walker chassis.")
    motors = [p for p in by_part_id.values() if is_motor_part(p["part"])]
    drive = data["expected"]["drives"][0]
    if (len(motors) != 1 or motors[0]["id"] != genome["motorPart"]
            or drive["motorPart"] != genome["motorPart"] or drive["speed"] != genome["speed"]
            or drive["maxTorque"] != genome["maxTorque"] or genome["speed"] == 0):
        raise AdapterError("The explicit walker motor choice differs from its single physical drive.")
    choices = genome["feet"]
    def mounted_plate(plate, layers=1):
        if layers > 2:
            return False
        for parent in by_part_id.values():
            if (parent["id"] == plate["id"] or parent["body"] != plate["body"]
                    or parent["part"] not in {"3701.dat", "3710.dat"}
                    or not np.allclose(parent["rotation"], plate["rotation"], rtol=0, atol=1e-6)):
                continue
            height = 24 if parent["part"] == "3701.dat" else 8
            top = np.array(parent["position"]) + height*np.array(parent["rotation"])[:, 1]
            if np.allclose(top, plate["position"], rtol=0, atol=1e-6):
                if parent["part"] == "3701.dat" or mounted_plate(parent, layers+1):
                    return True
        return False
    regions = {foot["id"]: foot for foot in data["feet"]}
    logical = {foot["id"]: foot for foot in data["motion"]["logicalFeet"]}
    if len(choices) != len(logical) or {c["foot"] for c in choices} != set(logical):
        raise AdapterError("Walker foot choices must cover all logical feet exactly once.")
    if any("contactShape" in foot for foot in logical.values()):
        from .ground import foot_regions
        from .support_surfaces import support_surfaces
        support_surfaces(data, foot_regions(data))
    for choice in choices:
        patches = [regions[name] for name in logical[choice["foot"]]["regions"]]
        source = by_part_id.get(choice["part"])
        if source is None:
            raise AdapterError("Walker foot choices must reference an actual source part.")
        contacts = [choice] + choice.get("additionalContacts", [])
        if (len({contact["part"] for contact in contacts}) != len(contacts)
                or {patch["partInstance"] for patch in patches} != {contact["part"] for contact in contacts}):
            raise AdapterError("Walker physical foot choices must cover their contact parts exactly once.")
        sole = source["part"] in {"3701.dat", "3710.dat"}
        shape = logical[choice["foot"]].get("contactShape")
        if shape is not None:
            for contact in contacts:
                physical = by_part_id.get(contact["part"])
                if physical is None or physical["body"] != source["body"]:
                    raise AdapterError("Additional soles must belong to their logical foot's physical body.")
                if contact is not choice and (
                        not sole or physical["part"] != source["part"]
                        or physical["position"][:2] != source["position"][:2]
                        or physical["rotation"] != source["rotation"]
                        or abs(physical["position"][2] - contact["depth"]) > 1e-6):
                    raise AdapterError("An additional sole must be a parallel physical brick at its declared depth.")
                selected = [patch for patch in patches if patch["partInstance"] == contact["part"]]
                positions = np.array([patch["position"] for patch in selected])
                local = (positions - np.array(physical["position"])) @ np.array(physical["rotation"])
                if shape == "rectangle":
                    underside = 8 if physical["part"] == "3710.dat" else 24
                    if physical["part"] == "3710.dat" and (
                            not mounted_plate(physical)
                            or abs(physical["position"][2] - contact["depth"]) > 1e-6):
                        raise AdapterError("Plate soles require one or two stud-aligned plates under a Technic brick.")
                    expected = np.array([[x, underside, z] for x in (-40, 40) for z in (-10, 10)])
                    valid = sole and len(selected) == 4 and np.allclose(local, expected, rtol=0, atol=1e-6)
                else:
                    half = 5 if physical["part"] == "32063.dat" else 10
                    valid = (not sole and len(selected) == 2
                             and np.allclose(local[:, :1], local[:1, :1], rtol=0, atol=1e-6)
                             and np.allclose(local[:, 2], local[0, 2], rtol=0, atol=1e-6)
                             and np.allclose(local[:, 1], [-half, half], rtol=0, atol=1e-6)
                             and abs(float(positions[:, 2].mean()) - contact["depth"]) < 1e-6)
                if not valid:
                    raise AdapterError("Bounded support differs from the actual sole underside or transverse beam toe.")
            continue
        if len(contacts) != 1:
            raise AdapterError("Additional physical soles require explicit bounded surfaces.")
        if (any(actual["partInstance"] != choice["part"] for actual in patches)
                or (not sole and (len(patches) != 1 or patches[0]["position"][2] != choice["depth"]))
                or (sole and (len(patches) != 4 or sorted(round(p["position"][2]-choice["depth"], 6) for p in patches) != [-10, -10, 10, 10]))):
            raise AdapterError("Walker foot choices differ from their physical contact regions.")


def logical_foot_count(manifest):
    if manifest["grammarVersion"] in ("technic-motion-v2", "technic-motion-v3"):
        return len(manifest["motion"]["logicalFeet"])
    if manifest["grammarVersion"] == "technic-motion-v1":
        raise AdapterError("This archived motion manifest has no logical foot grouping; regenerate it with the current compiler.")
    return len(manifest["feet"])


def library_fingerprint(library: Path, part_names: list[str]) -> str:
    """Reject missing recursive dependencies; upstream otherwise uses empty geometry."""
    if not (library / "parts").is_dir() or not (library / "p").is_dir():
        raise AdapterError(f"Expected an LDraw library containing parts/ and p/: {library}")
    contents: dict[Path, bytes] = {}

    def visit(name: str, parent: Path):
        relative = Path(name.replace("\\", "/"))
        locations = [parent / relative, library / "parts" / relative, library / "p" / relative]
        found = next((candidate.resolve() for candidate in locations if candidate.is_file()), None)
        if found is None:
            raise AdapterError(f"Missing LDraw dependency {name!r}, referenced from {parent}")
        if found in contents:
            return
        raw = found.read_bytes()
        contents[found] = raw
        for line in raw.decode("utf-8-sig").splitlines():
            tokens = line.split()
            if tokens and tokens[0] == "1" and len(tokens) >= 15:
                visit(" ".join(tokens[14:]), found.parent)

    for name in sorted(set(part_names)):
        visit(name, library / "parts")
    digest = hashlib.sha256()
    for file, raw in sorted(contents.items(), key=lambda item: str(item[0])):
        digest.update(str(file.relative_to(library)).replace("\\", "/").encode("utf-8"))
        digest.update(b"\0")
        digest.update(raw)
        digest.update(b"\0")
    return digest.hexdigest()


@dataclass
class CheckedModel:
    manifest: dict
    scene: PhysicsScene
    body_indices: dict[str, int]
    joint_indices: dict[str, int]
    report: dict
    build: LDrawBuild


def _part_identities(manifest, build):
    if len(build.parts) != len(manifest["parts"]):
        raise AdapterError("Parsed part count differs from the flat LDraw manifest.")
    identities = {}
    for actual, expected in zip(build.parts, manifest["parts"], strict=True):
        transform = np.eye(4)
        transform[:3, :3] = expected["rotation"]
        transform[:3, 3] = expected["position"]
        if (actual.part_id.lower() != expected["part"].lower()
                or actual.color != expected["color"]
                or not np.allclose(actual.transform, transform, atol=1e-8, rtol=0)):
            raise AdapterError(f"Parsed placement differs for {expected['id']}.")
        if not actual.triangles:
            raise AdapterError(f"No geometry for {expected['id']}; incomplete parts library.")
        identities[id(actual)] = expected
    return identities


def verify_construction_connectivity(proof, manifest, build):
    identities = _part_identities(manifest, build)
    by_id = {expected["id"]: (index, actual) for index, actual in enumerate(build.parts)
             for expected in [identities[id(actual)]]}
    frame = next(step for step in proof["steps"] if step["goal"] == "frame")
    chassis = manifest["chassis"]
    roots = {part for part in frame["parts"] if part in by_id
             and identities[id(by_id[part][1])]["body"] == chassis}
    required = {part["id"] for part in manifest["parts"] if part["body"] == chassis} - roots
    paths = [goal for goal in proof["goals"] if goal["kind"] == "rigid-path-to-frame"]
    if not roots or {goal["subject"] for goal in paths} != required:
        raise AdapterError("Construction proof must require a rigid path for every new chassis part.")
    library_goals = [goal["id"] for goal in proof["goals"] if goal["kind"] == "exact-rigid-connectivity"]
    if library_goals != ["library-connectivity"]:
        raise AdapterError("Construction proof omitted its exact connectivity obligation.")
    structural = [(i, part) for i, part in enumerate(build.parts) if not is_connector(part.part_id)]
    engagements = {}
    failed, verified = [], []
    for step in proof["steps"]:
        for bond in step["bonds"]:
            connector_id, a, b = bond["connector"], bond["partA"], bond["partB"]
            if len({connector_id, a, b}) != 3 or not {connector_id, a, b}.issubset(by_id):
                raise AdapterError("Construction bond references invalid physical parts.")
            _, connector = by_id[connector_id]
            if classify_connector(connector.part_id) != ConnectorType.FRICTION_PIN:
                raise AdapterError("Rigid bracket bonds require friction pins, not free bearings.")
            if connector_id not in engagements:
                engagements[connector_id] = dict(_find_port_connections(connector, structural))
            selected = engagements[connector_id]
            shaft = _connector_shaft(connector)
            failed_parts, ports = [], []
            for suffix, part_id in (("A", a), ("B", b)):
                index, actual = by_id[part_id]
                position = np.array([bond[f"position{suffix}"][axis] for axis in ("x", "y", "z")])
                axis = np.array([bond[f"axis{suffix}"][coord] for coord in ("x", "y", "z")])
                if not np.isfinite(position).all() or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis), 1):
                    raise AdapterError("Construction bond has an invalid port witness.")
                # Primitive ports can lie on a socket face, +/-10 LDU from its catalogue center.
                matching = [port for port in actual.ports
                            if np.linalg.norm(np.cross(port.position-position, axis)) <= PORT_POSITION_TOLERANCE
                            and abs(float(np.dot(port.position-position, axis))) <= 10 + PORT_POSITION_TOLERANCE
                            and abs(float(np.dot(port.orientation, axis))) > .999
                            and _port_on_shaft(port, *shaft)
                            and _determine_connection_type(ConnectorType.FRICTION_PIN, port.port_type, actual, port) == "rigid"]
                if (identities[id(actual)]["body"] != chassis or selected.get(index) != "rigid" or not matching):
                    failed_parts.append(part_id)
                else:
                    ports.append({"part": part_id, "position": matching[0].position.tolist(),
                                  "axis": matching[0].orientation.tolist(), "kind": matching[0].port_type.name})
            record = {"goal": step["goal"], "connector": connector_id, "partA": a, "partB": b}
            if failed_parts:
                failed.append({**record, "failedParts": failed_parts,
                               "engagements": [{"part": identities[id(build.parts[index])]["id"], "kind": kind}
                                               for index, kind in selected.items()]})
            else:
                verified.append({**record, "ports": ports})
    mount_pins = {part for step in proof["steps"] if step["rule"] in ("two-beam-covered-pin-bracket", "three-beam-covered-pin-bracket", "multi-beam-covered-pin-bracket")
                  for part in step["parts"] if classify_connector(by_id[part][1].part_id) == ConnectorType.FRICTION_PIN}
    if not mount_pins.issubset({bond["connector"] for step in proof["steps"] for bond in step["bonds"]}):
        raise AdapterError("Construction proof omitted a bracket pin's rigid bond.")
    adjacency = {}
    for bond in verified:
        for a, b in ((bond["partA"], bond["partB"]), (bond["partB"], bond["partA"])):
            adjacency.setdefault(a, []).append((b, bond["connector"]))
    witnessed = {root: {"part": root, "frame": root, "edges": []} for root in sorted(roots)}
    queue = list(witnessed)
    for part in queue:
        for other, connector in adjacency.get(part, []):
            if other not in witnessed:
                witnessed[other] = {"part": other, "frame": witnessed[part]["frame"],
                                    "edges": witnessed[part]["edges"] + [{"from": part, "to": other, "connector": connector}]}
                queue.append(other)
    disconnected = sorted(required - witnessed.keys())
    passed = not failed and not disconnected
    return {"schemaVersion": 1, "scope": proof["scope"], "chassis": chassis,
            "status": "pass" if passed else "reject", "verifiedGoals": library_goals if passed else [],
            "bonds": verified, "failedBonds": failed, "disconnectedParts": disconnected,
            "paths": [witnessed[part] for part in sorted(required & witnessed.keys())]}


def _shared_pivot_matches(declared, members, scene):
    reference = declared[0]
    axis = np.asarray(reference["axis"])
    point = np.asarray(reference["position"]) * LDU_TO_METERS
    candidates = [
        index for index, actual in enumerate(scene.joints)
        if actual.joint_type == JointType.REVOLUTE
        and {actual.unit_a_index, actual.unit_b_index}.issubset(members)
        and abs(float(np.dot(actual.axis, axis))) / np.linalg.norm(actual.axis) > 1 - 1e-7
        and np.linalg.norm(np.cross(actual.position - point, axis)) < 1e-6
    ]
    reached = {next(iter(members))}
    for _ in members:
        for index in candidates:
            actual = scene.joints[index]
            pair = {actual.unit_a_index, actual.unit_b_index}
            if reached.intersection(pair):
                reached.update(pair)
    return candidates, reached == members and len(candidates) == len(declared)


def verify_joint_preservation(proof, manifest, build, scene):
    identities = _part_identities(manifest, build)
    by_id = {expected["id"]: actual for actual in build.parts for expected in [identities[id(actual)]]}
    walking = next(step for step in proof["steps"] if step["goal"] == "walking")
    owner = next(step for step in proof["steps"] if step["goal"] == "drive")
    goals = [goal for goal in proof["goals"] if goal["kind"] == "exact-joint-preservation"]
    pivots = {f"pivot-{pivot['id']}": pivot for pivot in manifest["motion"]["sharedPivots"]
              if f"pivot-{pivot['id']}" in walking["parts"]}
    if len(goals) != len(pivots) or {goal["subject"] for goal in goals} != set(pivots):
        raise AdapterError("Construction proof must preserve every retained shared pivot exactly once.")
    physical = {step["goal"] for step in proof["steps"] if step["parts"]}
    if any(goal["id"] not in owner["introduced"] or not physical.issubset(goal["requires"]) for goal in goals):
        raise AdapterError("Joint preservation must be owned by the drive production and depend on every physical producer.")
    structural = [(index, part) for index, part in enumerate(build.parts) if not is_connector(part.part_id)]
    units = {id(part): index for index, unit in enumerate(scene.units) for part in unit.bricks}
    expected_parts = {}
    for entry in manifest["parts"]:
        if entry["body"] is not None:
            expected_parts.setdefault(entry["body"], set()).add(entry["id"])
    records, used = [], set()
    for goal in goals:
        connector = goal["subject"]
        pivot = pivots[connector]
        selected = [(build.parts[index], kind) for index, kind in _find_port_connections(by_id[connector], structural)]
        expected = set(pivot["bodies"])
        bodies = {identities[id(part)]["body"] for part, _ in selected}
        unexpected = sorted(identities[id(part)]["id"] for part, _ in selected if identities[id(part)]["body"] not in expected)
        failures, merged_parts, merged_bodies, split_parts, split_bodies = [], set(), set(), set(), set()
        if bodies != expected:
            failures.append(f"selected bodies {sorted(bodies)} differ from {sorted(expected)}")
        if any(kind != "revolute" for _, kind in selected):
            failures.append("non-revolute connector engagement")
        members = {units[id(part)] for part, _ in selected}
        for part, _ in selected:
            body = identities[id(part)]["body"]
            actual_members = {identities[id(brick)]["id"] for brick in scene.units[units[id(part)]].bricks}
            missing = expected_parts[body] - actual_members
            if missing:
                split_parts.update(missing)
                split_bodies.add(body)
            for brick in scene.units[units[id(part)]].bricks:
                entry = identities[id(brick)]
                if entry["body"] != body:
                    merged_parts.add(entry["id"])
                    merged_bodies.update([body, entry["body"]])
        if merged_bodies:
            failures.append("intended bodies joined an unrelated rigid partition")
        if split_bodies:
            failures.append("intended rigid body split into fragments")
        declared = [joint for joint in manifest["expected"]["joints"] if joint["connector"] == connector]
        if not failures:
            candidates, matched = _shared_pivot_matches(declared, members, scene)
            if not matched or used.intersection(candidates):
                failures.append("revolute joint tree, axis or position changed")
            else:
                used.update(candidates)
        records.append({"goal": goal["id"], "connector": connector, "expectedBodies": sorted(expected),
                        "selected": [{"part": identities[id(part)]["id"], "body": identities[id(part)]["body"], "kind": kind}
                                     for part, kind in selected],
                        "unexpectedParts": unexpected, "mergedParts": sorted(merged_parts),
                        "mergedBodies": sorted(merged_bodies), "failures": failures,
                        "splitParts": sorted(split_parts), "splitBodies": sorted(split_bodies),
                        "status": "reject" if failures else "pass"})
    passed = all(record["status"] == "pass" for record in records)
    return {"schemaVersion": 1, "scope": proof["scope"], "status": "pass" if passed else "reject",
            "verifiedGoals": [record["goal"] for record in records if record["status"] == "pass"], "pivots": records}


def compare_scene(manifest: dict, build, scene: PhysicsScene):
    identities = _part_identities(manifest, build)

    detected = {}
    members = {}
    for index, unit in enumerate(scene.units):
        ids = set()
        for brick in unit.bricks:
            part = identities.get(id(brick))
            if part is None or part["body"] is None:
                raise AdapterError("Simulator assigned an unexpected part to a rigid body.")
            ids.add(part["id"])
        detected[index] = ids
    for part in manifest["parts"]:
        if part["body"] is not None:
            members.setdefault(part["body"], set()).add(part["id"])
    mapping = {}
    for body, expected_members in members.items():
        matches = [index for index, ids in detected.items() if ids == expected_members]
        if len(matches) != 1:
            overlapping = {str(index): sorted(ids) for index, ids in detected.items() if ids & expected_members}
            raise AdapterError(f"Rigid-body partition mismatch for {body}: expected {sorted(expected_members)}, detected {overlapping}")
        mapping[body] = matches[0]
    if len(mapping) != len(scene.units):
        raise AdapterError("Unexpected additional rigid bodies.")

    joint_mapping = {}
    used_joints = set()
    if manifest["grammarVersion"] in MOTION_GRAMMARS:
        for pivot in manifest["motion"]["sharedPivots"]:
            declared = [joint for joint in manifest["expected"]["joints"]
                        if joint["connector"] == f"pivot-{pivot['id']}"]
            members = {mapping[body] for body in pivot["bodies"]}
            reference = declared[0]
            candidates, matched = _shared_pivot_matches(declared, members, scene)
            if not matched or used_joints.intersection(candidates):
                connector = reference["connector"]
                pin = next((part for part in build.parts if identities[id(part)]["id"] == connector), None)
                structural = [(index, part) for index, part in enumerate(build.parts) if not is_connector(part.part_id)]
                selected = [] if pin is None else [
                    (identities[id(build.parts[index])]["id"], identities[id(build.parts[index])]["body"], kind)
                    for index, kind in _find_port_connections(pin, structural)]
                competing = [part for part, body, _ in selected if body not in pivot["bodies"]]
                raise AdapterError(
                    f"Shared pivot reconstruction mismatch: {pivot['id']}. "
                    f"Connector {connector} selected {selected}, expected bodies {pivot['bodies']}.",
                    repair_hints={"kind": "joint-port-engagement", "parts": sorted({connector, *competing}),
                                  "bodies": sorted(pivot["bodies"])})
            for joint, index in zip(declared, candidates, strict=True):
                joint_mapping[joint["id"]] = index
            used_joints.update(candidates)
    for joint in manifest["expected"]["joints"]:
        if joint["id"] in joint_mapping:
            continue
        endpoints = {mapping[joint["bodyA"]], mapping[joint["bodyB"]]}
        axis = np.array(joint["axis"])
        point = np.array(joint["position"]) * LDU_TO_METERS
        matches = []
        for index, actual in enumerate(scene.joints):
            if {actual.unit_a_index, actual.unit_b_index} != endpoints or actual.joint_type != JointType.REVOLUTE:
                continue
            alignment = abs(float(np.dot(actual.axis, axis))) / np.linalg.norm(actual.axis)
            offset = np.linalg.norm(np.cross(actual.position - point, axis))
            if alignment > 1 - 1e-7 and offset < 1e-6:
                matches.append(index)
        if len(matches) != 1 or matches[0] in used_joints:
            raise AdapterError(f"Joint mismatch for {joint['id']}: {matches}")
        joint_mapping[joint["id"]] = matches[0]
        used_joints.add(matches[0])
    if len(used_joints) != len(scene.joints):
        raise AdapterError(f"Unexpected joints: expected {len(used_joints)}, detected {len(scene.joints)}.")

    expected_gears = {}
    for gear in manifest["expected"]["gears"]:
        a, b = mapping[gear["bodyA"]], mapping[gear["bodyB"]]
        pair = tuple(sorted((a, b)))
        if pair in expected_gears:
            raise AdapterError("Duplicate expected gear mesh.")
        expected_gears[pair] = gear["ratio"] if a < b else 1 / gear["ratio"]
    actual_gears = {}
    for gear in scene.gears:
        a, b = gear.unit_a_index, gear.unit_b_index
        pair = tuple(sorted((a, b)))
        if pair in actual_gears:
            raise AdapterError("Duplicate detected gear mesh.")
        actual_gears[pair] = gear.ratio if a < b else 1 / gear.ratio
    if expected_gears.keys() != actual_gears.keys() or any(
        not math.isclose(ratio, actual_gears[pair], rel_tol=1e-7) for pair, ratio in expected_gears.items()
    ):
        raise AdapterError(f"Gear graph mismatch: expected {expected_gears}, detected {actual_gears}.")
    expected_motor_joints = [joint_mapping[drive["joint"]] for drive in manifest["expected"]["drives"]]
    if sorted(expected_motor_joints) != sorted(motor.joint_index for motor in scene.motors):
        raise AdapterError("Motor inference selected the wrong driven joint(s).")
    reached = {mapping[manifest["chassis"]]}
    while True:
        previous = len(reached)
        for joint in scene.joints:
            if joint.unit_a_index in reached or joint.unit_b_index in reached:
                reached.update((joint.unit_a_index, joint.unit_b_index))
        if previous == len(reached):
            break
    if len(reached) != len(scene.units):
        raise AdapterError("Disconnected mechanism.")
    return mapping, joint_mapping


class PartParserCache:
    """Reuse library geometry/ports only for plain, library-only LDraw builds."""

    def __init__(self):
        self.invalidate()

    def invalidate(self):
        self._library = None
        self._dependencies = {}
        self._parser = None

    def parser_for(self, path, library, parts):
        lines = path.read_text(encoding="utf-8").splitlines()
        tokens = [line.split(maxsplit=14) for line in lines if line.strip()]
        if (any(line[:2] == ["0", "FILE"] for line in (line.upper().split() for line in lines))
                or any(line[0] not in ("0", "1") for line in tokens)
                or {line[-1] for line in tokens if line[0] == "1"} != set(parts)
                or any((path.parent / part).exists() for part in parts)):
            return LDrawParser(parts_dir=library)
        if self._library != library:
            self.invalidate()
        dependencies = {part: library_fingerprint(library, [part]) for part in set(parts) | self._dependencies.keys()}
        if (self._library != library or self._parser is None
                or any(part in self._dependencies and self._dependencies[part] != fingerprint
                       for part, fingerprint in dependencies.items())):
            self._parser = LDrawParser(parts_dir=library)
            self._dependencies = {}
            self._library = library
        self._dependencies.update(dependencies)
        return self._parser


def _canonical_chassis_scene(scene, body_indices, chassis):
    chassis_index = body_indices[chassis]
    if chassis_index == 0:
        return scene, body_indices
    order = [chassis_index] + [index for index in range(len(scene.units)) if index != chassis_index]
    remap = {old: new for new, old in enumerate(order)}
    canonical = replace(
        scene, units=[scene.units[index] for index in order],
        joints=[replace(joint, unit_a_index=remap[joint.unit_a_index], unit_b_index=remap[joint.unit_b_index])
                for joint in scene.joints],
        gears=[replace(gear, unit_a_index=remap[gear.unit_a_index], unit_b_index=remap[gear.unit_b_index])
               for gear in scene.gears])
    return canonical, {body: remap[index] for body, index in body_indices.items()}


def check(path: Path, library: Path, *, parser_cache: PartParserCache | None = None,
          construction_proof: dict | None = None) -> CheckedModel:
    path, library = path.resolve(), library.expanduser().resolve()
    manifest = load_manifest(path)
    fingerprint = library_fingerprint(library, [part["part"] for part in manifest["parts"]])
    distribution = importlib.metadata.distribution("lego-technic-sim")
    direct = distribution.read_text("direct_url.json")
    source = json.loads(direct) if direct else {}
    installed_revision = source.get("vcs_info", {}).get("commit_id")
    if installed_revision and installed_revision != SIMULATOR_REVISION:
        raise AdapterError(f"Installed simulator revision {installed_revision} differs from {SIMULATOR_REVISION}.")
    parsed = False
    try:
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            model = path.parent / manifest["modelFile"]
            parser = (LDrawParser(parts_dir=library) if parser_cache is None else
                      parser_cache.parser_for(model, library, [part["part"] for part in manifest["parts"]]))
            build = parser.parse_build(model)
            scene = build_units_and_joints(build)
        parsed = True
    finally:
        if not parsed and parser_cache is not None:
            parser_cache.invalidate()
    if caught:
        if parser_cache is not None:
            parser_cache.invalidate()
        raise AdapterError("Simulator warnings: " + "; ".join(str(item.message) for item in caught))
    connectivity, preservation = None, None
    if construction_proof is not None:
        preservation = verify_joint_preservation(construction_proof, manifest, build, scene)
        preservation.update(ldrawSha256=manifest["ldrawSha256"], partsDependencySha256=fingerprint)
        connectivity = verify_construction_connectivity(construction_proof, manifest, build)
        connectivity.update(ldrawSha256=manifest["ldrawSha256"], partsDependencySha256=fingerprint)
        competition = any(pivot["unexpectedParts"] for pivot in preservation["pivots"] if pivot["status"] == "reject")
        if preservation["status"] != "pass" and (competition or connectivity["status"] == "pass"):
            raise JointPreservationError(preservation, connectivity)
        if connectivity["status"] != "pass":
            error = ConstructionConnectivityError(connectivity)
            error.joint_preservation = preservation
            raise error
    try:
        bodies, joints = compare_scene(manifest, build, scene)
    except AdapterError as error:
        error.construction_connectivity = connectivity
        error.joint_preservation = preservation
        raise
    scene, bodies = _canonical_chassis_scene(scene, bodies, manifest["chassis"])
    report = {
        "schemaVersion": 1, "status": "compatible", "modelKind": manifest["modelKind"],
        "ldrawSha256": manifest["ldrawSha256"], "partsDependencySha256": fingerprint,
        "simulatorRevision": installed_revision,
        "expectedSimulatorRevision": SIMULATOR_REVISION,
        "parts": len(build.parts), "bodies": len(scene.units), "joints": len(scene.joints),
        "gearMeshes": len(scene.gears), "motors": len(scene.motors),
        "bodyIndices": bodies, "jointIndices": joints,
        "warnings": manifest["warnings"] + [
            "Upstream compatibility units exclude connectors; evaluation reattaches their geometry/mass.",
            *([] if installed_revision else ["Installed simulator revision cannot be verified from package metadata."]),
        ],
    }
    if connectivity is not None:
        report["constructionConnectivity"] = connectivity
        report["jointPreservation"] = preservation
    return CheckedModel(manifest, scene, bodies, joints, report, build)
