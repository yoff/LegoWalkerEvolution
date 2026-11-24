"""Part hulls and thin axle rods, with narrowly verified socket exemptions."""

from dataclasses import dataclass, replace
from copy import deepcopy
from itertools import combinations
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from lego_technic_sim.ldraw.model import LDrawPart
from lego_technic_sim.mujoco_export import _write_stl
from lego_technic_sim.physics.connection_ports import PortType
from lego_technic_sim.physics.connectors import ConnectorType, classify_connector
from lego_technic_sim.physics.model import PhysicsScene
from lego_technic_sim.physics.motor_detection import is_motor_part

from .bridge import AdapterError, CheckedModel, LDU_TO_METERS

AXLE_ROD_RADIUS_METERS = 0.0012
MAX_STL_FACES = 200000


@dataclass(frozen=True)
class CollisionPart:
    instance: str
    geometry: LDrawPart
    unit: int


def to_mujoco(value):
    return np.asarray(value)[..., [0, 2, 1]] * [1, -1, -1]


def vector_text(value):
    return " ".join(f"{coordinate:.12g}" for coordinate in value)


def prepare_parts(checked: CheckedModel, *, profile=None):
    """Attach every connector, without changing the checked structural graph."""
    from .payload import load_profile, part_mass_center
    parts = checked.manifest["parts"]
    geometry = dict(zip((part["id"] for part in parts), checked.build.parts, strict=True))
    vertices_by_id = {
        part_id: np.array([vertex for triangle in item.triangles
                           for vertex in (triangle.v0, triangle.v1, triangle.v2)])
        for part_id, item in geometry.items()
    }
    units = {part["id"]: checked.body_indices[part["body"]]
             for part in parts if part["body"] is not None}
    exemptions = {}
    joints_by_connector = {}
    for joint in checked.manifest["expected"]["joints"]:
        joints_by_connector.setdefault(joint["connector"], []).append(joint)
    for source in parts:
        if source["body"] is not None:
            continue
        connector = geometry[source["id"]]
        kind = classify_connector(source["part"])
        if kind not in (ConnectorType.AXLE, ConnectorType.FRICTION_PIN, ConnectorType.FRICTIONLESS_PIN):
            raise AdapterError(f"No collision attachment recipe for {source['id']}.")
        axis = np.asarray(source["rotation"])[:, 0]
        # Use the real mesh extent, not an axis-aligned world bounding box.
        vertices = vertices_by_id[source["id"]]
        distances = (vertices - connector.position) @ axis
        lo, hi = distances.min(), distances.max()
        mates = []
        rigid_units = set()
        for target in parts:
            if target["body"] is None:
                continue
            for port in geometry[target["id"]].ports:
                compatible = port.port_type in (PortType.ROUND_HOLE, PortType.AXLE_HOLE)
                if kind != ConnectorType.AXLE:
                    compatible = port.port_type == PortType.ROUND_HOLE
                delta = port.position - connector.position
                along = float(delta @ axis)
                if (compatible and abs(float(port.orientation @ axis)) > 1 - 1e-7
                        and np.linalg.norm(np.cross(delta, axis)) < 1e-5
                        and lo - 1e-5 <= along <= hi + 1e-5):
                    target_depths = (vertices_by_id[target["id"]]-connector.position) @ axis
                    insertion = min(hi, target_depths.max())-max(lo, target_depths.min())
                    if insertion <= 1e-5:
                        continue
                    mates.append(target)
                    if (kind == ConnectorType.FRICTION_PIN
                            or (kind == ConnectorType.AXLE and port.port_type == PortType.AXLE_HOLE
                                and not is_motor_part(target["part"]))):
                        rigid_units.add(units[target["id"]])
                    exemptions[tuple(sorted((source["id"], target["id"])))] = "verified socket"
                    break
        joints = joints_by_connector.get(source["id"], [])
        if len(joints) > 1:
            first = joints[0]
            joint_axis = np.asarray(first["axis"])
            joint_axis = joint_axis / np.linalg.norm(joint_axis)
            for joint in joints:
                if (np.linalg.norm(np.cross(np.asarray(joint["axis"]), joint_axis)) > 1e-7
                        or np.linalg.norm(np.cross(np.asarray(joint["position"])-first["position"], joint_axis)) > 1e-5):
                    raise AdapterError(f"Shared connector {source['id']} has noncoaxial hinge declarations.")
            reached = {first["bodyA"]}
            for _ in joints:
                for joint in joints:
                    if reached.intersection((joint["bodyA"], joint["bodyB"])):
                        reached.update((joint["bodyA"], joint["bodyB"]))
            declared = {joint[key] for joint in joints for key in ("bodyA", "bodyB")}
            if reached != declared or len(joints) != len(declared)-1:
                raise AdapterError(f"Shared connector {source['id']} must declare a connected hinge tree.")
        if kind == ConnectorType.FRICTIONLESS_PIN:
            if not joints:
                raise AdapterError(f"No hinge attachment for {source['id']}.")
            owner = checked.body_indices[joints[0]["bodyB"]]
        else:
            if len(rigid_units) != 1:
                raise AdapterError(f"Connector {source['id']} has ambiguous rigid attachment: {rigid_units}.")
            owner = rigid_units.pop()
        allowed_units = ({checked.body_indices[joint[key]] for joint in joints for key in ("bodyA", "bodyB")}
                         if joints else {owner})
        if not mates or owner not in {units[part["id"]] for part in mates}:
            raise AdapterError(f"Connector {source['id']} does not engage its attachment body.")
        if any(units[part["id"]] not in allowed_units for part in mates):
            raise AdapterError(f"Connector {source['id']} intersects an undeclared socket/body.")
        units[source["id"]] = owner

    # Exempt only the two actual meshing gear parts, never their entire bodies.
    for gear in checked.manifest["expected"]["gears"]:
        teeth = {"3647.dat": 8, "3648b.dat": 24, "3649.dat": 40}
        endpoints = [[part for part in parts if part["body"] == gear[body]
                      and part["part"] in teeth] for body in ("bodyA", "bodyB")]
        pairs = [(a, b) for a in endpoints[0] for b in endpoints[1]
                 if math.isclose(teeth[a["part"]] / teeth[b["part"]], gear["ratio"], abs_tol=1e-10)
                 and math.isclose(a["position"][2], b["position"][2], abs_tol=1e-6)
                 and math.isclose(float(np.linalg.norm(np.asarray(a["position"])-b["position"])),
                                  (teeth[a["part"]]+teeth[b["part"]])*1.25, abs_tol=1e-6)]
        if len(pairs) != 1:
            raise AdapterError("No unambiguous part-level gear-contact recipe.")
        exemptions[tuple(sorted(part["id"] for part in pairs[0]))] = "idealized gear teeth"

    attached = [CollisionPart(part["id"], geometry[part["id"]], units[part["id"]]) for part in parts]
    declarations = checked.manifest.get("payloads", [])
    if declarations and profile is None:
        profile = load_profile()
    payloads = {payload["partInstance"]: profile["components"][payload["product"]] for payload in declarations}
    mass_properties = {part.instance: part_mass_center(part, payloads.get(part.instance)) for part in attached}
    updated_units = []
    for index, unit in enumerate(checked.scene.units):
        connectors = [part for part in attached
                      if part.unit == index and classify_connector(part.geometry.part_id) is not None]
        components = [mass_properties[part.instance] for part in attached if part.unit == index]
        mass = sum(value[0] for value in components)
        weighted = sum((component_mass*center for component_mass, center in components), np.zeros(3))
        if not math.isfinite(mass) or mass <= 0 or not np.isfinite(weighted).all():
            raise AdapterError(f"Invalid prepared body mass properties: unit_{index}.")
        center = weighted / mass
        vertices = np.concatenate([vertices_by_id[part.instance] for part in attached if part.unit == index])
        if (np.any(center < vertices.min(axis=0)*LDU_TO_METERS-1e-6)
                or np.any(center > vertices.max(axis=0)*LDU_TO_METERS+1e-6)):
            raise AdapterError(f"Prepared body center outside physical geometry: unit_{index}.")
        updated_units.append(replace(unit, bricks=[*unit.bricks, *(part.geometry for part in connectors)],
                                     mass=mass, center_of_mass=center))
    scene = replace(checked.scene, units=updated_units)
    return scene, attached, exemptions


def split_large_visuals(root, scene, parts, mesh_dir):
    asset = root.find("asset")
    bodies = {body.attrib["name"]: body for body in root.findall(".//body")}
    for index, unit in enumerate(scene.units):
        if sum(len(brick.triangles) for brick in unit.bricks) <= MAX_STL_FACES:
            continue
        name = f"mesh_{index}"
        original = asset.find(f"mesh[@name='{name}']")
        if original is None:
            raise AdapterError(f"Missing upstream visual mesh for oversized unit {index}.")
        body = bodies[f"unit_{index}"]
        for geom in list(body.findall("geom")):
            if geom.get("mesh") != name:
                continue
            if geom.get("contype") != "0" or geom.get("conaffinity") != "0":
                raise AdapterError("Only non-colliding visualization meshes may be split.")
            for number, part in enumerate(parts):
                if part.unit != index:
                    continue
                mesh_name = f"mesh_part_{number}"
                if asset.find(f"mesh[@name='{mesh_name}']") is None:
                    vertices = np.array([v for t in part.geometry.triangles for v in (t.v0, t.v1, t.v2)])
                    local = to_mujoco(vertices * LDU_TO_METERS - unit.center_of_mass)
                    filename = f"visual_part_{number}.stl"
                    _write_stl(mesh_dir / filename, local, np.arange(len(local)).reshape(-1, 3))
                    ET.SubElement(asset, "mesh", name=mesh_name, file=filename)
                visual = deepcopy(geom)
                visual.set("name", f"visual_part_{number}")
                visual.set("mesh", mesh_name)
                body.append(visual)
            body.remove(geom)
        asset.remove(original)


def validate_floor_contact_time_constant(value: float | None, timestep: float):
    if value is not None and (
            type(value) not in (int, float) or not math.isfinite(value)
            or value <= 0 or value < 2*timestep):
        raise AdapterError("floor_contact_time_constant must be finite, positive and at least two timesteps; "
                           "smaller values would be silently clamped by MuJoCo.")


def configure_floor_contacts(root: ET.Element, time_constant: float | None):
    if time_constant is None:
        return
    option = root.find("option")
    if option is None or "timestep" not in option.attrib:
        raise AdapterError("Explicit floor contacts require an explicit simulation timestep.")
    validate_floor_contact_time_constant(time_constant, float(option.attrib["timestep"]))
    pairs = [pair for pair in root.findall("contact/pair")
             if "ground" in (pair.get("geom1"), pair.get("geom2"))]
    if not pairs:
        raise AdapterError("Explicit floor contacts require active part-floor pairs, not an anchored rig.")
    for pair in pairs:
        pair.set("solref", vector_text([time_constant, 1.0]))


def install_contacts(root: ET.Element, scene: PhysicsScene, parts: list[CollisionPart],
                     exemptions: dict, mesh_dir: Path, *, enabled: bool, anchor: bool,
                     friction: float, floor_contact_time_constant: float | None = None):
    """Explicit part pairs also cover parent/child bodies and loop neighbors."""
    bodies = {body.attrib["name"]: body for body in root.findall(".//body")}
    for geom in root.findall(".//geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        geom.set("friction", f"{friction} 0.005 0.0001")
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    names = {}
    for index, part in enumerate(parts):
        if len(part.geometry.triangles) > MAX_STL_FACES:
            raise AdapterError(f"Individual part {part.instance} exceeds MuJoCo's {MAX_STL_FACES}-face STL limit.")
        name = f"part_{index}"
        names[part.instance] = name
        vertices = np.array([vertex for triangle in part.geometry.triangles
                             for vertex in (triangle.v0, triangle.v1, triangle.v2)])
        local = to_mujoco(vertices * LDU_TO_METERS - scene.units[part.unit].center_of_mass)
        if classify_connector(part.geometry.part_id) == ConnectorType.AXLE:
            axis = part.geometry.transform[:3, 0]
            distances = (vertices - part.geometry.position) @ axis
            endpoints = part.geometry.position + np.array([distances.min(), distances.max()])[:, None] * axis
            endpoints = to_mujoco(endpoints * LDU_TO_METERS - scene.units[part.unit].center_of_mass)
            ET.SubElement(bodies[f"unit_{part.unit}"], "geom", name=name, type="cylinder",
                          fromto=vector_text(endpoints.reshape(-1)),
                          size=str(AXLE_ROD_RADIUS_METERS), contype="0", conaffinity="0",
                          group="3", rgba="0 0 0 0")
            continue
        faces = np.arange(len(local)).reshape(-1, 3)
        _write_stl(mesh_dir / f"{name}.stl", local, faces)
        ET.SubElement(asset, "mesh", name=f"mesh_{name}", file=f"{name}.stl")
        ET.SubElement(bodies[f"unit_{part.unit}"], "geom", name=name, type="mesh",
                      mesh=f"mesh_{name}", contype="0", conaffinity="0",
                      group="3", rgba="0 0 0 0")
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    # Upstream body-pair exclusions must not hide non-mating parts of a hinge.
    contact.clear()
    pairs = 0
    for left, right in combinations(parts, 2):
        if (not enabled or left.unit == right.unit
                or tuple(sorted((left.instance, right.instance))) in exemptions):
            continue
        ET.SubElement(contact, "pair", geom1=names[left.instance], geom2=names[right.instance],
                      condim="3", friction=f"{friction} {friction} 0.005 0.0001 0.0001")
        pairs += 1
    if not anchor:
        for name in names.values():
            ET.SubElement(contact, "pair", geom1=name, geom2="ground",
                          condim="3", friction=f"{friction} {friction} 0.005 0.0001 0.0001")
    configure_floor_contacts(root, floor_contact_time_constant)
    split_large_visuals(root, scene, parts, mesh_dir)
    return {
        "mode": "part-hulls-and-axle-rods" if enabled else "ground-only-preview",
        "axleRodRadiusMeters": AXLE_ROD_RADIUS_METERS,
        "partGeometries": len(parts),
        "connectorGeometries": sum(classify_connector(part.geometry.part_id) is not None for part in parts),
        "selfContactPairs": pairs,
        "bodyPairExclusions": 0,
        "geometryInstances": {name: instance for instance, name in names.items()},
        "socketAndGearExemptions": [{"parts": list(pair), "reason": reason}
                                    for pair, reason in sorted(exemptions.items())],
    }
