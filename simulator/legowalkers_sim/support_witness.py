"""Isolate an archived foot hull for a zero-step support-classification witness."""

import argparse
import contextlib
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sqlite3
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from .bridge import AdapterError, LDU_TO_METERS, write_json
from .collision import to_mujoco, vector_text
from .ground import GroundContacts
from .support_surfaces import VERSION


def witness(archive, trial, output):
    archive, output = Path(archive).resolve(), Path(output).resolve()
    if output.is_relative_to(archive) or (output.exists() and any(output.iterdir())):
        raise AdapterError("Write a support witness to an empty directory outside the scientific archive.")
    with contextlib.closing(sqlite3.connect(f'{(archive / "archive.sqlite3").as_uri()}?mode=ro', uri=True)) as db:
        row = db.execute("SELECT b.design FROM evaluations e JOIN births b ON b.id=e.birth WHERE e.id=?", (trial,)).fetchone()
    if row is None:
        raise AdapterError("Support witness requires an existing archived trial.")
    manifest_path = archive / "designs" / row[0] / "candidate/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    source = archive / "evaluations" / f"{trial:04d}"
    gate = json.loads((source / "simulation-gate.json").read_text())
    touching = gate["diagnostics"]["groundContact"]["forbiddenContacts"][0]["part"]
    part = next(part for part in manifest["parts"] if part["id"] == touching)
    indices = [index for index, foot in enumerate(manifest["feet"]) if foot.get("partInstance") == touching]
    if not indices:
        raise AdapterError("The selected rejection is not on a designated foot part.")
    model = mujoco.MjModel.from_xml_path(str(source / "model.xml"))
    state = mujoco.MjData(model)
    mujoco.mj_forward(model, state)
    mapping = json.loads((source / "collision-model.json").read_text())["geometryInstances"]
    geom_name = next(name for name, instance in mapping.items() if instance == touching)
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
        raise AdapterError("Support witness requires the actual archived foot mesh.")
    mesh = model.geom_dataid[geom]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    vertices = model.mesh_vert[start:start+count] @ state.geom_xmat[geom].reshape(3, 3).T + state.geom_xpos[geom]
    start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    faces = model.mesh_face[start:start+count]
    points = np.array([state.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"foot_{index}")]
                       for index in indices])
    old_feet = [deepcopy(manifest["feet"][index]) for index in indices]
    kind = "rectangle" if part["part"] == "3701.dat" else "capsule"
    new_feet = deepcopy(old_feet)
    new_points = points.copy()
    if kind == "capsule":
        if len(indices) != 1:
            raise AdapterError("Beam regression witness requires one original spherical toe region.")
        half = 5 if part["part"] == "32063.dat" else 10
        axis = to_mujoco(np.array(part["rotation"])[:, 1])
        new_points = points[0] + np.array([-half, half])[:, None] * LDU_TO_METERS * axis
        new_feet = [{**old_feet[0], "id": f"toe-{index}"} for index in range(2)]
    elif len(indices) != 4:
        raise AdapterError("Shoe regression witness requires its four original underside corners.")
    logical_id = old_feet[0]["logicalFoot"]
    for foot, point in zip(new_feet, new_points, strict=True):
        foot["position"] = (point / LDU_TO_METERS).tolist()
    repaired = {
        "feet": new_feet, "parts": manifest["parts"], "groundWheels": [],
        "motion": {"logicalFeet": [{"id": logical_id, "body": part["body"], "contactShape": kind,
                                   "regions": [foot["id"] for foot in new_feet]}]},
    }
    legacy = {"feet": old_feet, "parts": manifest["parts"], "groundWheels": []}
    shift = -float(vertices[:, 2].min()) - .0001
    root = ET.Element("mujoco")
    ET.SubElement(root, "option", gravity="0 0 0")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "mesh", name="foot", vertex=vector_text(vertices.reshape(-1)),
                  face=" ".join(str(int(index)) for index in faces.reshape(-1)))
    world = ET.SubElement(root, "worldbody")
    fixture = "plane"
    if kind == "rectangle":
        normal = np.cross(points[1]-points[0], points[2]-points[0])
        normal /= np.linalg.norm(normal)
        if normal[2] < 0:
            normal = -normal
        probe = points.mean(axis=0) + [0, 0, shift] - normal * .0009
        ET.SubElement(world, "geom", name="ground", type="sphere", size=".001", pos=vector_text(probe))
        fixture = "localized-underside-probe"
    else:
        ET.SubElement(world, "geom", name="ground", type="plane", size="1 1 .1")
    body = ET.SubElement(world, "body", pos=vector_text([0, 0, shift]))
    ET.SubElement(body, "freejoint")
    ET.SubElement(body, "geom", name="foot", type="mesh", mesh="foot")
    output.mkdir(parents=True, exist_ok=True)
    path = output / "static-foot.xml"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    isolated = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(isolated)
    mujoco.mj_forward(isolated, data)
    points[:, 2] += shift
    new_points[:, 2] += shift
    old_monitor, new_monitor = GroundContacts(legacy), GroundContacts(repaired)
    contacts = []
    for contact in data.contact:
        if contact.dist > 0:
            continue
        position = contact.pos.copy()
        old_monitor.observe(touching, position, float(contact.dist), 0, points)
        new_monitor.observe(touching, position, float(contact.dist), 0, new_points)
        contacts.append({"positionMeters": position.tolist(), "distanceMeters": float(contact.dist)})
    if not contacts:
        raise AdapterError("Isolated static witness produced no touching foot contacts.")
    negative = GroundContacts(repaired)
    negative.observe("non-foot-rocker", contacts[0]["positionMeters"], -.0001, 0, new_points)
    if kind == "rectangle":
        normal = np.cross(new_points[1]-new_points[0], new_points[2]-new_points[0])
        outside = new_points.mean(axis=0) + normal / np.linalg.norm(normal) * .0096
    else:
        outside = new_points.mean(axis=0) + to_mujoco(np.array(part["rotation"])[:, 2]) * .03
    negative.observe(touching, outside, -.0001, 0, new_points)
    report = {
        "schemaVersion": 1, "version": VERSION, "mode": "isolated-static-support-witness",
        "sourceArchive": str(archive), "sourceTrial": trial, "partInstance": touching, "part": part["part"],
        "sourceManifestSha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "sourceModelSha256": hashlib.sha256((source / "model.xml").read_bytes()).hexdigest(),
        "sourceGateSha256": hashlib.sha256((source / "simulation-gate.json").read_bytes()).hexdigest(),
        "contactShape": kind, "groundFixture": fixture,
        "unchangedRadiusMeters": old_feet[0]["contactRadiusMeters"],
        "contacts": contacts, "legacy": old_monitor.report(), "bounded": new_monitor.report(),
        "negativePredicateProbes": negative.report(),
        "simulatedSteps": 0, "simulationSeconds": 0, "campaignTrials": 0,
        "limitations": [
            "Same archived foot hull at its initial orientation, against a plane or localized underside probe; not a replay of the recorded contact-time state.",
            "Only the foot is isolated. This does not establish whole-walker clearance, gate survival or walking.",
            "Negative predicate probes are classification tests, not observed physical contacts.",
            "No archived measurement or qualification was changed.",
        ],
    }
    write_json(output / "witness.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--trial", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = witness(args.archive, args.trial, args.output)
    print(json.dumps({key: result[key] for key in ("contactShape", "legacy", "bounded", "negativePredicateProbes", "simulatedSteps")}, indent=2))


if __name__ == "__main__":
    main()
