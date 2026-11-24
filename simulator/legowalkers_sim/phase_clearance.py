"""Prescribed full-assembly collision checks; never integrate dynamics."""

import math
from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from .bridge import AdapterError, LDU_TO_METERS, write_json
from .collision import to_mujoco, vector_text

VERSION = "prescribed-phase-clearance-v1"
MAX_FRAMES = 2329


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _frames(document, manifest, bodies):
    if (not isinstance(document, dict) or document.get("version") != "prescribed-motion-phases-v1"
            or any(document.get(key) != manifest["motion"][key]
                   for key in ("mechanismId", "genotypeId", "assemblyId"))
            or not isinstance(document.get("parts"), list)
            or any(not isinstance(part, str) for part in document["parts"])
            or sorted(document["parts"]) != sorted(p["id"] for p in manifest["parts"])):
        raise AdapterError("Prescribed phases must identify the exact complete physical assembly.")
    frames = document.get("frames")
    if not isinstance(frames, list) or not 37 <= len(frames) <= MAX_FRAMES:
        raise AdapterError("Prescribed clearance requires 37 to 2329 bounded phase samples.")
    result = []
    for frame in frames:
        if not isinstance(frame, dict) or not _number(frame.get("inputAngleRadians")):
            raise AdapterError("Prescribed input angles must be finite numbers.")
        poses = frame.get("bodies")
        if (not isinstance(poses, list) or len(poses) != len(bodies)
                or any(not isinstance(pose, dict) or not isinstance(pose.get("id"), str) for pose in poses)
                or {pose["id"] for pose in poses} != set(bodies)):
            raise AdapterError("Every phase must cover every reconstructed rigid body exactly once.")
        by_id = {}
        for pose in poses:
            origin = pose.get("originLdu")
            if (not isinstance(origin, list) or len(origin) != 3 or not all(_number(v) for v in origin)
                    or not _number(pose.get("angleRadians"))):
                raise AdapterError("Prescribed rigid-body poses must be finite planar transforms.")
            by_id[pose["id"]] = (np.array(origin) * LDU_TO_METERS, pose["angleRadians"])
        result.append((frame["inputAngleRadians"], by_id))
    angles = np.array([angle for angle, _ in result])
    if (abs(angles[0]) > 1e-8 or not np.all(np.diff(angles) > 0)
            or np.max(np.diff(angles)) > 2 * math.pi / 36 + 1e-8
            or abs(angles[-1] / (2 * math.pi) - round(angles[-1] / (2 * math.pi))) > 1e-6
            or not 1 - 1e-8 <= angles[-1] / (2 * math.pi) <= 32 + 1e-8):
        raise AdapterError("Prescribed clearance needs ordered samples spanning one to 32 motor turns.")
    return result


def _rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def evaluate_phases(checked, model_path, collision, document, *, tolerance):
    import mujoco

    if not _number(tolerance) or tolerance <= 0:
        raise AdapterError("Prescribed clearance requires a positive finite penetration tolerance.")
    frames = _frames(document, checked.manifest, checked.body_indices)
    report = {
        "version": VERSION, "status": "incomplete", "sampledPoses": 0,
        "plannedPoses": len(frames), "motorTurnsCovered": frames[-1][0] / (2 * math.pi),
        "dynamicTrials": 0, "simulationSeconds": 0.0, "contacts": [],
        "penetrationToleranceMeters": tolerance, "partCount": len(checked.manifest["parts"]),
        "limitations": [
            "Prescribed kinematics using the existing simulator collision geometry and socket exemptions.",
            "Finite samples, not continuous collision certification; between-sample collisions can be missed.",
            "No ground-support, settling, deformation, load or walking qualification.",
        ],
    }
    if document.get("endpointRepeats") is not True or document.get("samplingFailure") is not None:
        report["reason"] = "incomplete-gait-sampling"
        return report
    for body in checked.body_indices:
        origin, angle = frames[0][1][body]
        end_origin, end_angle = frames[-1][1][body]
        if (np.linalg.norm(end_origin - origin) > 1e-8
                or np.max(np.abs(_rotation(end_angle - angle) - np.eye(3))) > 1e-6):
            report["reason"] = "nonrepeating-body-pose"
            return report

    model_path = Path(model_path)
    original = mujoco.MjModel.from_xml_path(str(model_path))
    reference = mujoco.MjData(original)
    mujoco.mj_forward(original, reference)
    root = ET.parse(model_path).getroot()
    world = root.find("worldbody")
    units = {int(body.attrib["name"].removeprefix("unit_")): body
             for body in root.findall(".//body") if body.get("name", "").startswith("unit_")}
    if (set(units) != set(checked.body_indices.values())
            or len(units) != len(checked.body_indices)
            or len(root.findall(".//body")) != len(units)):
        raise AdapterError("Prescribed clearance requires an unambiguous complete physical body map.")
    centers = collision.get("unitCentersMeters")
    if (not isinstance(centers, dict) or set(centers) != {str(index) for index in units}
            or any(not isinstance(p, list) or len(p) != 3 or not all(_number(v) for v in p)
                   for p in centers.values())):
        raise AdapterError("Prescribed clearance requires the exported physical unit centers.")
    instances = collision["geometryInstances"]
    if set(instances.values()) != {p["id"] for p in checked.manifest["parts"]}:
        raise AdapterError("Prescribed clearance cannot omit physical part collision geometry.")
    for section in ("actuator", "sensor", "equality", "tendon", "keyframe"):
        element = root.find(section)
        if element is not None:
            root.remove(element)
    initial = {}
    for index, template in sorted(units.items()):
        name = f"unit_{index}"
        old_id = mujoco.mj_name2id(original, mujoco.mjtObj.mjOBJ_BODY, name)
        initial[index] = (reference.xpos[old_id].copy(), reference.xquat[old_id].copy())
        body = deepcopy(template)
        for child in list(body):
            if child.tag in ("body", "joint", "freejoint"):
                body.remove(child)
        body.set("pos", vector_text(initial[index][0]))
        body.set("quat", vector_text(initial[index][1]))
        ET.SubElement(body, "freejoint", name=f"phase_{index}")
        if template in list(world):
            world.remove(template)
        world.append(body)
    phase_path = model_path.with_name("phase-model.xml")
    ET.ElementTree(root).write(phase_path, encoding="unicode")
    try:
        model = mujoco.MjModel.from_xml_path(str(phase_path))
    except ValueError as error:
        raise AdapterError(f"MuJoCo rejected the prescribed-motion model: {error}") from error
    data = mujoco.MjData(model)
    addresses = {}
    for body, index in checked.body_indices.items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"phase_{index}")
        addresses[body] = int(model.jnt_qposadr[joint])
    chassis = checked.body_indices[checked.manifest["chassis"]]
    lift = initial[chassis][0] - np.array(centers[str(chassis)])
    conversion = to_mujoco(np.eye(3)).T
    contacts = {}
    for angle, poses in frames:
        for body, index in checked.body_indices.items():
            origin, rotation = poses[body]
            start_origin, start_rotation = frames[0][1][body]
            delta = conversion @ _rotation(rotation - start_rotation) @ conversion.T
            position = (to_mujoco(origin) + lift
                        + delta @ (initial[index][0] - lift - to_mujoco(start_origin)))
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, delta.reshape(9))
            orientation = np.empty(4)
            mujoco.mju_mulQuat(orientation, quaternion, initial[index][1])
            address = addresses[body]
            data.qpos[address:address + 3] = position
            data.qpos[address + 3:address + 7] = orientation
        mujoco.mj_forward(model, data)
        if data.time != 0:
            raise AdapterError("Prescribed clearance must never advance simulation time.")
        if report["sampledPoses"] == 0:
            for name in instances:
                before = mujoco.mj_name2id(original, mujoco.mjtObj.mjOBJ_GEOM, name)
                after = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                if (before < 0 or after < 0
                        or not np.allclose(reference.geom_xpos[before], data.geom_xpos[after], atol=1e-9, rtol=0)
                        or not np.allclose(reference.geom_xmat[before], data.geom_xmat[after], atol=1e-9, rtol=0)):
                    raise AdapterError("Prescribed initial geometry differs from the checked physical assembly.")
        for contact in data.contact[:data.ncon]:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g))
                     for g in (contact.geom1, contact.geom2)]
            if all(name in instances for name in names) and -float(contact.dist) > tolerance:
                pair = tuple(sorted(instances[name] for name in names))
                if pair[0] == pair[1]:
                    continue
                entry = {"parts": list(pair), "depthMeters": -float(contact.dist), "inputAngleRadians": angle}
                if pair not in contacts or entry["depthMeters"] > contacts[pair]["depthMeters"]:
                    contacts[pair] = entry
        report["sampledPoses"] += 1
    report.update(
        status="collision" if contacts else "clear-at-sampled-phases",
        contactPairs=len(contacts), contactsTruncated=len(contacts) > 64,
        contacts=sorted(contacts.values(), key=lambda c: (-c["depthMeters"], c["parts"]))[:64],
        initialGeometryMatches=True,
    )
    write_json(model_path.with_name("phase-clearance.json"), report)
    return report
