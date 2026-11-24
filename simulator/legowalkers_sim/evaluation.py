from __future__ import annotations

import importlib.metadata
import hashlib
import math
from copy import deepcopy
from dataclasses import asdict, dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from lego_technic_sim.mujoco_export import generate_mjcf

from .bridge import AdapterError, CheckedModel, LDU_TO_METERS, logical_foot_count, write_json
from .schema import MIN_WALKER_FEET, MAX_FEET
from .collision import install_contacts, prepare_parts, to_mujoco, vector_text, validate_floor_contact_time_constant
from .payload import apply_payloads, install_inertia, load_profile
from .ground import GroundContacts, GroundPenetrationStats
from .joint_tree import prioritize_shared_pivots
from .simulation_gate import SimulationPrefixGate, validate_short_gate
from . import motor


@dataclass(frozen=True)
class EvaluationOptions:
    duration: float = 5.0
    settle: float = 1.0
    timestep: float = 0.001
    friction: float = 1.0
    speed: float = motor.NO_LOAD_SPEED
    torque: float = 0.4
    anchor: bool = False
    self_collision: bool = True
    capture_poses: bool = False
    payload_profile: str | None = None
    cycle_aware: bool = False
    short_gate: float = 0.0
    motor_model: str = motor.MODEL
    floor_contact_time_constant: float | None = None

    def validate(self):
        for key, value in asdict(self).items():
            if isinstance(value, float) and not math.isfinite(value):
                raise AdapterError(f"{key} must be finite.")
        if not 0 <= self.settle < self.duration <= 60:
            raise AdapterError("Require 0 <= settle < duration <= 60 seconds.")
        if not 0.0001 <= self.timestep <= 0.005:
            raise AdapterError("timestep must be between 0.0001 and 0.005 seconds.")
        if self.friction <= 0 or self.friction > 3 or self.torque <= 0 or self.speed == 0:
            raise AdapterError("Require 0 < friction <= 3, nonzero signed speed, and positive torque.")
        for key in ("duration", "settle"):
            steps = getattr(self, key) / self.timestep
            if not math.isclose(steps, round(steps), abs_tol=1e-7):
                raise AdapterError(f"{key} must be a whole number of timesteps.")
        validate_short_gate(self.short_gate, self.timestep)
        validate_floor_contact_time_constant(self.floor_contact_time_constant, self.timestep)
        if self.anchor and self.floor_contact_time_constant is not None:
            raise AdapterError("Explicit floor contacts cannot be applied to an anchored rig.")
        motor.validate(self.motor_model, self.speed, self.torque)


def ranking_rejections(options: EvaluationOptions, *, full_cycle: bool, interference: bool,
                       fallen: bool, closure_ok: bool, distance: float, logical_feet: int | None,
                       forbidden_ground: bool = False, rigid_towing: bool = False) -> list[str]:
    return [reason for invalid, reason in [
        (not options.self_collision, "ground-only-preview"),
        (options.anchor, "anchored-chassis"),
        (options.motor_model == "ideal-servo", "ideal-motor-preview"),
        (interference, "self-interference"),
        (not full_cycle, "incomplete-motor-cycle"),
        (fallen, "excessive-tilt"),
        (not closure_ok, "joint-closure-error"),
        (distance <= 0, "nonpositive-forward-motion"),
        (forbidden_ground, "forbidden-ground-contact"),
        (rigid_towing, "rigid-wagon-hitch"),
        (logical_feet is None, "unknown-logical-foot-count"),
        (logical_feet is not None and (type(logical_feet) is not int
                                      or not MIN_WALKER_FEET <= logical_feet <= MAX_FEET),
         "logical-foot-count-out-of-range"),
    ] if invalid]


def motor_speed(manifest, options):
    polarity = (math.copysign(1, manifest["genome"]["speed"])
                if manifest["grammarVersion"] == "technic-motion-v3"
                else manifest["genome"].get("motorDirection", 1))
    return options.speed * polarity


def prepare_xml(checked: CheckedModel, output: Path, options: EvaluationOptions):
    """Small, explicit corrections to upstream's idealized MJCF export."""
    options.validate()
    manifest = checked.manifest
    profile = load_profile(options.payload_profile)
    scene, parts, exemptions = prepare_parts(checked, profile=profile)
    write_json(output / "mass-preparation.json", {
        "version": "part-local-mass-v2",
        "coordinateFrame": "LDraw axes scaled to meters",
        "method": "Part-local mesh integration, actual payload profiles and each connector attached once",
        "units": [
            {"body": index, "upstreamMassKg": original.mass, "preparedMassKg": prepared.mass,
             "upstreamCenterMeters": original.center_of_mass.tolist(),
             "preparedCenterMeters": prepared.center_of_mass.tolist(),
             "hasPayload": any(part.unit == index and part.instance in {
                 item["partInstance"] for item in manifest.get("payloads", [])} for part in parts)}
            for index, (original, prepared) in enumerate(zip(checked.scene.units, scene.units, strict=True))
        ],
    })
    scene, tensors = apply_payloads(scene, parts, manifest, profile)
    if manifest["grammarVersion"] == "technic-motion-v3":
        from .mass_layout import analyze_layout
        write_json(output / "mass-layout.json", analyze_layout(checked, scene, parts, profile, inertia_tensors=tensors))
    if not scene.motors:
        raise AdapterError("Evaluation requires a detected motor; use check for passive fixtures.")
    chassis_index = checked.body_indices[manifest["chassis"]]
    if chassis_index != 0 or scene.joints[scene.motors[0].joint_index].unit_a_index != 0:
        raise AdapterError("This pinned upstream exporter requires chassis unit 0 as the motor's body A.")
    mesh_dir = output / "meshes"
    mesh_dir.mkdir(parents=True)
    root = ET.fromstring(generate_mjcf(scene, timestep=options.timestep,
                                     motor_speed=abs(options.speed), mesh_dir=mesh_dir))
    write_json(output / "joint-tree.json", prioritize_shared_pivots(
        root, scene, manifest, checked.body_indices))
    from .spatial_gears import install_bevel_couplings
    write_json(output / "gear-couplings.json", install_bevel_couplings(root, scene))
    root.find("compiler").set("meshdir", "meshes")
    bodies = {body.attrib["name"]: body for body in root.findall(".//body")}
    install_inertia(bodies, tensors)
    collision_model = install_contacts(
        root, scene, parts, exemptions, mesh_dir, enabled=options.self_collision,
        anchor=options.anchor, friction=options.friction,
        floor_contact_time_constant=options.floor_contact_time_constant,
    )
    collision_model["unitCentersMeters"] = {
        str(index): to_mujoco(unit.center_of_mass).tolist() for index, unit in enumerate(scene.units)}
    equality = root.find("equality")
    if equality is not None:
        # Two points on the hinge axis preserve its orientation, not just its pivot.
        for constraint in list(equality.findall("connect")):
            joint_index = int(constraint.attrib["name"].removeprefix("loop_"))
            joint = scene.joints[joint_index]
            axis = to_mujoco(joint.axis)
            axis /= np.linalg.norm(axis)
            anchor = np.fromstring(constraint.attrib["anchor"], sep=" ")
            attributes = dict(constraint.attrib)
            attributes["name"] += "_axis"
            attributes["anchor"] = vector_text(anchor + axis * 0.008)
            ET.SubElement(equality, "connect", attributes)

    motor.install(root, manifest, options)

    if manifest["grammarVersion"] == "technic-fourbar-v5" and manifest["genome"]["towWagon"]:
        for name in ("drawbar-walker-pitch", "drawbar-wagon-pitch"):
            index = checked.joint_indices[name]
            hinge = root.find(f".//joint[@name='joint_{index}']")
            if hinge is None:
                raise AdapterError(f"Drawbar pitch hinge {name} must remain an independent tree joint.")
            hinge.set("damping", "0")

    joint_sites = []
    for index, joint in enumerate(manifest["expected"]["joints"]):
        names = []
        point = to_mujoco(np.array(joint["position"]) * LDU_TO_METERS)
        for endpoint in ("bodyA", "bodyB"):
            unit_index = checked.body_indices[joint[endpoint]]
            local = point - to_mujoco(scene.units[unit_index].center_of_mass)
            name = f"pivot_{index}_{endpoint}"
            ET.SubElement(bodies[f"unit_{unit_index}"], "site", name=name,
                          pos=vector_text(local), size="0.001", rgba="0 0 0 0")
            names.append(name)
        joint_sites.append(names)
    foot_sites = []
    for index, foot in enumerate(manifest["feet"]):
        unit_index = checked.body_indices[foot["body"]]
        local = to_mujoco(np.array(foot["position"]) * LDU_TO_METERS) - to_mujoco(scene.units[unit_index].center_of_mass)
        name = f"foot_{index}"
        ET.SubElement(bodies[f"unit_{unit_index}"], "site", name=name,
                      pos=vector_text(local), size="0.002", rgba="1 0 0 1")
        foot_sites.append(name)
    chassis = bodies["unit_0"]
    minimum_z = min(
        -float(vertex[1]) * LDU_TO_METERS
        for unit in scene.units for brick in unit.bricks
        for triangle in brick.triangles for vertex in (triangle.v0, triangle.v1, triangle.v2)
    )
    position = np.fromstring(chassis.attrib["pos"], sep=" ")
    position[2] += 0.001 - minimum_z
    chassis.set("pos", vector_text(position))
    if options.anchor:
        chassis.remove(chassis.find("freejoint"))
    ET.indent(root)
    xml_path = output / "model.xml"
    ET.ElementTree(root).write(xml_path, encoding="utf-8", xml_declaration=True)
    write_json(output / "collision-model.json", collision_model)
    return xml_path, joint_sites, foot_sites, collision_model


def closure_sample(joint, endpoint_a, endpoint_b, time, motor_angles):
    separation = endpoint_b - endpoint_a
    return {
        "joint": joint["id"], "bodyA": joint["bodyA"], "bodyB": joint["bodyB"],
        "timeSeconds": time, "errorMeters": float(np.linalg.norm(separation)),
        "endpointAMeters": endpoint_a.tolist(), "endpointBMeters": endpoint_b.tolist(),
        "separationVectorMeters": separation.tolist(),
        "motorAnglesRadians": motor_angles.tolist(),
        "motorPhaseDegrees": (np.degrees(motor_angles) % 360).tolist(),
    }


def evaluate(checked: CheckedModel, output: Path, options: EvaluationOptions, *, on_gate_passed=None) -> dict:
    options.validate()
    from .cycles import CycleProgress, planned_options
    options, gait_plan = planned_options(checked.manifest, options)
    gait_progress = CycleProgress(gait_plan, motor_speed(checked.manifest, options)) if gait_plan else None
    if output.exists() and any(output.iterdir()):
        raise AdapterError(f"Evaluation output directory must be empty: {output}")
    try:
        import mujoco
    except ModuleNotFoundError as error:
        raise AdapterError("MuJoCo is missing; install the project's [physics] extra.") from error
    output.mkdir(parents=True, exist_ok=True)
    xml_path, joint_sites, foot_sites, collision_model = prepare_xml(checked, output, options)
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    except ValueError as error:
        raise AdapterError(f"MuJoCo rejected the generated model: {error}") from error
    data = mujoco.MjData(model)
    if options.motor_model == motor.MODEL and np.any(model.actuator_biastype != mujoco.mjtBias.mjBIAS_NONE):
        raise AdapterError("Calibrated motors require direct torque actuators, not velocity servos.")
    mujoco.mj_forward(model, data)
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "unit_0")
    site_ids = [[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in pair] for pair in joint_sites]
    foot_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in foot_sites]
    motor_qpos = [
        model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{checked.joint_indices[drive['joint']]}"
        )] for drive in checked.manifest["expected"]["drives"]
    ]
    initial_motor_angles = None
    effective_speed = motor_speed(checked.manifest, options)
    pose_frames = []
    pose_steps = {round(frame / 50 / options.timestep)
                  for frame in range(round(options.duration * 50) + 1)}
    pose_bodies = {f"unit_{index}": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"unit_{index}")
                   for index in range(len(checked.scene.units))}
    previous_quaternions = {}

    def sample_pose():
        poses = {}
        for name, index in pose_bodies.items():
            quaternion = data.xquat[index].copy()
            if name in previous_quaternions and quaternion @ previous_quaternions[name] < 0:
                quaternion *= -1
            previous_quaternions[name] = quaternion
            poses[name] = {"position": (data.xpos[index] * 25).tolist(), "quaternion": quaternion.tolist()}
        pose_frames.append({"time": float(data.time), "bodies": poses})

    if options.capture_poses:
        sample_pose()
    frames = []
    next_sample = 0.0
    start = None
    work = 0.0
    max_tilt = 0.0
    max_closure = 0.0
    closure_peaks = {}
    first_closure_failure = None

    def closure_diagnosis():
        return {
            "schemaVersion": 1, "toleranceMeters": 0.002,
            "scope": "measurement interval after settling",
            "coordinates": "MuJoCo world coordinates, meters",
            "motorAngleReference": "absolute joint qpos; phase modulo one turn",
            "firstExceeded": first_closure_failure,
            "jointPeaks": sorted(closure_peaks.values(), key=lambda sample: sample["errorMeters"], reverse=True),
            "interpretation": "Measured pivot separation, not a proven physical disconnection or root cause. "
                              "Constraint drift, incompatible geometry and loading require further investigation.",
        }
    peak_force = 0.0
    penetrations = {}
    geometry_instances = collision_model["geometryInstances"]
    contact_names = {index: geometry_instances.get(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index))
                     for index in range(model.ngeom)}
    ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    ground_contacts = GroundContacts(checked.manifest)
    ground_penetration = GroundPenetrationStats()
    from .stance_support import StanceSupport, VERSION as STANCE_VERSION, contact_hull
    stance = StanceSupport(options.settle, anchored=options.anchor)
    stance_frames = []
    stance_contacts = []

    def stance_diagnosis():
        write_json(output / "ground-penetration.json", ground_penetration.report())
        report = stance.report()
        write_json(output / "stance-support.json", {
            "schemaVersion": 1, "version": STANCE_VERSION, "startTime": options.settle,
            "ldrawSha256": checked.manifest["ldrawSha256"],
            "summary": report, "frames": stance_frames,
        })
        return report

    def record_contacts():
        stance_contacts.clear()
        for index, contact in enumerate(data.contact):
            left, right = contact_names[contact.geom1], contact_names[contact.geom2]
            if ground_id in (contact.geom1, contact.geom2):
                instance = right if contact.geom1 == ground_id else left
                if instance is None:
                    raise AdapterError("Ground contact has no declared part instance.")
                kind = ground_contacts.observe(instance, contact.pos, float(contact.dist), float(data.time),
                                               data.site_xpos[foot_ids])
                ground_penetration.observe(instance, kind, float(contact.dist), float(data.time))
                if kind in ("foot", "wheel"):
                    force = np.empty(6)
                    mujoco.mj_contactForce(model, data, index, force)
                    if not np.isfinite(force).all():
                        raise AdapterError("Non-finite ground reaction in stance diagnostics.")
                    if force[0] > 0:
                        stance_contacts.append(contact.pos[:2].tolist())
            if left is not None and right is not None and contact.dist < 0:
                pair = tuple(sorted((left, right)))
                depth = -float(contact.dist)
                if pair not in penetrations or depth > penetrations[pair]["depthMeters"]:
                    penetrations[pair] = {"parts": list(pair), "depthMeters": depth, "time": float(data.time)}

    record_contacts()
    min_height = math.inf
    steps = round(options.duration / options.timestep)
    settle_steps = round(options.settle / options.timestep)
    gate = SimulationPrefixGate(output, options.short_gate, options.timestep, settle_steps, steps, on_pass=on_gate_passed)
    if gate.active:
        gate.observe(0, float(data.time), max_tilt, max_closure, deepcopy(penetrations), deepcopy(ground_contacts.report()), frames,
                     stance_diagnosis=stance_diagnosis)
    for step in range(steps):
        if step == settle_steps:
            mujoco.mj_forward(model, data)
            start = data.xpos[chassis].copy()
            initial_motor_angles = data.qpos[motor_qpos].copy()
            if gait_progress:
                gait_progress.observe(float(data.time), 0.0, float(start[0]))
        if step < settle_steps:
            data.ctrl[:] = 0.0
        elif options.motor_model == motor.MODEL:
            data.ctrl[:] = [motor.torque_at(float(speed), effective_speed, options.torque)
                            for speed in data.actuator_velocity]
        else:
            data.ctrl[:] = effective_speed
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        if options.capture_poses and step + 1 in pose_steps:
            sample_pose()
        record_contacts()
        if np.any(data.warning.number):
            raise AdapterError(f"MuJoCo issued numerical warnings at t={data.time:.6f}: {data.warning.number.tolist()}")
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise AdapterError(f"Non-finite simulation state at t={data.time:.6f}.")
        if step >= settle_steps and not options.anchor:
            polygon = contact_hull(stance_contacts)
            center = data.subtree_com[chassis].copy()
            stance.observe(float(data.time), center, polygon)
            stance_frames.append({
                "time": float(data.time), "centerOfMassMeters": center.tolist(),
                "supportPolygonXYMeters": polygon,
                "chassisRotation": data.xmat[chassis].reshape(3, 3).tolist(),
            })
        if step >= settle_steps:
            if gait_progress:
                gait_progress.observe(float(data.time),
                                      float((data.qpos[motor_qpos[0]]-initial_motor_angles[0])/(2*math.pi)),
                                      float(data.xpos[chassis, 0]))
            work += float(np.abs(data.actuator_force * data.actuator_velocity).sum()) * options.timestep
            peak_force = max(peak_force, float(np.max(np.abs(data.actuator_force))))
        tilt = math.acos(float(np.clip(data.xmat[chassis].reshape(3, 3)[2, 2], -1, 1)))
        closure = max((float(np.linalg.norm(data.site_xpos[a] - data.site_xpos[b])) for a, b in site_ids), default=0.0)
        if step >= settle_steps:
            samples = []
            for joint, (a, b) in zip(checked.manifest["expected"]["joints"], site_ids, strict=True):
                sample = closure_sample(joint, data.site_xpos[a], data.site_xpos[b],
                                        float(data.time), data.qpos[motor_qpos])
                samples.append(sample)
                previous = closure_peaks.get(joint["id"])
                if previous is None or sample["errorMeters"] > previous["errorMeters"]:
                    closure_peaks[joint["id"]] = sample
            if first_closure_failure is None and closure > 0.002:
                first_closure_failure = max(samples, key=lambda sample: sample["errorMeters"])
            max_tilt = max(max_tilt, tilt)
            min_height = min(min_height, float(data.xpos[chassis, 2]))
            max_closure = max(max_closure, closure)
        if data.time + 1e-9 >= next_sample or step == steps - 1:
            frames.append({
                "time": float(data.time), "chassis": data.xpos[chassis].tolist(),
                "quaternion": data.xquat[chassis].tolist(),
                "feet": [data.site_xpos[index].tolist() for index in foot_ids],
                "closureErrorMeters": closure,
            })
            next_sample += 1 / 60
        if gate.active:
            gate.observe(step + 1, float(data.time), max_tilt, max_closure, deepcopy(penetrations),
                         deepcopy(ground_contacts.report()), frames, closure_diagnosis=closure_diagnosis,
                         stance_diagnosis=stance_diagnosis)
    mujoco.mj_forward(model, data)
    if start is None or initial_motor_angles is None:
        raise AdapterError("No measurement interval was executed.")
    displacement = data.xpos[chassis] - start
    mass = float(model.body_mass.sum())
    distance = float(displacement[0])
    fallen = max_tilt > math.pi / 3
    closure_ok = max_closure <= 0.002
    motor_turns = ((data.qpos[motor_qpos] - initial_motor_angles) / (2 * math.pi)).tolist()
    full_cycle = bool(motor_turns) and all(abs(turns) >= 1 for turns in motor_turns)
    maximum_penetration = max((entry["depthMeters"] for entry in penetrations.values()), default=0.0)
    interference = maximum_penetration > 0.0001
    logical_feet = (None if checked.manifest["grammarVersion"] == "technic-motion-v1"
                    else logical_foot_count(checked.manifest))
    ranking_reasons = ranking_rejections(
        options, full_cycle=full_cycle, interference=interference,
        fallen=fallen, closure_ok=closure_ok, distance=distance, logical_feet=logical_feet,
        forbidden_ground=bool(ground_contacts.forbidden),
        rigid_towing=checked.manifest["genome"].get("towWagon", False)
        and checked.manifest["grammarVersion"] != "technic-fourbar-v5",
    )
    gait = {**gait_plan, **gait_progress.report()} if gait_plan else None
    if gait is not None:
        if gait["completeCycles"] < gait["requiredCycles"]:
            ranking_reasons.append("incomplete-gait-cycles")
        if not gait["repeatableForwardMotion"]:
            ranking_reasons.append("nonrepeatable-forward-motion")
    result = {
        "schemaVersion": 1, "status": "interference" if interference else "evaluated", "compatibility": checked.report,
        "regime": "anchored-mechanism" if options.anchor else "free-walker",
        "options": asdict(options),
        "control": {"effectiveSpeedRadPerSecond": effective_speed,
                    "speedMeaning": "requested-unloaded-speed" if options.motor_model == motor.MODEL else "servo-target"},
        "groundContact": ground_contacts.report(),
        "stanceSupport": stance_diagnosis(),
        "closureDiagnosis": closure_diagnosis(),
        "payloadProfile": load_profile(options.payload_profile),
        "collision": {
            **collision_model, "penetrationToleranceMeters": 0.0001,
            "maxSelfPenetrationMeters": maximum_penetration,
            "selfContacts": sorted(penetrations.values(), key=lambda entry: entry["depthMeters"], reverse=True),
            "fullMotorCycle": full_cycle,
        },
        "versions": {package: importlib.metadata.version(package) for package in ("mujoco", "numpy", "lego-technic-sim", "legowalkers-sim")},
        "metrics": {
            "logicalFootCount": logical_feet,
            "measurementSeconds": options.duration - options.settle,
            "forwardMeters": distance,
            "speedMetersPerSecond": distance / (options.duration - options.settle),
            "lateralMeters": float(displacement[1]), "verticalMeters": float(displacement[2]),
            "absoluteMechanicalWorkJoules": work,
            "costOfTransport": work / (mass * 9.81 * distance) if distance > 1e-6 else None,
            "simulatedMassKg": mass,
            "maxTiltDegrees": math.degrees(max_tilt),
            "minimumChassisHeightMeters": min_height,
            "maxClosureErrorMeters": max_closure,
            "peakActuatorForceNm": peak_force,
            "motorTurns": motor_turns,
            "fallen": fallen, "closureWithinTolerance": closure_ok,
            "eligibleForRanking": not ranking_reasons,
            "rankingRejectionReasons": ranking_reasons,
        },
        "motorModel": motor.definition(options.motor_model, options.speed, options.torque),
        "groundPenetration": ground_penetration.report(),
        "adapterCorrections": [
            "Loop-closing hinges use two point constraints along their common axis.",
            ("PF XL motors use calibrated open-loop 9 V torque-speed control and a conservative stall cap."
             if options.motor_model == motor.MODEL else
             "Preview-only velocity servos have explicit torque limits, not a physical motor curve."),
            "The complete assembly starts 1 mm above the floor; anchored rigs disable floor contact.",
            "Axles use 1.2 mm radius rods; other parts use individual hulls with explicit contact pairs.",
            "Connector meshes contribute estimated ABS mass and center of mass.",
            "All physical ground contacts remain enabled; non-foot/non-wheel support disqualifies the design.",
            *(["Electronics replace mesh-derived mass; payload chassis uses per-part box inertia and the parallel-axis theorem."]
              if checked.manifest.get("payloads") else []),
        ],
        "warnings": [
            "Conservative individual part hulls fill holes; verified socket pairs and meshing teeth are exempted.",
            "Thin axle rods detect core crossings, not all grazing contacts with the real cross-shaped shaft.",
            "Connector masses use mesh volume and ABS density, not measured weights; inertia remains approximate.",
            "Mechanical work is not electrical energy.",
            *(["Motor calibration assumes regulated 9 V; battery sag, receiver losses and thermal protection are not simulated."]
              if options.motor_model == motor.MODEL else []),
            "No physical buildability or successful gait is certified by this evaluation.",
            *(["Loaded battery/receiver masses and centers are reference estimates; mounts are ideal rigid attachments."]
              if checked.manifest.get("payloads") else
              ["External power is idealized; no tether drag or cable forces are simulated."]),
            *([] if options.self_collision else ["Ground-only preview: not eligible for ranking."]),
            *(["Anchored mechanism rig: floor contact disabled."] if options.anchor else []),
        ],
    }
    if gait is not None:
        result["gait"] = gait
        result["metrics"]["speedMetersPerSecond"] = gait["minimumCycleSpeedMetersPerSecond"]
    if options.short_gate:
        result["simulationGate"] = gate.report
    write_json(output / "trajectory.json", {"schemaVersion": 1, "frames": frames})
    if options.capture_poses:
        write_json(output / "poses.json", {
            "schemaVersion": 1, "sampleFps": 50, "renderScale": 25, "frames": pose_frames,
            "modelSha256": hashlib.sha256(xml_path.read_bytes()).hexdigest(),
            "ldrawSha256": checked.manifest["ldrawSha256"],
        })
    write_json(output / "closure-diagnosis.json", closure_diagnosis())
    write_json(output / "result.json", result)
    return result
