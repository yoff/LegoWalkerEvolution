"""Prepare, replay and publish the collision-enabled reference preview."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import sys
from pathlib import Path
from urllib.parse import quote

import numpy as np
from lego_technic_sim.ldraw.parser import LDrawParser

from legowalkers_sim.bridge import AdapterError, library_fingerprint, load_manifest, write_json
from legowalkers_sim.collision import to_mujoco

SAMPLE_FPS = 50
RENDER_SCALE = 25
PRESETS = {
    "thumbnail": {"engine": "BLENDER_WORKBENCH", "label": "Workbench thumbnail", "fps": 1,
                  "width": 480, "height": 300, "samples": 8},
    "draft": {"engine": "BLENDER_WORKBENCH", "label": "Workbench", "fps": 10,
              "width": 720, "height": 450, "samples": 8},
    "preview": {"engine": "BLENDER_EEVEE_NEXT", "label": "EEVEE", "fps": 15,
                "width": 960, "height": 600, "samples": 16},
    "quality": {"engine": "BLENDER_EEVEE_NEXT", "label": "EEVEE", "fps": 25,
                "width": 1600, "height": 1000, "samples": 64},
}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))

def render_vertices(manifest, build):
    if len(build.parts) != len(manifest["parts"]):
        raise AdapterError("Parsed render part count differs from the manifest.")
    vertices = []
    for source, geometry in zip(manifest["parts"], build.parts, strict=True):
        if source["part"].lower() != geometry.part_id.lower():
            raise AdapterError(f"Unexpected render part: {source['id']}.")
        points = np.array([vertex for triangle in geometry.triangles
                           for vertex in (triangle.v0, triangle.v1, triangle.v2)])
        if not len(points) or not np.isfinite(points).all():
            raise AdapterError(f"Missing or non-finite render geometry: {source['id']}.")
        vertices.append(points)
    if not vertices:
        raise AdapterError("A render requires actual LDraw parts.")
    return vertices


def prepare_still(args):
    manifest = load_manifest(args.manifest)
    fingerprint = library_fingerprint(args.library, [part["part"] for part in manifest["parts"]])
    if args.assets.exists() and any(args.assets.iterdir()):
        raise AdapterError(f"Preview assets directory must be empty: {args.assets}")
    build = LDrawParser(parts_dir=args.library).parse_build(args.manifest.parent / manifest["modelFile"])
    vertices = render_vertices(manifest, build)
    minimum_z = min(float(-points[:, 1].max()) * 0.0004 for points in vertices)
    shift = np.array([0, 0, 0.001-minimum_z]) * RENDER_SCALE
    parts = []
    for source, points in zip(manifest["parts"], vertices, strict=True):
        unique, inverse = np.unique(to_mujoco(points)*0.01+shift, axis=0, return_inverse=True)
        parts.append({"id": source["id"], "color": source["color"], "body": "assembly",
                      "vertices": unique.tolist(), "faces": inverse.reshape(-1, 3).tolist()})
    metadata = {
        "schemaVersion": 1, "renderKind": "assembly-still", "manifest": str(args.manifest.resolve()),
        "ldrawSha256": manifest["ldrawSha256"], "partsDependencySha256": fingerprint,
        "preset": args.preset, "renderPresets": PRESETS, "renderScale": RENDER_SCALE,
        "bodies": {"assembly": {"position": [0, 0, 0], "quaternion": [1, 0, 0, 0]}},
        "note": "Initial LEGO assembly only; no simulation, measured motion or walking qualification.",
    }
    args.assets.mkdir(parents=True, exist_ok=True)
    write_json(args.assets / "metadata.json", metadata)
    (args.assets / "geometry.json").write_text(json.dumps(parts, allow_nan=False, separators=(",", ":")))
    print(f"Prepared static assembly of {len(parts)} real LDraw parts; zero physics steps.", flush=True)


def prepare(args):
    manifest = load_manifest(args.manifest)
    result = read_json(args.evaluation / "result.json")
    captured = args.evaluation / "poses.json"
    if not captured.exists() and (result["status"] != "evaluated" or not result["options"]["anchor"]
            or not result["options"]["self_collision"] or not result["collision"]["fullMotorCycle"]):
        raise AdapterError("Preview requires captured evaluation poses or a collision-enabled, anchored, full-cycle evaluation.")
    if manifest["ldrawSha256"] != result["compatibility"]["ldrawSha256"]:
        raise AdapterError("Preview manifest differs from the tested model.")
    fingerprint = library_fingerprint(args.library, [part["part"] for part in manifest["parts"]])
    if fingerprint != result["compatibility"]["partsDependencySha256"]:
        raise AdapterError("LDraw library differs from the evaluated library.")
    if args.assets.exists() and any(args.assets.iterdir()):
        raise AdapterError(f"Preview assets directory must be empty: {args.assets}")

    import mujoco

    xml_path = args.evaluation / "model.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    build = LDrawParser(parts_dir=args.library).parse_build(args.manifest.parent / manifest["modelFile"])
    all_vertices = render_vertices(manifest, build)
    minimum_z = min(float(-vertices[:, 1].max()) * 0.0004 for vertices in all_vertices)
    shift = np.array([0, 0, 0.001 - minimum_z]) * RENDER_SCALE
    instances = {instance: name for name, instance in result["collision"]["geometryInstances"].items()}
    bodies = {}
    parts = []
    for source, geometry, vertices in zip(manifest["parts"], build.parts, all_vertices, strict=True):
        if source["part"].lower() != geometry.part_id.lower():
            raise AdapterError(f"Unexpected render part: {source['id']}.")
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, instances[source["id"]])
        if geom < 0:
            raise AdapterError(f"No physics attachment for {source['id']}.")
        body = int(model.geom_bodyid[geom])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if not np.allclose(data.xquat[body], [1, 0, 0, 0], atol=1e-12):
            raise AdapterError("Preview expects world-aligned body frames at rest.")
        rest = data.xpos[body] * RENDER_SCALE
        bodies[name] = {"position": rest.tolist(), "quaternion": data.xquat[body].tolist()}
        local = to_mujoco(vertices) * 0.01 + shift - rest
        unique, inverse = np.unique(local, axis=0, return_inverse=True)
        parts.append({
            "id": source["id"], "color": source["color"], "body": name,
            "vertices": unique.tolist(), "faces": inverse.reshape(-1, 3).tolist(),
        })
    metadata = {
        "schemaVersion": 1, "evaluation": str(args.evaluation.resolve()),
        "manifest": str(args.manifest.resolve()),
        "modelSha256": hashlib.sha256(xml_path.read_bytes()).hexdigest(),
        "preset": args.preset, "renderPresets": PRESETS, "sampleFps": SAMPLE_FPS,
        "renderScale": RENDER_SCALE, "result": result, "bodies": bodies,
    }
    args.assets.mkdir(parents=True, exist_ok=True)
    write_json(args.assets / "metadata.json", metadata)
    (args.assets / "geometry.json").write_text(
        json.dumps(parts, allow_nan=False, separators=(",", ":")), encoding="utf-8")
    if captured.exists():
        poses = read_json(captured)
        if (poses["sampleFps"] != SAMPLE_FPS or poses["renderScale"] != RENDER_SCALE
                or poses["modelSha256"] != metadata["modelSha256"]
                or poses["ldrawSha256"] != manifest["ldrawSha256"]
                or set(poses["frames"][-1]["bodies"]) != set(bodies)
                or not math.isclose(poses["frames"][-1]["time"], result["options"]["duration"], abs_tol=1e-8)):
            raise AdapterError("Captured poses do not match the evaluation/render contract.")
        final = read_json(args.evaluation / "trajectory.json")["frames"][-1]
        if not np.allclose(np.array(poses["frames"][-1]["bodies"]["unit_0"]["position"]) / RENDER_SCALE,
                           final["chassis"], atol=1e-8, rtol=0):
            raise AdapterError("Captured chassis pose differs from the evaluated trajectory.")
        write_json(args.assets / "poses.json", poses)
    print(f"Prepared {len(parts)} real LDraw parts on {len(bodies)} physics bodies.", flush=True)


def replay(args):
    import mujoco

    metadata = read_json(args.assets / "metadata.json")
    if metadata.get("renderKind") == "assembly-still":
        raise AdapterError("A static assembly render has no evaluated physics to replay.")
    evaluation = Path(metadata["evaluation"])
    xml_path = evaluation / "model.xml"
    if hashlib.sha256(xml_path.read_bytes()).hexdigest() != metadata["modelSha256"]:
        raise AdapterError("Physics model changed after preview preparation.")
    result = metadata["result"]
    options = result["options"]
    manifest = load_manifest(Path(metadata["manifest"]))
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bodies = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
              for name in metadata["bodies"]}
    motor_qpos = [model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT,
        f"joint_{result['compatibility']['jointIndices'][drive['joint']]}",
    )] for drive in manifest["expected"]["drives"]]
    steps = round(options["duration"] / options["timestep"])
    settle_steps = round(options["settle"] / options["timestep"])
    sample_fps = metadata["sampleFps"]
    sample_steps = {round(frame / sample_fps / options["timestep"])
                    for frame in range(round(options["duration"] * sample_fps) + 1)}
    frames = []
    previous = {}

    def sample():
        poses = {}
        for name, index in bodies.items():
            quaternion = data.xquat[index].copy()
            if name in previous and float(quaternion @ previous[name]) < 0:
                quaternion *= -1
            previous[name] = quaternion
            poses[name] = {
                "position": (data.xpos[index] * RENDER_SCALE).tolist(),
                "quaternion": quaternion.tolist(),
            }
        frames.append({"time": float(data.time), "bodies": poses})

    sample()
    initial_angles = None
    for step in range(steps):
        if step == settle_steps:
            mujoco.mj_forward(model, data)
            initial_angles = data.qpos[motor_qpos].copy()
        data.ctrl[:] = 0 if step < settle_steps else result.get("control", {}).get("effectiveSpeedRadPerSecond", options["speed"])
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        if np.any(data.warning.number) or not np.isfinite(data.qpos).all():
            raise AdapterError(f"Invalid physics replay at t={data.time}.")
        if step + 1 in sample_steps:
            sample()
        if (step + 1) % round(2 / options["timestep"]) == 0:
            print(f"Replayed {data.time:.1f}/{options['duration']} simulated seconds.", flush=True)
    if initial_angles is None:
        raise AdapterError("Replay did not execute a measurement interval.")
    turns = (data.qpos[motor_qpos] - initial_angles) / (2 * math.pi)
    if not np.allclose(turns, result["metrics"]["motorTurns"], atol=1e-8, rtol=0):
        raise AdapterError(f"Replay motor turns differ from evaluation: {turns.tolist()}.")
    final = read_json(evaluation / "trajectory.json")["frames"][-1]
    chassis = bodies["unit_0"]
    feet = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"foot_{i}")
            for i in range(len(manifest["feet"]))]
    if (not np.allclose(data.xpos[chassis], final["chassis"], atol=1e-8, rtol=0)
            or not np.allclose(data.site_xpos[feet], final["feet"], atol=1e-8, rtol=0)):
        raise AdapterError("Replay final chassis/feet differ from the recorded evaluation.")
    expected_frames = round(options["duration"] * sample_fps) + 1
    if len(frames) != expected_frames:
        raise AdapterError(f"Expected {expected_frames} animation frames, got {len(frames)}.")
    write_json(args.assets / "poses.json", {
        "schemaVersion": 1, "fps": sample_fps, "frames": frames,
        "verifiedMotorTurns": turns.tolist(), "verificationTolerance": 1e-8,
    })
    print(f"Verified replay: {len(frames)} frames, {turns.tolist()} motor turns.", flush=True)


def publish(args):
    metadata = read_json(args.assets / "metadata.json")
    render = read_json(args.assets / "render-info.json")
    result = metadata["result"]
    template = Path(__file__).with_name("preview.html").read_text(encoding="utf-8")
    assets = {
        "IMAGE": (args.assets / "walker-v2.png", "image/png"),
        "VIDEO": (args.assets / "walker-v2-anchored.mp4", "video/mp4"),
        "OLD_FREE": (args.pages / "walker-free.mp4", "video/mp4"),
        "OLD_ANCHORED": (args.pages / "walker-anchored.mp4", "video/mp4"),
    }
    for path, _ in assets.values():
        if not path.is_file() or path.stat().st_size == 0:
            raise AdapterError(f"Missing preview media: {path}")
    for embed, name in [(False, "index.html"), (True, "preview.html")]:
        html = template
        for key, (path, mime) in assets.items():
            uri = ("data:" + mime + ";base64," + base64.b64encode(path.read_bytes()).decode("ascii")
                   if embed else quote(Path(os.path.relpath(path, args.pages)).as_posix()))
            html = html.replace(f"@@{key}@@", uri)
        values = {
            "TURNS": f"{result['metrics']['motorTurns'][0]:.2f}",
            "PENETRATION": f"{result['collision']['maxSelfPenetrationMeters'] * 1000:.5f}",
            "CLOSURE": f"{result['metrics']['maxClosureErrorMeters'] * 1000:.3f}",
            "DURATION": f"{result['options']['duration']:g}",
            "TORQUE": f"{result['options']['torque']:g}",
            "SPEED": f"{result['options']['speed']:+g}",
            "PENETRATION_TOLERANCE": f"{result['collision']['penetrationToleranceMeters'] * 1000:g}",
            "ROD_RADIUS": f"{result['collision']['axleRodRadiusMeters'] * 1000:g}",
            "FPS": str(render["fps"]),
            "ENGINE": render["label"],
            "PRESET": render["preset"],
            "PARTS": str(result["compatibility"]["parts"]),
            "BODIES": str(result["compatibility"]["bodies"]),
            "ASSET_BASE": quote(Path(os.path.relpath(args.assets, args.pages)).as_posix()),
            "MODEL_LINK": quote(Path(os.path.relpath(
                Path(metadata["manifest"]).parent / "model.ldr", args.pages)).as_posix()),
            "RESULT_LINK": quote(Path(os.path.relpath(
                Path(metadata["evaluation"]) / "result.json", args.pages)).as_posix()),
        }
        for key, value in values.items():
            html = html.replace(f"@@{key}@@", value)
        if "@@" in html:
            raise AdapterError("Unresolved preview template placeholder.")
        (args.pages / name).write_text(html, encoding="utf-8")
    print(f"Published relative and embedded-media pages in {args.pages}.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("prepare", "prepare-still", "replay", "publish"):
        item = commands.add_parser(command)
        item.add_argument("--assets", type=Path, required=True)
        if command in ("prepare", "prepare-still"):
            item.add_argument("--manifest", type=Path, required=True)
            if command == "prepare":
                item.add_argument("--evaluation", type=Path, required=True)
            item.add_argument("--library", type=Path, required=True)
            item.add_argument("--preset", choices=PRESETS, default="draft")
        elif command == "publish":
            item.add_argument("--pages", type=Path, required=True)
    args = parser.parse_args()
    for name, value in vars(args).items():
        if isinstance(value, Path):
            setattr(args, name, value.expanduser().resolve())
    try:
        {"prepare": prepare, "prepare-still": prepare_still, "replay": replay, "publish": publish}[args.command](args)
    except (AdapterError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
