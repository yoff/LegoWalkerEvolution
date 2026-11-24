"""Run with Blender --background --factory-startup --python ... -- --assets DIR --mode still|animation."""

import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import bpy
from mathutils import Quaternion, Vector

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--assets", type=Path, required=True)
parser.add_argument("--mode", choices=["still", "animation"], required=True)
parser.add_argument("--preset", choices=["thumbnail", "draft", "preview", "quality"])
parser.add_argument("--video-format", choices=["mp4", "webm"], default="mp4")
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
assets = args.assets.resolve()
metadata = json.loads((assets / "metadata.json").read_text())
if metadata.get("renderKind") == "assembly-still" and args.mode != "still":
    raise ValueError("A static assembly has no evaluated animation poses.")
parts = json.loads((assets / "geometry.json").read_text())
poses = (json.loads((assets / "poses.json").read_text())["frames"]
         if args.mode == "animation" else [{"bodies": metadata["bodies"]}])
preset = args.preset or metadata["preset"]
settings = metadata["renderPresets"][preset]

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
thumbnail = args.mode == "still" and metadata.get("renderKind") == "assembly-still" and preset == "thumbnail"
scene.render.engine = "BLENDER_EEVEE_NEXT" if args.mode == "still" and not thumbnail else settings["engine"]
scene.render.resolution_x = 1600 if args.mode == "still" and not thumbnail else settings["width"]
scene.render.resolution_y = 1000 if args.mode == "still" and not thumbnail else settings["height"]
scene.render.resolution_percentage = 100
scene.eevee.taa_render_samples = 64 if args.mode == "still" else settings["samples"]
scene.render.fps = settings["fps"]
if scene.render.engine == "BLENDER_WORKBENCH":
    scene.display.render_aa = "8"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.studio_light = "paint.sl"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_shadows = False
    scene.display.shading.show_cavity = True
    scene.display.shading.cavity_type = "BOTH"
    scene.view_settings.view_transform = "Standard"
    scene.view_settings.exposure = 0.5
scene.world.use_nodes = True
scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.28, 0.36, 0.48, 1)
scene.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.45
bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"
bpy.context.preferences.filepaths.save_version = 0


def material(name, color, roughness=0.28):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    result.diffuse_color = color
    shader = result.node_tree.nodes["Principled BSDF"]
    shader.inputs["Base Color"].default_value = color
    shader.inputs["Roughness"].default_value = roughness
    return result


def linear_hex(value):
    srgb = [int(value[index:index+2], 16) / 255 for index in (0, 2, 4)]
    return tuple(channel / 12.92 if channel <= 0.04045 else ((channel+0.055)/1.055)**2.4
                 for channel in srgb) + (1,)


colors = {0: "1B2A34", 1: "1E5AA8", 2: "00852B", 4: "B40000", 7: "8A928D", 8: "545955", 14: "FAC80A"}
materials = {color: material(f"LDraw {color}", linear_hex(value)) for color, value in colors.items()}
bodies = {}
for name, pose in metadata["bodies"].items():
    body = bpy.data.objects.new(name, None)
    scene.collection.objects.link(body)
    body.rotation_mode = "QUATERNION"
    body.location = pose["position"]
    body.rotation_quaternion = pose["quaternion"]
    bodies[name] = body

local_bounds = {}
for part in parts:
    mesh = bpy.data.meshes.new(part["id"])
    mesh.from_pydata(part["vertices"], [], part["faces"])
    mesh.update()
    obj = bpy.data.objects.new(part["id"], mesh)
    scene.collection.objects.link(obj)
    obj.parent = bodies[part["body"]]
    if part["color"] not in materials:
        raise ValueError(f"No preview material for LDraw color {part['color']}.")
    obj.data.materials.append(materials[part["color"]])
    for axis in range(3):
        values = [vertex[axis] for vertex in part["vertices"]]
        key = part["body"], axis
        lo, hi = min(values), max(values)
        if key in local_bounds:
            lo, hi = min(lo, local_bounds[key][0]), max(hi, local_bounds[key][1])
        local_bounds[key] = lo, hi

for sample in poses:
    for name, pose in sample["bodies"].items():
        body = bodies[name]
        body.location = pose["position"]
        body.rotation_quaternion = pose["quaternion"]
        if args.mode == "animation":
            frame = 1 + sample["time"] * settings["fps"]
            body.keyframe_insert(data_path="location", frame=frame)
            body.keyframe_insert(data_path="rotation_quaternion", frame=frame)

minimum = Vector((math.inf, math.inf, math.inf))
maximum = Vector((-math.inf, -math.inf, -math.inf))
for sample in poses:
    for name, pose in sample["bodies"].items():
        rotation = Quaternion(pose["quaternion"])
        center = Vector(pose["position"])
        for corner in itertools.product(*(local_bounds[name, axis] for axis in range(3))):
            point = center + rotation @ Vector(corner)
            minimum = Vector(tuple(min(minimum[axis], point[axis]) for axis in range(3)))
            maximum = Vector(tuple(max(maximum[axis], point[axis]) for axis in range(3)))
center = (minimum + maximum) / 2
span = max(maximum - minimum)
bpy.ops.mesh.primitive_plane_add(size=200, location=(center.x, center.y, minimum.z-0.18))
floor = bpy.context.object
floor.name = "Studio floor (not a physics contact surface)"
floor.data.materials.append(material("Studio slate", (0.1384, 0.1912, 0.2623, 1), 0.68))

bpy.ops.object.camera_add()
camera = bpy.context.object
camera.location = center + Vector((1.05, 1.8, 1.1)).normalized() * span * 2.5
camera.rotation_euler = (center-camera.location).to_track_quat("-Z", "Y").to_euler()
camera.data.type = "ORTHO"
camera.data.lens = 50
rotation = camera.rotation_euler.to_matrix()
right, up = rotation.col[0], rotation.col[1]
corners = [Vector(corner) for corner in itertools.product(*zip(minimum, maximum))]
width = max(point.dot(right) for point in corners) - min(point.dot(right) for point in corners)
height = max(point.dot(up) for point in corners) - min(point.dot(up) for point in corners)
camera.data.ortho_scale = max(width, height*scene.render.resolution_x/scene.render.resolution_y) * 1.15
scene.camera = camera

for name, offset, energy, size in [
    ("Large soft key", (0.1, 0.2, 1.2), 2400, 0.75),
    ("Front fill", (-0.7, 0.65, 0.4), 1700, 0.65),
    ("Rear rim", (0.5, -0.75, 0.8), 2800, 0.55),
]:
    data = bpy.data.lights.new(name, "AREA")
    data.energy = energy
    data.shape = "DISK"
    data.size = size * span
    light = bpy.data.objects.new(name, data)
    scene.collection.objects.link(light)
    light.location = center + Vector(offset) * span
    light.rotation_euler = (center-light.location).to_track_quat("-Z", "Y").to_euler()

scene.frame_start = 1
scene.frame_end = round(metadata["result"]["options"]["duration"] * settings["fps"]) + 1 if args.mode == "animation" else 1
scene.frame_set(1)
if args.mode == "still":
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(assets / "walker-v2.png")
    if not thumbnail:
        bpy.ops.wm.save_as_mainfile(filepath=str(assets / "walker-v2.blend"))
    bpy.ops.render.render(write_still=True)
else:
    scene.render.image_settings.file_format = "FFMPEG"
    scene.render.ffmpeg.format = "WEBM" if args.video_format == "webm" else "MPEG4"
    scene.render.ffmpeg.codec = "WEBM" if args.video_format == "webm" else "H264"
    scene.render.ffmpeg.constant_rate_factor = "HIGH"
    scene.render.ffmpeg.ffmpeg_preset = "GOOD"
    scene.render.filepath = str(assets / f"walker-v2-anchored.{args.video_format}")
    bpy.ops.wm.save_as_mainfile(filepath=str(assets / "walker-v2-animated.blend"))
    (assets / "render-info.json").unlink(missing_ok=True)
    started = time.perf_counter()
    bpy.ops.render.render(animation=True)
    (assets / "render-info.json").write_text(json.dumps({
        **settings, "preset": preset, "frames": scene.frame_end, "videoFormat": args.video_format,
        "simulationSeconds": metadata["result"]["options"]["duration"],
        "renderElapsedSeconds": time.perf_counter() - started,
    }, indent=2) + "\n")
