"""Render deterministic animated SVG comparisons of solved mutation witnesses."""

import argparse
import html
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

COLORS = ("#62758a", "#c17b08", "#b74545", "#267750", "#8057af", "#18808b")


def animation(parent, child, operation, realizations=None):
    documents = (parent, child)
    for document in documents:
        if document["mode"] != "planar-kinematics-not-dynamics" or len(document["frames"]) < 2:
            raise ValueError("Mutation illustrations require solved kinematic trajectories.")
        if any(frame["maxResidualLdu"] > 1e-7 for frame in document["frames"]):
            raise ValueError("A mutation illustration contains an unsolved frame.")
    frames = [document["frames"][:-1] for document in documents]
    realizations = realizations or (None, None)
    teeth = {"3647.dat": 8, "3648b.dat": 24, "3649.dat": 40}
    outlines = {
        "32063.dat": ((0, 0, -50), (0, 0, 50)),
        "32524.dat": ((0, 0, -60), (0, 0, 60)),
        "32525.dat": ((0, 0, -100), (0, 0, 100)),
        "32271.dat": ((0, 0, 0), (0, 0, 120), (32, 0, 144)),
    }
    hardware = {
        "58119.dat": ("Battery", "#6b55a0", ((-100, -25, 0), (100, -25, 0), (100, 25, 0), (-100, 25, 0))),
        "58121.dat": ("Motor", "#a54e3e", ((-50, -49, 60), (50, -49, 60), (50, 49, 60), (-50, 49, 60))),
        "58123.dat": ("Receiver", "#426e8b", ((-40, -40, 0), (40, -40, 0), (40, 0, 0), (-40, 0, 0))),
    }
    if len(frames[0]) != len(frames[1]):
        raise ValueError("Parent and child must use the same sampling resolution.")

    def world(pose, site):
        x, y = site["position"]["x"], site["position"]["y"]
        c, s = math.cos(pose["angle"]), math.sin(pose["angle"])
        return pose["origin"][0] + c*x - s*y, pose["origin"][1] + s*x + c*y

    def depth_point(point, depth):
        return point[0]+.3*depth, point[1]-.2*depth

    def part_point(part, local):
        axes = part["rotation"]
        return [part["position"][axis] + sum(axes[basis][axis]*value for basis, value in zip(("x", "y", "z"), local))
                for axis in ("x", "y", "z")]

    def body_depths(realization):
        return {body["body"]: sum(part["position"]["z"] for part in body["parts"])/len(body["parts"])
                for body in realization["bodies"]} if realization else {}

    def foot_depths(realization):
        return {foot["foot"]: foot["depth"] for foot in realization["feet"]} if realization else {}

    points = []
    for document, samples, realization in zip(documents, frames, realizations, strict=True):
        depths = body_depths(realization)
        if realization:
            for body in realization["bodies"]:
                for part in body["parts"]:
                    if part["part"] in hardware:
                        for local in hardware[part["part"]][2]:
                            p = part_point(part, local)
                            points.append(depth_point(p, p[2]))
        for frame in samples:
            poses = {pose["body"]: pose for pose in frame["poses"]}
            points.extend(depth_point(world(poses[body["id"]], site), depths.get(body["id"], 0))
                          for body in document["program"]["bodies"] for site in body["sites"])
    left, right = min(p[0] for p in points), max(p[0] for p in points)
    top, bottom = min(p[1] for p in points), max(p[1] for p in points)
    if any(realizations):
        left, right, top, bottom = left-50, right+50, top-50, bottom+50
    scale = min(510 / max(1, right-left), 290 / max(1, bottom-top))
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="450" viewBox="0 0 1200 450" role="img">',
           f"<title>{html.escape(operation)}: parent and child, synchronized motor phase</title>",
           "<desc>Solved planar kinematics, not dynamic simulation, physical LEGO animation or evidence of walking.</desc>",
           '<rect width="1200" height="450" fill="#f5f8fc"/>',
           '<g font-family="system-ui,sans-serif" fill="#203147">',
           f'<text x="24" y="28" font-size="20" font-weight="bold">{html.escape(operation)} — before / after</text>']
    count = len(frames[0])
    for panel, (document, samples, realization) in enumerate(zip(documents, frames, realizations, strict=True)):
        program = document["program"]
        depths = body_depths(realization)
        contacts = foot_depths(realization)

        def project(point):
            return 30 + panel*600 + (point[0]-left)*scale, 100 + (point[1]-top)*scale

        def coordinates(point):
            x, y = project(point)
            return f"{x:.3f},{y:.3f}"

        label = "Parent" if panel == 0 else "Mutated child"
        svg.append(f'<text x="{24+600*panel}" y="62" font-size="16">{label}: '
                   f'{len(program["bodies"])} bodies, {len(program["hinges"])} hinges, '
                   f'{len(program["feet"])} logical feet</text>')
        for foot in program["feet"]:
            path = [next(item["position"] for item in frame["feet"] if item["id"] == foot["id"]) for frame in samples]
            trace = " ".join(coordinates(depth_point(point, contacts.get(foot["id"], 0))) for point in path+[path[0]])
            svg.append(f'<polyline points="{trace}" fill="none" stroke="#3186cc" opacity=".35"/>')
        for index, frame in enumerate(samples):
            poses = {pose["body"]: pose for pose in frame["poses"]}
            svg.append(f'<g visibility="{"visible" if index == 0 else "hidden"}">')
            values = ["hidden"]*count
            values[index] = "visible"
            svg.append('<animate attributeName="visibility" calcMode="discrete" dur="4s" repeatCount="indefinite" '
                       f'values="{";".join(values)}" keyTimes="{";".join(f"{n/count:.8f}" for n in range(count))}"/>')
            for i, body in enumerate(program["bodies"]):
                positions = [depth_point(world(poses[body["id"]], site), depths.get(body["id"], 0)) for site in body["sites"]]
                dash = 'stroke-dasharray="4 4"' if body["grounded"] else ""
                for point in positions[1:]:
                    svg.append(f'<polyline points="{coordinates(positions[0])} {coordinates(point)}" fill="none" '
                               f'stroke="{COLORS[i % len(COLORS)]}" stroke-width="3" {dash}/>')
                for point in positions:
                    x, y = project(point)
                    svg.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="2.5" fill="{COLORS[i % len(COLORS)]}"/>')
            if realization:
                initial = {pose["body"]: pose for pose in samples[0]["poses"]}
                for body in realization["bodies"]:
                    before, after = initial[body["body"]], poses[body["body"]]
                    angle = after["angle"]-before["angle"]
                    c, s = math.cos(angle), math.sin(angle)

                    def placed(part, local=(0, 0, 0)):
                        p = part_point(part, local)
                        x, y = p[0]-before["origin"][0], p[1]-before["origin"][1]
                        return depth_point((after["origin"][0]+c*x-s*y, after["origin"][1]+s*x+c*y), p[2])

                    for part in body["parts"]:
                        if part["part"] in hardware:
                            name, color, shape = hardware[part["part"]]
                            path = " ".join(coordinates(placed(part, local)) for local in shape)
                            svg.append(f'<polygon points="{path}" fill="{color}" fill-opacity=".18" stroke="{color}"/>')
                            x, y = project(placed(part, shape[0]))
                            svg.append(f'<text x="{x:.3f}" y="{y+13:.3f}" font-size="11" fill="{color}">{name}</text>')
                        if part["part"] in teeth:
                            x, y = project(placed(part))
                            radius = teeth[part["part"]]*1.25*scale
                            svg.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{radius:.3f}" fill="#e5b857" '
                                       'fill-opacity=".15" stroke="#af7300" stroke-width="1"/>')
                            phase = after["angle"]
                            svg.append(f'<path d="M {x:.3f},{y:.3f} l {radius*math.cos(phase):.3f},{radius*math.sin(phase):.3f}" '
                                       'stroke="#af7300" stroke-width="2"/>')
                            svg.append(f'<text x="{x:.3f}" y="{y-4:.3f}" font-size="9" fill="#735000">{teeth[part["part"]]}T</text>')
                        if part["part"] in outlines:
                            path = " ".join(coordinates(placed(part, local)) for local in outlines[part["part"]])
                            svg.append(f'<polyline points="{path}" fill="none" stroke="#45736b" stroke-width="7" opacity=".4"/>')
            for foot in frame["feet"]:
                x, y = project(depth_point(foot["position"], contacts.get(foot["id"], 0)))
                svg.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="5" fill="#1876c1" stroke="white"/>')
            svg.append("</g>")
    svg.append('<text x="24" y="420" font-size="14">Synchronized motor input; blue curves: foot paths. '
               'Schematic hardware; solved kinematics only — not a walking qualification.</text></g></svg>')
    result = "\n".join(svg)+"\n"
    ET.fromstring(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for directory in sorted(args.source.iterdir()):
        if not directory.is_dir():
            continue
        mutation = json.loads((directory / "mutation.json").read_text())
        documents = [json.loads((directory / name / "motion.json").read_text()) for name in ("parent", "child")]
        target = args.output / f"mutation-{mutation['operation']}.svg"
        realization_paths = [directory / f"lego-{name}" / "realization.json" for name in ("parent", "child")]
        realizations = [json.loads(path.read_text()) for path in realization_paths] if all(path.exists() for path in realization_paths) else None
        target.write_text(animation(*documents, mutation["operation"], realizations), encoding="utf-8")
        print(target)


if __name__ == "__main__":
    main()
