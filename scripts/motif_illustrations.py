"""Draw construction-rule witnesses as matched-scale port/shaft schematics, not LEGO meshes."""

import argparse
import html
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


def illustration(example):
    panels = (example["before"], example["after"])
    if {part["id"] for part in panels[0]} != {part["id"] for part in panels[1]}:
        raise ValueError("Motif illustrations must retain the same part instances.")

    def project(point):
        if not all(math.isfinite(point[key]) for key in ("x", "y", "z")):
            raise ValueError("Motif coordinates must be finite.")
        return point["x"]+.75*point["z"], point["y"]-.45*point["z"]

    points = [project(point) for panel in panels for part in panel for point in part["ports"]+part["shaft"]]
    if not points:
        raise ValueError("Motif illustration has no supported ports or shafts.")
    left, right = min(p[0] for p in points), max(p[0] for p in points)
    top, bottom = min(p[1] for p in points), max(p[1] for p in points)
    scale = min(470/max(1, right-left), 215/max(1, bottom-top), 6)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="420" viewBox="0 0 1200 420" role="img">',
           f'<title>{html.escape(example["title"])}: before and after construction</title>',
           '<desc>Matched-scale projected catalog ports and shaft intervals. Not a LEGO mesh, '
           'assembly sequence, collision proof or dynamic simulation. Both examples have identical planar motion.</desc>',
           '<defs><marker id="direction" markerWidth="7" markerHeight="7" refX="6" refY="3" orient="auto">'
           '<path d="M0,0 L6,3 L0,6" fill="#172c44"/></marker></defs>',
           '<rect width="1200" height="420" rx="12" fill="#f5f8fc"/>',
           '<g font-family="system-ui,sans-serif" fill="#203147">',
           f'<text x="24" y="30" font-size="21" font-weight="bold">{html.escape(example["title"])}</text>',
           f'<text x="24" y="57" font-size="15">{html.escape(example["caption"])}</text>',
           '<path d="M600,78 V345" stroke="#d2dce6"/>']
    for index, panel in enumerate(panels):
        origin = index*600
        svg.append(f'<text x="{origin+28}" y="91" font-size="17" font-weight="bold">'
                   f'{"Before" if index == 0 else "After"}</text>')

        def position(point):
            x, y = project(point)
            return origin+65+(x-left)*scale, 116+(y-top)*scale

        # Shared projection bounds keep a layer shift or pin-size change visible at the same scale.
        for part in sorted(panel, key=lambda part: bool(part["shaft"])):
            color = "#c17b08" if part["changed"] else "#73879a"
            svg.append(f'<g><title>{html.escape(part["id"])}: {html.escape(part["part"])}</title>')
            if part["shaft"]:
                a, b = [position(point) for point in part["shaft"]]
                length = math.sqrt(sum((part["shaft"][1][axis]-part["shaft"][0][axis])**2 for axis in ("x", "y", "z")))
                svg.append(f'<path d="M{a[0]:.2f},{a[1]:.2f} L{b[0]:.2f},{b[1]:.2f}" '
                           f'stroke="{color}" stroke-width="10" stroke-linecap="round"/>')
                start = tuple(.65*x+.35*y for x, y in zip(a, b, strict=True))
                end = tuple(.35*x+.65*y for x, y in zip(a, b, strict=True))
                svg.append(f'<path d="M{start[0]:.2f},{start[1]:.2f} L{end[0]:.2f},{end[1]:.2f}" '
                           'stroke="#172c44" stroke-width="2" marker-end="url(#direction)"/>')
                x, y = tuple((x+y)/2 for x, y in zip(a, b, strict=True))
                svg.append(f'<path d="M{x-5:.2f},{y-5:.2f} l10,10 m-10,0 l10,-10" stroke="#172c44" stroke-width="2"/>')
                svg.append(f'<text x="{(a[0]+b[0])/2+12:.2f}" y="{(a[1]+b[1])/2-12:.2f}" '
                           f'font-size="12">{html.escape(part["part"])} · {length:g} LDU</text>')
            else:
                line = " ".join(f"{x:.2f},{y:.2f}" for x, y in map(position, part["ports"]))
                svg.append(f'<polyline points="{line}" fill="none" stroke="{color}" stroke-width="15" stroke-linecap="round"/>')
                for point in part["ports"]:
                    x, y = position(point)
                    svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="#f5f8fc" stroke="{color}" stroke-width="2"/>')
            svg.append('</g>')
    svg.extend(['<text x="24" y="371" font-size="14">Amber: changed part · Gray: unchanged context · Arrow: local pin axis · ×: shaft midpoint</text>',
                '<text x="24" y="399" font-size="14">Port/shaft schematic, not LEGO mesh or physics. Depth is projected diagonally; planar motion is unchanged.</text>',
                '</g></svg>'])
    value = "\n".join(svg)+"\n"
    ET.fromstring(value)
    return value


def write_gallery(source, output):
    builder = json.loads((source / "builder.json").read_text())
    gallery = json.loads((source / "gallery.json").read_text())
    if gallery["schemaVersion"] != 1 or gallery["mode"] != "construction-schematic-not-dynamics":
        raise ValueError("Expected construction-rule gallery witnesses.")
    examples = gallery["examples"]
    if {example["motif"] for example in examples} != {motif["id"] for motif in builder["catalogue"]}:
        raise ValueError("Every catalogue motif needs an illustration.")
    identifiers = [example["id"] for example in examples]
    if len(set(identifiers)) != len(identifiers) or any(not name or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-" for c in name) for name in identifiers):
        raise ValueError("Illustration IDs must be unique safe filenames.")
    rendered = [(example["id"], illustration(example)) for example in examples]
    output.mkdir(parents=True, exist_ok=True)
    for name, svg in rendered:
        (output / f"{name}.svg").write_text(svg)
    metadata = {"builder": builder, "mode": gallery["mode"],
                "examples": [{key: value for key, value in example.items() if key not in ("before", "after")}
                             for example in examples]}
    (output / "catalogue.json").write_text(json.dumps(metadata, indent=2)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    write_gallery(args.input, args.output)


if __name__ == "__main__":
    main()
