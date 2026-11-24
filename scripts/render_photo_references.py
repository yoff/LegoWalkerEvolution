"""Render explicitly incomplete photo-reference layouts without a native manifest or physics."""

import argparse
import html
import json
from pathlib import Path
import subprocess

import numpy as np
from lego_technic_sim.ldraw.parser import LDrawParser

from legowalkers_sim.bridge import AdapterError, library_fingerprint, write_json
from legowalkers_sim.collision import to_mujoco
from preview import PRESETS, RENDER_SCALE, render_vertices


def render(source, library, blender):
    summary = json.loads((source / "references.json").read_text())
    panels = []
    for entry in summary["references"]:
        directory = source / entry["id"]
        reference = json.loads((directory / "reference.json").read_text())
        if reference["nativeCandidate"] or reference["dynamicTrials"] != 0:
            raise AdapterError("Expected an explicitly untested photo layout reference.")
        assets = directory / "render"
        if assets.exists() and any(assets.iterdir()):
            raise AdapterError(f"Render output must be empty: {assets}")
        model = directory / "layout-study.ldr"
        build = LDrawParser(parts_dir=library).parse_build(model)
        vertices = render_vertices({"parts": reference["parts"]}, build)
        minimum_z = min(float(-points[:, 1].max()) * .0004 for points in vertices)
        shift = np.array([0, 0, .001-minimum_z]) * RENDER_SCALE
        parts = []
        for part, points in zip(reference["parts"], vertices, strict=True):
            unique, inverse = np.unique(to_mujoco(points)*.01+shift, axis=0, return_inverse=True)
            color = {71: 7, 72: 8}.get(part["color"], part["color"])
            parts.append({"id": part["id"], "color": color, "body": "layout",
                          "vertices": unique.tolist(), "faces": inverse.reshape(-1, 3).tolist()})
        write_json(assets / "metadata.json", {
            "schemaVersion": 1, "renderKind": "assembly-still", "preset": "thumbnail",
            "renderPresets": PRESETS, "renderScale": RENDER_SCALE,
            "classification": reference["classification"],
            "note": "Incomplete layout approximation; mesh parsing is not construction acceptance.",
            "renderColorSubstitutions": {"71": 7, "72": 8},
            "partsDependencySha256": library_fingerprint(library, [part["part"] for part in reference["parts"]]),
            "bodies": {"layout": {"position": [0, 0, 0], "quaternion": [1, 0, 0, 0]}},
            "dynamicTrials": 0,
        })
        write_json(assets / "geometry.json", parts)
        subprocess.run([str(blender), "--background", "--factory-startup", "--threads", "2",
                        "--python-exit-code", "1", "--python", str(Path(__file__).with_name("render_preview.py")),
                        "--", "--assets", str(assets), "--mode", "still", "--preset", "thumbnail"], check=True)
        if not (assets / "walker-v2.png").is_file():
            raise AdapterError(f"Renderer did not produce an image: {assets}")
        panels.append(f'<section><h2>{html.escape(entry["title"])}</h2>'
                      f'<img src="{entry["id"]}/render/walker-v2.png" alt="Incomplete LEGO layout reference">'
                      f'<p><a href="{entry["id"]}/single-leg/preview.html">Single-leg animation</a> | '
                      f'<a href="{entry["id"]}/preview.html">Four-leg kinematics</a> | '
                      f'<a href="{entry["id"]}/layout-study.ldr">LDraw layout</a> | '
                      f'<a href="{entry["id"]}/reference.json">Assumptions</a></p></section>')
    (source / "rendered.html").write_text(
        '<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">'
        '<title>Photo walker reference layout</title><style>'
        'body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:1rem;background:#eef2f7;color:#203147}'
        'section{background:white;padding:1rem;margin:1rem 0;border-radius:12px}img{width:100%;max-width:720px}'
        '</style><h1>Photo walker reference layout</h1>'
        '<p>One design represented by both photo builds; phase-adjustment hardware is out of scope.</p>'
        '<p><strong>Not a digitized original or buildable reconstruction.</strong> Confirmed linkage topology, '
        'estimated dimensions. Missing connectors, bracing and shaft distribution; motor, gears and payloads '
        'are layout proxies. Broad shoes are not reconstructed. '
        'These images are not collision, mass-layout or walking validation. Zero dynamics.</p>'
        '<p><a href="index.html">Motion diagrams</a></p>' + "".join(panels) + '</html>', encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--ldraw-library", type=Path, required=True)
    parser.add_argument("--blender", type=Path, required=True)
    args = parser.parse_args()
    render(args.input.resolve(), args.ldraw_library.resolve(), args.blender.resolve())
