"""Render the checked leg's actual assembly, without advancing physics."""

import argparse
import html
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

from legowalkers_sim.bridge import AdapterError, write_json
from preview import prepare_still


def render(source, output, library, blender):
    source, output = source.resolve(), output.resolve()
    library = library.resolve()
    if output.exists() and any(output.iterdir()):
        raise AdapterError(f"Render output must be empty: {output}")
    if output == source or source in output.parents:
        raise AdapterError("Keep renders outside the immutable fixture.")
    assets = output / "render"
    prepare_still(SimpleNamespace(manifest=source / "manifest.json", assets=assets,
                                 library=library, preset="thumbnail"))
    geometry = json.loads((assets / "geometry.json").read_text())
    for part in geometry:
        part["color"] = {71: 7, 72: 8}.get(part["color"], part["color"])
    write_json(assets / "geometry.json", geometry)
    metadata = json.loads((assets / "metadata.json").read_text())
    metadata["renderColorSubstitutions"] = {"71": 7, "72": 8}
    metadata["dynamicTrials"] = 0
    write_json(assets / "metadata.json", metadata)
    subprocess.run([str(blender), "--background", "--factory-startup", "--threads", "2",
                    "--python-exit-code", "1", "--python", str(Path(__file__).with_name("render_preview.py")),
                    "--", "--assets", str(assets), "--mode", "still", "--preset", "thumbnail"], check=True)
    if not (assets / "walker-v2.png").is_file():
        raise AdapterError("Renderer did not produce an image.")
    (output / "index.html").write_text(
        '<!doctype html><html lang="en"><meta charset="utf-8"><title>Physical single-leg fixture</title>'
        '<style>body{font:16px system-ui;max-width:1000px;margin:2rem auto;padding:1rem;background:#eef2f7}'
        'img{max-width:100%;background:white;border-radius:12px}p{max-width:760px}</style>'
        '<h1>Physical double-parallelogram leg</h1><img src="render/walker-v2.png" alt="Actual single-leg fixture parts">'
        '<p>23 actual parts, including the molded L, smooth shared pin, hinged shoe and supported direct motor. '
        'The long vertical beam is the fixed test stand behind the leg, not a proposed walker chassis.</p>'
        '<p>This isolates the confirmed leg mechanism. The motor directly drives the crank for this fixture; '
        'the photographed reduction train and whole-robot mass layout remain unresolved. '
        'No dynamics or walking test. Initial-pose checks do not establish full-cycle clearance.</p>'
        f'<p><a href="{html.escape((source / "model.ldr").as_uri(), quote=True)}">LDraw assembly</a> | '
        f'<a href="{html.escape((source / "cycle.svg").as_uri(), quote=True)}">Kinematic foot cycle</a> | '
        f'<a href="{html.escape((source / "fixture.json").as_uri(), quote=True)}">Fixture provenance</a></p></html>',
        encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ldraw-library", type=Path, required=True)
    parser.add_argument("--blender", type=Path, required=True)
    args = parser.parse_args()
    render(args.input, args.output, args.ldraw_library, args.blender)
