"""Run a preserved experiment using its hash-verified frozen implementation."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def verify_snapshot(directory):
    definition = json.loads((directory / "protocol.json").read_text())
    snapshot = directory / "protocol-source"
    if not definition.get("sourceSha256") or not definition.get("generatorBinarySha256"):
        raise ValueError("This archive has no complete pinned runtime; prepare an upgraded experiment.")
    for name, digest in definition["sourceSha256"].items():
        path = (snapshot / name).resolve()
        if not path.is_relative_to(snapshot.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Pinned experiment source changed: {name}")
    for name, digest in definition["generatorBinarySha256"].items():
        path = snapshot / "generator" / name
        if Path(name).name != name or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Pinned experiment compiler changed: {name}")
    profile = snapshot / "selected-payloads.json"
    if profile.exists() and json.loads(profile.read_text()) != definition["payloadProfile"]:
        raise ValueError("Pinned experiment payload profile changed.")
    return snapshot


def run(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("--output", type=Path, required=True)
    args, _ = parser.parse_known_args(argv)
    directory = args.output.resolve()
    try:
        snapshot = verify_snapshot(directory)
        # Execute as a script, not -m, so no current package modules have been imported.
        sys.path.insert(0, str(snapshot / "simulator"))
        from legowalkers_sim import evolution, gallery, screening
        original_screen = screening.screen_batch

        def screen(source, output, library, **options):
            digest = screening._digest(screening._protocol(options.get("load_model", True)))
            return original_screen(source, Path(output) / digest, library, **options)

        screening.screen_batch = screen
        project = Path(__file__).resolve().parents[2]

        def publish(archive):
            subprocess.run([sys.executable, "-m", "legowalkers_sim.evolution", "--output",
                            str(archive.directory), "--publish-only"], cwd=project, check=True)

        gallery.publish = publish
        evolution.publish = publish
        arguments = list(argv if argv is not None else sys.argv[1:])
        if "--generator" in arguments:
            arguments[arguments.index("--generator") + 1] = str(snapshot / "generator/LegoWalkers.Cli.dll")
        return evolution.main(arguments)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"ERROR: Pinned resume failed: {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
