"""Materialize portable motion/realization sources as a fresh native seed pool."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

from .bridge import AdapterError, load_manifest, write_json
from .evolution import Generator, ROOT
from .native_episode import _queue


def materialize(population, output, generator, selected=None):
    population, output = Path(population).resolve(), Path(output).resolve()
    descriptor = population / "population.json"
    data = json.loads(descriptor.read_text())
    entries = data.get("seeds")
    if data.get("schemaVersion") != 1 or not isinstance(entries, list) or not entries:
        raise AdapterError("Expected a nonempty versioned portable seed population.")
    ids = [entry["id"] for entry in entries]
    if len(set(ids)) != len(ids):
        raise AdapterError("Portable seed IDs must be unique.")
    if selected is not None and (not selected or not set(selected) <= set(ids)):
        raise AdapterError("Requested seed IDs must belong to the portable population.")
    if output.exists():
        raise AdapterError("Seed materialization requires a fresh output directory.")
    batches = []
    for entry in entries:
        if selected is not None and entry["id"] not in selected:
            continue
        source = (population / entry["directory"]).resolve()
        destination = (output / entry["id"]).resolve()
        if source.parent != population or destination.parent != output:
            raise AdapterError("Portable seeds must use immediate population subdirectories.")
        for name in ("program.json", "realization.json"):
            digest = hashlib.sha256((source / name).read_bytes()).hexdigest()
            if entry["sha256"].get(name) != digest:
                raise AdapterError(f"Portable seed {entry['id']} changed: {name}.")
        command = [*generator.command, "realize-walker",
                   "--program", str(source / "program.json"), "--genome", str(source / "realization.json"),
                   "--output", str(destination), "--steps", "96"]
        result = subprocess.run(command, capture_output=True, text=True, timeout=120)
        if result.returncode != 0:
            raise AdapterError(f"Seed {entry['id']} did not lower: {result.stderr.strip() or result.stdout.strip()}")
        summary, _ = _queue(destination)
        candidate, = summary["generated"]
        genome = load_manifest(destination / candidate["directory"] / "manifest.json")["genome"]
        original = json.loads((source / "realization.json").read_text())
        if any(genome.get(key) != original.get(key) for key in ("speed", "maxTorque", "remote", "towWagon")):
            raise AdapterError(f"Seed {entry['id']} changed electrical control or payload mode.")
        batches.append(entry["id"])
    write_json(output / "seeds.json", {
        "schemaVersion": 1, "batches": batches,
        "sourcePopulationSha256": hashlib.sha256(descriptor.read_bytes()).hexdigest(),
        "qualification": "Motion/control source only; current construction, exact admission and local evaluation remain mandatory.",
    })
    return batches


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--population", type=Path, default=ROOT / "seeds/recommended")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", action="append", help="Materialize only these IDs; repeat for multiple seeds.")
    parser.add_argument("--dotnet", type=Path, default=Path(shutil.which("dotnet") or ROOT / ".tools/dotnet/dotnet"))
    parser.add_argument("--generator", type=Path, default=ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    args = parser.parse_args(argv)
    try:
        batches = materialize(args.population, args.output, Generator(args.dotnet, args.generator), args.seed)
    except (AdapterError, OSError, json.JSONDecodeError, subprocess.TimeoutExpired) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"Materialized {len(batches)} motion seeds; no dynamic trials or inherited fitness.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
