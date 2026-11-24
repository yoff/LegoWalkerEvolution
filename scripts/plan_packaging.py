"""Prepare measured-geometry mass estimates for an offline F# packaging experiment."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from legowalkers_sim.bridge import AdapterError, LDU_TO_METERS, check, write_json
from legowalkers_sim.collision import prepare_parts
from legowalkers_sim.payload import load_profile, part_mass_center

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(value):
    return dict(zip(("x", "y", "z"), map(float, value), strict=True))


def mass_properties(checked, profile):
    _, parts, _ = prepare_parts(checked, profile=profile)
    owners = {index: body for body, index in checked.body_indices.items()}
    payloads = {item["partInstance"]: profile["components"][item["product"]]
                for item in checked.manifest.get("payloads", [])}
    records = []
    for part in parts:
        mass, center = part_mass_center(part, payloads.get(part.instance))
        vertices = np.array([v for triangle in part.geometry.triangles
                             for v in (triangle.v0, triangle.v1, triangle.v2)])
        local = (vertices-part.geometry.position) @ part.geometry.transform[:3, :3]
        if (not len(local) or not np.isfinite(local).all()
                or part.unit not in owners):
            raise AdapterError(f"Invalid packaging geometry or body owner: {part.instance}")
        center_ldu = center/LDU_TO_METERS
        if part.instance in payloads and (
                np.any(center_ldu < vertices.min(axis=0)-1e-6)
                or np.any(center_ldu > vertices.max(axis=0)+1e-6)):
            raise AdapterError(f"Packaging payload center outside its geometry: {part.instance}")
        records.append({
            "id": part.instance, "part": part.geometry.part_id, "body": owners[part.unit],
            "massKg": mass, "center": vector(center_ldu),
            "low": vector(local.min(axis=0)), "high": vector(local.max(axis=0)),
            "source": "payload-profile-estimate" if part.instance in payloads else "ABS-mesh-estimate",
        })
    if len(records) != len(checked.manifest["parts"]):
        raise AdapterError("Packaging mass input must cover every part, including connectors.")
    return sorted(records, key=lambda item: item["id"])


def run(parent, output, library, dotnet, payload_profile=None):
    parent, output = parent.resolve(), output.resolve()
    if parent == output or parent in output.parents or output in parent.parents:
        raise AdapterError("Packaging output must be separate from its immutable parent.")
    if output.exists() and any(output.iterdir()):
        raise AdapterError("Packaging experiment output must be empty.")
    profile = load_profile(payload_profile)
    checked = check(parent / "manifest.json", library)
    records = mass_properties(checked, profile)
    output.mkdir(parents=True, exist_ok=True)
    properties = output / "mass-properties.json"
    write_json(properties, {
        "schemaVersion": 1,
        "parentManifestSha256": digest(parent / "manifest.json"),
        "programSha256": digest(parent / "program.json"),
        "realizationSha256": digest(parent / "realization.json"),
        "dependencySha256": checked.report["partsDependencySha256"],
        "payloadProfileSha256": hashlib.sha256(json.dumps(
            profile, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
        "parts": records,
    })
    write_json(output / "payload-profile.json", profile)
    command = [str(dotnet.resolve()), "fsi", str(ROOT / "scripts/packaging_planner.fsx"),
               "--parent", str(parent), "--mass-properties", str(properties),
               "--output", str(output / "plans")]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    if result.returncode:
        raise AdapterError(f"F# packaging planner failed:\n{result.stderr.strip()}\n{result.stdout.strip()}")
    print(result.stdout.strip())
    return output / "plans"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parent", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--library", type=Path, default=ROOT / ".tools/ldraw")
    parser.add_argument("--dotnet", type=Path, default=ROOT / ".tools/dotnet/dotnet")
    parser.add_argument("--payload-profile", type=Path)
    args = parser.parse_args()
    try:
        run(args.parent, args.output, args.library, args.dotnet, args.payload_profile)
    except (AdapterError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
