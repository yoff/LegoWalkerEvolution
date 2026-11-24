"""Combine offline drive layouts with measured part mass and clearance envelopes."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from lego_technic_sim.ldraw.parser import LDrawParser

from legowalkers_sim.bridge import AdapterError, LDU_TO_METERS, library_fingerprint, load_manifest, write_json
from legowalkers_sim.collision import CollisionPart
from legowalkers_sim.payload import part_mass_center
from plan_packaging import ROOT, digest, vector


def prepared_part(source, geometry, owner, kind):
    if kind not in ("gear", "shaft", "motor"):
        raise AdapterError(f"Unsupported combined drive geometry kind: {kind}")
    transform = np.eye(4)
    rotation = source["rotation"]
    transform[:3, :3] = [[rotation[column][row] for column in "xyz"] for row in "xyz"]
    transform[:3, 3] = [source["position"][axis] for axis in "xyz"]
    if (source["part"] != geometry.part_id or source["color"] != geometry.color
            or not np.allclose(geometry.transform, transform, atol=1e-8, rtol=0)):
        raise AdapterError(f"Combined input placement mismatch: {source['id']}")
    vertices = np.array([v for tri in geometry.triangles for v in (tri.v0, tri.v1, tri.v2)])
    local = (vertices-geometry.position) @ transform[:3, :3]
    if not len(local) or not np.isfinite(local).all():
        raise AdapterError(f"Combined input geometry invalid: {source['id']}")
    mass, center = part_mass_center(CollisionPart(source["id"], geometry, 0))
    low, high = local.min(axis=0), local.max(axis=0)
    record = {
        "id": source["id"], "part": source["part"], "body": owner,
        "massKg": mass, "center": vector(center/LDU_TO_METERS),
        "low": vector(low), "high": vector(high), "source": "ABS-mesh-estimate",
    }
    position = transform[:3, 3]
    if kind == "gear":
        if not np.allclose(transform[:3, 2], [0, 0, 1], atol=1e-8, rtol=0):
            raise AdapterError("Combined prototype requires transverse gear axes.")
        radius = float(np.linalg.norm(local[:, :2], axis=1).max())
        world_low = position + [-radius, -radius, low[2]]
        world_high = position + [radius, radius, high[2]]
        start = end = position
    elif kind == "shaft":
        if not np.allclose(transform[:3, 0], [0, 0, 1], atol=1e-8, rtol=0):
            raise AdapterError("Combined prototype requires transverse axle axes.")
        radius = float(np.linalg.norm(local[:, 1:], axis=1).max())
        start = position+transform[:3, 0]*low[0]
        end = position+transform[:3, 0]*high[0]
        world_low = np.minimum(start, end)-radius
        world_high = np.maximum(start, end)+radius
    else:
        return record, None
    envelope = {
        "id": source["id"], "body": owner, "kind": kind,
        "low": vector(world_low), "high": vector(world_high),
        "segmentStart": vector(start), "segmentEnd": vector(end), "radius": radius,
    }
    return record, envelope


def layout_properties(layout, build):
    gears = {item["placement"]["id"]: item for item in layout["gears"]}
    shafts = {item["placement"]["id"]: item for item in layout["shafts"]}
    sources = ([item["placement"] for item in layout["gears"]]
               + [item["placement"] for item in layout["shafts"]] + [layout["motor"]["placement"]])
    if len(sources) != len(build.parts):
        raise AdapterError("Combined drive part count differs from the LDraw subassembly.")
    records, envelopes = [], []
    for source, geometry in zip(sources, build.parts, strict=True):
        if source["id"] in gears:
            owner, kind = source["body"], "gear"
        elif source["id"] in shafts:
            owner, kind = source["id"].removeprefix("constructed-axle-"), "shaft"
        else:
            owner, kind = source["body"], "motor"
        record, envelope = prepared_part(source, geometry, owner, kind)
        records.append(record)
        if envelope:
            envelopes.append(envelope)
    return records, envelopes, [{key: value for key, value in source.items() if key != "body"} for source in sources]


def run(transmissions, mass_input, output, library, dotnet):
    transmissions, mass_input, output = transmissions.resolve(), mass_input.resolve(), output.resolve()
    for source in (transmissions, mass_input.parent):
        if output == source or source in output.parents or output in source.parents:
            raise AdapterError("Combined output must be separate from its immutable inputs.")
    report_path = transmissions / "transmission-plan.json"
    report = json.loads(report_path.read_text())
    base = json.loads(mass_input.read_text())
    parent = Path(report["parent"]).resolve()
    for source in (parent, transmissions):
        if output == source or source in output.parents or output in source.parents:
            raise AdapterError("Combined output must be separate from its immutable inputs.")
    if output.exists() and any(output.iterdir()):
        raise AdapterError("Combined output must be empty.")
    if digest(mass_input) != report["massInputSha256"]:
        raise AdapterError("Combined source mass input differs from the transmission experiment.")
    for filename, key in (("program.json", "programSha256"), ("realization.json", "realizationSha256"),
                          ("manifest.json", "parentManifestSha256")):
        if digest(parent / filename) != base[key]:
            raise AdapterError(f"Combined source input changed: {filename}")
    selected = [layout for layout in report["layouts"]
                if layout["id"].startswith(("root-02-interior-", "root-04-interior-", "root-05-interior-"))]
    if len(selected) != 6:
        raise AdapterError("Expected six central/upper/lower interior Birth 15 motor layouts.")
    parser = LDrawParser(parts_dir=library.resolve())
    layouts, dependencies = [], set()
    for layout in selected:
        folder = transmissions / layout["id"]
        records, envelopes, placements = layout_properties(layout, parser.parse_build(folder / "transmission.ldr"))
        comparison = layout["comparison"]
        if (comparison["maxBodyPositionErrorLdu"] > 1e-5 or comparison["maxBodyAngleErrorRadians"] > 1e-6
                or comparison["maxFootPositionErrorLdu"] > 1e-5):
            raise AdapterError(f"Drive layout does not preserve walking: {layout['id']}")
        dependencies.update(part["part"] for part in records)
        layouts.append({
            "id": layout["id"], "root": layout["root"], "clockScale": comparison["motorClockScale"],
            "motorSpeed": layout["motor"]["speed"], "maxTorque": layout["motor"]["maxTorque"],
            "driveProgramPath": str((folder / "drive-program.json").resolve()),
            "driveProgramSha256": digest(folder / "drive-program.json"),
            "ldrawPath": str((folder / "transmission.ldr").resolve()),
            "ldrawSha256": digest(folder / "transmission.ldr"),
            "parts": records, "placements": placements, "envelopes": envelopes,
            "bearingTargets": [target for shaft in layout["shafts"] for target in shaft["bearingTargets"]],
        })
    manifest = load_manifest(parent / "manifest.json")
    source_program = json.loads((parent / "program.json").read_text())["program"]
    shaft_ids = {source_program["motor"]["body"]}
    shaft_ids.update(edge[key] for edge in source_program["transmissions"] for key in ("input", "output"))
    chassis = next(body["id"] for body in source_program["bodies"] if body["grounded"])
    bearings = {joint["connector"]: next(joint[key] for key in ("bodyA", "bodyB") if joint[key] != chassis)
                for joint in manifest["expected"]["joints"]
                if chassis in (joint["bodyA"], joint["bodyB"])
                and any(joint[key] in shaft_ids for key in ("bodyA", "bodyB"))}
    legacy_gear_ids = {part["id"] for part in manifest["parts"]
                       if part["part"] in ("3647.dat", "3648b.dat", "3649.dat") and part["body"] in shaft_ids}
    legacy_bush_ids = {part["id"] for part in manifest["parts"]
                       if part["part"] in ("3713.dat", "4265c.dat") and part["body"] in shaft_ids}
    removed = legacy_gear_ids | legacy_bush_ids | set(bearings)
    build = parser.parse_build(parent / manifest["modelFile"])
    if len(build.parts) != len(manifest["parts"]):
        raise AdapterError("Combined parent geometry part count mismatch.")
    legacy_envelopes, legacy_placements = [], []
    base_parts = {item["id"]: item for item in base["parts"]}
    for source, geometry in zip(manifest["parts"], build.parts, strict=True):
        if source["id"] not in legacy_gear_ids | set(bearings):
            continue
        rotation = source["rotation"]
        placement = {**source, "position": vector(source["position"]),
                     "rotation": {column: vector(np.array(rotation)[:, i]) for i, column in enumerate("xyz")}}
        _, envelope = prepared_part(placement, geometry, base_parts[source["id"]]["body"],
                                    "gear" if source["id"] in legacy_gear_ids else "shaft")
        legacy_envelopes.append(envelope)
        legacy_placements.append({key: placement[key] for key in ("id", "part", "color", "position", "rotation")})
    output.mkdir(parents=True, exist_ok=True)
    input_path = output / "combined-input.json"
    write_json(input_path, {
        "schemaVersion": 1, "parent": str(parent), "sourceMass": base,
        "sourceMassPath": str(mass_input), "sourceMassSha256": digest(mass_input),
        "drivePlanPath": str(report_path), "drivePlanSha256": digest(report_path),
        "freshDependencySha256": library_fingerprint(library, sorted(dependencies)),
        "removedLegacyIds": sorted(removed), "legacyEnvelopes": legacy_envelopes,
        "legacyPlacements": legacy_placements, "drives": layouts,
    })
    result = subprocess.run([
        str(dotnet.resolve()), "fsi", str(ROOT / "scripts/combined_planner.fsx"),
        "--input", str(input_path), "--output", str(output / "plans"),
    ], cwd=ROOT, capture_output=True, text=True)
    if result.returncode:
        raise AdapterError(f"F# combined planner failed:\n{result.stderr.strip()}\n{result.stdout.strip()}")
    print(result.stdout.strip())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transmissions", type=Path, required=True)
    parser.add_argument("--mass-properties", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library", type=Path, default=ROOT / ".tools/ldraw")
    parser.add_argument("--dotnet", type=Path, default=ROOT / ".tools/dotnet/dotnet")
    args = parser.parse_args()
    try:
        run(args.transmissions, args.mass_properties, args.output, args.library, args.dotnet)
    except (AdapterError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
