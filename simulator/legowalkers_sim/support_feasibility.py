"""Offline sampled ground-reach diagnostics; no dynamics and no live rejection rule."""

import argparse
from collections import Counter, defaultdict
import contextlib
import hashlib
from html import escape
import json
import math
from pathlib import Path
import sqlite3
import subprocess
import warnings

import numpy as np

from .bridge import AdapterError, LDU_TO_METERS, PartParserCache, load_manifest, write_json
from .cycles import cycle_plan
from .evaluation import EvaluationOptions
from .evolution import is_qualified
from .ground import foot_regions
from .support_surfaces import support_surfaces

VERSION = "sampled-support-feasibility-v1"
LIMITATIONS = [
    "Diagnostic only: no admission, selection, qualification or live rejection rule is changed.",
    "Finite motor-phase and chassis-tilt samples cannot prove impossibility between samples.",
    "Potential foot contacts use the intersection of mesh-depth and region-depth bounds, not confirmed physical contacts.",
    "Only mesh vertices outside allowed regions obstruct the plane; unsampled face interiors may also obstruct it.",
    "Connectors without a declared rigid owner are omitted, making clearance optimistic.",
    "No COM balance, friction, contact forces, self-collision, dynamic leaning or walking is established.",
    "One-foot support and airborne phases can be valid dynamically; two-foot counts are diagnostic, not requirements.",
]


def _read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _origin(pose):
    value = pose["origin"]
    return np.asarray([value[axis] for axis in ("x", "y", "z")] if isinstance(value, dict) else value,
                      dtype=float)


def ground_normals():
    """Downward ground normals in robot coordinates, including upright first."""
    result = [{"pitchDegrees": 0, "rollDegrees": 0, "normal": [0., 1., 0.]}]
    for pitch in range(-60, 61, 15):
        for roll in range(-60, 61, 15):
            p, r = math.radians(pitch), math.radians(roll)
            normal = [math.sin(p)*math.cos(r), math.cos(p)*math.cos(r), math.sin(r)]
            if (pitch or roll) and normal[1] >= .5-1e-12:
                result.append({"pitchDegrees": pitch, "rollDegrees": roll, "normal": normal})
    return result


def _local(points, pose):
    return (points-_origin(pose)) @ _rotation(pose["angle"])


def _world(points, pose):
    return points @ _rotation(pose["angle"]).T + _origin(pose)


def analyze(manifest, motion, vertices, *, contact_tolerance_meters=.001):
    """Sweep declared rigid-body poses and actual LDraw vertices, without a physics model."""
    if not math.isfinite(contact_tolerance_meters) or not 0 <= contact_tolerance_meters <= .004:
        raise AdapterError("Support diagnostic contact tolerance must be between zero and 4 mm.")
    program, frames = motion["program"], motion["frames"]
    bodies = {body["id"]: body for body in program["bodies"]}
    if len(frames) < 2:
        raise AdapterError("Support diagnostic requires a sampled motion cycle.")
    for frame in frames:
        if {pose["body"] for pose in frame["poses"]} != set(bodies):
            raise AdapterError("Support diagnostic requires every body pose in every frame.")
        values = [value for pose in frame["poses"] for value in [*_origin(pose), pose["angle"]]]
        if not np.isfinite(values).all() or not math.isfinite(frame["inputAngle"]):
            raise AdapterError("Support diagnostic requires finite poses and motor phases.")
    feet = manifest["feet"]
    if not feet:
        raise AdapterError("Support diagnostic requires designated feet.")
    points = np.asarray([foot["position"] for foot in feet], dtype=float)
    radii = np.asarray([region["radiusMeters"] for region in foot_regions(manifest)])/LDU_TO_METERS
    if (points.shape != (len(feet), 3) or not np.isfinite(points).all()
            or not np.isfinite(radii).all() or np.any(radii < 0)):
        raise AdapterError("Support diagnostic requires finite contact geometry.")
    groups = defaultdict(list)
    for index, foot in enumerate(feet):
        groups[foot.get("logicalFoot", foot["id"])].append(index)
        if foot["body"] not in bodies:
            raise AdapterError("Support diagnostic foot has no motion body.")
    # Realization placements and contact anchors use the first solved motor sample.
    initial = {pose["body"]: pose for pose in frames[0]["poses"]}
    foot_local = [_local(points[index:index+1], initial[foot["body"]])[0]
                  for index, foot in enumerate(feet)]
    regions = foot_regions(manifest)
    surfaces = support_surfaces(manifest, regions)
    physical_feet = points*LDU_TO_METERS
    clouds = defaultdict(list)
    omitted = []
    vertex_count = 0
    for part in manifest["parts"]:
        values = np.asarray(vertices[part["id"]], dtype=float)
        if values.ndim != 2 or values.shape[1] != 3 or not len(values) or not np.isfinite(values).all():
            raise AdapterError(f"Support diagnostic requires finite mesh vertices: {part['id']}.")
        if part["body"] is None:
            omitted.append(part["id"])
            continue
        if part["body"] not in bodies:
            raise AdapterError(f"Support diagnostic mesh has no motion body: {part['id']}.")
        values = np.unique(values, axis=0)
        allowed = np.zeros(len(values), dtype=bool)
        for region in regions:
            if region["partInstance"] == part["id"]:
                allowed |= np.linalg.norm(values-points[region["site"]], axis=1) <= (
                    region["radiusMeters"]/LDU_TO_METERS)
        for surface in surfaces:
            if surface.part == part["id"]:
                allowed |= np.array([surface.contains(value*LDU_TO_METERS, physical_feet) for value in values])
        vertex_count += int((~allowed).sum())
        clouds[part["body"]].append((part["id"], _local(values, initial[part["body"]]), ~allowed))
    if not clouds:
        raise AdapterError("Support diagnostic requires physical obstruction geometry.")
    orientations = ground_normals()
    normals = np.asarray([item["normal"] for item in orientations])
    tolerance = contact_tolerance_meters/LDU_TO_METERS
    names = list(groups)
    minimum_gaps = np.full((len(names), len(normals)), np.inf)
    participation = np.zeros((len(names), len(normals)), dtype=int)
    clear_counts = np.zeros(len(normals), dtype=int)
    two_counts = np.zeros(len(normals), dtype=int)
    max_contacts = np.zeros(len(normals), dtype=int)
    upright = []
    witness = None
    projected = {}
    for frame_index, frame in enumerate(frames):
        poses = {pose["body"]: pose for pose in frame["poses"]}
        foot_points = np.asarray([_world(foot_local[index:index+1], poses[foot["body"]])[0]
                                  for index, foot in enumerate(feet)])
        depths = foot_points @ normals.T + radii[:, None]
        plane = np.full(len(normals), -np.inf)
        obstruction = np.full(len(normals), -np.inf)
        blockers = [None]*len(normals)
        mesh_depths = {}
        for body, parts in clouds.items():
            pose = poses[body]
            local_normals = normals @ _rotation(pose["angle"])
            origin_depth = _origin(pose) @ normals.T
            for part, values, forbidden in parts:
                key = (part, pose["angle"], tuple(_origin(pose)))
                if key not in projected:
                    projections = values @ local_normals.T + origin_depth
                    projected[key] = (projections.max(axis=0),
                                      projections[forbidden].max(axis=0) if forbidden.any() else None)
                mesh_depths[part], depth = projected[key]
                plane = np.maximum(plane, mesh_depths[part])
                if depth is None:
                    continue
                deeper = depth > obstruction
                for index in np.flatnonzero(deeper):
                    blockers[index] = part
                obstruction = np.maximum(obstruction, depth)
        # A contact region is not solid geometry: its radius must not push the
        # ground plane below the actual foot mesh and hide a low chassis or heel.
        bounded_depths = np.asarray([
            np.minimum(depths[index], mesh_depths[regions[index]["partInstance"]])
            for index in range(len(feet))])
        logical_depths = np.asarray([bounded_depths[indices].max(axis=0) for indices in groups.values()])
        gaps = np.maximum(0, plane[None, :]-logical_depths)
        minimum_gaps = np.minimum(minimum_gaps, gaps)
        clear = obstruction < plane-1e-7
        touching = gaps <= tolerance
        contacts = touching.sum(axis=0)
        participation += touching & clear[None, :]
        clear_counts += clear
        two_counts += clear & (contacts >= 2)
        max_contacts = np.maximum(max_contacts, np.where(clear, contacts, 0))
        upright.append({
            "frame": frame_index, "inputAngle": frame["inputAngle"],
            "potentialLogicalContacts": [name for index, name in enumerate(names) if touching[index, 0]],
            "clearOfForbiddenVertices": bool(clear[0]), "limitingPart": blockers[0],
            "forbiddenClearanceMeters": (float(plane[0]-obstruction[0])*LDU_TO_METERS
                                         if math.isfinite(obstruction[0]) else None),
        })
        if witness is None and np.any(clear & (contacts >= 2)):
            index = int(np.flatnonzero(clear & (contacts >= 2))[0])
            witness = {"frame": frame_index, "inputAngle": frame["inputAngle"],
                       **orientations[index], "potentialLogicalContacts": [
                           name for foot_index, name in enumerate(names) if touching[foot_index, index]]}
    alerts = []
    if not clear_counts.any():
        alerts.append("no-sampled-clear-ground-pose")
    if not two_counts.any():
        alerts.append("no-sampled-two-foot-ground-pose")
    elif not two_counts[0]:
        alerts.append("sampled-two-foot-support-requires-leaning")
    feet_report = []
    for index, name in enumerate(names):
        if not participation[index].any():
            alerts.append(f"logical-foot-never-reaches-clear-ground:{name}")
        elif not participation[index, 0]:
            alerts.append(f"logical-foot-requires-leaning:{name}")
        feet_report.append({
            "id": name, "uprightMinimumShortfallMeters": float(minimum_gaps[index, 0])*LDU_TO_METERS,
            "tiltedMinimumShortfallMeters": float(minimum_gaps[index].min())*LDU_TO_METERS,
            "uprightClearContactSamples": int(participation[index, 0]),
            "allClearContactSamples": int(participation[index].sum()),
        })
    return {
        "version": VERSION, "mode": "optimistic-sampled-ground-reach-not-balance",
        "frames": len(frames), "orientations": len(normals),
        "motorTurns": (frames[-1]["inputAngle"]-frames[0]["inputAngle"])/(2*math.pi),
        "contactToleranceMeters": contact_tolerance_meters,
        "maximumSampledTiltDegrees": 60, "tiltGridDegrees": 15,
        "uprightClearFrames": int(clear_counts[0]), "uprightTwoFootFrames": int(two_counts[0]),
        "maximumUprightLogicalContacts": int(max_contacts[0]),
        "maximumTiltedLogicalContacts": int(max_contacts.max()),
        "alerts": alerts, "feet": feet_report, "optimisticPoseWitness": witness,
        "uprightFrames": upright, "forbiddenVertices": vertex_count,
        "omittedUnownedConnectors": omitted, "simulatedSteps": 0, "campaignTrials": 0,
        "liveGateChanged": False, "limitations": LIMITATIONS,
    }


def _meshes(manifest, source, library, cache, stock_meshes):
    parser = cache.parser_for(source / manifest["modelFile"], library,
                              [part["part"] for part in manifest["parts"]])
    lines = [line.split(maxsplit=14) for line in (source / manifest["modelFile"]).read_text().splitlines()
             if line.strip()]
    if any(line[0] not in ("0", "1") or line[:2] == ["0", "FILE"] for line in lines):
        raise AdapterError("Support diagnostic requires a flat, library-only exported LDraw model.")
    placements = [line for line in lines if line[0] == "1"]
    if len(placements) != len(manifest["parts"]):
        raise AdapterError("Support geometry does not match the manifest part count.")
    vertices = {}
    for part, actual in zip(manifest["parts"], placements, strict=True):
        if len(actual) != 15:
            raise AdapterError("Invalid exported LDraw placement.")
        values = np.asarray([float(value) for value in actual[2:14]])
        if (part["part"] != actual[14]
                or not np.allclose(values[:3], part["position"], atol=1e-7, rtol=0)
                or not np.allclose(values[3:].reshape(3, 3), part["rotation"], atol=1e-7, rtol=0)):
            raise AdapterError(f"Support geometry differs from manifest: {part['id']}.")
        stock = (library / "parts" / part["part"]).resolve()
        if not stock.is_relative_to(library) or not stock.is_file() or (source / part["part"]).exists():
            raise AdapterError(f"Support diagnostic requires an existing library stock part: {part['part']}.")
        key = (parser, stock)
        if key not in stock_meshes:
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                triangles = parser.parse_part(stock)
            if caught:
                cache.invalidate()
                raise AdapterError("LDraw support geometry warnings: "+"; ".join(str(item.message) for item in caught))
            local = np.array([value for triangle in triangles
                              for value in (triangle.v0, triangle.v1, triangle.v2)])
            if local.ndim != 2 or local.shape[1] != 3 or not len(local) or not np.isfinite(local).all():
                raise AdapterError(f"Support diagnostic requires finite stock mesh vertices: {stock}.")
            stock_meshes[key] = np.unique(local, axis=0)
        vertices[part["id"]] = stock_meshes[key] @ np.asarray(part["rotation"]).T + part["position"]
    return vertices


def _cohort(archives):
    designs = {}
    for archive in archives:
        with contextlib.closing(sqlite3.connect(f'{(archive/"archive.sqlite3").as_uri()}?mode=ro', uri=True)) as db:
            db.row_factory = sqlite3.Row
            db.execute("BEGIN")
            rows = db.execute("SELECT id,design,status,operation FROM births ORDER BY id").fetchall()
            measurements = defaultdict(list)
            for table in ("evaluations", "reused_evaluations"):
                for row in db.execute(f"SELECT b.design,e.result FROM {table} e JOIN births b ON e.birth=b.id "
                                      "WHERE e.result IS NOT NULL"):
                    result = json.loads(row["result"])
                    measurements[row["design"]].append({
                        "qualified": is_qualified(result), "status": result["status"],
                        "rejectionReasons": result.get("metrics", {}).get("rankingRejectionReasons", []),
                    })
        for row in rows:
            source = archive / "designs" / row["design"] / "candidate"
            entry = designs.setdefault(row["design"], {"design": row["design"], "source": source,
                                                       "births": [], "measurements": []})
            entry["births"].append({"archive": str(archive), **dict(row)})
            entry["measurements"].extend(measurements[row["design"]])
    return list(designs.values())


def page(report):
    rows = []
    for case in report["designs"]:
        result = case.get("diagnostic")
        refs = ", ".join(f'{Path(b["archive"]).name} birth {b["id"]}' for b in case["births"])
        if result is None:
            detail, counts = "Excluded: "+case["exclusion"], "-"
        else:
            detail = "; ".join(result["alerts"]) or "No sampled reach warning"
            counts = f'{result["maximumUprightLogicalContacts"]} / {result["maximumTiltedLogicalContacts"]}'
            gaps = ", ".join(f'{foot["id"]}: {1000*foot["uprightMinimumShortfallMeters"]:.1f} mm'
                             for foot in result["feet"])
            detail += "\nMinimum upright ground shortfall: "+gaps
        measurements = case["measurements"]
        label = ("qualified" if any(item["qualified"] for item in measurements)
                 else "measured, not qualified" if measurements else "unmeasured")
        rows.append(f'<tr><td><a href="{escape(case["source"], quote=True)}/cycle.svg">{escape(refs)}</a></td>'
                    f'<td>{label}</td><td>{counts}</td><td style="white-space:pre-line">{escape(detail)}</td></tr>')
    limits = "".join(f"<li>{escape(text)}</li>" for text in LIMITATIONS)
    return ('<!doctype html><meta charset="utf-8"><title>Sampled ground-reach diagnostic</title>'
            '<style>body{font:15px system-ui;max-width:1300px;margin:2rem auto;padding:1rem}'
            'table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:.5rem;text-align:left}</style>'
            '<h1>Sampled ground-reach diagnostic</h1><p>No dynamics. No live gate changes.</p>'
            f'<pre>{escape(json.dumps(report["summary"], indent=2))}</pre>'
            '<p>Contact counts are optimistic maxima: upright / including sampled leaning. They do not prove balance.</p>'
            '<table><tr><th>Archived design</th><th>Existing observation</th><th>Logical contacts</th>'
            '<th>Diagnostic</th></tr>'+''.join(rows)+'</table><h2>Limits</h2><ul>'+limits+'</ul>'
            '<p><a href="support-feasibility.json">Full JSON, per-foot gaps and phase diagnostics</a></p>')


def run(archives, output, library, dotnet, generator, *, steps=72):
    archives = [Path(path).resolve() for path in archives]
    output, library = Path(output).resolve(), Path(library).resolve()
    if not archives or not 36 <= steps <= 3600:
        raise AdapterError("Supply archived cohorts and 36 to 3600 samples per motor turn.")
    for archive in archives:
        if not (archive / "archive.sqlite3").is_file():
            raise AdapterError(f"Not an existing archive: {archive}")
        # Also protect sibling campaign versions and its workbench/registry.
        protected = archive.parent.parent if archive.parent.name == "versions" else archive
        if output.is_relative_to(protected) or protected.is_relative_to(output):
            raise AdapterError("Support diagnostic output must be outside the scientific campaign.")
    if output.exists():
        raise AdapterError("Support diagnostic requires a fresh output directory.")
    if not library.is_dir() or not Path(dotnet).is_file() or not Path(generator).is_file():
        raise AdapterError("Existing LDraw library, dotnet executable and generator DLL are required.")
    cohort = _cohort(archives)
    if not cohort:
        raise AdapterError("The selected archives contain no births.")
    output.mkdir(parents=True)
    cache, results, motions, stock_meshes = PartParserCache(), [], {}, {}
    for index, case in enumerate(cohort):
        source = case.pop("source")
        case["source"] = str(source)
        if not (source / "program.json").is_file():
            case["exclusion"] = "No declared motion program; no trajectory was invented."
            results.append(case)
            continue
        manifest = load_manifest(source / "manifest.json")
        program = _read(source / "program.json")["program"]
        turns = cycle_plan(manifest, EvaluationOptions(speed=100, settle=0))["motorTurnsPerCycle"]
        if turns > 32:
            case["exclusion"] = f"Declared gait needs {turns} motor turns; exporter cap is 32."
            results.append(case)
            continue
        key = hashlib.sha256((source / "program.json").read_bytes()).hexdigest()
        case["sourceSha256"] = {name: hashlib.sha256((source/name).read_bytes()).hexdigest()
                               for name in ("manifest.json", manifest["modelFile"], "program.json")}
        if key not in motions:
            target = output / "motions" / key
            target.parent.mkdir(exist_ok=True)
            with (target.parent / f"{key}.log").open("w") as log:
                completed = subprocess.run(
                    [str(dotnet), str(generator), "motion", "--program", str(source / "program.json"),
                     "--output", str(target), "--steps", str(steps), "--turns", str(turns)],
                    stdout=log, stderr=subprocess.STDOUT, timeout=600)
            if completed.returncode:
                raise AdapterError(f"Motion export failed for {source}; see {target.parent / (key+'.log')}.")
            motion = _read(target / "motion.json")
            if motion["program"] != program:
                raise AdapterError("Motion export changed the archived program.")
            if len(motion["frames"]) != steps*turns+1:
                raise AdapterError("Motion export did not cover the requested full gait cycle.")
            if not math.isclose(motion["frames"][-1]["inputAngle"]-motion["frames"][0]["inputAngle"],
                                turns*2*math.pi, abs_tol=1e-8):
                raise AdapterError("Motion export has the wrong motor-phase coverage.")
            motions[key] = motion
        motion = motions[key]
        print(f'{index+1}/{len(cohort)}: loading meshes for birth {case["births"][0]["id"]}', flush=True)
        vertices = _meshes(manifest, source, library, cache, stock_meshes)
        print(f'{index+1}/{len(cohort)}: sweeping poses for birth {case["births"][0]["id"]}', flush=True)
        case["diagnostic"] = analyze(manifest, motion, vertices)
        case["declaredGaitMotorTurns"] = turns
        if any(hashlib.sha256((source/name).read_bytes()).hexdigest() != value
               for name, value in case["sourceSha256"].items()):
            raise AdapterError(f"Archived design changed during diagnosis: {source}")
        results.append(case)
        print(f'{index+1}/{len(cohort)}: {case["births"][0]["id"]}: '
              f'{len(case["diagnostic"]["alerts"])} diagnostic warnings', flush=True)
        write_json(output / "progress.json", {"completed": len(results), "total": len(cohort)})
    analyzed = [case for case in results if "diagnostic" in case]
    measured = [case for case in analyzed if case["measurements"]]
    qualified = [case for case in measured if any(item["qualified"] for item in case["measurements"])]
    report = {
        "version": VERSION, "archives": [str(path) for path in archives], "stepsPerMotorTurn": steps,
        "summary": {
            "uniqueDesigns": len(results), "analyzed": len(analyzed), "excluded": len(results)-len(analyzed),
            "measuredControls": len(measured), "qualifiedControls": len(qualified),
            "qualifiedControlsWithWarnings": sum(bool(case["diagnostic"]["alerts"]) for case in qualified),
            "warningCounts": dict(Counter(kind for case in analyzed
                                          for kind in {alert.split(":")[0]
                                                       for alert in case["diagnostic"]["alerts"]})),
            "simulatedSteps": 0, "campaignTrials": 0, "liveGateChanged": False,
        }, "designs": results, "limitations": LIMITATIONS,
    }
    write_json(output / "support-feasibility.json", report)
    (output / "support-feasibility.html").write_text(page(report), encoding="utf-8")
    (output / "progress.json").unlink()
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--dotnet", type=Path, required=True)
    parser.add_argument("--generator", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=72)
    args = parser.parse_args(argv)
    report = run(args.archive, args.output, args.library, args.dotnet, args.generator, steps=args.steps)
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
