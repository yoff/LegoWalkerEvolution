"""Bounded, persisted candidate screening. This module never advances physics."""

from __future__ import annotations

import hashlib
from contextlib import nullcontext
import importlib.metadata
import json
from pathlib import Path
import sqlite3

import numpy as np

from .bridge import (
    AdapterError, MOTION_GRAMMARS, SIMULATOR_REVISION, check, library_fingerprint, load_manifest, logical_foot_count, write_json,
)
from .collision import prepare_parts
from .evaluation import EvaluationOptions, prepare_xml
from .mass_layout import analyze_layout
from .payload import apply_payloads, load_profile
from .screening_reports import ScreeningProgress, rejection_detail, repair_hints

SCREEN_VERSION = "candidate-screen-v1"
PENETRATION_TOLERANCE = 0.0001
GATE_ORDER = ["structure", "mobility", "motion-potential", "lego-realizability", "geometry"]


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _protocol(load_model):
    from lego_technic_sim.physics import unit_builder
    sources = [Path(__file__).with_name(name) for name in
               ("screening.py", "screening_reports.py", "bridge.py", "collision.py", "evaluation.py", "joint_tree.py", "spatial_gears.py", "stance_support.py", "motion_carry.py", "motor.py", "payload.py",
                "ground.py", "schema.py", "cycles.py", "mass_layout.py", "phase_clearance.py")]
    sources.extend(sorted(Path(unit_builder.__file__).parent.glob("*.py")))
    fingerprint = hashlib.sha256()
    for path in sources:
        fingerprint.update(path.name.encode()+b"\0"+path.read_bytes()+b"\0")
    try:
        mujoco_version = importlib.metadata.version("mujoco") if load_model else None
    except importlib.metadata.PackageNotFoundError as error:
        raise AdapterError("MuJoCo is missing; install the [physics] extra or use --no-model-load.") from error
    return {
        "screenVersion": SCREEN_VERSION, "sourceSha256": fingerprint.hexdigest(),
        "simulatorRevision": SIMULATOR_REVISION, "loadModel": load_model,
        "mujocoVersion": mujoco_version,
        "selfPenetrationToleranceMeters": PENETRATION_TOLERANCE,
        "payloadProfileSha256": _digest(load_profile()),
    }


def _selected_protocol(load_model, payload_profile):
    protocol = _protocol(load_model)
    if payload_profile is not None:
        protocol = {**protocol, "payloadProfileSha256": _digest(load_profile(payload_profile))}
    return protocol


def cache_directory(output, load_model=True, *, payload_profile=None):
    """Keep changed screening implementations separate without deleting old reports."""
    return Path(output) / _digest(_selected_protocol(load_model, payload_profile))


def _decision(gate, outcome, code, diagnostics=()):
    return {"gate": gate, "outcome": outcome, "code": code, "diagnostics": list(diagnostics)}


def _load_only(checked, output, payload_profile=None, progress=None):
    import mujoco
    with progress.step("export") if progress else nullcontext():
        xml, joint_sites, _, collision = prepare_xml(checked, output, EvaluationOptions(payload_profile=payload_profile))
    with progress.step("load") if progress else nullcontext():
        try:
            model = mujoco.MjModel.from_xml_path(str(xml.resolve()))
        except ValueError as error:
            raise AdapterError(f"MuJoCo rejected the generated model: {error}") from error
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        if float(data.time) != 0.0:
            raise AdapterError("A load-only gate must not advance simulation time.")
    maximum_closure = 0.0
    for a, b in joint_sites:
        indices = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) for name in (a, b)]
        maximum_closure = max(maximum_closure, float(np.linalg.norm(data.site_xpos[indices[0]]-data.site_xpos[indices[1]])))
    instances = collision["geometryInstances"]
    penetrations = []
    for contact in data.contact[:data.ncon]:
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
                 for index in (contact.geom1, contact.geom2)]
        if all(name in instances for name in names) and -float(contact.dist) > PENETRATION_TOLERANCE:
            penetrations.append({"parts": [instances[name] for name in names],
                                 "depthMeters": -float(contact.dist)})
    return {
        "simulationSeconds": float(data.time), "dynamicTrials": 0,
        "maxClosureErrorMeters": maximum_closure,
        "initialSelfInterference": sorted(penetrations, key=lambda item: -item["depthMeters"]),
        "modelFile": str(xml.resolve()),
    }


def screen_batch(source: Path, output: Path, library: Path, *, max_checks=32, load_model=True,
                 payload_profile=None, observer=None, candidate_ids=None, parser_cache=None):
    if not 1 <= max_checks <= 32:
        raise AdapterError("max-checks must be between 1 and 32.")
    source, output, library = source.resolve(), output.resolve(), library.resolve()
    if output == source or source in output.parents:
        raise AdapterError("Screening output must be outside the immutable generation batch.")
    batch = json.loads((source/"candidates.json").read_text(encoding="utf-8"))
    required = {"schemaVersion", "gateVersion", "mechanismId", "gates", "generated", "rejected", "counts"}
    if (not isinstance(batch, dict) or not required.issubset(batch)
            or batch["schemaVersion"] != 1 or batch["gateVersion"] != "motion-gates-v1"
            or not isinstance(batch["generated"], list) or len(batch["generated"]) > 32
            or not all(isinstance(entry, dict) and {"directory", "assemblyId", "genotypeId"}.issubset(entry)
                       for entry in batch["generated"])
            or not isinstance(batch["gates"], list) or not all(isinstance(gate, dict) for gate in batch["gates"])
            or not isinstance(batch["rejected"], list)):
        raise AdapterError("Expected a bounded F# realization batch.")
    counts = batch["counts"]
    count_fields = {"proposals", "realizationAttempts", "rejected", "duplicates", "exported", "dynamicTrials"}
    if (not isinstance(counts, dict) or set(counts) != count_fields
            or any(type(value) is not int or value < 0 for value in counts.values())
            or counts["proposals"] != 1 or counts["dynamicTrials"] != 0 or counts["realizationAttempts"] > 32
            or counts["exported"] != len(batch["generated"]) or counts["rejected"] != len(batch["rejected"])
            or counts["realizationAttempts"] != counts["exported"]+counts["rejected"]+counts["duplicates"]):
        raise AdapterError("Invalid bounded realization-attempt counts.")
    entries = batch["generated"]
    if candidate_ids is not None:
        if not candidate_ids or set(candidate_ids) - {entry["genotypeId"] for entry in entries}:
            raise AdapterError("Selected screening candidates must belong to the immutable batch.")
        entries = [entry for entry in entries if entry["genotypeId"] in candidate_ids]
    names = [entry.get("directory") for entry in entries]
    if any(not isinstance(name, str) or not name for name in names) or len(names) != len(set(names)):
        raise AdapterError("Realization candidate directories must be nonempty and unique.")
    shared_gates = batch["gates"]
    if any(gate.get("outcome") == "reject" for gate in shared_gates) and entries:
        raise AdapterError("A rejected motion cannot export surviving realizations.")
    output.mkdir(parents=True, exist_ok=True)
    protocol = _selected_protocol(load_model, payload_profile)
    with sqlite3.connect(output/"candidates.sqlite3") as db:
        db.execute("CREATE TABLE IF NOT EXISTS protocol (id INTEGER PRIMARY KEY CHECK(id=1), json TEXT NOT NULL)")
        db.execute("""CREATE TABLE IF NOT EXISTS assemblies (
            screening_id TEXT PRIMARY KEY, assembly_id TEXT NOT NULL, report TEXT NOT NULL)""")
        db.execute("""CREATE TABLE IF NOT EXISTS batches (
            id INTEGER PRIMARY KEY, source TEXT NOT NULL, summary TEXT NOT NULL,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP)""")
        previous = db.execute("SELECT json FROM protocol WHERE id=1").fetchone()
        if previous is not None and json.loads(previous[0]) != protocol:
            raise AdapterError("Screening protocol/source changed; choose a fresh output archive.")
        db.execute("INSERT OR IGNORE INTO protocol VALUES (1, ?)", (json.dumps(protocol),))
        db.commit()
        results, checks, duplicates, deferred, rejected, cached_rejections = [], 0, 0, 0, 0, 0
        for entry in entries:
            if observer:
                observer(entry["genotypeId"])
            candidate = (source/entry["directory"]).resolve()
            if candidate.parent != source:
                raise AdapterError("Candidate paths must be immediate subdirectories of the batch.")
            gates = []
            screening_id = None
            failed_gate = "simulator-reconstruction"
            report = {"candidate": str(candidate), "assemblyId": entry["assemblyId"],
                      "genotypeId": entry["genotypeId"], "mechanismId": batch["mechanismId"],
                      "simulationSeconds": 0, "dynamicTrials": 0}
            progress = ScreeningProgress(output, entry["genotypeId"])
            try:
                progress.start("inputs")
                manifest = load_manifest(candidate/"manifest.json")
                if manifest["grammarVersion"] not in MOTION_GRAMMARS:
                    raise AdapterError("This screening batch requires a motion-realization manifest.")
                if manifest["grammarVersion"] in ("technic-motion-v2", "technic-motion-v3"):
                    report["logicalFootCount"] = logical_foot_count(manifest)
                identity = manifest["motion"]
                model_lines = (candidate/manifest["modelFile"]).read_text(encoding="utf-8").splitlines()
                physical_hash = hashlib.sha256("\n".join(sorted(
                    line.strip() for line in model_lines if line.startswith("1 "))).encode()).hexdigest()
                if (identity["assemblyId"] != physical_hash or identity["assemblyId"] != entry["assemblyId"]
                        or identity["genotypeId"] != entry["genotypeId"]
                        or identity["mechanismId"] != batch["mechanismId"]):
                    raise AdapterError("Batch, genome, mechanism and physical assembly identities disagree.")
                gates = json.loads((candidate/"gates.json").read_text(encoding="utf-8"))
                if ([gate.get("gate") for gate in gates] != GATE_ORDER
                        or any(gate.get("outcome") not in ("pass", "warning") for gate in gates)):
                    raise AdapterError("All five ordered preflight gates must pass or warn before reconstruction.")
                progress.complete()
                with progress.step("dependencies"):
                    dependencies = library_fingerprint(library, [part["part"] for part in manifest["parts"]])
                phase_path = candidate / "motion-phases.json"
                phase_hash = hashlib.sha256(phase_path.read_bytes()).hexdigest() if phase_path.is_file() else None
                screening_id = _digest({"assemblyId": physical_hash, "protocol": protocol, "parts": dependencies,
                                        "prescribedPhasesSha256": phase_hash})
                previous = db.execute("SELECT report FROM assemblies WHERE screening_id=?", (screening_id,)).fetchone()
                if previous is not None:
                    duplicates += 1
                    previous_report = json.loads(previous[0])
                    previous_status = previous_report["status"]
                    cached_rejections += previous_status == "rejected"
                    report.update(status="duplicate", screeningId=screening_id,
                                  previousStatus=previous_status,
                                  gates=gates+[_decision("deduplication", "duplicate", "physical-assembly-already-screened")])
                    if previous_status == "rejected":
                        report["rejectionDetail"] = rejection_detail(previous_report)
                        if "repairHints" in previous_report:
                            report["repairHints"] = previous_report["repairHints"]
                        if "loadOnly" in previous_report:
                            report["loadOnly"] = previous_report["loadOnly"]
                        if "constructionConnectivity" in previous_report:
                            report["constructionConnectivity"] = previous_report["constructionConnectivity"]
                        if "jointPreservation" in previous_report:
                            report["jointPreservation"] = previous_report["jointPreservation"]
                        if "phaseClearance" in previous_report:
                            report["phaseClearance"] = previous_report["phaseClearance"]
                    results.append(report)
                    progress.finish("cached", f"Saved screening outcome: {previous_status}; not rerun.")
                    if observer:
                        observer(entry["genotypeId"], report)
                    continue
                if checks >= max_checks:
                    deferred += 1
                    report.update(status="deferred", reason="reconstruction-attempt-limit",
                                  gates=gates+[_decision("simulator-reconstruction", "not-run", "attempt-limit")])
                    results.append(report)
                    progress.finish("deferred")
                    if observer:
                        observer(entry["genotypeId"], report)
                    continue
                checks += 1
                with progress.step("reconstruction"):
                    checked = (check(candidate/"manifest.json", library) if parser_cache is None else
                               check(candidate/"manifest.json", library, parser_cache=parser_cache))
                with progress.step("collision"):
                    profile = load_profile(payload_profile)
                    prepared = prepare_parts(checked, profile=profile)
                if manifest["grammarVersion"] == "technic-motion-v3":
                    failed_gate = "mass-layout"
                    with progress.step("mass"):
                        scene, parts, _ = prepared
                        scene, tensors = apply_payloads(scene, parts, manifest, profile)
                        report["massLayout"] = analyze_layout(checked, scene, parts, profile, inertia_tensors=tensors)
                        gates.append(report["massLayout"]["gate"])
                    failed_gate = "simulator-reconstruction"
                report["compatibility"] = checked.report
                if load_model:
                    model_dir = output/"models"/screening_id
                    if model_dir.exists() and any(model_dir.iterdir()):
                        raise AdapterError(f"Interrupted model export exists; inspect it and use a fresh archive: {model_dir}")
                    model_dir.mkdir(parents=True, exist_ok=True)
                    report["loadOnly"] = _load_only(checked, model_dir, payload_profile, progress)
                    loaded = report["loadOnly"]
                    progress.start("closure")
                    if loaded["maxClosureErrorMeters"] > 0.002:
                        raise AdapterError("Initial MuJoCo hinge closure exceeds 2 mm.")
                    progress.complete(detail=f"Maximum closure error: {loaded['maxClosureErrorMeters'] * 1000:.6g} mm (limit 2 mm).")
                    progress.start("contacts")
                    if loaded["initialSelfInterference"]:
                        gates.append(_decision("simulator-reconstruction", "reject", "initial-self-interference",
                                               [json.dumps(loaded["initialSelfInterference"])]))
                        report["status"] = "rejected"
                        rejected += 1
                        progress.complete("rejected", rejection_detail(report))
                    else:
                        gates.append(_decision("simulator-reconstruction", "pass", "reconstructed-and-loaded-at-time-zero"))
                        report["status"] = "ready-for-dynamics"
                        progress.complete(detail="No initial contacts above 0.1 mm penetration.")
                        if phase_path.is_file():
                            from .phase_clearance import evaluate_phases
                            failed_gate = "full-phase-clearance"
                            progress.start("phases")
                            model_path = Path(loaded["modelFile"])
                            collision = json.loads((model_path.parent / "collision-model.json").read_text())
                            phase = evaluate_phases(
                                checked, model_path, collision, json.loads(phase_path.read_text()),
                                tolerance=PENETRATION_TOLERANCE)
                            report["phaseClearance"] = phase
                            if phase["status"] != "clear-at-sampled-phases":
                                detail = ("; ".join(
                                    f"{' / '.join(c['parts'])}: {c['depthMeters'] * 1000:.3f} mm at {c['inputAngleRadians']:.6g} rad"
                                    for c in phase["contacts"]) or phase.get("reason", "incomplete-phase-clearance"))
                                gates.append(_decision("full-phase-clearance", "reject",
                                                       "prescribed-motion-self-interference" if phase["status"] == "collision"
                                                       else "incomplete-phase-clearance", [detail]))
                                report["status"] = "rejected"
                                rejected += 1
                                hints = repair_hints(report)
                                if hints is not None:
                                    report["repairHints"] = hints
                                progress.complete("rejected", detail)
                            else:
                                gates.append(_decision("full-phase-clearance", "pass", "clear-at-sampled-prescribed-phases"))
                                progress.complete(detail=f"{phase['sampledPoses']} prescribed poses checked; no dynamics advanced.")
                        elif manifest["grammarVersion"] == "technic-motion-v3":
                            gates.append(_decision("full-phase-clearance", "warning", "legacy-export-without-prescribed-phases",
                                                   ["Re-export to check the complete physical assembly over the geared gait."]))
                else:
                    gates.append(_decision("simulator-reconstruction", "warning", "reconstructed-not-loaded"))
                    report["status"] = "reconstructed-not-loaded"
                if report["status"] != "rejected":
                    gates.append(_decision("dynamic-evaluation", "not-run", "no-dynamic-trial-authorized"))
            except (AdapterError, OSError, json.JSONDecodeError) as error:
                rejected += 1
                gates.append(_decision(failed_gate, "reject",
                                       "invalid-mass-layout" if failed_gate == "mass-layout" else "reconstruction-failed",
                                       [str(error)]))
                report.update(status="rejected", error=str(error))
                if isinstance(error, AdapterError) and error.construction_connectivity is not None:
                    report["constructionConnectivity"] = error.construction_connectivity
                if isinstance(error, AdapterError) and error.joint_preservation is not None:
                    report["jointPreservation"] = error.joint_preservation
                if isinstance(error, AdapterError) and error.repair_hints is not None:
                    report["repairHints"] = error.repair_hints
                hints = repair_hints(report)
                if hints is not None:
                    report["repairHints"] = hints
            finally:
                if progress.document["status"] == "running":
                    progress.finish(report.get("status", "interrupted"),
                                    report.get("error", "" if "status" in report else "Screening interrupted."))
            report.update(gates=gates, screeningId=screening_id)
            if screening_id is not None:
                db.execute("INSERT OR REPLACE INTO assemblies VALUES (?, ?, ?)",
                           (screening_id, report["assemblyId"], json.dumps(report, allow_nan=False)))
                db.commit()
                write_json(output/"reports"/f"{screening_id}.json", report)
            results.append(report)
            if observer:
                observer(entry["genotypeId"], report)
        summary = {
            "schemaVersion": 1, "protocol": protocol, "mechanismId": batch["mechanismId"],
            "motionGates": shared_gates, "realizationRejections": batch["rejected"], "results": results,
            "counts": {**batch["counts"], "reconstructionAttempts": checks,
                       "motionRejections": sum(gate.get("outcome") == "reject" for gate in shared_gates),
                       "screeningRejections": rejected, "cachedRejections": cached_rejections,
                       "screeningDuplicates": duplicates, "deferred": deferred,
                       "ready": sum(r["status"] == "ready-for-dynamics" for r in results), "dynamicTrials": 0},
        }
        cursor = db.execute("INSERT INTO batches(source, summary) VALUES (?, ?)",
                            (str(source), json.dumps(summary, allow_nan=False)))
        db.commit()
        write_json(output/f"screen-{cursor.lastrowid:04d}.json", summary)
        return summary
