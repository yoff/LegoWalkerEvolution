"""Bounded offline packaging realization, exact screening and feedback repair."""

import argparse
import hashlib
from html import escape
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

from legowalkers_sim.bridge import AdapterError, PartParserCache, library_fingerprint, write_json
from legowalkers_sim.payload import load_profile
from legowalkers_sim import screening
from plan_packaging import ROOT, digest, mass_properties


def _generate(arguments, output, dotnet):
    command = [str(dotnet.resolve()), "fsi", str(ROOT / "scripts/realize_packaging.fsx"),
               *arguments, "--output", str(output), "--limit", "2"]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=600)
    if result.returncode:
        raise AdapterError(f"Packaging construction failed:\n{result.stderr}\n{result.stdout}")
    batch = json.loads((output / "candidates.json").read_text())
    if batch["counts"]["dynamicTrials"] != 0:
        raise AdapterError("Packaging construction unexpectedly declared dynamics.")
    return batch


def _proof_generate(arguments, output, dotnet):
    result = subprocess.run(
        [str(dotnet.resolve()), "fsi", str(ROOT / "scripts/proof_packaging.fsx"),
         *arguments, "--output", str(output)],
        cwd=ROOT, capture_output=True, text=True, timeout=600,
    )
    if result.returncode:
        raise AdapterError(f"Construction proof search failed:\n{result.stderr}\n{result.stdout}")
    batch = json.loads((output / "candidates.json").read_text())
    checkpoint = json.loads((output / "search-checkpoint.json").read_text())
    if batch["counts"]["dynamicTrials"] or len(batch["generated"]) > 1:
        raise AdapterError("Proof search must yield at most one load-only oracle obligation.")
    if bool(batch["generated"]) != bool(checkpoint["state"]["pending"]):
        raise AdapterError("Proof search checkpoint and physical witness disagree.")
    state = checkpoint["state"]
    if state["visited"] > state["maximumStates"] or len(state["frontier"]) > state["maximumFrontier"]:
        raise AdapterError("Proof search exceeded its persisted construction bounds.")
    for entry in batch["generated"]:
        candidate = output / entry["directory"]
        proof = json.loads((candidate / "construction-proof.json").read_text())
        if proof != state["pending"] or checkpoint["oracleScope"] != arguments[arguments.index("--oracle-scope")+1]:
            raise AdapterError("Exported proof differs from its pending request/oracle checkpoint.")
        genome = json.loads((candidate / "realization.json").read_text())
        physical = [p["id"] for b in genome["bodies"] for p in b["parts"]]
        physical += [p["id"] for p in genome["mounts"]] + [c["placement"]["id"] for c in genome["connections"]]
        produced = [part for step in proof["steps"] for part in step["parts"]]
        if len(produced) != len(set(produced)) or set(produced) != set(physical):
            raise AdapterError("Construction proof does not uniquely account for the physical stock.")
        assigned = set()
        goals = {goal["id"]: goal for goal in proof["goals"]}
        for step in proof["steps"]:
            if step["goal"] in assigned or not set(step["dependencies"]).issubset(assigned):
                raise AdapterError("Construction proof violates its dependency trail.")
            if not set(goals[step["goal"]]["requires"]).issubset(step["dependencies"]):
                raise AdapterError("Construction proof omitted a prerequisite dependency.")
            assigned.add(step["goal"])
        if any(not goal["external"] for goal in proof["goals"] if goal["id"] not in assigned):
            raise AdapterError("Exported construction has unresolved local obligations.")
        manifest = json.loads((candidate / "manifest.json").read_text())
        if (manifest["motion"]["assemblyId"] != entry["assemblyId"] or manifest["motion"]["genotypeId"] != entry["genotypeId"]
                or manifest["ldrawSha256"] != digest(candidate / "model.ldr")):
            raise AdapterError("Construction proof's physical candidate identity changed.")
    return batch


def _oracle_scope(inputs, library):
    source = json.loads((Path(inputs["parent"]) / "realization.json").read_text())
    parts = [p["part"] for b in source["bodies"] for p in b["parts"]]
    parts += [p["part"] for p in source["mounts"]]
    parts += [c["placement"]["part"] for c in source["connections"]]
    parts += [p["part"] for drive in inputs["drives"] for p in drive["placements"]]
    parts += ["32523.dat", "32316.dat", "32524.dat", "32525.dat", "6536.dat", "4459.dat", "6558.dat",
              "3705.dat", "44294.dat", "3707.dat", "60485.dat", "3708.dat", "32002.dat", "3673.dat", "32556.dat"]
    files = [ROOT / "src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll",
             *(ROOT / "scripts" / name for name in (
                 "proof_packaging.fsx", "PackagingRealizationSupport.fsx", "construct_packaging.py",
                 "packaging_balance.fsx", "PackagingSupport.fsx", "plan_packaging.py"))]
    value = {"protocol": screening._selected_protocol(True, None),
             "librarySha256": library_fingerprint(library, parts),
             "builderFiles": {str(path.relative_to(ROOT)): digest(path) for path in files}}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _proof_summary(folder):
    state = json.loads((folder / "search-checkpoint.json").read_text())["state"]
    return {
        "checkpoint": str(folder / "search-checkpoint.json"), "status": state["status"],
        "visited": state["visited"], "pruned": state["pruned"], "discarded": state["discarded"],
        "nogoods": len(state["nogoods"]), "internalRejections": state["rejections"],
    }


def _screen(batch_folder, entry, output, library, cache, dotnet):
    captured = []
    check = screening.check
    proof_path = batch_folder / entry.get("directory", "") / "construction-proof.json"
    proof = json.loads(proof_path.read_text()) if proof_path.is_file() else None

    def capture(*args, **kwargs):
        if proof is not None:
            kwargs["construction_proof"] = proof
        checked = check(*args, **kwargs)
        captured.append(checked)
        return checked

    with patch.object(screening, "check", side_effect=capture):
        result = screening.screen_batch(
            batch_folder, output, library, max_checks=1, candidate_ids=[entry["genotypeId"]],
            parser_cache=cache,
        )
    report = result["results"][0]
    if captured and proof is not None:
        report["constructionConnectivity"] = captured[0].report["constructionConnectivity"]
        report["jointPreservation"] = captured[0].report["jointPreservation"]
    spent = result["counts"]["reconstructionAttempts"]
    if result["counts"]["dynamicTrials"] or report["simulationSeconds"]:
        raise AdapterError("Offline construction check advanced dynamics.")
    if captured:
        checked = captured[0]
        records = mass_properties(checked, load_profile())
        candidate = batch_folder / entry["directory"]
        write_json(output / "mass-properties.json", {
            "schemaVersion": 1, "parentManifestSha256": digest(candidate / "manifest.json"),
            "programSha256": digest(candidate / "program.json"),
            "realizationSha256": digest(candidate / "realization.json"),
            "dependencySha256": checked.report["partsDependencySha256"],
            "payloadProfileSha256": result["protocol"]["payloadProfileSha256"], "parts": records,
        })
        mass = {"status": "reconstructed-part-ledger", "parts": len(records),
                "totalMassKg": sum(part["massKg"] for part in records),
                "path": str(output / "mass-properties.json")}
        result = subprocess.run([
            str(dotnet.resolve()), "fsi", str(ROOT / "scripts/packaging_balance.fsx"),
            "--parent", str(candidate), "--mass-properties", str(output / "mass-properties.json"),
            "--output", str(output / "phase-balance.json"),
        ], cwd=ROOT, capture_output=True, text=True, timeout=600)
        if result.returncode:
            raise AdapterError(f"Constructed mass balance failed:\n{result.stderr}\n{result.stdout}")
        balance = json.loads((output / "phase-balance.json").read_text())
        mass["phaseBalance"] = {key: balance[key] for key in ("meanHeightLdu", "worstMarginLdu", "outsideSupportSamples")}
    else:
        mass = {"status": "unavailable-reconstruction-rejected"}
    from legowalkers_sim.screening_reports import repair_hints
    hints = repair_hints(report)
    if hints is not None:
        report["repairHints"] = hints
    return report, spent, mass


def run(combined, output, library, dotnet, max_checks=4, *, proof_search=False, max_states=512, max_frontier=64,
        motor_depth_tolerance=0, motor_reduction=1):
    if not 1 <= max_checks <= 32:
        raise AdapterError("max-checks must be between one and 32.")
    if not 1 <= max_states <= 16384 or not 1 <= max_frontier <= 1024:
        raise AdapterError("Proof bounds require 1..16384 states and 1..1024 frontier nodes.")
    if not 0 <= motor_depth_tolerance <= 120 or (motor_depth_tolerance and not proof_search):
        raise AdapterError("Motor depth tolerance requires proof search and must be within 0..120 LDU.")
    if motor_reduction not in (1, 5) or (motor_reduction != 1 and not proof_search):
        raise AdapterError("Motor reduction requires proof search and must be 1 or 5.")
    combined, output = combined.resolve(), output.resolve()
    if output == combined or combined in output.parents or output in combined.parents:
        raise AdapterError("Construction output must be separate from immutable inputs.")
    if output.exists():
        raise AdapterError("Construction output must be fresh.")
    input_path, plan_path = combined / "combined-input.json", combined / "plans/combined-plan.json"
    inputs = json.loads(input_path.read_text())
    plans = json.loads(plan_path.read_text())
    parent = Path(inputs["parent"]).resolve()
    if output == parent or parent in output.parents or output in parent.parents:
        raise AdapterError("Construction output must be separate from the parent.")
    if digest(input_path) != plans["inputSha256"]:
        raise AdapterError("Combined input hash differs from the selected plans.")
    selected = []
    for prefix in ("root-05-interior-rearward-output", "root-04-interior-rearward-output"):
        candidates = [plan for plan in plans["plans"] if plan["driveId"] == prefix]
        if not candidates:
            raise AdapterError(f"Missing offline layout: {prefix}")
        selected.append(min(candidates, key=lambda p: (
            p["metrics"]["clearanceScore"], -p["metrics"]["worstMarginLdu"], p["metrics"]["meanHeightLdu"], p["id"])))
    immutable = {path: digest(path) for path in [input_path, plan_path, parent / "program.json",
                                                parent / "realization.json", parent / "manifest.json"]}
    if proof_search:
        for drive in inputs["drives"]:
            for key in ("driveProgramPath", "ldrawPath"):
                path = Path(drive[key])
                immutable[path] = digest(path)
    oracle_scope = _oracle_scope(inputs, library) if proof_search else None
    output.mkdir(parents=True)
    cache, checks, outcomes, pending = PartParserCache(), 0, [], []
    for plan in selected:
        folder = output / plan["id"]
        batch_folder = folder / "initial"
        arguments = ["--input", str(input_path), "--plans", str(plan_path), "--plan", plan["id"]]
        if proof_search:
            arguments += ["--oracle-scope", oracle_scope, "--max-states", str(max_states), "--max-frontier", str(max_frontier)]
            if motor_depth_tolerance:
                arguments += ["--motor-depth-tolerance", str(motor_depth_tolerance)]
            if motor_reduction != 1:
                arguments += ["--motor-reduction", str(motor_reduction)]
            batch = _proof_generate(arguments, batch_folder, dotnet)
        else:
            batch = _generate(arguments, batch_folder, dotnet)
        outcome = {"planId": plan["id"], "initialRejections": batch["rejected"], "evaluations": [],
                   "repairRejections": [], "unverifiedCandidates": []}
        if proof_search:
            outcome["proofSearch"] = _proof_summary(batch_folder)
        outcomes.append(outcome)
        pending.extend((0 if index == 0 else 2, batch_folder, entry, 0, outcome)
                       for index, entry in enumerate(batch["generated"]))
    while pending:
        pending.sort(key=lambda item: item[0])
        _, source, entry, generation, outcome = pending.pop(0)
        folder = output / outcome["planId"]
        candidate = source / entry["directory"]
        if checks >= max_checks:
            outcome["unverifiedCandidates"].append({"candidate": str(candidate), "reason": "shared-check-budget-exhausted"})
            continue
        check_folder = folder / f"check-{len(outcome['evaluations'])+1:02}"
        report, spent, mass = _screen(source, entry, check_folder, library, cache, dotnet)
        checks += spent
        outcome["evaluations"].append({"candidate": str(candidate), "generation": generation, "report": report, "mass": mass})
        if proof_search:
            checkpoint_path = source / "search-checkpoint.json"
            checkpoint = json.loads(checkpoint_path.read_text())
            evidence_path = check_folder / "proof-oracle-evidence.json"
            write_json(evidence_path, report)
            verdict_path = check_folder / "proof-oracle-verdict.json"
            write_json(verdict_path, {
                "scope": checkpoint["state"]["scope"], "assemblyId": entry["assemblyId"],
                "genotypeId": entry["genotypeId"], "evidencePath": str(evidence_path),
                "evidenceSha256": digest(evidence_path),
            })
            continuation = folder / f"continuation-{len(outcome['evaluations']):02}"
            batch = _proof_generate(["--resume", str(checkpoint_path), "--verdict", str(verdict_path),
                                     "--oracle-scope", oracle_scope], continuation, dotnet)
            outcome["proofSearch"] = _proof_summary(continuation)
            pending.extend((1, continuation, child, generation+1, outcome) for child in batch["generated"])
        elif report["status"] == "rejected" and "repairHints" in report and generation == 0:
            feedback_path = check_folder / "feedback.json"
            write_json(feedback_path, report["repairHints"])
            repair_folder = folder / f"repair-{len(outcome['evaluations']):02}"
            repairs = _generate(["--repair-parent", str(candidate), "--feedback", str(feedback_path)],
                                repair_folder, dotnet)
            outcome["repairRejections"].extend(repairs["rejected"])
            pending.extend((1 if index == 0 else 3, repair_folder, repair, 1, outcome)
                           for index, repair in enumerate(repairs["generated"]))
    for path, expected in immutable.items():
        if digest(path) != expected:
            raise AdapterError(f"Immutable construction input changed: {path}")
    if proof_search and _oracle_scope(inputs, library) != oracle_scope:
        raise AdapterError("Construction oracle, parts library or builder scope changed during the experiment.")
    evaluations = [entry for outcome in outcomes for entry in outcome["evaluations"]]
    ready = sum(entry["report"]["status"] == "ready-for-dynamics" for entry in evaluations)
    result = {
        "schemaVersion": 1, "version": "offline-construction-proof-v1" if proof_search else "offline-packaging-construction-v1",
        "status": "load-only-ready" if ready else "no-qualified-realization-within-bounds",
        "inputSha256": immutable[input_path], "plansSha256": immutable[plan_path],
        "coreAssemblySha256": digest(ROOT / "src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"),
        "maxChecks": max_checks, "reconstructionAttempts": checks, "ready": ready,
        "physicsTrials": 0, "liveContributions": 0, "outcomes": outcomes,
        "proofSearch": proof_search, "oracleScope": oracle_scope,
        "motorDepthToleranceLdu": motor_depth_tolerance,
        "motorReduction": motor_reduction,
        "limitations": [
            "A bounded failure is not proof that the abstract plan is impossible.",
            "Realization retains the source rails, bridges and structural grounded-link anchors; new mounts and bearing brackets are constructed stock parts.",
            "Declared motion/target constraints are not bypassed by repairs; exact reconstruction, mass and collision gates remain authoritative.",
            ("Scoped exact-witness nogoods guide resumable dependency backtracking; frontier/state limits are not impossibility proofs."
             if proof_search else "Only one feedback generation is explored per initial construction; all checks share one fixed budget."),
            "The first construction from each selected layout precedes feedback checks, so one layout cannot monopolize the comparison budget.",
            "No live pipeline activation, contribution or dynamics authorization has occurred.",
        ],
    }
    write_json(output / "construction-experiment.json", result)
    rows = []
    for outcome in outcomes:
        rows.append(f"<h2>{escape(outcome['planId'])}</h2>")
        for rejected in outcome["initialRejections"] + outcome["repairRejections"]:
            rows.append(f"<pre>{escape(json.dumps(rejected, indent=2))}</pre>")
        for evaluation in outcome["evaluations"]:
            report = evaluation["report"]
            relative = Path(evaluation["candidate"]).relative_to(output).as_posix()
            rows.append(f"<p><strong>{escape(report['status'])}</strong>: "
                        f"{escape(report.get('error', 'Passed load-only gates; dynamics untested.'))}</p>"
                        f'<p><a href="{relative}/model.ldr">Complete proposed LDraw assembly</a> | '
                        f'<a href="{relative}/packaging-construction.json">Construction derivation</a></p>'
                        f"<pre>{escape(json.dumps(evaluation['mass'], indent=2))}</pre>")
            if proof_search:
                rows.append(f'<p><a href="{relative}/construction-proof.json">Obligations and production witnesses</a></p>')
        if outcome["unverifiedCandidates"]:
            rows.append(f"<pre>{escape(json.dumps(outcome['unverifiedCandidates'], indent=2))}</pre>")
        if "proofSearch" in outcome:
            checkpoint = Path(outcome["proofSearch"]["checkpoint"]).relative_to(output).as_posix()
            rows.append(f'<p><a href="{checkpoint}">Resumable search checkpoint and scoped nogoods</a></p>')
            rows.append(f"<pre>{escape(json.dumps(outcome['proofSearch'], indent=2))}</pre>")
    html = ("<!doctype html><html lang=en><meta charset=utf-8><title>Packaging construction gate experiment</title>"
            "<style>body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:1rem}pre{white-space:pre-wrap}"
            ".warning{background:#fff0cb;padding:1rem}</style><h1>Packaging realization and feedback repair</h1>"
            f"<p class=warning>{escape(result['status'])}. {checks}/{max_checks} exact reconstruction checks; "
            f"{ready} load-only-ready assemblies. Zero physics trials. Nothing admitted to the live experiment.</p>"
            + "".join(rows) + '<p><a href="construction-experiment.json">Full gate and repair evidence</a></p></html>')
    (output / "index.html").write_text(html)
    print(json.dumps({key: result[key] for key in ("status", "reconstructionAttempts", "ready", "physicsTrials")}, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--combined", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--library", type=Path, default=ROOT / ".tools/ldraw")
    parser.add_argument("--dotnet", type=Path, default=ROOT / ".tools/dotnet/dotnet")
    parser.add_argument("--max-checks", type=int, default=4)
    parser.add_argument("--proof-search", action="store_true", help="Use the shared resumable construction theory; remains offline.")
    parser.add_argument("--max-states", type=int, default=512)
    parser.add_argument("--max-frontier", type=int, default=64)
    parser.add_argument("--motor-depth-tolerance", type=float, default=0,
                        help="Explicit new-design allowance for motor axial relocation, in LDU (0..120); existing layouts remain immutable.")
    parser.add_argument("--motor-reduction", type=int, choices=(1, 5), default=1,
                        help="Explicit new scoped 8T-to-40T root reduction; rebuild supports beyond the walking-link planes.")
    args = parser.parse_args()
    try:
        run(args.combined, args.output, args.library, args.dotnet, args.max_checks,
            proof_search=args.proof_search, max_states=args.max_states, max_frontier=args.max_frontier,
            motor_depth_tolerance=args.motor_depth_tolerance, motor_reduction=args.motor_reduction)
    except (AdapterError, OSError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
