"""Replay an opt-in F# connection grammar against an immutable archived cohort."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
from html import escape
import json
from pathlib import Path
import sqlite3
import subprocess

from .bridge import AdapterError, PartParserCache, write_json
from .screening import screen_batch
from .screening_reports import rejections


def _read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _zero_steps(*args, **kwargs):
    raise AdapterError("Dynamics are forbidden in the connection grammar pilot.")


def _ready(report):
    return report["status"] == "ready-for-dynamics" or (
        report["status"] == "duplicate" and report.get("previousStatus") == "ready-for-dynamics"
    )


def _cohort(archive):
    reports = rejections(archive)
    with sqlite3.connect(f"{(archive/'archive.sqlite3').as_uri()}?mode=ro", uri=True) as db:
        rows = db.execute(
            "SELECT id,design,status,operation FROM births WHERE status IN "
            "('screening-rejected','simulation-rejected','evaluated') ORDER BY id"
        ).fetchall()
    result = []
    for birth, design, status, operation in rows:
        source = archive / "designs" / design / "candidate"
        manifest = _read(source / "manifest.json")
        # Retained imports have no screening observation in this cohort.
        if operation in ("import-motion-walker", "user-contribution"):
            continue
        info = reports.get(manifest["motion"]["genotypeId"])
        report = _read(archive / info["report"]) if info else {}
        contacts = report.get("loadOnly", {}).get("initialSelfInterference", [])
        result.append({
            "birth": birth, "source": str(source), "status": status,
            "mechanismId": manifest["motion"]["mechanismId"],
            "expectedInitialPass": status != "screening-rejected",
            "pairs": [item["parts"] for item in contacts],
            "excluded": status == "screening-rejected" and not contacts,
            "exclusionReason": info["detail"] if info and not contacts else None,
        })
    return result


def _generate(case, pairs, output, dotnet, generator, limit):
    output.mkdir(parents=True, exist_ok=False)
    feedback = output / "conflicts.json"
    write_json(feedback, {"pairs": pairs})
    command = [str(dotnet), str(generator), "grammar-walker", "--parent", case["source"],
               "--output", str(output / "candidates"), "--conflicts", str(feedback),
               "--limit", str(limit)]
    with (output / "generator.log").open("w") as log:
        completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=600)
    if completed.returncode not in (0, 1) or not (output / "candidates/candidates.json").is_file():
        raise AdapterError(f"F# grammar failed for birth {case['birth']}; inspect {output/'generator.log'}.")
    summary = _read(output / "candidates/candidates.json")
    if summary["counts"]["dynamicTrials"] != 0:
        raise AdapterError("Grammar exports unexpectedly declare dynamics.")
    source_program = (Path(case["source"]) / "program.json").read_bytes()
    for entry in summary["generated"]:
        candidate = output / "candidates" / entry["directory"]
        if (candidate / "program.json").read_bytes() != source_program:
            raise AdapterError(f"Grammar altered the motion program bytes: {candidate}")
        if _read(candidate / "manifest.json")["motion"]["mechanismId"] != case["mechanismId"]:
            raise AdapterError("Grammar altered the mechanism identity.")
    return summary, command


def _screen(source, entry, output, library, parser_cache=None):
    result = screen_batch(source, output, library, max_checks=1, candidate_ids=[entry["genotypeId"]],
                          parser_cache=parser_cache)
    report = result["results"][0]
    loaded = report.get("loadOnly")
    if result["counts"]["dynamicTrials"] != 0 or loaded and (
        loaded["simulationSeconds"] != 0 or loaded["dynamicTrials"] != 0
    ):
        raise AdapterError("A grammar pilot check advanced dynamics.")
    return {
        "directory": str(source / entry["directory"]), "genotypeId": entry["genotypeId"],
        "assemblyId": entry["assemblyId"], "ready": _ready(report), "report": report,
        "reconstructionAttempts": result["counts"]["reconstructionAttempts"],
    }


def run(campaign, version, output, library, dotnet, generator, limit=8, fresh_baselines=False):
    import mujoco

    if not 1 <= limit <= 32:
        raise AdapterError("Pilot limit must be between one and 32 alternatives per motion context.")
    campaign, output = campaign.resolve(), output.resolve()
    archive = (campaign / "versions" / version).resolve()
    if archive.parent != (campaign / "versions").resolve() or not (archive / "archive.sqlite3").is_file():
        raise AdapterError("Expected an existing campaign version.")
    if output == campaign or campaign in output.parents or output in campaign.parents:
        raise AdapterError("Pilot output must be outside the live campaign.")
    if output.exists():
        raise AdapterError("Pilot output must be fresh; refusing to overwrite evidence.")
    if not library.is_dir() or not generator.is_file() or not dotnet.is_file():
        raise AdapterError("Existing LDraw library, generator DLL and dotnet executable are required.")
    cohort = _cohort(archive)
    if not cohort:
        raise AdapterError("The selected version has no completed screening evidence.")
    output.mkdir(parents=True)
    parser_cache = PartParserCache()
    started = datetime.now(timezone.utc).isoformat()
    saved_steps = {name: getattr(mujoco, name) for name in ("mj_step", "mj_step1", "mj_step2")}
    try:
        for name in saved_steps:
            setattr(mujoco, name, _zero_steps)
        baselines, counterexamples, exclusions = [], [], []
        representatives = {}
        for case in cohort:
            if case["pairs"]:
                counterexamples.append(case)
                representatives.setdefault(case["mechanismId"], case)
                continue
            if case["excluded"]:
                exclusions.append(case)
                continue
            folder = output / f"baseline-{case['birth']:04d}"
            batch, command = _generate(case, [], folder, dotnet, generator, 1)
            if len(batch["generated"]) != 1:
                raise AdapterError(f"The inherited baseline must export exactly once for birth {case['birth']}.")
            source_assembly = _read(Path(case["source"]) / "manifest.json")["motion"]["assemblyId"]
            inherited = batch["generated"][0]
            if inherited["assemblyId"] != source_assembly:
                raise AdapterError("Inherited grammar production changed assembly identity.")
            if fresh_baselines:
                checked = _screen(folder / "candidates", inherited, output / "screening", library, parser_cache)
            else:
                checked = {
                    "directory": str(folder / "candidates" / inherited["directory"]),
                    "assemblyId": source_assembly, "genotypeId": inherited["genotypeId"],
                    "ready": case["expectedInitialPass"], "reconstructionAttempts": 0,
                    "report": {"status": "archived-disposition-not-rescreened", "source": str(archive / "archive.sqlite3"),
                               "birth": case["birth"], "archivedBirthStatus": case["status"],
                               "identityCheck": "same-assembly-id-and-program-bytes"},
                }
            baselines.append({"case": case, "excluded": False, "result": checked, "command": command,
                              "freshlyScreened": fresh_baselines,
                              "matchesArchivedDisposition": checked["ready"] == case["expectedInitialPass"]
                              if fresh_baselines else None})
            mode = "fresh reconstruction" if fresh_baselines else "identity verified; archived verdict"
            print(f"Passing control birth {case['birth']} ({mode}): "
                  f"{'pass' if checked['ready'] else 'reject'}", flush=True)
            write_json(output / "progress.json", {"phase": "baselines", "results": baselines})
        repairs = []
        for case in representatives.values():
            folder = output / f"repair-{case['birth']:04d}"
            batch, command = _generate(case, case["pairs"], folder, dotnet, generator, limit)
            attempts = []
            for entry in batch["generated"]:
                checked = _screen(folder / "candidates", entry, output / "screening", library, parser_cache)
                attempts.append(checked)
                print(f"Grammar birth {case['birth']} {entry['directory']}: "
                      f"{'pass' if checked['ready'] else 'reject'}", flush=True)
                if checked["ready"]:
                    break
            repairs.append({
                "case": case, "command": command, "nativeSearch": batch["search"],
                "exportedAlternatives": len(batch["generated"]), "attempts": attempts,
                "recoveredInitialPass": any(a["ready"] for a in attempts),
            })
            write_json(output / "progress.json", {"phase": "repairs", "results": repairs})
    finally:
        for name, function in saved_steps.items():
            setattr(mujoco, name, function)
    checks = [b["result"] for b in baselines if not b["excluded"]] + [
        attempt for repair in repairs for attempt in repair["attempts"]
    ]
    summary = {
        "schemaVersion": 1, "startedUtc": started, "finishedUtc": datetime.now(timezone.utc).isoformat(),
        "archive": str(archive), "generator": str(generator),
        "generatorSha256": hashlib.sha256(generator.read_bytes()).hexdigest(),
        "coreSha256": hashlib.sha256((generator.parent / "LegoWalkers.Core.dll").read_bytes()).hexdigest(),
        "baselineMode": "fresh-reconstruction" if fresh_baselines else "identity-and-archived-disposition",
        "mode": "connection-link-grammar-zero-step-pilot", "dynamicTrials": 0, "simulationSeconds": 0,
        "baselines": baselines, "counterexamples": counterexamples, "exclusions": exclusions, "repairs": repairs,
        "counts": {
            "passingControlCases": len(baselines), "failedAssemblyCounterexamples": len(counterexamples),
            "excludedNonGeometricCases": len(exclusions),
            "freshBaselineChecks": sum(b.get("freshlyScreened", False) for b in baselines),
            "baselineDispositionMismatches": sum(b["matchesArchivedDisposition"] is False for b in baselines if not b["excluded"]),
            "initialPassingControls": sum(b["case"]["expectedInitialPass"] for b in baselines if not b["excluded"]),
            "passingControlsPreserved": sum(b["case"]["expectedInitialPass"] and b["result"]["ready"]
                                           for b in baselines if not b["excluded"]),
            "failedMotionContexts": len(repairs),
            "recoveredMotionContexts": sum(r["recoveredInitialPass"] for r in repairs),
            "grammarAlternativesScreened": sum(len(r["attempts"]) for r in repairs),
            "reconstructionAttempts": sum(c["reconstructionAttempts"] for c in checks),
        },
        "limitations": [
            "The grammar realizes existing link graphs; it does not yet generate arbitrary new motion topologies.",
            "Search is bounded and heuristic-ranked; an unsuccessful context is not an impossible mechanism.",
            "Static hull checks do not certify detailed physical collision freedom, axial retention or walking.",
            "No grammar promotion, live archive mutation, evaluation or trial authorization occurred.",
        ],
    }
    write_json(output / "pilot.json", summary)
    _write_html(output, summary)
    return summary


def _write_html(output, summary):
    rows = []
    for baseline in summary["baselines"]:
        case = baseline["case"]
        disposition = "Excluded: no archived geometric rejection" if baseline["excluded"] else (
            ("Fresh: " if baseline["freshlyScreened"] else "Identity verified; archived: ")
            + ("initial pass" if baseline["result"]["ready"] else "rejected"))
        rows.append(f"<tr><td>{case['birth']}</td><td>{escape(case['status'])}</td><td>{disposition}</td></tr>")
    repairs = []
    for repair in summary["repairs"]:
        winner = next((a for a in repair["attempts"] if a["ready"]), None)
        evidence = ""
        if winner:
            relative = Path(winner["directory"]).relative_to(output).as_posix()
            evidence = f'<a href="{relative}/grammar.json">derivation</a> / <a href="{relative}/model.ldr">LDraw</a>'
        reasons = Counter(a["report"].get("error") or ",".join(g["code"] for g in a["report"]["gates"]
                          if g["outcome"] == "reject") for a in repair["attempts"] if not a["ready"])
        repairs.append(
            f"<tr><td>{repair['case']['birth']}</td><td>{repair['exportedAlternatives']}</td>"
            f"<td>{len(repair['attempts'])}</td><td>{'Initial pass recovered' if winner else 'Not recovered within bounds'}</td>"
            f"<td>{evidence}<br>{escape(json.dumps(reasons))}</td></tr>"
        )
    text = f"""<!doctype html><html lang="en"><meta charset="utf-8"><title>Connection grammar pilot</title>
<style>body{{font:16px/1.5 system-ui;max-width:1100px;margin:30px auto;padding:16px}}
table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #aaa;padding:8px;text-align:left}}pre{{white-space:pre-wrap}}</style>
<h1>Connection-and-link grammar: zero-step pilot</h1>
<p>Explicit link-layer and revolute-stack productions in F#, followed by independent simulator reconstruction
and time-zero hull checking. Bounds and derivation traces are retained, including unsuccessful searches.</p>
<p>Baseline mode: <strong>{escape(summary['baselineMode'])}</strong>. In identity mode controls retain their
recorded screening verdict; unchanged assembly IDs and exact motion bytes are checked, not freshly reconstructed.
Every attempted new alternative receives normal fresh screening.</p>
<p><strong>No dynamics, trial authorization or live policy promotion.</strong> <a href="pilot.json">Full evidence</a>.</p>
<pre>{escape(json.dumps(summary['counts'], indent=2))}</pre>
<h2>Representative failed motion contexts</h2><table><tr><th>Source birth</th><th>Exports</th>
<th>Checked</th><th>Result</th><th>Evidence / rejected alternatives</th></tr>{''.join(repairs)}</table>
<h2>Passing assembly controls only</h2><p>Failed assemblies are counterexamples supplying constraints,
not constructions the grammar is required to reproduce. Non-geometric failures are excluded.</p>
<table><tr><th>Birth</th><th>Archived status</th><th>Control evidence</th></tr>{''.join(rows)}</table>
<h2>Limits</h2><ul>{''.join('<li>'+escape(item)+'</li>' for item in summary['limitations'])}</ul></html>"""
    (output / "pilot.html").write_text(text, encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--dotnet", type=Path, required=True)
    parser.add_argument("--generator", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=8)
    parser.add_argument("--fresh-baselines", action="store_true")
    args = parser.parse_args(argv)
    summary = run(args.campaign, args.version, args.output, args.library.resolve(),
                  args.dotnet.resolve(), args.generator.resolve(), args.limit, args.fresh_baselines)
    print(json.dumps(summary["counts"], indent=2))


if __name__ == "__main__":
    main()
