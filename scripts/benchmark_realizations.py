"""Compare inherited assemblies with bounded constructive alternatives, without stepping physics."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from legowalkers_sim.bridge import AdapterError, load_manifest, write_json
from legowalkers_sim.screening import screen_batch
from legowalkers_sim.screening_reports import repair_hints

ROOT = Path(__file__).resolve().parents[1]


def _assert_load_only(screened, allowance):
    if screened["counts"]["dynamicTrials"] != 0:
        raise AdapterError("A load-only benchmark must not spend dynamics trials.")
    if any(report.get("dynamicTrials") != 0 or report.get("simulationSeconds") != 0
           for report in screened["results"]):
        raise AdapterError("A load-only benchmark must not advance simulation time.")
    checks = screened["counts"].get("reconstructionAttempts")
    if type(checks) is not int or not 0 <= checks <= allowance:
        raise AdapterError("Load-only screening exceeded its reconstruction allowance.")


def benchmark(candidates, output, library, dotnet, generator, limit, payload_profile=None, repair_rounds=0,
              builder_policy=None, max_checks=None):
    if not candidates:
        raise AdapterError("At least one fixed benchmark candidate is required.")
    if not 1 <= limit <= 32:
        raise AdapterError("Realization benchmark limit must be between one and 32.")
    if type(repair_rounds) is not int or not 0 <= repair_rounds <= 2:
        raise AdapterError("Realization benchmark repair rounds must be between zero and two.")
    if max_checks is None:
        max_checks = min(32, limit*(1+repair_rounds))
    if type(max_checks) is not int or not 1 <= max_checks <= 32:
        raise AdapterError("Per-case reconstruction budget must be between one and 32.")
    if any(output.resolve() == candidate.resolve() or candidate.resolve() in output.resolve().parents
           for candidate in candidates):
        raise AdapterError("Benchmark output must stay outside its immutable input candidates.")
    if output.exists() and any(output.iterdir()):
        raise AdapterError("Realization benchmark output must be empty.")
    output.mkdir(parents=True, exist_ok=True)
    policy_args = []
    if builder_policy is not None:
        policy = json.loads(builder_policy.read_text())
        snapshot = output / "builder-policy.json"
        write_json(snapshot, policy)
        policy_args = ["--builder-policy", str(snapshot.resolve())]
    cases = []
    for index, candidate in enumerate(candidates, 1):
        candidate = candidate.resolve()
        original = load_manifest(candidate / "manifest.json")
        root = output / f"{index:04d}"
        started = time.perf_counter()
        command = [str(dotnet), str(generator), "construct-walker", "--parent", str(candidate),
                   "--output", str(root / "generated"), *policy_args, "--limit", str(limit)]
        generated = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        if generated.returncode:
            raise AdapterError(f"Construction failed for {candidate}: {generated.stderr.strip() or generated.stdout.strip()}")
        summary = json.loads((root / "generated/candidates.json").read_text())
        if summary["mechanismId"] != original["motion"]["mechanismId"]:
            raise AdapterError("Construction changed the benchmark's motion description.")
        allowance = min(limit, max_checks)
        screened = screen_batch(root / "generated", root / "screening", library, max_checks=allowance,
                                payload_profile=payload_profile)
        _assert_load_only(screened, allowance)
        reports = list(screened["results"])
        generation_counts, screening_counts = [summary["counts"]], [screened["counts"]]
        history = []
        stop_reason = "repair-round-limit" if repair_rounds else "repairs-disabled"
        for number in range(1, repair_rounds+1):
            if any(report.get("previousStatus", report["status"]) == "ready-for-dynamics" for report in reports):
                break
            remaining = max_checks-sum(counts["reconstructionAttempts"] for counts in screening_counts)
            if remaining == 0:
                stop_reason = "check-budget"
                break
            failed = next((report for report in reversed(screened["results"]) if repair_hints(report)), None)
            if failed is None:
                stop_reason = "no-supported-feedback"
                break
            feedback = root / f"feedback-{number:03d}.json"
            write_json(feedback, repair_hints(failed))
            batch = root / f"repair-{number:03d}"
            command = [str(dotnet), str(generator), "construct-walker", "--parent", failed["candidate"],
                       "--output", str(batch), "--feedback", str(feedback), *policy_args, "--limit", str(limit)]
            repaired = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            if repaired.returncode not in (0, 1) or not (batch / "candidates.json").exists():
                raise AdapterError(f"Construction repair failed: {repaired.stderr.strip() or repaired.stdout.strip()}")
            attempt = json.loads((batch / "candidates.json").read_text())
            if (attempt["mechanismId"] != original["motion"]["mechanismId"]
                    or attempt["parentGenotypeId"] != failed["genotypeId"]
                    or attempt.get("builderPolicyId") != summary.get("builderPolicyId")):
                raise AdapterError("Construction repair changed its motion or parent identity.")
            generation_counts.append(attempt["counts"])
            history.append({"parentGenotypeId": failed["genotypeId"],
                            "feedback": str(feedback.relative_to(output)), "generation": attempt["counts"]})
            if not attempt["generated"]:
                stop_reason = "no-compatible-repair"
                break
            allowance = min(limit, remaining)
            screened = screen_batch(batch, root / f"screening-repair-{number:03d}", library,
                                    max_checks=allowance, payload_profile=payload_profile)
            _assert_load_only(screened, allowance)
            screening_counts.append(screened["counts"])
            reports.extend(screened["results"])
        ready = [report for report in reports
                 if report.get("previousStatus", report["status"]) == "ready-for-dynamics"]
        if sum(counts["reconstructionAttempts"] for counts in screening_counts) >= max_checks:
            stop_reason = "check-budget"
        baseline = next((report for report in reports if report["genotypeId"] == original["motion"]["genotypeId"]), None)
        if baseline is None:
            raise AdapterError("Construction benchmark omitted its exact inherited baseline.")
        case = {
            "candidate": str(candidate), "mechanismId": summary["mechanismId"],
            "builderPolicyId": summary.get("builderPolicyId"),
            "baselineReady": baseline.get("previousStatus", baseline["status"]) == "ready-for-dynamics",
            "anyReady": bool(ready), "passingRealizations": len(ready),
            "generation": {key: sum(counts[key] for counts in generation_counts) for key in summary["counts"]},
            "screening": {key: sum(counts[key] for counts in screening_counts) for key in screening_counts[0]},
            "repairHistory": history,
            "termination": "ready" if ready else stop_reason,
            "elapsedSeconds": time.perf_counter() - started,
            "report": str((root / "screening").relative_to(output)),
        }
        cases.append(case)
        write_json(root / "case.json", case)
        print(f"Case {index}: baseline={case['baselineReady']}; any ready={case['anyReady']}; "
              f"{case['passingRealizations']}/{len(reports)} passed; {case['elapsedSeconds']:.1f}s", flush=True)
    result = {
        "schemaVersion": 2, "constructionVersion": summary["constructionVersion"],
        "builderPolicyId": summary.get("builderPolicyId"),
        "limitPerBatch": limit, "maxChecksPerCase": max_checks, "maxRepairRounds": repair_rounds, "cases": cases,
        "baselinePassingCases": sum(case["baselineReady"] for case in cases),
        "casesWithPassingRealization": sum(case["anyReady"] for case in cases),
        "recoveredCases": sum(case["anyReady"] and not case["baselineReady"] for case in cases),
        "dynamicTrials": 0, "simulationSeconds": 0,
    }
    write_json(output / "benchmark.json", result)
    return result


def compare_policies(training, heldout, policies, output, library, dotnet, generator,
                     max_checks=3, repair_rounds=2, payload_profile=None):
    if not training or not heldout or not 1 <= len(policies) <= 7:
        raise AdapterError("Policy comparison needs training and held-out cases, plus one to seven alternative policies.")
    if len(training) > 32 or len(heldout) > 32:
        raise AdapterError("Each policy-comparison cohort is limited to 32 fixed cases.")
    if type(max_checks) is not int or not 1 <= max_checks <= 32:
        raise AdapterError("Per-case reconstruction budget must be between one and 32.")
    if type(repair_rounds) is not int or not 0 <= repair_rounds <= 2:
        raise AdapterError("Realization benchmark repair rounds must be between zero and two.")
    cohorts = []
    for candidates in (training, heldout):
        manifests = [load_manifest(candidate / "manifest.json") for candidate in candidates]
        genotypes = [manifest["motion"]["genotypeId"] for manifest in manifests]
        if len(set(genotypes)) != len(genotypes):
            raise AdapterError("A policy-comparison cohort must not repeat a genotype.")
        cohorts.append(manifests)
    for key in ("genotypeId", "mechanismId", "assemblyId"):
        if ({manifest["motion"][key] for manifest in cohorts[0]}
                & {manifest["motion"][key] for manifest in cohorts[1]}):
            raise AdapterError(f"Training and held-out cases overlap by {key}; split complete motions, not sibling realizations.")
    if output.exists() and any(output.iterdir()):
        raise AdapterError("Policy comparison output must be empty.")
    if any(output.resolve() == case.resolve() or case.resolve() in output.resolve().parents
           for case in [*training, *heldout]):
        raise AdapterError("Policy comparison output must stay outside its immutable input candidates.")
    descriptors = []
    for policy in [None, *policies]:
        command = [str(dotnet), str(generator), "builder-catalogue"]
        if policy is not None:
            command += ["--builder-policy", str(policy.resolve())]
        response = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        if response.returncode:
            raise AdapterError(f"Builder policy validation failed: {response.stderr.strip()}")
        descriptor = json.loads(response.stdout)
        if (not isinstance(descriptor, dict) or not isinstance(descriptor.get("policy"), dict)
                or not isinstance(descriptor.get("catalogueVersion"), str)
                or not isinstance(descriptor.get("policyId"), str) or len(descriptor["policyId"]) != 64):
            raise AdapterError("Expected a versioned F# builder-policy descriptor.")
        descriptors.append(descriptor)
    if len({entry["policyId"] for entry in descriptors}) != len(descriptors):
        raise AdapterError("Policy comparison must not repeat the same normalized policy.")
    default_id = descriptors[0]["policyId"]
    catalogue = descriptors[0]["catalogueVersion"]
    if any(entry["catalogueVersion"] != catalogue for entry in descriptors):
        raise AdapterError("Policies must use the same construction catalogue.")
    output.mkdir(parents=True, exist_ok=True)
    binaries = {path: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (generator, generator.with_name("LegoWalkers.Core.dll"))}
    write_json(output / "comparison-input.json", {
        "training": [str(path.resolve()) for path in training],
        "heldout": [str(path.resolve()) for path in heldout],
        "policies": descriptors, "maxChecksPerCase": max_checks, "maxRepairRounds": repair_rounds,
        "generatorSha256": binaries[generator],
        "coreSha256": binaries[generator.with_name("LegoWalkers.Core.dll")],
        "maximumReconstructionChecks": max_checks*(len(training)*len(descriptors)+len(heldout)),
    })
    results = []
    for descriptor in sorted(descriptors, key=lambda entry: entry["policyId"]):
        root = output / descriptor["policyId"]
        policy_file = root / "policy.json"
        write_json(policy_file, descriptor["policy"])
        measured = benchmark(training, root / "training", library, dotnet, generator, 1,
                             payload_profile, repair_rounds, policy_file, max_checks)
        if measured["builderPolicyId"] != descriptor["policyId"]:
            raise AdapterError("Measured builder policy differs from the selected policy.")
        results.append({"policyId": descriptor["policyId"],
                        "ready": measured["casesWithPassingRealization"],
                        "checks": sum(case["screening"]["reconstructionAttempts"] for case in measured["cases"]),
                        "report": str((root / "training/benchmark.json").relative_to(output))})
    # Held-out results never participate in selection; ties retain the current default.
    selected = min(results, key=lambda row: (-row["ready"], row["checks"],
                                             row["policyId"] != default_id, row["policyId"]))
    selected_file = output / selected["policyId"] / "policy.json"
    verification = benchmark(heldout, output / "heldout", library, dotnet, generator, 1,
                             payload_profile, repair_rounds, selected_file, max_checks)
    if verification["builderPolicyId"] != selected["policyId"]:
        raise AdapterError("Held-out verification used a different builder policy.")
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in binaries.items()):
        raise AdapterError("Generator binaries changed during the policy comparison; results cannot be promoted.")
    result = {
        "schemaVersion": 1, "catalogueVersion": catalogue,
        "defaultPolicyId": default_id, "selectedPolicyId": selected["policyId"],
        "selection": "Training readiness descending, reconstruction checks ascending; ties retain default then policy ID.",
        "training": results, "heldout": {
            "ready": verification["casesWithPassingRealization"], "cases": len(heldout),
            "checks": sum(case["screening"]["reconstructionAttempts"] for case in verification["cases"]),
            "report": "heldout/benchmark.json"},
        "maxChecksPerCase": max_checks, "maxRepairRounds": repair_rounds,
        "reconstructionChecks": sum(row["checks"] for row in results)
                                + sum(case["screening"]["reconstructionAttempts"] for case in verification["cases"]),
        "splitLimitations": "Disjoint recorded motions and assemblies, not proof of topology/family-independent generalization.",
        "promotion": "retain-default" if selected["policyId"] == default_id else "manual-review-required",
        "dynamicTrials": 0, "simulationSeconds": 0,
    }
    write_json(output / "policy-comparison.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, action="append")
    parser.add_argument("--training-candidate", type=Path, action="append")
    parser.add_argument("--heldout-candidate", type=Path, action="append")
    parser.add_argument("--compare-policy", type=Path, action="append")
    parser.add_argument("--builder-policy", type=Path)
    parser.add_argument("--max-checks", type=int)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ldraw-library", type=Path, default=ROOT / ".tools/ldraw")
    parser.add_argument("--dotnet", type=Path, default=ROOT / ".tools/dotnet/dotnet")
    parser.add_argument("--generator", type=Path,
                        default=ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--payload-profile", type=Path)
    parser.add_argument("--repair-rounds", type=int)
    args = parser.parse_args(argv)
    try:
        if args.compare_policy:
            if args.candidate or args.builder_policy or args.limit not in (None, 1):
                raise AdapterError("Policy comparison uses explicit training/held-out cohorts and one candidate per batch.")
            result = compare_policies(
                args.training_candidate, args.heldout_candidate, args.compare_policy, args.output.resolve(),
                args.ldraw_library.resolve(), args.dotnet.resolve(), args.generator.resolve(),
                args.max_checks if args.max_checks is not None else 3,
                args.repair_rounds if args.repair_rounds is not None else 2, args.payload_profile)
        else:
            if args.training_candidate or args.heldout_candidate:
                raise AdapterError("Training/held-out cohorts require --compare-policy.")
            result = benchmark(args.candidate, args.output.resolve(), args.ldraw_library.resolve(),
                               args.dotnet.resolve(), args.generator.resolve(),
                               args.limit if args.limit is not None else 4, args.payload_profile,
                               args.repair_rounds if args.repair_rounds is not None else 0,
                               args.builder_policy, args.max_checks)
        print(json.dumps({key: value for key, value in result.items() if key != "cases"}, indent=2))
        return 0
    except (AdapterError, OSError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
