"""Reconstruct native parallelogram constructions and load at time zero; never step."""

import argparse
import json
from pathlib import Path

from check_photo_leg_fixture import check_engagement
from legowalkers_sim.bridge import AdapterError, check, write_json
from legowalkers_sim.screening import _load_only


def verify(source, output, library):
    source, output = source.resolve(), output.resolve()
    if output == source or source in output.parents:
        raise AdapterError("Keep check evidence outside the immutable benchmark.")
    if output.exists() and any(output.iterdir()):
        raise AdapterError(f"Check output must be empty: {output}")
    benchmark = json.loads((source / "benchmark.json").read_text())
    if (benchmark["classification"] != "native-construction-benchmark-not-campaign-founder"
            or benchmark["dynamicTrials"] != 0 or benchmark["liveMutationFamiliesChanged"]):
        raise AdapterError("Expected an isolated native construction benchmark.")
    cases = benchmark["cases"]
    if (len(cases) != 2 or {case["orientation"] for case in cases} != {"outward", "inward"}
            or any(len(case["constructions"]) != 2
                   or {entry["side"] for entry in case["constructions"]} != {-1, 1} for case in cases)):
        raise AdapterError("Expected exactly two L orientations and two depth sides each.")
    results = []
    for case in benchmark["cases"]:
        for entry in case["constructions"]:
            directory = (source / entry["directory"]).resolve()
            if source not in directory.parents:
                raise AdapterError("Construction path leaves the benchmark.")
            report = {"directory": entry["directory"], "assemblyId": entry["assemblyId"]}
            try:
                checked = check(directory / "manifest.json", library)
                if checked.manifest["motion"]["assemblyId"] != entry["assemblyId"]:
                    raise AdapterError("Benchmark and manifest identities disagree.")
                counts = {key: checked.report[key] for key in ("bodies", "joints", "motors", "gearMeshes")}
                if counts != {"bodies": 8, "joints": 10, "motors": 1, "gearMeshes": 0}:
                    raise AdapterError(f"Unexpected reconstructed graph: {counts}")
                report["reconstruction"] = checked.report
                report["connectorEngagement"] = check_engagement(checked)
                loaded = _load_only(checked, output / entry["directory"] / "load-only")
                report["loadOnly"] = loaded
                if loaded["simulationSeconds"] != 0 or loaded["dynamicTrials"] != 0:
                    raise AdapterError("Benchmark checking must never advance time.")
                report["fixtureClosureComparison"] = {
                    "limitMeters": 1e-6,
                    "withinLimit": loaded["maxClosureErrorMeters"] <= 1e-6,
                }
                report["status"] = "initial-interference" if loaded["initialSelfInterference"] else "pass"
            except AdapterError as error:
                report.update(status="rejected", error=str(error))
            results.append(report)
    passed = len(results) == 4 and all(result["status"] == "pass" for result in results)
    report = {"schemaVersion": 1, "status": "pass" if passed else "rejected",
              "dynamicTrials": 0, "simulationSeconds": 0, "results": results,
              "limits": [
                  "Uses standard reconstruction and zero-time interference gates; never advances physics.",
                  "The control fixture's additional 1-micrometer regression limit is reported, not a native screening gate.",
                  "No swept-clearance, dynamics, walking, founder-reachability or live mutation activation claim.",
              ]}
    write_json(output / "check.json", report)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ldraw-library", type=Path, required=True)
    args = parser.parse_args()
    report = verify(args.input, args.output, args.ldraw_library)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["status"] == "pass" else 2)
