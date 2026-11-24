import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from legowalkers_sim.bridge import AdapterError, write_json
from test_native_episode import native_fixture

SPEC = importlib.util.spec_from_file_location(
    "benchmark_realizations", Path(__file__).parents[2] / "scripts/benchmark_realizations.py")
assert SPEC is not None and SPEC.loader is not None
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)


def test_benchmark_compares_exact_inherited_assembly_with_same_motion_repairs(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    candidate = source / "candidate-0"
    original = (candidate / "manifest.json").read_bytes()
    commands = []

    def generate(command, **kwargs):
        commands.append(command)
        output = Path(command[command.index("--output")+1])
        write_json(output / "candidates.json", {
            "mechanismId": data["motion"]["mechanismId"], "constructionVersion": "technic-construction-v1",
            "counts": {"realizationAttempts": 2, "exported": 2, "dynamicTrials": 0},
        })
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(benchmark.subprocess, "run", generate)
    monkeypatch.setattr(benchmark, "screen_batch", lambda *args, **kwargs: {
        "counts": {"dynamicTrials": 0, "reconstructionAttempts": 2},
        "results": [{"genotypeId": data["motion"]["genotypeId"], "status": "rejected",
                     "dynamicTrials": 0, "simulationSeconds": 0},
                    {"genotypeId": "repair", "status": "ready-for-dynamics",
                     "dynamicTrials": 0, "simulationSeconds": 0}],
    })
    output = tmp_path / "benchmark"
    measured = benchmark.benchmark([candidate], output, tmp_path, tmp_path / "dotnet",
                                   tmp_path / "generator", 2)
    assert measured["baselinePassingCases"] == 0 and measured["casesWithPassingRealization"] == 1
    assert measured["recoveredCases"] == 1 and measured["dynamicTrials"] == measured["simulationSeconds"] == 0
    assert commands[0][2] == "construct-walker" and commands[0][-1] == "2"
    assert json.loads((output / "benchmark.json").read_text()) == measured
    assert original == (candidate / "manifest.json").read_bytes()
    with pytest.raises(AdapterError, match="must be empty"):
        benchmark.benchmark([candidate], output, tmp_path, tmp_path / "dotnet", tmp_path / "generator", 2)


@pytest.mark.parametrize("limit", [0, 33])
def test_benchmark_enforces_explicit_realization_bounds(tmp_path, limit):
    with pytest.raises(AdapterError, match="one and 32"):
        benchmark.benchmark([tmp_path], tmp_path / "output", tmp_path, tmp_path, tmp_path, limit)


def test_benchmark_requires_a_fixed_case(tmp_path):
    with pytest.raises(AdapterError, match="At least one"):
        benchmark.benchmark([], tmp_path / "output", tmp_path, tmp_path, tmp_path, 2)


@pytest.mark.parametrize(("trials", "report"), [
    (1, {"dynamicTrials": 0, "simulationSeconds": 0}),
    (0, {"dynamicTrials": 1, "simulationSeconds": 0}),
    (0, {"dynamicTrials": 0, "simulationSeconds": 0.001}),
    (0, {"dynamicTrials": 0}),
])
def test_benchmark_rejects_dynamics_or_unverified_zero_time(tmp_path, monkeypatch, trials, report):
    source, data = native_fixture(tmp_path)

    def generate(command, **kwargs):
        output = Path(command[command.index("--output")+1])
        write_json(output / "candidates.json", {"mechanismId": data["motion"]["mechanismId"]})
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(benchmark.subprocess, "run", generate)
    monkeypatch.setattr(benchmark, "screen_batch", lambda *args, **kwargs: {
        "counts": {"dynamicTrials": trials}, "results": [report],
    })
    output = tmp_path / "benchmark"
    with pytest.raises(AdapterError, match="load-only benchmark"):
        benchmark.benchmark([source / "candidate-0"], output, tmp_path, tmp_path, tmp_path, 2)
    assert not (output / "benchmark.json").exists()


@pytest.mark.parametrize("max_checks", [None, 3])
def test_benchmark_feeds_latest_rejection_into_bounded_same_motion_repair(tmp_path, monkeypatch, max_checks):
    source, data = native_fixture(tmp_path)
    mechanism = data["motion"]["mechanismId"]
    baseline = data["motion"]["genotypeId"]
    commands = []

    def generate(command, **kwargs):
        commands.append(command)
        output = Path(command[command.index("--output")+1])
        repairing = "--feedback" in command
        count = 1 if repairing else 2
        summary = {"mechanismId": mechanism, "constructionVersion": "technic-construction-v2",
                   "parentGenotypeId": "shorter-pin" if repairing else baseline,
                   "generated": [{"directory": "candidate-001"}],
                   "counts": {"realizationAttempts": count, "exported": count, "dynamicTrials": 0}}
        write_json(output / "candidates.json", summary)
        return SimpleNamespace(returncode=0)

    def screen(batch, output, library, **kwargs):
        repairing = batch.name.startswith("repair-")
        assert kwargs["max_checks"] == (1 if repairing and max_checks == 3 else 2)
        reports = ([{"genotypeId": "layer-repair", "status": "ready-for-dynamics"}] if repairing else
                   [{"genotypeId": baseline, "status": "rejected"},
                    {"genotypeId": "shorter-pin", "status": "rejected"}])
        for report in reports:
            report.update(candidate=str(batch / report["genotypeId"]), dynamicTrials=0, simulationSeconds=0)
            if report["status"] == "rejected":
                report["repairHints"] = {"kind": "rigid-body-partition", "parts": ["beam-b"], "bodies": ["chassis"]}
        return {"counts": {"dynamicTrials": 0, "reconstructionAttempts": len(reports)}, "results": reports}

    monkeypatch.setattr(benchmark.subprocess, "run", generate)
    monkeypatch.setattr(benchmark, "screen_batch", screen)
    output = tmp_path / "benchmark"
    measured = benchmark.benchmark([source / "candidate-0"], output, tmp_path, tmp_path, tmp_path, 2,
                                   repair_rounds=2, max_checks=max_checks)
    assert len(commands) == 2 and "--feedback" in commands[1]
    assert measured["recoveredCases"] == 1 and measured["maxRepairRounds"] == 2
    case = measured["cases"][0]
    assert case["generation"]["realizationAttempts"] == case["screening"]["reconstructionAttempts"] == 3
    assert case["repairHistory"][0]["parentGenotypeId"] == "shorter-pin"
    assert case["baselineReady"] is False and case["anyReady"] is True


def test_total_check_budget_stops_feedback_and_freezes_custom_policy(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    commands, allowances = [], []
    policy = tmp_path / "policy.json"
    write_json(policy, {"schemaVersion": 1})

    def generate(command, **kwargs):
        commands.append(command)
        assert "--builder-policy" in command
        snapshot = Path(command[command.index("--builder-policy")+1])
        assert snapshot != policy and json.loads(snapshot.read_text()) == {"schemaVersion": 1}
        output = Path(command[command.index("--output")+1])
        write_json(output / "candidates.json", {
            "mechanismId": data["motion"]["mechanismId"], "constructionVersion": "test",
            "builderPolicyId": "a"*64, "counts": {"realizationAttempts": 2, "dynamicTrials": 0}})
        return SimpleNamespace(returncode=0)

    def screen(*args, **kwargs):
        allowances.append(kwargs["max_checks"])
        return {"counts": {"dynamicTrials": 0, "reconstructionAttempts": 1},
                "results": [{"genotypeId": data["motion"]["genotypeId"], "status": "rejected",
                             "dynamicTrials": 0, "simulationSeconds": 0,
                             "repairHints": {"kind": "rigid-body-partition", "parts": ["beam"], "bodies": []}}]}

    monkeypatch.setattr(benchmark.subprocess, "run", generate)
    monkeypatch.setattr(benchmark, "screen_batch", screen)
    measured = benchmark.benchmark([source / "candidate-0"], tmp_path / "benchmark", tmp_path, tmp_path,
                                   tmp_path, 2, repair_rounds=2, builder_policy=policy, max_checks=1)
    assert len(commands) == 1 and allowances == [1]
    assert measured["cases"][0]["termination"] == "check-budget"
    assert measured["maxChecksPerCase"] == 1 and measured["limitPerBatch"] == 2


@pytest.mark.parametrize("checks", [0, 33, True])
def test_total_check_budget_rejects_invalid_allowances(tmp_path, checks):
    with pytest.raises(AdapterError, match="one and 32"):
        benchmark.benchmark([tmp_path], tmp_path / "out", tmp_path, tmp_path, tmp_path, 1, max_checks=checks)


def comparison_cases(tmp_path):
    cases = []
    for index, token in enumerate(("a", "b")):
        root = tmp_path / str(index)
        root.mkdir()
        source, data = native_fixture(root)
        data["motion"].update(genotypeId=token*64, mechanismId=token*64, assemblyId=token*64)
        data["genome"]["mechanismId"] = token*64
        (source / "candidate-0/manifest.json").write_text(json.dumps(data))
        cases.append(source / "candidate-0")
    return cases


def comparison_generator(tmp_path, monkeypatch, *, duplicate=False, default_token="a"):
    generator = tmp_path / "generator.dll"
    generator.write_bytes(b"generator")
    generator.with_name("LegoWalkers.Core.dll").write_bytes(b"core")

    def describe(command, **kwargs):
        alternative = "--builder-policy" in command and not duplicate
        return SimpleNamespace(returncode=0, stdout=json.dumps({
            "policyId": ("b" if alternative else default_token)*64,
            "catalogueVersion": "test", "policy": {"alternative": alternative}}))

    monkeypatch.setattr(benchmark.subprocess, "run", describe)
    return generator


def test_policy_selection_uses_only_training_then_verifies_selected_policy(tmp_path, monkeypatch):
    train, heldout = comparison_cases(tmp_path)
    generator = comparison_generator(tmp_path, monkeypatch)
    calls = []

    def measure(cases, output, library, dotnet, binary, limit, profile, rounds, policy, checks):
        alternative = json.loads(policy.read_text())["alternative"]
        assert limit == 1 and rounds == 1 and checks == 2
        calls.append((cases, alternative))
        ready = alternative if cases == [train] else not alternative
        return {"builderPolicyId": ("b" if alternative else "a")*64,
                "casesWithPassingRealization": int(ready),
                "cases": [{"screening": {"reconstructionAttempts": 2}}]}

    monkeypatch.setattr(benchmark, "benchmark", measure)
    output = tmp_path / "comparison"
    measured = benchmark.compare_policies([train], [heldout], [tmp_path / "policy.json"], output,
                                          tmp_path, tmp_path, generator, max_checks=2, repair_rounds=1)
    assert calls == [([train], False), ([train], True), ([heldout], True)]
    assert measured["selectedPolicyId"] == "b"*64 and measured["heldout"]["ready"] == 0
    assert measured["promotion"] == "manual-review-required"
    assert measured["dynamicTrials"] == measured["simulationSeconds"] == 0
    assert json.loads((output / "policy-comparison.json").read_text()) == measured


@pytest.mark.parametrize("default_checks", [1, 2])
def test_policy_ties_retain_default_and_duplicate_normalized_policies_are_rejected(tmp_path, monkeypatch, default_checks):
    train, heldout = comparison_cases(tmp_path)
    generator = comparison_generator(tmp_path, monkeypatch, default_token="f")

    def measure(cases, output, library, dotnet, binary, limit, profile, rounds, policy, checks):
        alternative = json.loads(policy.read_text())["alternative"]
        return {"builderPolicyId": ("b" if alternative else "f")*64,
                "casesWithPassingRealization": 1,
                "cases": [{"screening": {"reconstructionAttempts": 1 if alternative else default_checks}}]}

    monkeypatch.setattr(benchmark, "benchmark", measure)
    measured = benchmark.compare_policies([train], [heldout], [tmp_path / "policy"], tmp_path / "comparison",
                                          tmp_path, tmp_path, generator)
    assert measured["selectedPolicyId"] == ("f" if default_checks == 1 else "b")*64
    assert measured["promotion"] == ("retain-default" if default_checks == 1 else "manual-review-required")
    comparison_generator(tmp_path, monkeypatch, duplicate=True)
    with pytest.raises(AdapterError, match="normalized policy"):
        benchmark.compare_policies([train], [heldout], [tmp_path / "policy"], tmp_path / "duplicate",
                                    tmp_path, tmp_path, generator)


def test_policy_split_rejects_repair_siblings_and_duplicate_cases_before_running_generator(tmp_path, monkeypatch):
    train, heldout = comparison_cases(tmp_path)
    path = heldout / "manifest.json"
    data = json.loads(path.read_text())
    data["motion"]["mechanismId"] = "a"*64
    data["genome"]["mechanismId"] = "a"*64
    path.write_text(json.dumps(data))
    monkeypatch.setattr(benchmark.subprocess, "run", lambda *args, **kwargs: pytest.fail("Generator must not run."))
    with pytest.raises(AdapterError, match="overlap by mechanismId"):
        benchmark.compare_policies([train], [heldout], [tmp_path / "policy"], tmp_path / "comparison",
                                    tmp_path, tmp_path, tmp_path)
    with pytest.raises(AdapterError, match="repeat a genotype"):
        benchmark.compare_policies([train, train], [heldout], [tmp_path / "policy"], tmp_path / "comparison",
                                    tmp_path, tmp_path, tmp_path)
