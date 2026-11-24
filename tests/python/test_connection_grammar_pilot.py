from pathlib import Path
from types import SimpleNamespace
import warnings

import pytest

from legowalkers_sim import connection_grammar_pilot as pilot
from legowalkers_sim import bridge
from legowalkers_sim.bridge import AdapterError, write_json


def test_pilot_never_accepts_dynamic_steps():
    with pytest.raises(AdapterError, match="Dynamics are forbidden"):
        pilot._zero_steps(None, None)


def test_cached_initial_pass_is_not_confused_with_rejected_duplicate():
    assert pilot._ready({"status": "ready-for-dynamics"})
    assert pilot._ready({"status": "duplicate", "previousStatus": "ready-for-dynamics"})
    assert not pilot._ready({"status": "duplicate", "previousStatus": "rejected"})
    assert not pilot._ready({"status": "rejected"})


def test_generation_checks_exact_motion_program_bytes(tmp_path, monkeypatch):
    source = tmp_path / "source"
    source.mkdir()
    (source / "program.json").write_text('{"same": true}\n')
    case = {"source": str(source), "birth": 17, "mechanismId": "motion"}

    def fake_run(command, **kwargs):
        destination = Path(command[command.index("--output") + 1])
        candidate = destination / "candidate-001"
        candidate.mkdir(parents=True)
        write_json(destination / "candidates.json", {
            "counts": {"dynamicTrials": 0}, "generated": [{"directory": "candidate-001"}],
        })
        (candidate / "program.json").write_text('{"same": false}\n')
        write_json(candidate / "manifest.json", {"motion": {"mechanismId": "motion"}})
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(pilot.subprocess, "run", fake_run)
    with pytest.raises(AdapterError, match="altered the motion program bytes"):
        pilot._generate(case, [], tmp_path / "generated", Path("dotnet"), Path("generator"), 1)


def test_screening_rejects_nonzero_time_even_if_report_says_ready(tmp_path, monkeypatch):
    monkeypatch.setattr(pilot, "screen_batch", lambda *args, **kwargs: {
        "counts": {"dynamicTrials": 0, "reconstructionAttempts": 1},
        "results": [{"status": "ready-for-dynamics", "loadOnly": {"simulationSeconds": .002, "dynamicTrials": 0}}],
    })
    with pytest.raises(AdapterError, match="advanced dynamics"):
        pilot._screen(tmp_path, {"directory": "one", "genotypeId": "g"}, tmp_path / "out", tmp_path)


def test_pilot_html_links_successful_derivation_and_ldraw(tmp_path):
    folder = tmp_path / "repair-0017/candidates/candidate-001"
    summary = {
        "counts": {"recoveredMotionContexts": 1}, "baselines": [], "limitations": ["Bounded."],
        "baselineMode": "identity-and-archived-disposition",
        "repairs": [{
            "case": {"birth": 17}, "exportedAlternatives": 1,
            "attempts": [{"ready": True, "directory": str(folder)}],
        }],
    }
    pilot._write_html(tmp_path, summary)
    html = (tmp_path / "pilot.html").read_text()
    assert "repair-0017/candidates/candidate-001/grammar.json" in html
    assert "repair-0017/candidates/candidate-001/model.ldr" in html
    assert "Initial pass recovered" in html
    assert "No dynamics" in html
    assert "Failed assemblies are counterexamples" in html


def test_part_parser_cache_revalidates_all_prior_dependencies(tmp_path, monkeypatch):
    a = tmp_path / "a.ldr"
    b = tmp_path / "b.ldr"
    a.write_text("1 0 0 0 0 1 0 0 0 1 0 0 0 1 a.dat\n")
    b.write_text("1 0 0 0 0 1 0 0 0 1 0 0 0 1 b.dat\n")
    revisions = {"a.dat": "old", "b.dat": "unchanged"}
    monkeypatch.setattr(bridge, "library_fingerprint", lambda library, parts: revisions[parts[0]])
    cache = bridge.PartParserCache()
    first = cache.parser_for(a, tmp_path, ["a.dat"])
    assert cache.parser_for(a, tmp_path, ["a.dat"]) is first
    revisions["a.dat"] = "new"
    second = cache.parser_for(b, tmp_path, ["b.dat"])
    assert second is not first
    assert cache.parser_for(b, tmp_path, ["b.dat"]) is second
    cache.invalidate()
    assert cache.parser_for(b, tmp_path, ["b.dat"]) is not second


def test_part_parser_cache_does_not_reuse_local_or_inline_models(tmp_path, monkeypatch):
    model = tmp_path / "model.ldr"
    model.write_text("1 0 0 0 0 1 0 0 0 1 0 0 0 1 a.dat\n")
    monkeypatch.setattr(bridge, "library_fingerprint", lambda *args: "fixed")
    cache = bridge.PartParserCache()
    first = cache.parser_for(model, tmp_path, ["a.dat"])
    (tmp_path / "a.dat").write_text("0 local override\n")
    assert cache.parser_for(model, tmp_path, ["a.dat"]) is not first
    model.write_text("0\tFILE\tmodel.ldr\n")
    assert cache.parser_for(model, tmp_path, []) is not first


def test_parser_warning_never_becomes_a_cached_success(tmp_path, monkeypatch):
    model = tmp_path / "model.ldr"
    model.write_text("1 0 0 0 0 1 0 0 0 1 0 0 0 1 a.dat\n")
    instances = []

    class Parser:
        def __init__(self, **kwargs):
            instances.append(self)

        def parse_build(self, path):
            warnings.warn("bad geometry", RuntimeWarning)
            return object()

    monkeypatch.setattr(bridge, "LDrawParser", Parser)
    monkeypatch.setattr(bridge, "load_manifest", lambda path: {"parts": [{"part": "a.dat"}], "modelFile": "model.ldr"})
    monkeypatch.setattr(bridge, "library_fingerprint", lambda *args: "fixed")
    monkeypatch.setattr(bridge, "build_units_and_joints", lambda build: object())
    cache = bridge.PartParserCache()
    for _ in range(2):
        with pytest.raises(AdapterError, match="bad geometry"):
            bridge.check(tmp_path / "manifest.json", tmp_path, parser_cache=cache)
    assert len(instances) == 2


def test_failed_assemblies_supply_constraints_but_are_not_preservation_controls(tmp_path, monkeypatch):
    campaign = tmp_path / "campaign"
    archive = campaign / "versions/v-test"
    archive.mkdir(parents=True)
    (archive / "archive.sqlite3").touch()
    library = tmp_path / "library"
    library.mkdir()
    dotnet, generator = tmp_path / "dotnet", tmp_path / "generator.dll"
    dotnet.touch()
    generator.touch()
    (tmp_path / "LegoWalkers.Core.dll").touch()
    good, bad = tmp_path / "good", tmp_path / "bad"
    write_json(good / "manifest.json", {"motion": {"assemblyId": "passing"}})
    common = {"excluded": False, "exclusionReason": None, "mechanismId": "motion"}
    passing = {**common, "birth": 13, "source": str(good), "status": "simulation-rejected",
               "expectedInitialPass": True, "pairs": []}
    failing = {**common, "birth": 17, "source": str(bad), "status": "screening-rejected",
               "expectedInitialPass": False, "pairs": [["beam", "pin"]]}
    monkeypatch.setattr(pilot, "_cohort", lambda archive: [passing, failing])
    generated = []

    def generate(case, pairs, output, *args):
        generated.append((case["birth"], pairs))
        return {
            "generated": [{"directory": "one", "assemblyId": "passing" if not pairs else "new", "genotypeId": "g"}],
            "search": {},
        }, ["fake-generator"]

    monkeypatch.setattr(pilot, "_generate", generate)
    monkeypatch.setattr(pilot, "_screen", lambda source, entry, *args: {
        "directory": str(source / entry["directory"]), "assemblyId": entry["assemblyId"],
        "ready": True, "reconstructionAttempts": 1, "report": {"status": "ready-for-dynamics"},
    })
    summary = pilot.run(campaign, "v-test", tmp_path / "output", library, dotnet, generator)
    assert generated == [(13, []), (17, [["beam", "pin"]])]
    assert summary["counts"]["passingControlCases"] == 1
    assert summary["counts"]["failedAssemblyCounterexamples"] == 1
    assert summary["counts"]["recoveredMotionContexts"] == 1
    assert summary["counts"]["freshBaselineChecks"] == 0
