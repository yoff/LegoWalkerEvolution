import json
import sqlite3
import subprocess
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from legowalkers_sim import motion_carry, native_episode
from legowalkers_sim.bridge import AdapterError, SIMULATOR_REVISION, load_manifest, write_json
from legowalkers_sim.evolution import Archive, Generator, ROOT
from test_native_discovery import Proposals, setup_discovery
from test_native_episode import native_fixture
from test_native_discovery import mock_constructor
from test_evolution import protocol


def pending(archive):
    return archive.db.execute(
        "SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth WHERE c.birth=1").fetchone()


@pytest.mark.integration
def test_actual_rerooted_construction_batch_binds_exported_mechanism_identity(tmp_path):
    generator = Generator(ROOT / ".tools/dotnet/dotnet",
                          ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    if not all(Path(path).is_file() for path in generator.command) or not (ROOT / ".venv/bin/z3").is_file():
        pytest.skip("Build the F# CLI and install Z3 for motion-first construction integration.")
    founder = tmp_path / "founder"
    generator.invoke("realize-walker", "--feet", 4, "--steps", 72, "--output", founder)
    parent = founder / "candidate-001"
    extension = next(edit for edit in generator.manual_edits(parent)["edits"]
                     if edit["operation"] == "shared-leg-lengthen-foot"
                     and any(change["path"].endswith("/contact") and 139.9 < change["after"][0] < 140.1
                             for change in edit["changed"]))
    extended = tmp_path / "extended-feet"
    extended_summary = generator.apply_edit(parent, extended, extension["id"], limit=1)
    assert extended_summary["generated"], extended_summary["gates"]
    parent = extended / extended_summary["generated"][0]["directory"]
    motor_edit = next(edit for edit in generator.manual_edits(parent)["edits"]
                      if edit["operation"] == "motor-layout"
                      and any(change["path"].endswith("->shaft-4") for change in edit["changed"])
                      and any(change["path"] == "hardware/motor/position" and change["after"][2] > 0
                              for change in edit["changed"]))
    rerooted = tmp_path / "rerooted-source"
    rerooted_summary = generator.apply_edit(parent, rerooted, motor_edit["id"], limit=1)
    assert rerooted_summary["generated"], rerooted_summary["gates"]
    parent = rerooted / rerooted_summary["generated"][0]["directory"]
    original = load_manifest(parent / "manifest.json")
    output = tmp_path / "constructed"
    summary = generator.build_native(parent, output)
    assert summary["generated"], summary["gates"]
    entry, = summary["generated"]
    candidate = output / entry["directory"]
    manifest = load_manifest(candidate / "manifest.json")
    assert manifest["motion"]["mechanismId"] != original["motion"]["mechanismId"]
    assert summary["mechanismId"] == manifest["motion"]["mechanismId"]
    queued, _ = native_episode._queue(output)
    assert queued == summary
    assert summary["counts"]["dynamicTrials"] == 0


def test_balance_model_uses_whole_robot_mass_and_support_bounds():
    layout = {"totalMassKg": 0.5, "centerOfMassLdu": [100, -50, 80],
              "sweptFootHullXZLdu": [[0, -100], [400, -100], [400, 200], [0, 200]],
              "components": [{"partInstance": part, "massKg": mass, "centerLdu": center}
                             for part, mass, center in [
                                 ("motor", .06, [40, 0, 200]), ("payload-battery", .2, [500, -120, 40]),
                                 ("payload-receiver", .025, [80, -80, 180])]]}
    model = motion_carry._balance_model(layout)
    assert model["supportCenterLdu"] == {"x": 200, "y": -50, "z": 50}
    assert model["supportHalfSpanLdu"] == {"x": 200, "y": 0, "z": 150}
    assert model["totalMassKg"] == .5 and model["centerOfMassLdu"] == {"x": 100, "y": -50, "z": 80}
    assert model["components"][0]["massKg"] == .06
    renamed = {**layout, "components": [
        {**c, "partInstance": "input-motor"} if c["partInstance"] == "motor" else c
        for c in layout["components"]]}
    assert motion_carry._balance_model(renamed, "input-motor")["components"][0]["part"] == "input-motor"
    with pytest.raises(AdapterError, match="Invalid source balance estimate"):
        motion_carry._balance_model({**layout, "totalMassKg": .1})
    with pytest.raises(AdapterError, match="degenerate"):
        motion_carry._balance_model({**layout, "sweptFootHullXZLdu": [[0, 0], [100, 0]]})


def test_balance_estimate_is_passed_into_each_constructor_alternative(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    layout = {"totalMassKg": .5, "centerOfMassLdu": [100, -50, 80],
              "sweptFootHullXZLdu": [[0, -100], [400, 200]],
              "components": [{"partInstance": p, "massKg": m, "centerLdu": [0, 0, 0]}
                             for p, m in [("motor", .06), ("payload-battery", .2), ("payload-receiver", .025)]]}
    layout["partMasses"] = [{"partInstance": "beam-a", "massKg": .01, "centerLdu": [0, 0, 0]}]
    monkeypatch.setattr(motion_carry, "_centering", lambda *args: (
        {"payload-battery": {"x": 200, "y": -120, "z": 40}}, layout))
    models = []

    def build(*args, balance_model, **kwargs):
        models.append(json.loads(balance_model.read_text()))
        return {"generated": [], "rejected": [], "gates": []}

    monkeypatch.setattr(Proposals, "build_native", build)
    try:
        native_episode.stage_native(archive, source)
        motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path)
        assert len(models) == 4
        assert all(model == motion_carry._balance_model(layout, scaffold_parts=["beam-a"]) for model in models)
        assert archive.counts() == {}
    finally:
        archive.db.close()

def test_same_genotype_construction_binds_new_proof_without_mutating_original_inputs(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    try:
        native_episode.stage_native(archive, source)
        original = dict(pending(archive))
        constructed = motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path)
        assert dict(pending(archive)) == original
        assert constructed["birth"] == 1 and constructed["design"] == original["design"]
        assert constructed["batch"] != original["batch"]
        assert archive.counts() == {}
        checked = SimpleNamespace(manifest=load_manifest(
            archive.directory / constructed["batch"] / constructed["directory"] / "manifest.json"))
        motion_carry.require_construction(archive, 1, checked)
        assert motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path) == constructed
        with pytest.raises(sqlite3.IntegrityError, match="immutable"):
            archive.db.execute("UPDATE motion_candidates SET batch='stale' WHERE birth=1")
        proof = archive.directory / constructed["batch"] / "motion-carry-forward.json"
        proof.write_text("{}")
        with pytest.raises(AdapterError, match="changed after"):
            motion_carry.require_construction(archive, 1, checked)
    finally:
        archive.db.close()


def test_unbuilt_motion_cannot_reserve_a_trial(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    try:
        native_episode.stage_native(archive, source)
        checked = SimpleNamespace(manifest=load_manifest(source / "candidate-0/manifest.json"))
        with pytest.raises(AdapterError, match="inherited hardware"):
            motion_carry.require_construction(archive, 1, checked)
        assert archive.counts() == {}
    finally:
        archive.db.close()


def test_construction_rejection_keeps_motion_source_without_spending_budget(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    try:
        native_episode.stage_native(archive, source)
        original = dict(pending(archive))
        monkeypatch.setattr(Proposals, "build_native", lambda *args, **kwargs: {
            "generated": [], "rejected": [{"code": "no-route"}], "gates": []})
        assert motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path) is None
        assert dict(pending(archive)) == original and archive.counts() == {}
        assert archive.db.execute("SELECT status FROM births WHERE id=1").fetchone()[0] == "construction-rejected"
        result = json.loads((archive.directory / "motion-builds/000001/result.json").read_text())
        assert len(result["attempts"]) == 2 and result["dynamicTrials"] == 0
    finally:
        archive.db.close()


def test_effective_duplicate_requests_are_carried_between_alternatives(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    searched, seen = [], []
    monkeypatch.setattr(motion_carry, "_centering", lambda *args: (
        {"payload-battery": {"x": 0, "y": 0, "z": 0}}, None))

    def build(generator, parent, output, *, layers, placements, seen_requests, balance_model):
        output.mkdir()
        request_id = str(layers) * 64
        duplicate = request_id in seen_requests
        seen.append(list(seen_requests))
        if not duplicate:
            searched.append(request_id)
        return {"generated": [], "rejected": [], "gates": [],
                "motionFirst": {"requestId": request_id,
                                "status": "duplicate-request" if duplicate else "productions-exhausted"}}

    monkeypatch.setattr(Proposals, "build_native", build)
    try:
        native_episode.stage_native(archive, source)
        assert motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path) is None
        outcome = json.loads((archive.directory / "motion-builds/000001/result.json").read_text())
        assert len(searched) == 2 and len(outcome["attempts"]) == 4
        assert [entry["status"] for entry in outcome["attempts"]] == [
            "construction-rejected", "construction-rejected", "duplicate-request", "duplicate-request"]
        assert seen == [[], ["1"*64], ["1"*64, "2"*64], ["1"*64, "2"*64]]
        assert archive.counts() == {}
    finally:
        archive.db.close()


def test_historical_motion_is_queued_unchanged_then_imported_and_built_with_live_progress(tmp_path, monkeypatch):
    source, _ = native_fixture(tmp_path)
    manifest_path = source / "candidate-0/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["simulator"]["revision"] = "0"*40
    write_json(manifest_path, manifest)
    original = manifest_path.read_bytes()
    base = protocol()
    mock_constructor(tmp_path, monkeypatch, base)
    archive = Archive(tmp_path / "deferred")
    archive.initialize(native_episode.native_protocol(base, source, discover=True, defer_construction=True), 1, 42, False)
    calls = []

    class Compiler(Proposals):
        def invoke(self, *args):
            calls.append(args[0])
            parent = (Path(args[args.index("--parent")+1]) if "--parent" in args
                      else Path(args[args.index("--program")+1]).parent)
            target = Path(args[args.index("--output")+1]) / "candidate-001"
            shutil.copytree(parent, target)
            current = json.loads((target / "manifest.json").read_text())
            current["simulator"]["revision"] = SIMULATOR_REVISION
            write_json(target / "manifest.json", current)
            if args[0] == "migrate-walker-legs":
                write_json(target.parent / "leg-family-conversion.json",
                           {"status": "no-matching-legs", "families": [], "unmatched": []})

        def build_native(self, parent, output, **kwargs):
            calls.append("build")
            assert archive.db.execute("SELECT stage FROM pipeline_events ORDER BY id DESC LIMIT 1").fetchone()[0] == "building"
            return super().build_native(parent, output, **kwargs)

    try:
        native_episode.stage_native(archive, source)
        assert calls == [] and archive.counts() == {}
        checked = SimpleNamespace(manifest=manifest)
        with pytest.raises(AdapterError, match="inherited hardware"):
            motion_carry.require_construction(archive, 1, checked)
        compiled = motion_carry.ensure_pending(archive, pending(archive), Compiler(), tmp_path)
        assert calls == ["realize-walker", "migrate-walker-legs", "build"]
        assert manifest_path.read_bytes() == original
        assert (archive.directory / "native-input/candidate-0/manifest.json").read_bytes() == original
        checked.manifest = load_manifest(archive.directory / compiled["batch"] / compiled["directory"] / "manifest.json")
        motion_carry.require_construction(archive, compiled["birth"], checked)
        assert archive.counts() == {}
    finally:
        archive.db.close()


def test_construction_timeout_is_recorded_without_spending_or_repeating_work(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    calls = []

    def timeout(*args, **kwargs):
        calls.append(args)
        args[2].mkdir()
        (args[2] / "partial.txt").write_text("unfinished")
        raise subprocess.TimeoutExpired(["build-walker"], 600)

    monkeypatch.setattr(Proposals, "build_native", timeout)
    try:
        native_episode.stage_native(archive, source)
        assert motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path) is None
        result = json.loads((archive.directory / "motion-builds/000001/result.json").read_text())
        assert result["status"] == "construction-timeout" and result["dynamicTrials"] == 0
        assert (archive.directory / "motion-builds/000001/attempt-001/partial.txt").exists()
        assert archive.db.execute("SELECT status FROM births WHERE id=1").fetchone()[0] == "construction-rejected"
        assert motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path) is None
        assert len(calls) == 1 and archive.counts() == {}
    finally:
        archive.db.close()


@pytest.mark.parametrize("change", ["reference", "clock", "speed", "proof", "balance"])
def test_generator_rejects_changed_intent_control_or_missing_construction_proof(tmp_path, monkeypatch, change):
    source, _ = native_fixture(tmp_path)
    parent = source / "candidate-0"
    output = tmp_path / "build"
    program = {"id": "retained-motion"}
    (parent / "program.json").write_text(json.dumps({"program": program}))
    balance_file = tmp_path / "balance-model.json"
    balance_file.write_text('{"totalMassKg": 0.5}')
    solver = tmp_path / "z3"
    solver.write_bytes(b"solver fixture")
    solver.chmod(0o755)

    def run(*args, **kwargs):
        command = args[0]
        assert command[command.index("--constraint-solver")+1] == str(solver)
        summary = Proposals().build_native(parent, output)
        summary.update(parentGenotypeId=load_manifest(parent / "manifest.json")["motion"]["genotypeId"],
                       counts={"exported": 1, "dynamicTrials": 0},
                       motionFirst={"packagingVersion": motion_carry.POLICY["packagingVersion"],
                                    "theoryVersion": motion_carry.POLICY["theoryVersion"]})
        summary["rejected"] = []
        (output / "candidates.json").write_text(json.dumps(summary))
        request = {"reference": {} if change == "reference" else program,
                   "drive": {"motorClockScale": 5 if change == "clock" else 1},
                   "groundClearance": {"minimumNonFootClearanceLdu": 8}}
        (output / "request.json").write_text(json.dumps(request))
        candidate = output / "candidate-001"
        for name in ("ground-plan.json", "construction-proof.json"):
            if change != "proof":
                (candidate / name).write_text("{}")
        if change == "speed":
            genome = json.loads((candidate / "realization.json").read_text())
            genome["speed"] = -genome["speed"]
            (candidate / "realization.json").write_text(json.dumps(genome))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr("legowalkers_sim.evolution.subprocess.run", run)
    with pytest.raises(AdapterError, match="reference intent|electrical control|construction proof|balance estimate"):
        Generator(Path("dotnet"), tmp_path / "generator.dll").build_native(
            parent, output, balance_model=balance_file if change == "balance" else None)
