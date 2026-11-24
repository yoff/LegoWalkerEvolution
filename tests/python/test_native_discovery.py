import json
import hashlib
import os
from pathlib import Path
import shutil
import sqlite3
from types import SimpleNamespace

import pytest

from legowalkers_sim import discovery, native_episode, screening
from legowalkers_sim.bridge import AdapterError, load_manifest
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import Archive, Generator, ROOT, identity
from test_evolution import protocol, result
from test_native_episode import native_fixture


class Proposals:
    def __init__(self, actions=(), siblings=1):
        self.actions = list(actions)
        self.siblings = siblings
        self.calls = []

    def build_native(self, parent, output, *, layers=1, placements=None):
        output.mkdir(parents=True)
        shutil.copytree(parent, output / "candidate-001")
        data = load_manifest(parent / "manifest.json")
        summary = {"schemaVersion": 1, "gateVersion": "motion-gates-v1", "mechanismId": data["motion"]["mechanismId"],
                   "generated": [{"directory": "candidate-001", "genotypeId": data["motion"]["genotypeId"],
                                  "assemblyId": data["motion"]["assemblyId"]}], "gates": []}
        (output / "candidates.json").write_text(json.dumps(summary))
        return summary

    def propose_native(self, parent, output, seed, attempt, limit):
        self.calls.append((parent, seed, attempt))
        parent_manifest = load_manifest(parent / "manifest.json")
        action = self.actions.pop(0) if self.actions else "offspring"
        output.mkdir(parents=True)
        summary = {
            "schemaVersion": 1, "gateVersion": "motion-gates-v1", "mutationVersion": discovery.POLICY["mutationVersion"],
            "parentGenotypeId": parent_manifest["motion"]["genotypeId"], "seed": seed, "attempt": attempt,
            "operation": "input-phase", "changed": [{"path": "motor/phase", "before": [0], "after": [1]}],
            "mutationSelection": {"policy": discovery.POLICY["mutationSelection"]["policy"],
                                  "selectedFamily": "input-phase",
                                  "availableFamilies": [{"operation": "input-phase", "edits": 2}]},
            "mechanismId": parent_manifest["motion"]["mechanismId"], "gates": [], "generated": [], "rejected": [],
            "counts": {"proposals": 1, "realizationAttempts": 0, "rejected": 0, "duplicates": 0, "exported": 0, "dynamicTrials": 0},
        }
        if action == "reject":
            summary["gates"] = [{"gate": "mobility", "outcome": "reject", "code": "unit-test-nonclosure", "diagnostics": []}]
        else:
            for number in range(1 if action == "duplicate" else min(limit, self.siblings)):
                name = f"candidate-{number:03d}"
                destination = output / name
                shutil.copytree(parent, destination)
                data = json.loads((destination / "manifest.json").read_text())
                if action != "duplicate":
                    token = f"{len(self.calls)}-{number}"
                    data["motion"]["mechanismId"] = identity({"motion": len(self.calls)})
                    data["genome"]["mechanismId"] = data["motion"]["mechanismId"]
                    data["genome"]["speed"] += len(self.calls)+number
                    data["expected"]["drives"][0]["speed"] = data["genome"]["speed"]
                    data["motion"]["genotypeId"] = identity(data["genome"])
                    data["motion"]["assemblyId"] = identity({"assembly": token})
                summary["mechanismId"] = data["motion"]["mechanismId"]
                (destination / "manifest.json").write_text(json.dumps(data))
                (destination / "realization.json").write_text(json.dumps(data["genome"]))
                summary["generated"].append({"directory": name, "genotypeId": data["motion"]["genotypeId"],
                                             "assemblyId": data["motion"]["assemblyId"]})
            summary["counts"].update(realizationAttempts=len(summary["generated"]), exported=len(summary["generated"]))
        (output / "candidates.json").write_text(json.dumps(summary))
        return summary


def backward_result():
    measured = result()
    measured["metrics"].update(
        forwardMeters=-.2, eligibleForRanking=False,
        motorTurns=[-2.2], logicalFootCount=4,
        rankingRejectionReasons=["nonpositive-forward-motion", "nonrepeatable-forward-motion"])
    measured["collision"]["fullMotorCycle"] = True
    measured["gait"] = {"version": "gait-cycles-v1", "motorTurnsPerCycle": 1,
                        "completeCycles": 2, "requiredCycles": 2, "cycleCoverage": 2.2,
                        "repeatableForwardMotion": False, "nominalCycleSeconds": 2.094,
                        "durationSeconds": 20, "minimumCycleSpeedMetersPerSecond": -.02,
                        "cycleIntervals": [{"forwardMeters": -.1, "seconds": 5, "speedMetersPerSecond": -.02},
                                          {"forwardMeters": -.1, "seconds": 5, "speedMetersPerSecond": -.02}]}
    return measured


@pytest.mark.parametrize("problem", ["collision", "incomplete", "mixed", "nonfinite"])
def test_motor_reversal_feedback_requires_clean_consistent_backward_cycles(problem):
    measured = backward_result()
    assert discovery.backwards_only(measured)
    if problem == "collision":
        measured["metrics"]["rankingRejectionReasons"].append("self-interference")
    elif problem == "incomplete":
        measured["gait"]["completeCycles"] = 1
    elif problem == "mixed":
        measured["gait"]["cycleIntervals"][0]["forwardMeters"] = .1
    else:
        measured["metrics"]["forwardMeters"] = float("nan")
    assert not discovery.backwards_only(measured)


@pytest.mark.parametrize("screening_workers", [1, 2])
def test_backward_gait_generates_one_normal_control_only_repair_then_is_measured(tmp_path, monkeypatch, screening_workers):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=2, screening_workers=screening_workers)
    if screening_workers == 2:
        from legowalkers_sim import prescreen
        from test_prescreen import screened_fixture
        actual_pool = prescreen.ScreeningPool
        monkeypatch.setattr(prescreen, "ScreeningPool",
                            lambda *args: actual_pool(*args, evaluator=screened_fixture))
    calls = []

    class Direction(Proposals):
        def manual_edits(self, parent):
            return {"edits": [{"id": "reverse", "operation": "motor-direction"}]}

        def apply_edit(self, parent, output, edit, limit=2):
            assert edit == "reverse" and limit == 1
            summary = Proposals(actions=["duplicate"]).propose_native(parent, output, 0, 0, limit)
            destination = output / summary["generated"][0]["directory"]
            data = json.loads((destination / "manifest.json").read_text())
            before = data["genome"]["speed"]
            data["genome"]["speed"] = -before
            data["expected"]["drives"][0]["speed"] = -before
            data["motion"]["genotypeId"] = identity(data["genome"])
            (destination / "manifest.json").write_text(json.dumps(data))
            (destination / "realization.json").write_text(json.dumps(data["genome"]))
            summary["generated"][0].update(genotypeId=data["motion"]["genotypeId"],
                                           assemblyId=data["motion"]["assemblyId"])
            summary.update(operation="motor-direction",
                           changed=[{"path": "genome/speed", "before": [before], "after": [-before]}])
            (output / "candidates.json").write_text(json.dumps(summary))
            return summary

    def evaluate(checked, output, options):
        calls.append(checked.manifest["genome"]["speed"])
        return backward_result() if len(calls) == 1 else result(qualified=True)

    monkeypatch.setattr(native_episode, "evaluate", evaluate)
    discovery.search_discovery(archive, Direction(), source, None, EvaluationOptions())
    assert len(calls) == 2 and calls[1] == -calls[0]
    assert archive.counts() == {"complete": 2}
    assert archive.db.execute("SELECT count(*) FROM native_assemblies").fetchone()[0] == 1
    assert archive.db.execute("SELECT count(*) FROM native_actuations").fetchone()[0] == 2
    assert archive.db.execute("SELECT count(*) FROM births WHERE status='duplicate'").fetchone()[0] == 0
    proposal, = archive.db.execute("SELECT summary FROM native_proposals").fetchall()
    summary = json.loads(proposal["summary"])
    assert summary["polarityRepairOf"] == 1
    assert summary["operation"] == "motor-direction"
    assert discovery._polarity_pending(archive) is None
    archive.db.close()


def mock_constructor(tmp_path, monkeypatch, base):
    from legowalkers_sim import motion_carry
    binaries = {}
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        path = tmp_path / name
        path.write_bytes(name.encode())
        binaries[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    base["generatorBinarySha256"] = binaries
    monkeypatch.setattr(Proposals, "command", ["dotnet", str(tmp_path / "LegoWalkers.Cli.dll")], raising=False)
    monkeypatch.setattr(motion_carry, "_centering", lambda *args: (None, None))


def setup_discovery(tmp_path, monkeypatch, budget=3, rejected_checks=False, screening_workers=1, replacement=False):
    source, data = native_fixture(tmp_path)
    archive = Archive(tmp_path / "archive")
    base = protocol()
    mock_constructor(tmp_path, monkeypatch, base)
    if replacement:
        base["evaluation"] = {"short_gate": 2}
    archive.initialize(native_episode.native_protocol(base, source, discover=True, screening_workers=screening_workers),
                       budget, 42, False)
    calls = []

    def screen(batch, output, library, max_checks, payload_profile=None, observer=None):
        entries = json.loads((batch / "candidates.json").read_text())["generated"]
        reports = [
            {"candidate": str(batch / entry["directory"]),
             "status": "rejected" if rejected_checks and entry["genotypeId"] != data["motion"]["genotypeId"] else "ready-for-dynamics"}
            for entry in entries]
        if observer:
            for entry, report in zip(entries, reports, strict=True):
                observer(entry["genotypeId"])
                observer(entry["genotypeId"], report)
        return {"counts": {"reconstructionAttempts": len(entries)}, "results": reports}

    def evaluate(checked, output, options):
        calls.append(checked.manifest["motion"]["genotypeId"])
        assert archive.counts().get("running") == 1
        measured = result(qualified=True)
        measured["metrics"]["speedMetersPerSecond"] = len(calls)
        measured["metrics"]["logicalFootCount"] = len(checked.manifest["feet"])
        return measured

    monkeypatch.setattr(discovery, "screen_batch", screen)
    monkeypatch.setattr(native_episode, "check", lambda path, library: SimpleNamespace(manifest=load_manifest(path), report={}))
    monkeypatch.setattr(native_episode, "mass_layout_gate", lambda *args: {"gate": {"outcome": "pass"}})
    monkeypatch.setattr(native_episode, "evaluate", evaluate)
    return source, data, archive, calls


@pytest.mark.parametrize("workers", [1, 2])
def test_discovery_generates_bounded_replacements_and_pauses_without_spending_full_allowance(tmp_path, monkeypatch, workers):
    from test_simulation_gate import rejected_or_complete

    source, data, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=4, replacement=True)
    native_episode.stage_native(archive, source)
    archive.finish(archive.reserve(1), result(qualified=True))
    monkeypatch.setattr(native_episode, "check", lambda path, library: SimpleNamespace(
        manifest=load_manifest(path), report={}, reject=True))
    monkeypatch.setattr(native_episode, "evaluate", rejected_or_complete)
    generator = Proposals()
    try:
        measured = discovery.search_discovery(archive, generator, source, tmp_path,
                                              EvaluationOptions(short_gate=2), workers=workers)
        assert measured["status"] == "gate-limit" and measured["failures"] == 0
        assert measured["cheapRejections"] == 9
        assert archive.counts() == {"complete": 1}
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 4
        assert archive.db.execute("SELECT COUNT(*) FROM simulation_attempts").fetchone()[0] == 9
        assert len(generator.calls) == 9
        assert archive.db.execute("SELECT status FROM simulation_gate_rounds").fetchone()[0] == "limit"
    finally:
        archive.db.close()


def test_native_parent_offspring_selection_and_immutable_motion_lineage(tmp_path, monkeypatch):
    source, data, archive, calls = setup_discovery(tmp_path, monkeypatch)
    generator = Proposals()
    try:
        native_episode.stage_native(archive, source)
        assert archive.counts() == {} and not calls
        measured = discovery.search_discovery(archive, generator, source, tmp_path, EvaluationOptions())
        assert measured["evaluations"] == {"complete": 3} and len(generator.calls) == 2
        births = archive.db.execute("SELECT * FROM births ORDER BY id").fetchall()
        assert births[0]["parent"] is None and births[1]["parent"] == 1
        proposal = json.loads(archive.db.execute(
            "SELECT summary FROM native_proposals ORDER BY id DESC LIMIT 1").fetchone()[0])
        decision = proposal["parentSelection"]
        assert proposal["mutationSelection"] == {
            "policy": "uniform-applicable-families-v1", "selectedFamily": proposal["operation"],
            "availableFamilies": [{"operation": "input-phase", "edits": 2}]}
        assert births[2]["parent"] == decision["selectedBirth"]
        assert [row["operation"] for row in births] == ["import-motion-walker", "input-phase", "input-phase"]
        assert json.loads(births[1]["changes"])[0]["path"] == "motor/phase"
        with pytest.raises(sqlite3.IntegrityError, match="provenance is immutable"):
            archive.db.execute("UPDATE births SET changes='[]' WHERE id=2")
        archive.db.rollback()
        with pytest.raises(sqlite3.IntegrityError, match="inputs are immutable"):
            archive.db.execute("UPDATE native_candidates SET batch='tampered' WHERE birth=2")
        archive.db.rollback()
        assert generator.calls[1][0].parent.name == births[births[2]["parent"]-1]["design"]
        assert len(set(calls)) == 3 and calls[0] == data["motion"]["genotypeId"]
        assert [row[0] for row in archive.db.execute(
            "SELECT stage FROM pipeline_events WHERE birth=1 ORDER BY id")] == [
                "planning", "building", "screening", "screened", "preflight", "ready"]
        assert archive.db.execute("SELECT count(*) FROM native_proposals WHERE status='generated'").fetchone()[0] == 2
        page = (archive.directory / "index.html").read_text()
        assert "Native motion proposals" in page and "Enabled mutations:" in page
        assert "add dyad" in page and "remove dyad" in page
        lineage = (archive.directory / "history/0003.html").read_text()
        assert "Recorded motion edits" in lineage and "candidate/program.json" in lineage
        discovery.search_discovery(archive, generator, source, tmp_path, EvaluationOptions())
        assert len(calls) == 3
    finally:
        archive.db.close()


def test_cheap_rejections_physical_duplicates_and_siblings_spend_no_extra_slots(tmp_path, monkeypatch):
    source, _, archive, calls = setup_discovery(tmp_path, monkeypatch)
    generator = Proposals(["reject", "duplicate"], siblings=2)
    try:
        discovery.search_discovery(archive, generator, source, tmp_path, EvaluationOptions())
        assert len(calls) == 3 and len(generator.calls) == 3
        assert archive.db.execute("SELECT count(*) FROM births WHERE status='duplicate'").fetchone()[0] == 1
        assert archive.db.execute("SELECT count(*) FROM native_proposals WHERE status='rejected'").fetchone()[0] == 1
        assert archive.db.execute("SELECT count(*) FROM native_assemblies").fetchone()[0] == 3
        assert all(row[0] < 8 for row in archive.db.execute("SELECT sum(allowance) FROM native_checks GROUP BY after_trial"))
    finally:
        archive.db.close()


@pytest.mark.parametrize("failure", ["proposals", "reconstruction"])
def test_native_discovery_bounds_are_persistent_and_pause_without_fabricated_trials(tmp_path, monkeypatch, failure):
    source, _, archive, calls = setup_discovery(tmp_path, monkeypatch, budget=2, rejected_checks=failure == "reconstruction")
    generator = Proposals(["reject"]*32 if failure == "proposals" else [])
    expected = 16 if failure == "proposals" else 8
    try:
        with pytest.raises(AdapterError, match="limit reached"):
            discovery.search_discovery(archive, generator, source, tmp_path, EvaluationOptions())
        assert archive.counts() == {"complete": 1} and len(calls) == 1
        assert archive.db.execute("SELECT status FROM run").fetchone()[0] == "paused"
        if failure == "proposals":
            assert len(generator.calls) == expected
        else:
            assert archive.db.execute("SELECT sum(allowance) FROM native_checks WHERE after_trial=1").fetchone()[0] == expected
        before = len(generator.calls)
        with pytest.raises(AdapterError, match="limit reached"):
            discovery.search_discovery(archive, generator, source, tmp_path, EvaluationOptions())
        assert len(calls) == 1 and len(generator.calls) == before
    finally:
        archive.db.close()


def test_native_interrupted_slot_and_proposal_are_not_replayed(tmp_path, monkeypatch):
    source, _, archive, calls = setup_discovery(tmp_path, monkeypatch, budget=1)
    try:
        native_episode.stage_native(archive, source)
        birth = archive.db.execute("SELECT id FROM births").fetchone()[0]
        archive.reserve(birth)
        archive.db.execute("""INSERT INTO native_proposals(parent,seed,attempt,after_trial,status,created)
                              VALUES (?,42,0,0,'running','unit-test')""", (birth,))
        archive.db.commit()
        discovery.search_discovery(archive, Proposals(), source, tmp_path, EvaluationOptions())
        assert not calls and archive.counts() == {"interrupted": 1}
        assert archive.db.execute("SELECT status FROM native_proposals").fetchone()[0] == "interrupted"
    finally:
        archive.db.close()


def test_native_batch_tampering_is_rejected_before_reserving_trial(tmp_path, monkeypatch):
    source, data, archive, calls = setup_discovery(tmp_path, monkeypatch)
    try:
        native_episode.stage_native(archive, source)
        destination = archive.directory / "native-input/candidate-0/program.json"
        destination.write_text('{"tampered":true}')
        with pytest.raises(AdapterError, match="Archived native inputs changed"):
            discovery.search_discovery(archive, Proposals(), source, tmp_path, EvaluationOptions())
        assert archive.counts() == {} and not calls
    finally:
        archive.db.close()


def test_generator_does_not_treat_broken_command_as_cheap_rejection(tmp_path, monkeypatch):
    generator = Generator("dotnet", "generator.dll")
    monkeypatch.setattr("legowalkers_sim.evolution.subprocess.run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=1, stderr="compiler failure"))
    with pytest.raises(AdapterError, match="Native F# proposal failed"):
        generator.propose_native(tmp_path, tmp_path / "proposal", 42, 0, 2)


@pytest.mark.parametrize("returncode,changed", [(1, True), (0, True), (1, False)])
def test_generator_accepts_only_a_consistent_gate_rejection(tmp_path, monkeypatch, returncode, changed):
    generator = Generator("dotnet", "generator.dll")
    batch = tmp_path / "proposal"
    batch.mkdir()
    summary = {"schemaVersion": 1, "mutationVersion": discovery.POLICY["mutationVersion"],
               "seed": 42, "attempt": 0, "changed": [{"path": "motor/phase"}] if changed else [],
               "generated": [], "rejected": [], "counts": {"dynamicTrials": 0, "exported": 0},
               "gates": [{"gate": "mobility", "outcome": "reject"}]}
    (batch / "candidates.json").write_text(json.dumps(summary))
    monkeypatch.setattr("legowalkers_sim.evolution.subprocess.run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=returncode, stderr=""))
    if returncode == 1 and changed:
        assert generator.propose_native(tmp_path, batch, 42, 0, 2) == summary
    else:
        with pytest.raises(AdapterError, match="inconsistent"):
            generator.propose_native(tmp_path, batch, 42, 0, 2)


def test_discovery_dashboard_requires_explicit_twelve_trial_authorization(tmp_path, monkeypatch):
    from legowalkers_sim.dashboard import EpisodeController
    source, _ = native_fixture(tmp_path)
    native = native_episode.native_protocol(protocol(), source, discover=True)
    launched = []
    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen",
                        lambda *args, **kwargs: launched.append(args) or SimpleNamespace(pid=123456, poll=lambda: None))
    controller = EpisodeController(tmp_path / "dashboard", native, ["test-worker", "--discover"])
    assert not launched and controller.status()["budget"] == 0
    assert controller.status()["nextEpisodeTrials"] == 12 and not controller.status()["finiteQueue"]
    assert controller.start()["budget"] == 12 and len(launched) == 1


@pytest.mark.integration
def test_actual_motion_mutation_and_two_physical_realizations_load_without_stepping(tmp_path, monkeypatch):
    library = os.environ.get("LDRAW_LIBRARY")
    if not library:
        pytest.skip("Set LDRAW_LIBRARY for actual mutated walker reconstruction.")
    generator = Generator(ROOT / ".tools/dotnet/dotnet",
                          ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    founder = tmp_path / "founder"
    generator.invoke("realize-walker", "--feet", 4, "--steps", 72, "--output", founder)
    batch = tmp_path / "offspring"
    parent = founder / "candidate-001"
    edit = next(edit for edit in generator.manual_edits(parent)["edits"]
                if edit["operation"] == "move-hinge-site")
    summary = generator.apply_edit(parent, batch, edit["id"])
    assert summary["operation"] == "move-hinge-site" and summary["changed"][0]["before"] != summary["changed"][0]["after"]
    assert summary["counts"]["exported"] == 2 and summary["counts"]["dynamicTrials"] == 0
    entries, _ = native_episode._queue(batch)
    assert len({entry["genotypeId"] for entry in entries["generated"]}) == 2
    assert len({entry["assemblyId"] for entry in entries["generated"]}) == 2
    import mujoco
    monkeypatch.setattr(mujoco, "mj_step", lambda *args, **kwargs: pytest.fail("Offspring load-only screening must not step physics."))
    screened = screening.screen_batch(batch, tmp_path / "screened", Path(library), max_checks=2)
    assert screened["counts"]["ready"] == 2, json.dumps(screened, indent=2)
    assert screened["counts"]["dynamicTrials"] == 0
    for report in screened["results"]:
        assert report["logicalFootCount"] == 4 and report["compatibility"]["motors"] == 1
        assert report["loadOnly"]["simulationSeconds"] == 0
        assert report["loadOnly"]["maxClosureErrorMeters"] < 1e-5
        assert report["loadOnly"]["initialSelfInterference"] == []
