import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest
from lego_technic_sim.physics.model import Unit

from legowalkers_sim import discovery, native_episode, prescreen, screening
from legowalkers_sim.bridge import AdapterError, ConstructionConnectivityError, compare_scene, load_manifest, write_json
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import Generator, identity
from legowalkers_sim.screening_reports import motion_repair_hints, repair_hints
from test_bridge import scene_data
from test_native_discovery import Proposals, setup_discovery
from test_native_episode import native_fixture
from test_screening import batch_fixture, mock_reconstruction


HINTS = {"kind": "rigid-body-partition", "parts": ["beam-b"], "bodies": ["chassis"]}

def initial_ground_report():
    return {
        "version": "simulation-prefix-gate-v1", "status": "rejected",
        "measurementSeconds": 0, "reasons": ["forbidden-ground-contact"],
        "diagnostics": {"groundContact": {
            "forbiddenContactDetected": True, "footContactSamples": 0, "wheelContactSamples": 0,
            "forbiddenContacts": [{"part": "lower-link"}, {"part": "parallel-link"}],
        }},
    }


def test_nonfoot_touchdown_before_any_support_targets_a_stock_fit_repair():
    assert motion_repair_hints(initial_ground_report()) == {
        "kind": "initial-foot-clearance", "parts": ["lower-link", "parallel-link"], "bodies": []}


@pytest.mark.parametrize("change", ["measured", "feet", "wheel", "passed", "missing-parts"])
def test_initial_foot_repair_does_not_generalize_to_balance_or_other_failures(change, caplog):
    report = initial_ground_report()
    if change == "measured":
        report["measurementSeconds"] = .1
    elif change == "feet":
        report["diagnostics"]["groundContact"]["footContactSamples"] = 1
    elif change == "wheel":
        report["diagnostics"]["groundContact"]["wheelContactSamples"] = 1
    elif change == "passed":
        report["status"] = "passed"
    else:
        report["diagnostics"]["groundContact"]["forbiddenContacts"] = [{}]
    assert motion_repair_hints(report) is None
    if change == "missing-parts":
        assert caplog.records


def test_partition_diagnostic_identifies_unexpected_parts_without_changing_physics_bridge():
    manifest, build, scene = scene_data()
    scene.units = [Unit([build.parts[0], build.parts[1]])]
    with pytest.raises(AdapterError, match="Rigid-body partition mismatch") as caught:
        compare_scene(manifest, build, scene)
    assert repair_hints({"error": str(caught.value)}) == HINTS


@pytest.mark.parametrize("detected, targets", [
    ("{'0': ['rail', 'anchor'], '1': ['socket']}", ["socket"]),
    ("{'0': ['rail'], '1': ['socket']}", ["rail", "socket"]),
    ("{'0': ['rail', 'anchor']}", ["socket"]),
])
def test_split_and_missing_expected_parts_are_precise_repair_targets(detected, targets):
    expected = "['rail', 'socket']" if targets == ["rail", "socket"] else "['rail', 'anchor', 'socket']"
    diagnostic = f"Rigid-body partition mismatch for chassis: expected {expected}, detected {detected}"
    assert repair_hints({"error": diagnostic}) == {
        "kind": "rigid-body-partition", "parts": targets, "bodies": ["chassis"]}

def test_cached_partition_rejection_preserves_structured_repair_targets(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path, count=1)
    mock_reconstruction(monkeypatch)

    def reject(*args):
        raise AdapterError("Rigid-body partition mismatch for chassis: expected ['beam-a'], detected {'0': ['beam-a', 'beam-b']}")

    monkeypatch.setattr(screening, "check", reject)
    output = tmp_path / "screening"
    first = screening.screen_batch(source, output, tmp_path, max_checks=1)
    second = screening.screen_batch(source, output, tmp_path, max_checks=1)
    assert repair_hints(first["results"][0]) == repair_hints(second["results"][0]) == HINTS
    assert second["counts"]["reconstructionAttempts"] == 0
    assert second["results"][0]["previousStatus"] == "rejected"

def test_exact_bond_failure_and_targets_survive_screening_and_cache(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path, count=1)
    mock_reconstruction(monkeypatch)
    evidence = {"status": "reject", "scope": "frozen", "chassis": "chassis", "verifiedGoals": [],
                "failedBonds": [{"connector": "pin", "partA": "anchor", "partB": "socket"}],
                "disconnectedParts": ["socket"]}

    def reject(*args):
        raise ConstructionConnectivityError(evidence)

    monkeypatch.setattr(screening, "check", reject)
    output = tmp_path / "screening"
    first = screening.screen_batch(source, output, tmp_path, max_checks=1)
    second = screening.screen_batch(source, output, tmp_path, max_checks=1)
    for report in (first["results"][0], second["results"][0]):
        assert report["constructionConnectivity"] == evidence
        assert report["repairHints"]["parts"] == ["anchor", "pin", "socket"]
    assert first["counts"]["dynamicTrials"] == second["counts"]["reconstructionAttempts"] == 0

def test_successful_connection_evidence_survives_a_later_partition_failure(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path, count=1)
    mock_reconstruction(monkeypatch)
    evidence = {"status": "pass", "scope": "frozen", "verifiedGoals": ["library-connectivity"]}
    preservation = {"status": "pass", "scope": "frozen", "verifiedGoals": ["preserve-pivot"]}

    def reject(*args):
        raise AdapterError(
            "Rigid-body partition mismatch for chassis: expected ['beam-a'], detected {'0': ['beam-a', 'beam-b']}",
            construction_connectivity=evidence, joint_preservation=preservation)

    monkeypatch.setattr(screening, "check", reject)
    result = screening.screen_batch(source, tmp_path / "screening", tmp_path, max_checks=1)
    report = result["results"][0]
    assert report["status"] == "rejected"
    assert report["constructionConnectivity"] == evidence
    assert report["jointPreservation"] == preservation
    assert report["repairHints"] == HINTS


def test_joint_engagement_targets_survive_screening_and_cache(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path, count=1)
    mock_reconstruction(monkeypatch)
    hints = {"kind": "joint-port-engagement", "parts": ["gear", "pivot-link"], "bodies": ["link", "brace"]}
    evidence = {"scope": "frozen", "status": "reject", "verifiedGoals": [], "pivots": []}

    def reject(*args):
        raise AdapterError("Shared pivot reconstruction mismatch: link.", repair_hints=hints, joint_preservation=evidence)

    monkeypatch.setattr(screening, "check", reject)
    output = tmp_path / "screening"
    first = screening.screen_batch(source, output, tmp_path, max_checks=1)
    second = screening.screen_batch(source, output, tmp_path, max_checks=1)
    assert first["results"][0]["repairHints"] == second["results"][0]["repairHints"] == hints
    assert first["results"][0]["jointPreservation"] == second["results"][0]["jointPreservation"] == evidence
    assert second["counts"]["reconstructionAttempts"] == 0


@pytest.mark.parametrize("diagnostic", [
    "Rigid-body partition mismatch for chassis: expected ['beam-a'], detected {'0': 3}",
    "Rigid-body partition mismatch for chassis: expected no-membership-data",
])
def test_unrecognized_partition_fields_are_logged_and_not_guessed(diagnostic, caplog):
    assert repair_hints({"error": diagnostic}) is None
    assert caplog.records


def feedback_report(candidate, manifest):
    return {"candidate": str(candidate), "genotypeId": manifest["motion"]["genotypeId"],
            "status": "rejected", "error": "Partition mismatch fixture.",
            "repairHints": HINTS, "dynamicTrials": 0, "simulationSeconds": 0}


class Repairs(Proposals):
    def __init__(self):
        self.calls = []

    def repair_native(self, parent, output, feedback, limit):
        self.calls.append(parent)
        assert limit == 1 and json.loads(feedback.read_text()) == HINTS
        data = load_manifest(parent / "manifest.json")
        before = data["motion"]["genotypeId"]
        name = "candidate-001"
        destination = output / name
        shutil.copytree(parent, destination)
        for body in data["genome"]["bodies"]:
            if body["body"] == "link":
                for part in body["parts"]:
                    part["position"]["z"] += 10
        for foot in data["genome"]["feet"]:
            foot["depth"] += 10
        for part in data["parts"]:
            if part["body"] == "link":
                part["position"][2] += 10
        for foot in data["feet"]:
            foot["position"][2] += 10
        data["motion"]["genotypeId"] = identity(data["genome"])
        data["motion"]["assemblyId"] = identity({"assembly": data["genome"]})
        write_json(destination / "manifest.json", data)
        write_json(destination / "realization.json", data["genome"])
        shutil.copyfile(feedback, output / "feedback.json")
        summary = {
            "schemaVersion": 1, "gateVersion": "motion-gates-v1",
            "mechanismId": data["motion"]["mechanismId"], "parentGenotypeId": before,
            "constructionVersion": discovery.POLICY["construction"]["version"],
            "operation": "repair-realization", "changed": [{"path": "construction", "before": [0], "after": [10]}],
            "gates": [], "rejected": [],
            "generated": [{"directory": name, "assemblyId": data["motion"]["assemblyId"],
                           "genotypeId": data["motion"]["genotypeId"]}],
            "counts": {"proposals": 1, "realizationAttempts": 1, "exported": 1,
                       "rejected": 0, "duplicates": 0, "dynamicTrials": 0},
        }
        write_json(output / "candidates.json", summary)
        return summary


@pytest.mark.parametrize("invalid", [None, "motion", "parent", "version", "attempts", "trials"])
def test_generator_validates_same_motion_parent_and_bounded_repair_summary(tmp_path, monkeypatch, invalid):
    source, _ = native_fixture(tmp_path)
    feedback = tmp_path / "feedback.json"
    write_json(feedback, HINTS)
    batch = tmp_path / "repair"
    summary = Repairs().repair_native(source / "candidate-0", batch, feedback, 1)
    if invalid == "motion":
        summary["mechanismId"] = "changed"
    elif invalid == "parent":
        summary["parentGenotypeId"] = "wrong-parent"
    elif invalid == "version":
        summary["constructionVersion"] = "unknown"
    elif invalid == "attempts":
        summary["counts"]["realizationAttempts"] = 2
    elif invalid == "trials":
        summary["counts"]["dynamicTrials"] = 1
    write_json(batch / "candidates.json", summary)
    monkeypatch.setattr("legowalkers_sim.evolution.subprocess.run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=0, stderr=""))
    generator = Generator("dotnet", "generator.dll")
    if invalid:
        with pytest.raises(AdapterError, match="inconsistent bounded construction"):
            generator.repair_native(source / "candidate-0", batch, feedback, 1)
    else:
        assert generator.repair_native(source / "candidate-0", batch, feedback, 1) == summary


def mark_rejected(archive, birth):
    row = archive.db.execute("SELECT design FROM births WHERE id=?", (birth,)).fetchone()
    candidate = archive.directory / "designs" / row["design"] / "candidate"
    report = feedback_report(candidate, load_manifest(candidate / "manifest.json"))
    write_json(archive.directory / "preflight" / str(birth) / "screen-0001.json", {"results": [report]})
    with archive.db:
        archive.db.execute("UPDATE births SET status='screening-rejected' WHERE id=?", (birth,))


def test_collision_feedback_preserves_pairs_even_with_legacy_hints():
    pairs = [["rail", "pin"], ["beam", "pin"]]
    hints = repair_hints({
        "repairHints": {"kind": "initial-self-interference", "parts": ["beam", "pin", "rail"], "bodies": []},
        "loadOnly": {"initialSelfInterference": [{"parts": pair, "depthMeters": .001} for pair in pairs]},
    })
    assert hints["pairs"] == pairs
    assert hints["parts"] == ["beam", "pin", "rail"]


@pytest.mark.parametrize("valid_version", [True, False])
@pytest.mark.parametrize("kind", ["initial-self-interference", "motion-self-interference"])
def test_collision_repairs_invoke_the_promoted_grammar_with_exact_pairs(tmp_path, monkeypatch, valid_version, kind):
    source, _ = native_fixture(tmp_path)
    feedback = tmp_path / "feedback.json"
    write_json(feedback, HINTS)
    output = tmp_path / "repair"
    summary = Repairs().repair_native(source / "candidate-0", output, feedback, 1)
    summary["grammarVersion"] = discovery.POLICY["construction"]["grammar"]["version"] if valid_version else "old"
    write_json(output / "candidates.json", summary)
    hints = {"kind": kind, "parts": ["rail", "pin"], "bodies": [],
             "pairs": [["rail", "pin"]]}
    write_json(feedback, hints)

    def invoke(command, **kwargs):
        assert command[2] == "construct-walker"
        conflicts = Path(command[command.index("--conflicts") + 1])
        selected = Path(command[command.index("--feedback") + 1])
        assert json.loads(conflicts.read_text()) == {"pairs": hints["pairs"]}
        assert json.loads(selected.read_text()) == {key: value for key, value in hints.items() if key != "pairs"}
        return SimpleNamespace(returncode=0, stderr="")

    monkeypatch.setattr("legowalkers_sim.evolution.subprocess.run", invoke)
    generator = Generator("dotnet", "generator.dll")
    if valid_version:
        assert generator.repair_native(source / "candidate-0", output, feedback, 1) == summary
    else:
        with pytest.raises(AdapterError, match="inconsistent bounded construction"):
            generator.repair_native(source / "candidate-0", output, feedback, 1)
    assert json.loads(feedback.read_text()) == hints


def moving_report():
    return {"version": "simulation-prefix-gate-v1", "status": "rejected", "reasons": ["self-interference"],
            "diagnostics": {"selfContacts": [
                {"parts": ["rail", "pin"], "depthMeters": .000101, "time": .504},
                {"parts": ["pin", "rail"], "depthMeters": .001, "time": 1},
                {"parts": ["noise", "pin"], "depthMeters": .000001, "time": .002}]}}


def test_moving_feedback_uses_measured_significant_pairs_not_numerical_contacts():
    hints = motion_repair_hints(moving_report())
    assert hints == {"kind": "motion-self-interference", "parts": ["pin", "rail"], "bodies": [],
                     "pairs": [["pin", "rail"]]}
    measured = {"metrics": {"rankingRejectionReasons": ["self-interference"]},
                "collision": moving_report()["diagnostics"]}
    assert motion_repair_hints(measured) == hints
    assert motion_repair_hints({**moving_report(), "status": "passed"}) is None
    assert motion_repair_hints({**moving_report(), "reasons": ["forbidden-ground-contact"]}) is None


@pytest.mark.parametrize("contact", [
    {"parts": ["rail", "rail"], "depthMeters": .001},
    {"parts": ["rail", "pin"], "depthMeters": float("nan")},
    {"parts": ["rail", "pin"], "depthMeters": True},
    {"parts": ["rail"], "depthMeters": .001},
    None,
])
def test_malformed_moving_contact_evidence_is_logged_not_guessed(contact, caplog):
    report = moving_report()
    report["diagnostics"]["selfContacts"] = [contact]
    assert motion_repair_hints(report) is None
    assert caplog.records


@pytest.mark.parametrize("full_trial,stance,initial_ground", [
    (False, False, False), (True, False, False),
    (False, True, False), (True, True, False), (False, False, True),
])
def test_moving_rejections_enter_bounded_repair_selection_without_inheriting_fitness(
        tmp_path, monkeypatch, full_trial, stance, initial_ground):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=1)
    try:
        native_episode.stage_native(archive, source)
        report = initial_ground_report() if initial_ground else moving_report()
        if stance:
            from test_stance_support import trace
            recorded = trace()
            recorded["ldrawSha256"] = json.loads((source / "candidate-0/manifest.json").read_text())["ldrawSha256"]
            report["reasons"] = ["forbidden-ground-contact"]
            report["diagnostics"].update(
                stanceSupport={"version": recorded["version"], "status": "measured", "unsupportedFraction": .2},
                groundContact={"forbiddenContactDetected": True})
        if full_trial:
            trial = archive.reserve(1)
            from test_evolution import result
            measured = result()
            measured["metrics"]["rankingRejectionReasons"] = report["reasons"]
            measured["collision"].update(report["diagnostics"])
            if stance:
                measured.update(stanceSupport=report["diagnostics"]["stanceSupport"],
                                groundContact=report["diagnostics"]["groundContact"])
            archive.finish(trial, measured)
            with archive.db:
                archive.db.execute("UPDATE births SET status='evaluated' WHERE id=1")
        else:
            from legowalkers_sim.parallel import ensure_attempt_schema
            ensure_attempt_schema(archive.db)
            with archive.db:
                archive.db.execute("INSERT INTO simulation_gate_rounds(id,full_budget,attempt_limit,created) VALUES (1,1,4,'now')")
                archive.db.execute("INSERT INTO simulation_attempts(id,birth,round,slot,status,started) VALUES (1,1,1,1,'rejected','now')")
                archive.db.execute("UPDATE births SET status='simulation-rejected' WHERE id=1")
            write_json(archive.directory / "simulation-attempts/0001/simulation-gate.json", report)
        expected = motion_repair_hints(report)
        if stance:
            relative = ("evaluations" if full_trial else "simulation-attempts") + "/0001/stance-support.json"
            write_json(archive.directory / relative, recorded)
            write_json((archive.directory / relative).with_name("mass-layout.json"), {
                "totalMassKg": 1., "components": [{"partInstance": "payload-battery", "massKg": .5}]})
            expected["supportTrace"] = relative
        repair = discovery._repair_pending(archive)
        assert repair["birth"] == 1 and repair["depth"] == 0
        assert repair["repairHints"] == expected
        before = archive.counts()
        generator = Repairs()
        def apply(parent, output, feedback, limit):
            assert json.loads(feedback.read_text()) == repair["repairHints"]
            partition_feedback = tmp_path / "partition-fixture.json"
            write_json(partition_feedback, HINTS)
            return generator.repair_native(parent, output, partition_feedback, limit)
        discovery._propose(archive, SimpleNamespace(repair_native=apply), sum(before.values()), 1)
        assert archive.counts() == before
        assert discovery._repair_pending(archive) is None
        child = archive.db.execute("SELECT parent,status,operation FROM births WHERE id=2").fetchone()
        assert tuple(child) == (1, "generated", "repair-realization")
        assert archive.db.execute("SELECT count(*) FROM evaluations WHERE birth=2").fetchone()[0] == 0
    finally:
        archive.db.close()


def test_repairs_are_one_per_rejected_parent_depth_bounded_and_immutable(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=1)
    generator = Repairs()
    try:
        native_episode.stage_native(archive, source)
        mark_rejected(archive, 1)
        assert discovery._neighborhood(archive) == 1
        discovery._propose(archive, generator, 0, 1)
        assert discovery._repair_pending(archive) is None
        mark_rejected(archive, 2)
        assert discovery._repair_pending(archive)["depth"] == 1
        discovery._propose(archive, generator, 0, 1)
        mark_rejected(archive, 3)
        assert discovery._repair_pending(archive) is None
        rows = archive.db.execute("SELECT parent,operation FROM births ORDER BY id").fetchall()
        assert [(row["parent"], row["operation"]) for row in rows[1:]] == [
            (1, "repair-realization"), (2, "repair-realization")]
        pending = archive.db.execute("SELECT files,batch FROM native_candidates WHERE birth=2").fetchone()
        files = json.loads(pending["files"])
        assert "feedback.json" in files
        (archive.directory / pending["batch"] / "feedback.json").write_text("{}")
        with pytest.raises(AdapterError, match="immutable birth"):
            discovery._screen_pending(
                archive, discovery._pending(archive) or {"batch": pending["batch"], "files": pending["files"]},
                tmp_path, 0)
        assert archive.counts() == {}
        assert archive.db.execute("SELECT count(*) FROM native_checks").fetchone()[0] == 0
    finally:
        archive.db.close()


def test_repairs_do_not_bypass_the_existing_proposal_cap(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=1)
    generator = Repairs()
    monkeypatch.setitem(discovery.POLICY, "maxProposalsPerTrial", 1)
    try:
        native_episode.stage_native(archive, source)
        mark_rejected(archive, 1)
        discovery._propose(archive, generator, 0, 1)
        mark_rejected(archive, 2)
        with pytest.raises(AdapterError, match="proposal limit"):
            discovery._propose(archive, generator, 0, 1)
        assert len(generator.calls) == 1 and archive.counts() == {}
    finally:
        archive.db.close()


def screened_feedback_fixture(batch, output, design, manifest, library, options, stage):
    data = load_manifest(manifest)
    if int(output.name) == 1:
        write_json(output / "test-protocol" / "screen-0001.json",
                   {"results": [feedback_report(Path(manifest).parent, data)]})
        return {"status": "screening-rejected", "checks": 1, "error": "Partition mismatch fixture."}
    stage("preflight")
    return {"status": "ready", "checks": 1, "checked": SimpleNamespace(manifest=data, report={})}


@pytest.mark.parametrize("workers", [1, 2])
def test_dispatch_repairs_rejected_founder_before_abandoning_its_motion(tmp_path, monkeypatch, workers):
    source, data, archive, calls = setup_discovery(tmp_path, monkeypatch, budget=1, screening_workers=workers)
    generator = Repairs()

    def screen(batch, output, library, max_checks, payload_profile=None, observer=None):
        summary = json.loads((batch / "candidates.json").read_text())
        entry = summary["generated"][0]
        candidate = batch / entry["directory"]
        manifest = load_manifest(candidate / "manifest.json")
        report = (feedback_report(candidate, manifest) if not generator.calls else
                  {"candidate": str(candidate), "genotypeId": entry["genotypeId"], "status": "ready-for-dynamics"})
        write_json(output / "screen-0001.json", {"results": [report]})
        return {"counts": {"reconstructionAttempts": 1}, "results": [report]}

    monkeypatch.setattr(discovery, "screen_batch", screen)
    actual_pool = prescreen.ScreeningPool
    monkeypatch.setattr(prescreen, "ScreeningPool",
                        lambda *args: actual_pool(*args, evaluator=screened_feedback_fixture))
    try:
        discovery.search_discovery(archive, generator, source, tmp_path, EvaluationOptions())
        assert archive.counts() == {"complete": 1}
        assert len(generator.calls) == len(calls) == 1
        assert archive.db.execute("SELECT status FROM births WHERE id=1").fetchone()[0] == "screening-rejected"
        assert archive.db.execute("SELECT birth FROM evaluations").fetchone()[0] == 2
        assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 2
        assert load_manifest(generator.calls[0] / "manifest.json")["motion"]["mechanismId"] == data["motion"]["mechanismId"]
        page = (archive.directory / "history/0002.html").read_text()
        assert "Recorded construction edits" in page
        assert "Screening feedback used for this repair" in page and "feedback.json" in page
    finally:
        archive.db.close()
