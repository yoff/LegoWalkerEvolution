import copy
import json
import shutil
from types import SimpleNamespace

import pytest

from legowalkers_sim import discovery, native_episode
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.campaign import definition, observation_sources, version_html
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import Archive, evidence_invalidation, fitness, is_qualified, trusted_result
from legowalkers_sim.gallery import display_result
from legowalkers_sim.observations import compatible_evaluation
from legowalkers_sim.pipeline import qualified_walkers
from legowalkers_sim.proposal_forest import nodes
from legowalkers_sim.simulation_gate import SimulationGateRejected
from legowalkers_sim import leg_family_migration, upgrades
from test_campaign import setup
from test_evolution import result
from test_native_discovery import Proposals, setup_discovery
from test_native_episode import native_fixture


OLD_REVISION = "7db4596fe7f970fadc7b53893be32b4b93249d9e"


@pytest.mark.parametrize("qualified", [True, False])
def test_invalidated_results_are_not_current_evidence_and_raw_results_survive(tmp_path, qualified):
    source, data = native_fixture(tmp_path)
    protocol = native_episode.native_protocol({"simulatorRevision": OLD_REVISION}, source, discover=True)
    archive = Archive(tmp_path / "legacy")
    try:
        archive.initialize(protocol, 1, 42, False)
        native_episode.stage_native(archive, source)
        measured = result(qualified=qualified)
        archive.finish(archive.reserve(1), measured)
        original = archive.db.execute("SELECT result FROM evaluations").fetchone()[0]
        adjusted = display_result({"result": original, "genome": json.dumps(data["genome"])}, protocol)
        assert adjusted["evidenceInvalidation"]
        assert adjusted["metrics"]["eligibleForRanking"] is False
        assert "invalidated-mass-properties" in adjusted["metrics"]["rankingRejectionReasons"]
        assert not is_qualified(adjusted)
        assert fitness(adjusted) < fitness(result())
        assert archive.outcomes() == []
        assert archive.best() is None
        assert qualified_walkers(archive.db, archive.directory) == []
        assert nodes(archive.db)[0].state == "invalidated"
        from legowalkers_sim.motif_opportunities import simulation_contacts
        contacts = simulation_contacts([{"id": "legacy", "archive": str(archive.directory)}])
        assert contacts["groups"] == []
        assert contacts["counts"]["invalidatedRecords"] == 1
        assert "excluded" in contacts["warnings"][0]
        archive.db.execute("""CREATE TABLE user_pool_entries(
            contribution TEXT,birth INTEGER,status TEXT,error TEXT)""")
        archive.db.execute("INSERT INTO user_pool_entries VALUES ('contribution',1,'eligible',NULL)")
        archive.db.commit()
        from legowalkers_sim.user_pool import status
        assert status(archive.directory)[0]["measurement"] == "invalidated; corrected simulation required"
        assert archive.db.execute("SELECT result FROM evaluations").fetchone()[0] == original
        assert measured == json.loads(original)
        assert archive.counts() == {"complete": 1}
    finally:
        archive.db.close()


def test_corrected_results_remain_qualified_and_legacy_cannot_be_reused(tmp_path):
    corrected = {"simulatorRevision": "1b70ff4b6c567b3c8541d5ef8ff4c56b4be24a5b"}
    measured = result(qualified=True)
    assert evidence_invalidation(corrected) is None
    assert trusted_result(measured, corrected) is measured
    assert is_qualified(trusted_result(measured, corrected))
    old = {"simulatorRevision": OLD_REVISION}
    assert compatible_evaluation(old, old, tmp_path) is False
    assert compatible_evaluation(old, corrected, tmp_path) is False


def test_historical_page_explains_invalidation_without_altering_saved_content():
    original = ('<html><body><span class="pill good">Best tested qualified walker</span>'
                '<table><tr><td>qualified</td></tr></table><video src="walk.mp4"></video></body></html>')
    html = version_html(original, "/versions/v0048/", readonly=True,
                        invalidation=evidence_invalidation({"simulatorRevision": OLD_REVISION}))
    assert "data-evidence-invalidation" in html
    assert "Invalidated historical qualification" in html
    assert "<td>invalidated</td>" in html
    assert 'src="/versions/v0048/walk.mp4"' in html
    assert "Best tested qualified walker" in original


@pytest.mark.parametrize("rejected", [True, False])
def test_requalification_checks_only_requested_walkers_then_stops(tmp_path, monkeypatch, rejected):
    from legowalkers_sim import user_pool
    source, _, original, _ = setup_discovery(tmp_path, monkeypatch, budget=3)
    targets = tmp_path / "targets"
    summary = Proposals(siblings=8).propose_native(source / "candidate-0", targets, 42, 0, 8)
    protocol = native_episode.native_protocol({**original.protocol(), "evaluation": {"short_gate": 2}},
                                             targets, discover=True)
    protocol["requalificationCandidates"] = [entry["genotypeId"] for entry in summary["generated"]]
    archive = Archive(tmp_path / "recheck")
    archive.initialize(protocol, 9, 42, False)
    calls = []

    def evaluate(checked, output, options, *, on_gate_passed):
        calls.append(checked.manifest["motion"]["genotypeId"])
        if rejected:
            raise SimulationGateRejected("forbidden ground contact")
        on_gate_passed()
        return result(qualified=True)

    monkeypatch.setattr(native_episode, "evaluate", evaluate)
    monkeypatch.setattr(user_pool, "sync", lambda *args, **kwargs: pytest.fail(
        "Unrelated contribution admission must not delay the targeted batch."))
    generator = Proposals()
    try:
        discovery.search_discovery(archive, generator, targets, tmp_path, EvaluationOptions(short_gate=2))
        assert calls == protocol["requalificationCandidates"]
        assert generator.calls == []
        assert archive.counts() == ({} if rejected else {"complete": 8})
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 9
        assert archive.db.execute("SELECT count(*) FROM simulation_attempts").fetchone()[0] == 8
        assert discovery.requalification_pending(archive) == []
    finally:
        archive.db.close()
        original.db.close()


def test_requalification_adds_allowance_once_and_does_not_reuse_requested_measurement(setup, tmp_path):
    campaign = setup.create()
    legacy = Archive(tmp_path / "pre-fix")
    source = setup.original / "native-input"
    try:
        old = copy.deepcopy(definition(setup.original))
        old["simulatorRevision"] = OLD_REVISION
        legacy.initialize(old, 3, 42, False)
        native_episode.stage_native(legacy, source)
        legacy.finish(legacy.reserve(1), result(qualified=True))
    finally:
        legacy.db.close()
    record = campaign._record("v0000", tmp_path / "pre-fix")
    campaign.state["versions"].insert(0, record)
    campaign._mark(record["archive"], record["id"])
    campaign._save()
    before = campaign.history()
    with pytest.raises(AdapterError, match="explicit allowance"):
        campaign.requalify(2)
    assert campaign.history() == before
    scheduled = campaign.requalify(1)
    assert scheduled["authorizedAdditionalTrials"] == 1
    assert scheduled["scheduledWalkers"] == 1
    assert len(setup.launched) == 1
    after = campaign.history()
    assert after["budget"] == before["budget"] + 1
    assert after["used"] == before["used"]
    protocol = definition(campaign.directory)
    assert protocol["requalificationCandidates"] == protocol["nativeCandidates"]
    assert observation_sources([setup.original], protocol, target=campaign.directory) == []
    archive = Archive(campaign.directory)
    try:
        assert archive.db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == 0
        assert len(discovery.requalification_pending(archive)) == 1
    finally:
        archive.db.close()
    with pytest.raises(AdapterError, match="already have a registered"):
        campaign.requalify(1)
    assert campaign.history()["budget"] == after["budget"]


def test_preparation_deduplicates_genotypes_that_converge_during_family_conversion(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    origin = tmp_path / "origin"
    rows = []
    for index, design in enumerate(("a"*64, "b"*64)):
        destination = origin / "designs" / design / "candidate"
        shutil.copytree(source / "candidate-0", destination)
        manifest = copy.deepcopy(data)
        manifest["motion"]["genotypeId"] = design
        (destination / "manifest.json").write_text(json.dumps(manifest))
        rows.append({"birth": index+1, "design": design, "sourceArchive": str(origin),
                     "founder": [str(origin), 1], "measured": False})
    monkeypatch.setattr(upgrades, "representatives", lambda sources: rows)
    monkeypatch.setattr(native_episode, "refresh_seed_exports", lambda source, cache, generator: source)

    def convert(source, cache, generator):
        destination = cache / "converted"
        shutil.copytree(source, destination)
        for batch in destination.glob("neighborhood-*"):
            manifest_path = batch / "candidate-001/manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["motion"]["genotypeId"] = "f"*64
            manifest_path.write_text(json.dumps(manifest))
            summary_path = batch / "candidates.json"
            summary = json.loads(summary_path.read_text())
            summary["generated"][0]["genotypeId"] = "f"*64
            summary_path.write_text(json.dumps(summary))
        return destination

    monkeypatch.setattr(leg_family_migration, "convert", convert)
    target, seeds = upgrades.prepare(origin, target=tmp_path / "prepared", generator=SimpleNamespace())
    assert native_episode.native_protocol({}, seeds, discover=True)["nativeCandidates"] == ["f"*64]
    assert len(json.loads((target / "upgrade.json").read_text())["parents"]) == 1


def test_ancestry_lookup_is_read_only_even_while_another_cursor_reads_history(tmp_path):
    from legowalkers_sim.campaign import database
    source, data = native_fixture(tmp_path)
    archive = Archive(tmp_path / "history")
    try:
        archive.initialize(native_episode.native_protocol({}, source, discover=True), 2, 42, False)
        native_episode.stage_native(archive, source)
        second, _ = archive.record_native_birth(data, parent=1)
        archive.finish(archive.reserve(1), result(qualified=True))
        archive.finish(archive.reserve(second), result(qualified=True))
        with database(archive.directory) as db:
            cursor = db.execute("SELECT * FROM evaluations ORDER BY id")
            assert next(cursor)["id"] == 1
            assert upgrades._origin(archive.directory, second) == (str(archive.directory), 1)
            assert next(cursor)["id"] == 2
    finally:
        archive.db.close()
