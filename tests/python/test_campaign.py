import contextlib
import copy
import hashlib
import json
from pathlib import Path
import re
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from legowalkers_sim import dashboard
from legowalkers_sim import observations
from legowalkers_sim.bridge import AdapterError, write_json
from legowalkers_sim.campaign import CampaignController, database, definition, require_active
from legowalkers_sim.evolution import Archive
from legowalkers_sim.gallery import publish
from legowalkers_sim.native_episode import native_protocol, stage_native
from legowalkers_sim.observations import reuse_observations
from legowalkers_sim.proposal_forest import campaign_nodes, campaign_snapshot, diagram, layout
from legowalkers_sim.upgrades import prepare
from test_evolution import result
from test_native_episode import native_fixture
from test_observations import measured_compatibility, physics_protocol


@pytest.fixture
def setup(tmp_path, monkeypatch):
    source, manifest = native_fixture(tmp_path)
    base = physics_protocol()
    original = tmp_path / "legacy"
    archive = Archive(original)
    archive.initialize(native_protocol(copy.deepcopy(base), source, discover=True), 3, 42, False)
    stage_native(archive, source)
    birth = archive.db.execute("SELECT id FROM births").fetchone()[0]
    measured = result(qualified=True)
    measured["compatibility"] = measured_compatibility(tmp_path, manifest)
    archive.finish(archive.reserve(birth), measured)
    write_json(original / "evaluations/0001/result.json", measured)
    publish(archive)
    archive.db.close()
    target = copy.deepcopy(base)
    target["sourceSha256"]["src/LegoWalkers.Core/WalkerEvolution.fs"] = "new-generation-only"
    launched = []
    monkeypatch.setattr(dashboard, "verify_snapshot", lambda path: path / "protocol-source")
    monkeypatch.setattr(dashboard.subprocess, "Popen",
                        lambda command, **kwargs: launched.append(command) or SimpleNamespace(pid=123456, poll=lambda: None))

    def load(path):
        value = dashboard.EpisodeController(path, definition(path), ["test-worker"])
        value.pinned = True
        return value

    def describe(seeds):
        return native_protocol(copy.deepcopy(target), seeds, discover=True)

    def build(path, seeds, selected, allowance):
        value = dashboard.EpisodeController(path, selected, ["test-worker"], initial_budget=allowance)
        archive = Archive(path)
        try:
            stage_native(archive, seeds)
            reuse_observations(archive, tmp_path)
            publish(archive)
        finally:
            archive.db.close()
        value.pinned = True
        return value

    def create():
        return CampaignController(tmp_path / "experiment", original, load=load, build=build, describe=describe)

    return SimpleNamespace(original=original, base=base, target=target, manifest=manifest,
                           create=create, launched=launched, build=build)


def test_upgrade_status_does_not_start_previews_before_activation(setup, monkeypatch):
    from legowalkers_sim.previews import PreviewController
    original = PreviewController.status
    calls = []

    def checked_status(previews, **options):
        if options.get("automatic", True):
            require_active(previews.directory)
        calls.append((previews.directory, options.get("automatic", True)))
        return original(previews, **options)

    monkeypatch.setattr(PreviewController, "status", checked_status)
    campaign = setup.create()
    campaign.ensure_latest()
    target = campaign.directory
    assert (target, False) in calls
    campaign.status()
    assert calls[-1] == (target, True)
    assert setup.launched == []


def test_updates_and_restarts_preserve_one_identity_and_transfer_allowance_once(setup):
    campaign = setup.create()
    identifier = campaign.status()["campaign"]
    before = (setup.original / "protocol.json").read_bytes()
    update = campaign.ensure_latest()
    assert update["updated"] and update["url"] == "/"
    state = campaign.status()
    assert (state["used"], state["budget"], state["remaining"], state["versionBudget"]) == (1, 3, 2, 2)
    assert setup.launched == []
    assert (setup.original / "protocol.json").read_bytes() == before
    assert campaign.ensure_latest()["updated"] is False
    restarted = setup.create()
    assert restarted.status()["campaign"] == identifier
    assert restarted.status()["remaining"] == 2
    assert restarted.ensure_latest()["updated"] is False
    assert len(restarted.history()["versions"]) == 2
    setup.target["sourceSha256"]["src/LegoWalkers.Core/WalkerEvolution.fs"] = "another-generation-update"
    assert restarted.ensure_latest()["updated"]
    assert restarted.status()["budget"] == 3 and restarted.status()["remaining"] == 2
    assert len(restarted.history()["versions"]) == 3
    with pytest.raises(AdapterError, match="read-only historical"):
        require_active(setup.original)
    historical = dashboard.EpisodeController(setup.original, definition(setup.original), ["test-worker"])
    assert historical.status()["canStart"] is False
    with pytest.raises(AdapterError, match="read-only historical"):
        historical.start()
    assert restarted.start()["budget"] == 2
    assert len(setup.launched) == 1 and restarted.status()["budget"] == 3


def test_generation_only_update_reuses_measurement_without_simulation(setup):
    campaign = setup.create()
    campaign.ensure_latest()
    archive = Archive(campaign.directory)
    try:
        assert archive.counts() == {}
        assert archive.best()["reused"] == 1
        assert archive.db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == 1
    finally:
        archive.db.close()
    assert "Current measurement available" in campaign.dashboard_page()
    assert setup.launched == []


@pytest.mark.parametrize("inherited", [False, True])
def test_family_conversion_preserves_measurement_founder_and_cross_version_ancestry(setup, tmp_path, inherited):
    import shutil
    from legowalkers_sim import population
    from legowalkers_sim.upgrades import _origin

    binary = tmp_path / "compiler"
    binary.mkdir()
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        (binary / name).write_bytes(b"fixture")

    def invoke(*args):
        source = Path(args[args.index("--parent")+1])
        target = Path(args[args.index("--output")+1]) / "candidate-001"
        shutil.copytree(source, target)
        manifest = json.loads((target / "manifest.json").read_text())
        manifest["genome"]["legFamilies"] = []
        manifest["motion"]["genotypeId"] = "f"*64
        write_json(target / "manifest.json", manifest)
        write_json(target / "realization.json", manifest["genome"])
        write_json(target.parent / "leg-family-conversion.json",
                   {"status": "converted", "families": [], "unmatched": []})

    campaign = setup.create()
    if inherited:
        campaign.ensure_latest()
        setup.target["sourceSha256"]["src/LegoWalkers.Core/WalkerEvolution.fs"] = "convert-inherited-parent"
    campaign.generator = SimpleNamespace(command=["dotnet", str(binary / "LegoWalkers.Cli.dll")], invoke=invoke)
    assert campaign.ensure_latest()["updated"]
    metadata = json.loads((campaign.directory / "upgrade.json").read_text())
    assert metadata["parents"][0]["targetDesign"] == "f"*64
    archive = Archive(campaign.directory)
    try:
        assert archive.counts() == {}
        assert archive.best()["design"] == "f"*64 and archive.best()["reused"] == 1
        birth = archive.best()["birth"]
        assert _origin(campaign.directory, birth) == (str(setup.original), 1)
        assert population.family_map(archive)[birth] == birth
        assert len(population.members(archive)) == 1
    finally:
        archive.db.close()
    forest = campaign_nodes(campaign.state["versions"], campaign.state["active"])
    carried = [node for node in forest if node.operation == "carry forward"]
    assert len(carried) == (2 if inherited else 1)
    assert carried[-1].parents == (("bv0002:1",) if inherited else ("bv0001:1",))
    assert "Current measurement available" in campaign.dashboard_page()
    assert campaign.status()["used"] == 1 and campaign.status()["budget"] == 3
    assert setup.launched == []


def test_simulator_revision_change_excludes_old_measurements_before_loading_manifests(setup, monkeypatch):
    from legowalkers_sim import campaign

    target = definition(setup.original)
    target["simulatorRevision"] = "new-simulator-revision"
    target["sourceSha256"]["simulator/legowalkers_sim/support_surfaces.py"] = "changed"

    def forbidden(*args):
        pytest.fail("A different simulator revision cannot reuse a historical measurement.")

    monkeypatch.setattr(campaign, "load_manifest", forbidden)
    assert campaign.observation_sources([setup.original], target) == []
    assert setup.launched == []


def test_qualified_walkers_survive_version_updates_without_counting_reuse_twice(setup):
    campaign = setup.create()
    initial = campaign.status()
    item = initial["pipeline"]["qualified"]["items"][0]
    assert initial["pipeline"]["qualified"]["count"] == 1
    assert item["version"] == initial["version"]
    assert item["prefix"] == f'/versions/{initial["version"]}/'
    assert item["qualification"]["qualified"]
    campaign.ensure_latest()
    updated = campaign.status()
    assert updated["pipeline"]["qualified"] == initial["pipeline"]["qualified"]
    assert updated["used"] == initial["used"]
    assert setup.launched == []
    page = campaign.dashboard_page()
    assert 'id="dashboard-navigation" aria-label="Dashboard sections"' in page
    assert 'id="qualified-walkers"' in page
    assert 'id="animation-rendering"' in page


def test_population_upgrade_uses_the_frozen_target_not_the_development_tree(setup, monkeypatch):
    from legowalkers_sim import campaign as campaigns
    from legowalkers_sim.upgrades import _population_seeds

    campaign = setup.create()
    campaign.ensure_latest()
    latest = campaign.directory
    original = campaigns.reusable
    checked = []

    def reusable(source, target, *, designs=None, target_source=None):
        assert target_source == latest
        checked.append(source)
        return original(source, target, designs=designs, target_source=target_source)

    monkeypatch.setattr(campaigns, "reusable", reusable)
    rows = _population_seeds([version["archive"] for version in campaign.state["versions"]])
    assert rows and checked == [setup.original]
    assert setup.launched == []


def test_optional_support_update_selects_and_reuses_identical_legacy_measurements(setup):
    name = "simulator/legowalkers_sim/support_surfaces.py"
    setup.target["sourceSha256"][name] = hashlib.sha256((observations.ROOT / name).read_bytes()).hexdigest()
    campaign = setup.create()
    assert campaign.ensure_latest()["updated"]
    with database(campaign.directory) as db:
        row = db.execute("SELECT b.design,r.result FROM reused_evaluations r JOIN births b ON b.id=r.birth").fetchone()
        assert row is not None
        assert db.execute("SELECT count(*) FROM evaluations").fetchone()[0] == 0
    path = campaign.directory / "designs" / row["design"] / "candidate/manifest.json"
    assert json.loads(path.read_text()) == setup.manifest
    assert all("contactShape" not in foot for foot in setup.manifest["motion"]["logicalFeet"])
    assert (campaign.status()["used"], campaign.status()["budget"]) == (1, 3)
    assert setup.launched == []


def test_unused_catalogue_extension_reuses_measurements_without_new_authorization(setup):
    setup.target["partsDependencySha256"] = "catalogue-with-L"
    (setup.original.parent / "parts/32526.dat").write_text("0 new unused L\n")
    campaign = setup.create()
    campaign.ensure_latest()
    with database(campaign.directory) as db:
        assert db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == 1
        assert db.execute("SELECT count(*) FROM evaluations").fetchone()[0] == 0
    state = campaign.status()
    assert (state["used"], state["budget"], state["remaining"]) == (1, 3, 2)
    assert setup.launched == []


def test_changed_used_part_blocks_activation_without_spending_trials(setup):
    part = setup.original.parent / "parts" / setup.manifest["parts"][0]["part"]
    part.write_text(part.read_text() + "0 changed used geometry\n")
    campaign = setup.create()
    with pytest.raises(AdapterError, match="changed recursive part dependencies"):
        campaign.ensure_latest()
    assert campaign.directory == setup.original
    assert len(campaign.history()["versions"]) == 1
    assert (campaign.status()["used"], campaign.status()["budget"]) == (1, 3)
    assert setup.launched == []


def test_population_upgrade_reuses_all_compatible_parents_without_new_authorization(setup):
    from legowalkers_sim import native_episode, population
    from test_native_discovery import Proposals

    campaign = setup.create()
    campaign.ensure_latest()
    archive = Archive(campaign.directory)
    try:
        parent = archive.best()
        batch = archive.directory / "extra-offspring"
        summary = Proposals().propose_native(
            archive.directory / "designs" / parent["design"] / "candidate", batch, 42, 0, 1)
        _, files = native_episode._queue(batch)
        entry = summary["generated"][0]
        manifest, _ = native_episode._copy_candidate(archive, batch, entry, files)
        birth, _ = archive.record_native_birth(manifest, parent=parent["birth"], native={
            "assembly": entry["assemblyId"], "batch": "extra-offspring",
            "directory": entry["directory"], "files": files})
        measured = result(qualified=True, speed=.1)
        measured["compatibility"] = measured_compatibility(setup.original.parent, manifest)
        trial = archive.reserve(birth)
        archive.finish(trial, measured)
        write_json(archive.directory / f"evaluations/{trial:04d}/result.json", measured)
        publish(archive)
        assert len(population.members(archive)) == 2
    finally:
        archive.db.close()
    def preserved(versions):
        snapshots = []
        for version in versions:
            directory = campaign.version_directory(version["id"])
            with database(directory) as db:
                snapshots.append(((directory / "protocol.json").read_bytes(), {
                    table: [tuple(row) for row in db.execute(f"SELECT * FROM {table} ORDER BY rowid")]
                    for table in ("run", "births", "designs", "evaluations", "reused_evaluations",
                                  "native_proposals", "native_candidates", "native_assemblies", "native_checks")
                }))
        return snapshots

    before = preserved(campaign.state["versions"])
    setup.target["sourceSha256"]["src/LegoWalkers.Core/WalkerEvolution.fs"] = "next-population-version"
    campaign.ensure_latest()
    archive = Archive(campaign.directory)
    try:
        assert archive.counts() == {}
        assert archive.db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == 2
        assert len(population.members(archive)) == 2
        assert len(set(population.family_map(archive).values())) == 1
    finally:
        archive.db.close()
    assert preserved(campaign.state["versions"][:-1]) == before
    status = campaign.status()
    assert (status["used"], status["budget"], status["remaining"]) == (2, 3, 1)
    assert setup.launched == []


@pytest.mark.parametrize("physics_change", [False, True])
def test_population_backfills_compatible_history_without_repeating_trials(setup, physics_change):
    from legowalkers_sim import native_episode, population
    from test_native_discovery import Proposals

    scaffold = setup.create()
    directory = setup.original.parent / "legacy-population"
    source = setup.original / "native-input"
    original = copy.deepcopy(native_protocol(setup.base, source, discover=True))
    del original["nativeDiscovery"]["population"]
    original["nativeDiscovery"]["version"] = "native-discovery-v5"
    archive = Archive(directory)
    try:
        archive.initialize(original, 17, 42, False)
        stage_native(archive, source)
        generator = Proposals()
        root_design = archive.db.execute("SELECT design FROM births WHERE id=1").fetchone()[0]
        for index in range(16):
            batch = archive.directory / f"offspring-{index}"
            summary = generator.propose_native(
                archive.directory / "designs" / root_design / "candidate", batch, 42, index, 1)
            _, files = native_episode._queue(batch)
            entry = summary["generated"][0]
            manifest, _ = native_episode._copy_candidate(archive, batch, entry, files)
            archive.record_native_birth(manifest, parent=1, native={
                "assembly": entry["assemblyId"], "batch": batch.name,
                "directory": entry["directory"], "files": files})
        for birth in range(1, 18):
            measured = result(qualified=True, speed=birth)
            measured["compatibility"] = measured_compatibility(setup.original.parent, setup.manifest)
            trial = archive.reserve(birth)
            archive.finish(trial, measured)
            write_json(archive.directory / f"evaluations/{trial:04d}/result.json", measured)
        publish(archive)
        before_trials = [tuple(row) for row in archive.db.execute("SELECT * FROM evaluations ORDER BY id")]
        before_protocol = (directory / "protocol.json").read_bytes()
    finally:
        archive.db.close()

    if physics_change:
        setup.target["evaluation"]["duration"] += 1
    campaign = CampaignController(directory.parent / "backfill", directory,
                                  load=scaffold.load, build=scaffold.build, describe=scaffold.describe)
    campaign.ensure_latest()
    with database(campaign.directory) as db:
        assert db.execute("SELECT count(*) FROM births").fetchone()[0] == 1
    setup.target["sourceSha256"]["src/LegoWalkers.Core/WalkerEvolution.fs"] = "backfill-compatible-population"
    campaign.ensure_latest()
    archive = Archive(campaign.directory)
    try:
        assert archive.counts() == {}
        assert archive.db.execute("SELECT count(*) FROM births").fetchone()[0] == (1 if physics_change else 12)
        assert archive.db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == (0 if physics_change else 12)
        assert len(population.members(archive)) == (0 if physics_change else 12)
        assert len(set(population.family_map(archive).values())) == 1
        parents = json.loads((campaign.directory / "upgrade.json").read_text())["parents"]
        assert len({parent["sourceDesign"] for parent in parents}) == len(parents)
        assert all(parent["founder"] == [str(directory), 1] for parent in parents)
        if not physics_change:
            assert sum(parent["sourceArchive"] == str(directory) for parent in parents) == 11
    finally:
        archive.db.close()
    with database(directory) as db:
        assert [tuple(row) for row in db.execute("SELECT * FROM evaluations ORDER BY id")] == before_trials
    assert (directory / "protocol.json").read_bytes() == before_protocol
    status = campaign.status()
    assert (status["used"], status["budget"], status["remaining"]) == (17, 17, 0)
    assert campaign.ensure_latest()["updated"] is False
    assert setup.launched == []


@pytest.mark.parametrize("physics_change", [False, True])
def test_forest_preserves_all_branches_across_updates_without_historical_badges(setup, physics_change):
    campaign = setup.create()

    def reject(directory):
        archive = Archive(directory)
        try:
            with archive.db:
                archive.db.execute("""
                    INSERT INTO native_proposals(parent,seed,attempt,after_trial,status,summary,error,created)
                    VALUES (1,42,0,0,'rejected','{"operation":"move-hinge-site","footCount":3}',NULL,'now')
                """)
        finally:
            archive.db.close()

    reject(campaign.directory)
    if physics_change:
        setup.target["evaluation"]["duration"] += 1
    campaign.ensure_latest()
    reject(campaign.directory)
    setup.target["sourceSha256"]["src/LegoWalkers.Core/WalkerEvolution.fs"] = "third-generation"
    campaign.ensure_latest()
    versions, active = campaign.state["versions"], campaign.state["active"]
    before = [(campaign.version_directory(v["id"]) / "archive.sqlite3").read_bytes() for v in versions]
    graph = {node.key: node for node in campaign_nodes(versions, active)}
    assert len(graph) == 5
    assert set(graph) == {"bv0001:1", "pv0001:1", "bv0002:1", "pv0002:1", "bv0003:1"}
    assert graph["bv0002:1"].parents == ("bv0001:1",)
    assert graph["bv0003:1"].parents == ("bv0002:1",)
    assert graph["bv0001:1"].carry_from is None
    assert graph["bv0002:1"].carry_from == "bv0001:1"
    assert graph["bv0003:1"].carry_from == "bv0002:1"
    assert graph["pv0001:1"].carry_from is None
    assert graph["pv0001:1"].parents == ("bv0001:1",)
    assert graph["pv0002:1"].parents == ("bv0002:1",)
    assert graph["pv0001:1"].state == graph["pv0002:1"].state == "rejected"
    assert graph["bv0001:1"].state == "qualified"
    assert graph["bv0001:1"].representative is graph["bv0002:1"].representative is None
    assert graph["bv0003:1"].representative == (None if physics_change else "champion")
    assert graph["bv0003:1"].operation == "carry forward"
    assert not graph["bv0001:1"].population
    assert not graph["bv0002:1"].population
    assert graph["pv0001:1"].href == "/versions/v0001/forest.html#proposal-1"
    assert graph["bv0001:1"].href == "/versions/v0001/history/0001.html"
    assert len(layout(list(graph.values()))) == 5
    markup = campaign_snapshot(versions, active)
    assert "5 nodes in this experiment across 3 versions" in markup
    assert ">v0001</text>" in markup and ">v0003</text>" in markup
    assert markup.count('data-champion-link=') == (0 if physics_change else 1)
    assert "carry forward" in diagram(list(graph.values()))
    assert [(campaign.version_directory(v["id"]) / "archive.sqlite3").read_bytes() for v in versions] == before
    assert (campaign.status()["used"], campaign.status()["budget"]) == (1, 3)
    assert setup.launched == []
    reject(campaign.directory)
    graph = {node.key: node for node in campaign_nodes(versions, active)}
    assert len(graph) == 6 and graph["pv0003:1"].state == "rejected"


def test_forest_rejects_unregistered_carry_forward_sources(setup):
    campaign = setup.create()
    campaign.ensure_latest()
    path = campaign.directory / "upgrade.json"
    provenance = json.loads(path.read_text())
    provenance["parents"][0]["sourceArchive"] = str(campaign.root / "unknown")
    write_json(path, provenance)
    with pytest.raises(AdapterError, match="carry-forward source is not registered"):
        campaign_snapshot(campaign.state["versions"], campaign.state["active"])


def test_an_archive_cannot_be_adopted_by_two_experiments(setup):
    campaign = setup.create()
    with pytest.raises(AdapterError, match="already belongs"):
        CampaignController(campaign.root.parent / "other-experiment", setup.original,
                           load=campaign.load, build=campaign.build, describe=campaign.describe)
    assert campaign.status()["remaining"] == 2 and setup.launched == []


def test_historical_animation_survives_multiple_generation_updates(setup):
    render = setup.original / "designs" / setup.manifest["motion"]["genotypeId"] / "render"
    render.mkdir()
    (render / "walker-v2-anchored.mp4").write_bytes(b"historical video fixture")
    campaign = setup.create()
    campaign.ensure_latest()
    setup.target["sourceSha256"]["src/LegoWalkers.Core/WalkerEvolution.fs"] = "third-generation"
    campaign.ensure_latest()
    page = campaign.dashboard_page()
    assert f'/versions/v0001/designs/{setup.manifest["motion"]["genotypeId"]}/render/walker-v2-anchored.mp4' in page
    assert "Media from v0001; earlier ancestry through v0002" in page
    assert "Current measurement available" in page and setup.launched == []


def test_physics_update_preserves_but_does_not_rerank_old_measurements(setup):
    setup.target["evaluation"]["duration"] += 1
    campaign = setup.create()
    campaign.ensure_latest()
    archive = Archive(campaign.directory)
    try:
        assert archive.best() is None and archive.counts() == {}
    finally:
        archive.db.close()
    assert "Historical measurement — needs evaluation under current rules" in campaign.dashboard_page()
    assert campaign.status()["used"] == 1 and setup.launched == []
    assert (setup.original / "evaluations/0001/result.json").exists()


def test_failed_update_keeps_active_version_and_cannot_spend_staging_allowance(setup):
    campaign = setup.create()
    old = campaign.directory
    def failed(*args):
        setup.build(*args)
        raise AdapterError("Model preparation fixture failed.")
    campaign.build = failed
    with pytest.raises(AdapterError, match="preparation"):
        campaign.ensure_latest()
    assert campaign.directory == old
    assert campaign.status()["remaining"] == 2 and campaign.status()["budget"] == 3
    assert "preparation" in campaign.status()["updateError"]
    with pytest.raises(AdapterError, match="historical"):
        require_active(campaign.root / "versions/v0002")
    campaign.build = setup.build
    assert campaign.ensure_latest()["version"] == "v0003"
    assert campaign.status()["remaining"] == 2 and setup.launched == []


@pytest.mark.parametrize("worker", ["episode", "rendering"])
def test_update_refuses_to_switch_while_a_worker_is_running(setup, monkeypatch, worker):
    campaign = setup.create()
    if worker == "episode":
        campaign.active.start()
    else:
        status = campaign.active.renderer.status()
        monkeypatch.setattr(campaign.active.renderer, "status", lambda: {**status, "running": True})
    with pytest.raises(AdapterError, match="Stop the episode and rendering"):
        campaign.ensure_latest()
    assert campaign.directory == setup.original
    assert len(campaign.history()["versions"]) == 1
    assert campaign.status()["remaining"] == 2


def test_update_quiesces_an_idle_render_watcher_without_spending_trials(setup, monkeypatch):
    campaign = setup.create()
    original = campaign.active
    status = original.renderer.status()
    stopped = []
    monkeypatch.setattr(original.renderer, "status", lambda: {
        **status, "running": not stopped, "busy": False, "queued": 0, "current": None})
    monkeypatch.setattr(original.renderer, "stop", lambda: stopped.append(True))
    before = campaign.history()
    assert campaign.ensure_latest()["updated"]
    assert stopped == [True]
    after = campaign.history()
    assert (after["used"], after["budget"]) == (before["used"], before["budget"])
    assert campaign.status()["remaining"] == 2
    assert not campaign.status()["running"]


def test_update_does_not_switch_if_idle_renderer_cannot_be_stopped(setup, monkeypatch):
    campaign = setup.create()
    status = campaign.active.renderer.status()
    monkeypatch.setattr(campaign.active.renderer, "status", lambda: {**status, "running": True, "busy": False})
    monkeypatch.setattr(campaign.active.renderer, "stop_if_idle", lambda: False)
    with pytest.raises(AdapterError, match="Stop the episode and rendering"):
        campaign.ensure_latest()
    assert campaign.directory == setup.original

def test_imported_branches_keep_all_authorization_and_latest_measured_representative(setup):
    branch, seeds = prepare(setup.original)
    archive = Archive(branch)
    archive.initialize(native_protocol(copy.deepcopy(setup.base), seeds, discover=True), 4, 42, False)
    stage_native(archive, seeds)
    birth = archive.db.execute("SELECT id FROM births").fetchone()[0]
    measured = result(qualified=True)
    measured["compatibility"] = measured_compatibility(setup.original.parent, setup.manifest)
    archive.finish(archive.reserve(birth), measured)
    write_json(branch / "evaluations/0001/result.json", measured)
    publish(archive)
    archive.db.close()
    campaign = setup.create()
    assert campaign.status()["used"] == 2 and campaign.status()["budget"] == 7
    campaign.ensure_latest()
    assert campaign.status()["remaining"] == 5 and campaign.status()["versionBudget"] == 5
    parents = json.loads((campaign.directory / "upgrade.json").read_text())["parents"]
    assert len(parents) == 1 and parents[0]["sourceArchive"] == str(branch)
    assert parents[0]["founder"] == [str(setup.original), 1]
    assert setup.launched == []


def test_unmeasured_founder_is_not_lost_during_an_update(setup):
    archive = Archive(setup.original)
    try:
        with archive.db:
            archive.db.execute("DELETE FROM evaluations")
    finally:
        archive.db.close()
    campaign = setup.create()
    campaign.ensure_latest()
    assert "Carried forward — not yet measured" in campaign.dashboard_page()
    assert campaign.status()["remaining"] == 3 and setup.launched == []


def test_http_stable_url_history_assets_and_read_only_controls(setup):
    setup.target["evaluation"]["duration"] += 1
    campaign = setup.create()
    campaign.ensure_latest()
    original_page = setup.original / "history/0001.html"
    preview = '<section class="panel"><h2>Saved preview</h2><img src="data:image/png;base64,cG5n"></section>'
    legacy_page = re.sub(
        r'<details class="birth-genes"><summary>(<h[23]>.*?</h[23]>)</summary>(<pre>.*?</pre>)</details>',
        r"\1\2", original_page.read_text(), flags=re.DOTALL).replace("</main>", preview + "</main>")
    original_page.write_text(legacy_page)
    original_index = (setup.original / "index.html").read_bytes()
    with ThreadingHTTPServer(("127.0.0.1", 0), dashboard.DashboardHandler) as server:
        server.controller = campaign
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        def request(path, data=None):
            req = Request(url+path, data=json.dumps(data).encode() if data is not None else None,
                          headers={"Content-Type": "application/json", "Origin": url})
            try:
                with urlopen(req, timeout=5) as response:
                    return response.status, response.read()
            except HTTPError as error:
                with contextlib.closing(error):
                    return error.code, error.read()
        try:
            code, page = request("/")
            assert code == 200 and b"One evolving experiment" in page
            assert b"needs evaluation under current rules" in page and b"/history.html" in page
            assert request("/history.html")[0] == 200
            assert page.count(b'id="proposal-forest"') == 1
            forest = json.loads(request("/api/forest")[1])["html"]
            assert "/versions/v0002/history/" in forest
            assert "/versions/v0001/history/" in forest
            assert 'data-live-key="forest-bv0001:1"' in forest
            assert 'data-parent="bv0001:1" data-child="bv0002:1"' in forest
            assert b"2 nodes in this experiment across 2 versions" in page
            code, full_forest = request("/forest.html")
            assert code == 200 and forest.encode() in full_forest
            assert b"function toggleZoom(view, x, y)" in full_forest
            code, old_forest = request("/versions/v0001/forest.html")
            assert code == 200 and b"Read-only version history" in old_forest
            assert b"/versions/v0001/history/" in old_forest
            assert b"/versions/v0002/history/" not in old_forest
            assert b"1 nodes in this version" in old_forest
            assert b"function toggleZoom(view, x, y)" in old_forest
            assert b"var(--forest-scale,1)" in old_forest
            history = json.loads(request("/api/history")[1])
            assert len(history["versions"]) == 2 and history["used"] == 1
            code, old = request("/versions/v0001/")
            assert code == 200 and b"Read-only version history" in old
            assert b'id="category-3-feet"' in old
            assert b'id="category-3-feet"' in page
            assert b'id="start-episode"' not in old and b'id="start-rendering"' not in old
            assert b"function toggleZoom(view, x, y)" in old
            assert b"var(--forest-scale,1)" in old
            assert (setup.original / "index.html").read_bytes() == original_index
            code, old_birth = request("/versions/v0001/history/0001.html")
            assert code == 200 and b"Read-only version history" in old_birth
            assert old_birth.index(b"<h2>Saved preview</h2>") < old_birth.index(b"<h2>Ancestry</h2>")
            assert old_birth.count(b'<details class="birth-genes">') == 2
            assert b'<details class="birth-genes" open' not in old_birth
            assert original_page.read_text() == legacy_page
            path = f'/versions/v0001/designs/{setup.manifest["motion"]["genotypeId"]}/candidate/model.ldr'
            assert request(path) == (200, (setup.original / path.split("/versions/v0001/")[1]).read_bytes())
            assert request("/versions/unknown/")[0] == 404
            assert request("/versions/v0001/%2e%2e/%2e%2e/private")[0] == 403
            state = json.loads(request("/api/status")[1])
            token = {"csrfToken": state["csrfToken"]}
            assert request("/versions/v0001/api/episode", token)[0] == 405
            assert request("/api/episode", {})[0] == 403
            assert request("/api/upgrade", token)[0] == 202
            assert campaign.status()["remaining"] == 2 and setup.launched == []
            assert request("/api/episode", token)[0] == 202
            assert campaign.status()["budget"] == 3 and len(setup.launched) == 1
        finally:
            server.shutdown()
            thread.join(timeout=5)
