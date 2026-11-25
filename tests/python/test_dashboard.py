import contextlib
import json
import os
import subprocess
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.dashboard import DashboardHandler, DashboardServer, EpisodeController
from legowalkers_sim.evolution import Archive, CATEGORIES, GRAMMAR, SEARCH_POLICY, category_genome
from test_evolution import genome, protocol, result


def test_preparation_queues_historical_inputs_without_export_build_or_admission(tmp_path, monkeypatch):
    from legowalkers_sim import dashboard, evolution, observations, user_pool
    from legowalkers_sim.bridge import write_json
    from legowalkers_sim.payload import load_profile
    from test_native_episode import native_fixture

    source, _ = native_fixture(tmp_path)
    manifest = source / "candidate-0/manifest.json"
    original = json.loads(manifest.read_text())
    original["simulator"]["revision"] = "0"*40
    write_json(manifest, original)
    target, campaign = tmp_path / "prepared", tmp_path / "campaign"
    write_json(target / "campaign-version.json", {"campaign": str(campaign), "version": "v0002"})
    write_json(campaign / "campaign.json", {
        "active": "v0002", "versions": [{"id": "v0002", "archive": str(target)}]})
    monkeypatch.setattr(dashboard, "protocol_definition", lambda *args: protocol())
    monkeypatch.setattr(evolution, "freeze_runtime", lambda *args: None)
    monkeypatch.setattr(dashboard, "verify_snapshot", lambda path: path / "protocol-source")
    monkeypatch.setattr(observations, "reuse_observations", lambda *args: None)
    monkeypatch.setattr(user_pool, "sync", lambda *args, **kwargs: pytest.fail("Preparation must not admit contributions"))
    monkeypatch.setattr(evolution.Generator, "invoke", lambda *args: pytest.fail("Preparation must not export or build walkers"))
    assert dashboard.main(["--prepare-only", "--output", str(target), "--candidate-batch", str(source),
                           "--discover", "--defer-construction", "--simulation-workers", "1", "--initial-budget", "3"]) == 0
    archive = Archive(target)
    try:
        assert load_profile(target / "protocol-source/selected-payloads.json") == archive.protocol()["payloadProfile"]
        assert archive.protocol()["deferredConstruction"] == "queued-motion-inputs-v1"
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 3
        assert archive.counts() == {}
        assert (target / "native-input/candidate-0/manifest.json").read_bytes() == manifest.read_bytes()
        from legowalkers_sim.bridge import load_manifest
        with pytest.raises(AdapterError, match="different simulator revision"):
            load_manifest(target / "native-input/candidate-0/manifest.json")
    finally:
        archive.db.close()


def test_dashboard_explains_empty_running_pipeline_without_inventing_candidate_activity():
    script = (Path(__file__).parents[2] / "simulator/legowalkers_sim/dashboard.js").read_text()
    start = script.index("    function pipelineNotice(state)")
    end = script.index("    function renderPipeline(state)", start)
    code = script[start:end] + """
const assert = require("node:assert/strict");
const state = {running: true, pipeline: {tracked: true, stages: [{count: 0}, {count: 0}]}};
assert.match(pipelineNotice(state), /contribution admission/);
assert.match(pipelineNotice(state), /no candidate-stage activity/);
state.pipeline.startup = {tracked: true};
assert.equal(pipelineNotice(state), null);
delete state.pipeline.startup;
state.running = false;
assert.equal(pipelineNotice(state), null);
state.running = true;
state.pipeline.stages[0].count = 1;
assert.equal(pipelineNotice(state), null);
state.pipeline.tracked = false;
assert.match(pipelineNotice(state), /frozen version does not record/);
"""
    subprocess.run(["node", "-e", code], check=True, capture_output=True, text=True)


def test_dashboard_polls_only_visible_tabs_and_throttles_unchanged_forests():
    script = (Path(__file__).parents[2] / "simulator/legowalkers_sim/dashboard.js").read_text()
    poll = script[script.index("    async function poll()"):script.index("    updateNavigation();\n    poll();")]
    forest = script[script.index('        const forest = document.getElementById("proposal-forest");'):
                    script.index("\n    previewRetry.addEventListener")]
    code = """
const assert = require("node:assert/strict");
let updates = 0, delay, now = 0, requests = 0, patches = 0;
let forestRevision = null, forestUpdatedAt = -Infinity;
const document = {hidden: false, getElementById() {return {};}};
const performance = {now: () => now};
const update = async () => {updates++;};
const setTimeout = (_, value) => {delay = value;};
const request = async url => {assert.equal(url, "/api/forest"); requests++; return {html: "forest"};};
const DOMParser = class {parseFromString() {return {getElementById: () => ({})};}};
const patch = () => {patches++;};
""" + poll + "\nasync function refresh(state) {\n" + forest + """
(async () => {
  await poll(); assert.equal(updates, 1); assert.equal(delay, 2000);
  document.hidden = true;
  await poll(); assert.equal(updates, 1); assert.equal(delay, 10000);
  document.hidden = false;
  await poll(); assert.equal(updates, 2);
  await refresh({revision: "a"}); assert.equal(requests, 1);
  now = 2000; await refresh({revision: "a"}); assert.equal(requests, 1);
  now = 10000; await refresh({revision: "a"}); assert.equal(requests, 2);
  now = 10001; await refresh({revision: "b"}); assert.equal(requests, 3);
  assert.equal(patches, 3);
})().catch(error => {console.error(error); process.exitCode = 1;});
"""
    subprocess.run(["node", "-e", code], check=True, capture_output=True, text=True)


def test_campaign_controls_show_explicit_preparation_progress_failure_and_readiness():
    script = (Path(__file__).parents[2] / "simulator/legowalkers_sim/dashboard.js").read_text()
    controls = script[script.index("    async function update()"):script.index("        status.textContent = state.pausing")] + "\n}"
    code = """
const assert = require("node:assert/strict");
const element = () => ({textContent: "", append(...items) {
    this.textContent += items.map(item => typeof item === "string" ? item : item.textContent).join("");
}});
const runtimeStatus = element(), upgrade = element(), upgradeStatus = element();
const experimentHistory = element(), button = element(), pause = element();
const document = {createElement: element};
let token, running, canStart, upgrading = false, starting = false, pausing = false;
const state = {campaign: "campaign", version: "v0001", running: false, canStart: false,
    updateAvailable: true, remaining: 2, rendering: {running: false}, simulationWorkers: 2,
    migration: {status: "required"}};
const request = async () => state;
""" + controls + """
(async () => {
    await update();
    assert.equal(button.disabled, true);
    assert.equal(pause.disabled, true);
    assert.match(button.textContent, /Resume.*preparation required/);
    assert.equal(upgrade.disabled, false);
    assert.match(upgradeStatus.textContent, /Explicit preparation required/);
    state.running = true;
    await update();
    assert.equal(pause.disabled, false);
    assert.equal(pause.textContent, "Pause episode");
    state.pausing = true;
    await update();
    assert.equal(pause.disabled, true);
    assert.match(pause.textContent, /Pausing/);
    state.pausing = false;
    state.pauseFailed = true;
    await update();
    assert.equal(pause.disabled, false);
    assert.equal(pause.textContent, "Retry pause");
    state.pauseFailed = false;
    state.running = state.pausing = false;
    state.migration = {status: "running", started: new Date().toISOString(),
        stage: "Rebuilding retained plans.", targetVersion: "v0002", log: "/migration.log"};
    await update();
    assert.equal(button.disabled, true);
    assert.equal(upgrade.disabled, true);
    assert.match(upgradeStatus.textContent, /Rebuilding retained plans.*v0002.*Elapsed.*Preparation log/);
    state.migration = {status: "failed", error: "Construction timed out"};
    await update();
    assert.equal(button.disabled, true);
    assert.equal(upgrade.disabled, false);
    assert.match(upgradeStatus.textContent, /Preparation failed.*timed out/);
    state.migration = {status: "ready"};
    state.canStart = true;
    state.updateAvailable = false;
    await update();
    assert.equal(button.disabled, false);
    assert.match(button.textContent, /Resume episode/);
    assert.match(upgradeStatus.textContent, /Ready to resume/);
})().catch(error => {console.error(error); process.exitCode = 1;});
"""
    subprocess.run(["node", "-e", code], check=True, capture_output=True, text=True)


def test_preparation_and_resume_controls_appear_together_before_candidate_pipeline():
    from legowalkers_sim.gallery import document

    html = document("Dashboard", "", dashboard=True)
    assert html.index('id="prepare-upgrade"') < html.index('id="start-episode"') < html.index('id="candidate-pipeline"')
    assert html.index('id="upgrade-status"') < html.index('id="candidate-pipeline"')


@pytest.mark.parametrize("path", ["/api/forest", "/forest.html", "/"])
def test_campaign_forest_rendering_does_not_block_status(path, tmp_path, monkeypatch):
    from legowalkers_sim import dashboard
    from legowalkers_sim.campaign import CampaignController

    campaign = CampaignController.__new__(CampaignController)
    campaign.mutex = threading.RLock()
    campaign.state = {"active": "v0001", "versions": [{"id": "v0001", "archive": str(tmp_path)}]}
    campaign.dashboard_page = lambda: "<html><main id=\"dashboard-content\"></main></html>"

    def status():
        with campaign.mutex:
            return {"running": False, "remaining": 3}

    campaign.status = status
    (tmp_path / "index.html").write_text("Current results")
    entered, release = threading.Event(), threading.Event()
    calls = []

    def forest(versions, active):
        calls.append(active)
        assert versions == [{"id": "v0001", "archive": str(tmp_path)}]
        assert active == "v0001"
        assert versions is not campaign.state["versions"]
        entered.set()
        assert release.wait(5)
        return "<section id=\"proposal-forest\"></section>"

    monkeypatch.setattr(dashboard, "campaign_snapshot", forest)
    with DashboardServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = campaign
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        outcomes, failures = [], []

        def read_forest():
            try:
                with urlopen(url + path, timeout=5) as response:
                    outcomes.append(response.status)
                    response.read()
            except Exception as error:
                failures.append(error)

        reader = threading.Thread(target=read_forest)
        second_reader = threading.Thread(target=read_forest)
        reader.start()
        second_reader.start()
        try:
            assert entered.wait(3)
            with urlopen(url + "/api/status", timeout=1) as response:
                assert json.load(response) == {"running": False, "remaining": 3}
        finally:
            release.set()
            reader.join(timeout=5)
            second_reader.join(timeout=5)
            server.shutdown()
            thread.join(timeout=5)
        assert not failures
        assert outcomes == [200, 200]
        assert calls == ["v0001"]


def test_campaign_forest_cache_shares_work_expires_and_tracks_result_revisions(tmp_path, monkeypatch):
    from legowalkers_sim import dashboard

    page = tmp_path / "index.html"
    page.write_text("Current results")
    versions = [{"id": "v0001", "archive": str(tmp_path)}]
    clock, calls = [0], []
    monkeypatch.setattr(dashboard.time, "monotonic", lambda: clock[0])

    def forest(versions, active):
        calls.append((versions, active))
        return f"Forest {len(calls)}"

    monkeypatch.setattr(dashboard, "campaign_snapshot", forest)
    with DashboardServer(("127.0.0.1", 0), DashboardHandler) as server:
        assert server.campaign_forest(versions, "v0001") == "Forest 1"
        clock[0] = 9
        assert server.campaign_forest([dict(version) for version in versions], "v0001") == "Forest 1"
        clock[0] = 10
        assert server.campaign_forest(versions, "v0001") == "Forest 2"
        page.write_text("Newly published current results")
        assert server.campaign_forest(versions, "v0001") == "Forest 3"
        newer = versions + [{"id": "v0002", "archive": str(tmp_path)}]
        assert server.campaign_forest(newer, "v0002") == "Forest 4"


def test_campaign_forest_cache_does_not_hide_generation_failure(tmp_path, monkeypatch):
    from legowalkers_sim import dashboard

    (tmp_path / "index.html").write_text("Current results")
    versions = [{"id": "v0001", "archive": str(tmp_path)}]

    def failing(*args):
        raise AdapterError("Ancestry evidence unavailable")

    monkeypatch.setattr(dashboard, "campaign_snapshot", failing)
    with DashboardServer(("127.0.0.1", 0), DashboardHandler) as server:
        with pytest.raises(AdapterError, match="Ancestry evidence unavailable"):
            server.campaign_forest(versions, "v0001")
        assert server.forest_cache is None
        monkeypatch.setattr(dashboard, "campaign_snapshot", lambda *args: "Recovered ancestry")
        assert server.campaign_forest(versions, "v0001") == "Recovered ancestry"


def test_startup_card_renders_named_states_elapsed_time_and_navigation_without_fake_births():
    script = (Path(__file__).parents[2] / "simulator/legowalkers_sim/dashboard.js").read_text()
    code = script[script.index("    function screeningChecks("):script.index("    function pipelineNotice(")]
    code += """
const assert = require("node:assert/strict");
const document = {createElement(tag) {return {tag, dataset: {}, children: [], textContent: "",
    append(...nodes) {this.children.push(...nodes);}};}};
const text = node => node.textContent + node.children.map(text).join(" ");
const links = node => (node.tag === "a" ? [node] : []).concat(node.children.flatMap(links));
const item = {key: "human", label: "4longer <test>", kind: "admission", status: "queued",
    elapsedSeconds: null, detail: "", history: null, log: "episode-0001.log"};
let card = startupCard(item, "/versions/v0031/");
assert.match(text(card), /4longer <test>/);
assert.match(text(card), /admission · queued/);
assert.doesNotMatch(text(card), /elapsed|Birth null|Continue with birth/);
assert.equal(links(card)[0].href, "/versions/v0031/episode-0001.log");
item.status = "running"; item.elapsedSeconds = 65.9;
assert.match(text(startupCard(item, "")), /running · 1m 5s elapsed/);
item.status = "complete"; item.history = "history/0013.html"; item.birth = 13;
card = startupCard(item, "/versions/v0031/");
assert.match(text(card), /completed/);
assert.match(text(card), /Continue with birth 13/);
assert.equal(links(card)[0].href, "/versions/v0031/history/0013.html");
item.status = "rejected"; item.detail = "Overlap";
assert.match(text(startupCard(item, "")), /rejected.*Overlap/);
"""
    subprocess.run(["node", "-e", code], check=True, capture_output=True, text=True)


@pytest.fixture
def controller(tmp_path, monkeypatch):
    processes = []

    def launch(command, **options):
        process = SimpleNamespace(pid=123456, poll=lambda: None, command=command, options=options)
        processes.append(process)
        return process

    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen", launch)
    value = EpisodeController(tmp_path / "campaign", {**protocol(), "grammarVersion": GRAMMAR}, ["test-worker"])
    value.launched = processes
    return value


def test_preview_automatic_start_waits_until_campaign_version_is_active(controller, tmp_path, monkeypatch):
    from legowalkers_sim.bridge import write_json
    registry = tmp_path / "registry"
    write_json(controller.directory / "campaign-version.json", {"campaign": str(registry), "version": "v0002"})
    write_json(registry / "campaign.json", {"active": "v0001", "versions": [
        {"id": "v0002", "archive": str(controller.directory)}]})
    requests = []
    monkeypatch.setattr(controller.previews, "status",
                        lambda **options: requests.append(options) or {"running": False, "error": None})
    state = controller.status()
    assert "read-only historical version" in state["error"]
    assert requests[-1]["automatic"] is False
    write_json(registry / "campaign.json", {"active": "v0002", "versions": [
        {"id": "v0002", "archive": str(controller.directory)}]})
    assert controller.status()["error"] is None
    assert requests[-1]["automatic"] is True
    assert controller.status(automatic_previews=False)["error"] is None
    assert requests[-1]["automatic"] is False


def test_dashboard_never_starts_a_worker_until_explicit_authorization(controller):
    state = controller.status()
    assert state["budget"] == state["used"] == 0
    assert not state["running"] and controller.launched == []
    assert "location.reload" not in (controller.directory / "index.html").read_text()
    started = controller.start()
    state = controller.status()
    assert started["budget"] == state["budget"] == 12 and state["running"]
    process = controller.launched[0]
    assert process.command[-5:] == ["--budget", "12", "--seed", "42", "--resume"]
    assert process.options["start_new_session"] and process.options["close_fds"]
    assert state["log"] == "episode-0001.log"
    assert json.loads((controller.directory / "progress.json").read_text())["budget"] == 12
    with pytest.raises(AdapterError, match="already running"):
        controller.start()
    assert len(controller.launched) == 1 and controller.status()["budget"] == 12


def test_remote_only_dashboard_keeps_twelve_slot_explicit_authorization(tmp_path, monkeypatch):
    launched = []
    def launch(command, **options):
        launched.append(command)
        return SimpleNamespace(pid=123456, poll=lambda: None)
    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen", launch)
    active = {**protocol(), "grammarVersion": GRAMMAR, "searchPolicy": SEARCH_POLICY,
              "categories": [category_genome(*item) for item in CATEGORIES]}
    controller = EpisodeController(tmp_path/"remote", active, ["test-worker"])
    assert controller.status()["budget"] == 0 and not launched
    html = (controller.directory/"index.html").read_text()
    assert html.count('<article class="card"') == 3
    assert "3 foot-count categories" in html and "Wired / external power" not in html
    assert controller.start()["budget"] == 12
    assert len(launched) == 1
    with pytest.raises(AdapterError, match="already running"):
        controller.start()
    assert len(launched) == 1 and controller.status()["budget"] == 12


def test_dashboard_can_authorize_four_unknown_future_candidates_instead_of_twelve(tmp_path, monkeypatch):
    launched = []
    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen",
                        lambda command, **options: launched.append(command) or SimpleNamespace(pid=123456, poll=lambda: None))
    controller = EpisodeController(tmp_path / "four-slots", {**protocol(), "grammarVersion": GRAMMAR},
                                   ["test-worker"], episode_trials=4)
    assert controller.status()["budget"] == 0 and controller.status()["nextEpisodeTrials"] == 4
    assert controller.start()["budget"] == 4
    assert launched[0][-5:] == ["--budget", "4", "--seed", "42", "--resume"]
    assert controller.status()["remaining"] == 4
    with pytest.raises(AdapterError, match="positive integer"):
        EpisodeController(tmp_path / "bad", protocol(), ["test-worker"], episode_trials=0)


def test_native_dashboard_stages_without_physics_and_authorizes_only_the_prepared_trial(tmp_path, monkeypatch):
    from test_native_episode import native_fixture
    from legowalkers_sim import native_episode
    from legowalkers_sim.gallery import publish
    source, data = native_fixture(tmp_path)
    native = native_episode.native_protocol(protocol(), source)
    launched = []
    def launch(command, **options):
        process = SimpleNamespace(pid=123456, poll=lambda: None, command=command)
        launched.append(process)
        return process
    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen", launch)
    def forbidden(*args, **kwargs):
        pytest.fail("Opening/staging a native dashboard must not evaluate or screen physics.")
    monkeypatch.setattr(native_episode, "evaluate", forbidden)
    monkeypatch.setattr(native_episode, "screen_batch", forbidden)
    controller = EpisodeController(tmp_path/"dashboard", native, ["test-worker"])
    archive = Archive(controller.directory)
    try:
        native_episode.stage_native(archive, source)
        preview = archive.directory/"designs"/data["motion"]["genotypeId"]/"assembly-render"
        preview.mkdir()
        (preview/"walker-v2.png").write_bytes(b"unit test image")
        publish(archive)
        assert controller.status()["budget"] == 0 and launched == []
        assert controller.status()["canStart"] and controller.status()["nextEpisodeTrials"] == 1
        page = (controller.directory/"index.html").read_text()
        assert "assembly-render/walker-v2.png" in page and "not measured motion" in page
        assert controller.start()["budget"] == 1
        birth = archive.db.execute("SELECT id FROM births").fetchone()[0]
        archive.finish(archive.reserve(birth), result())
        archive.set_status("complete")
        launched[0].poll = lambda: 0
        state = controller.status()
        assert state["used"] == state["budget"] == 1 and not state["canStart"]
        assert state["nextEpisodeTrials"] == 0 and not state["running"]
        with pytest.raises(AdapterError, match="no repeat"):
            controller.start()
        assert len(launched) == 1
    finally:
        archive.db.close()


@pytest.mark.parametrize("workers", [2, 4])
def test_native_parallel_status_lists_individual_trials_under_one_episode(tmp_path, monkeypatch, workers):
    from legowalkers_sim import native_episode
    from test_native_episode import native_fixture

    source, manifest = native_fixture(tmp_path)
    native = native_episode.native_protocol(protocol(), source, discover=True, simulation_workers=workers,
                                           screening_workers=2)
    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen",
                        lambda *args, **kwargs: SimpleNamespace(pid=123456, poll=lambda: None))
    controller = EpisodeController(tmp_path / "parallel-status", native, ["test-worker"], episode_trials=4)
    controller.start()
    archive = Archive(controller.directory)
    try:
        pids = list(range(123457, 123457 + workers))
        for pid in pids:
            birth = archive.record_native_birth(manifest)[0]
            trial = archive.reserve(birth)
            with archive.db:
                archive.db.execute("INSERT INTO trial_workers VALUES (?,?,?)",
                                   (trial, pid, f"evaluations/{trial:04d}.log"))
        state = controller.status()
        assert state["simulationWorkers"] == workers and state["running"]
        assert state["screeningWorkers"] == 2
        assert state["used"] == workers and state["budget"] == 4
        assert [trial["pid"] for trial in state["activeTrials"]] == pids
        assert all(trial["design"] == manifest["motion"]["genotypeId"] for trial in state["activeTrials"])
        assert all(trial["preview"] is None for trial in state["activeTrials"])
        archive.finish(1, result())
        assert len(controller.status()["activeTrials"]) == workers - 1
    finally:
        archive.db.close()


def test_running_gallery_preview_availability_updates_without_starting_rendering(controller):
    controller.start()
    archive = Archive(controller.directory)
    try:
        birth, design = archive.record_birth(genome(), None, "founder", [], "generated")
        trial = archive.reserve(birth)
        state = controller.status()
        assert len(state["activeTrials"]) == 1
        assert state["activeTrials"][0]["preview"] is None
        assert state["activeTrials"][0]["category"] == "four-bar-2-wired-solo"
        root = controller.directory / "designs" / design
        for folder, name, kind in (("candidate", "cycle.svg", "motion-diagram"),
                                   ("assembly-render", "walker-v2.png", "assembly"),
                                   ("render", "walker-v2.png", "assembly")):
            path = root / folder / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"unit test preview")
            preview = controller.status()["activeTrials"][0]["preview"]
            assert preview == {"url": f"designs/{design}/{folder}/{name}", "kind": kind}
            assert controller.status()["used"] == 1
            assert not controller.renderer.status()["running"]
            assert len(controller.launched) == 1
        archive.finish(trial, result())
        assert controller.status()["activeTrials"] == []
        text = (controller.directory / "index.html").read_text()
        assert 'id="candidate-pipeline"' in text and 'id="assembly-preview-status"' in text
    finally:
        archive.db.close()


def test_native_dashboard_recovers_spent_interrupted_slot_without_authorizing_a_repeat(tmp_path, monkeypatch):
    from test_native_episode import native_fixture
    from legowalkers_sim import native_episode
    source, data = native_fixture(tmp_path)
    process = SimpleNamespace(pid=123456, poll=lambda: None)
    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen", lambda *args, **kwargs: process)
    controller = EpisodeController(tmp_path/"dashboard", native_episode.native_protocol(protocol(), source), ["worker"])
    archive = Archive(controller.directory)
    try:
        native_episode.stage_native(archive, source)
        controller.start()
        archive.reserve(archive.db.execute("SELECT id FROM births").fetchone()[0])
        process.poll = lambda: 2
        state = controller.status()
        assert not state["running"] and not state["canStart"]
        assert archive.counts() == {"interrupted": 1}
        with pytest.raises(AdapterError, match="no repeat"):
            controller.start()
    finally:
        archive.db.close()

def test_failed_worker_resumes_unspent_authorization_without_extending_budget(controller):
    controller.start()
    controller.launched[0].poll = lambda: 2
    state = controller.status()
    assert not state["running"] and "code 2" in state["error"] and state["remaining"] == 12
    controller.start()
    assert len(controller.launched) == 2 and controller.status()["budget"] == 12


def test_completed_episode_needs_another_click_to_authorize_twelve_more(controller):
    controller.start()
    archive = Archive(controller.directory)
    try:
        for _ in range(12):
            birth, _ = archive.record_birth(genome(), None, "test", [], "generated")
            archive.finish(archive.reserve(birth), result())
        archive.set_status("complete")
    finally:
        archive.db.close()
    controller.launched[0].poll = lambda: 0
    assert controller.status()["remaining"] == 0
    assert not controller.status()["running"] and len(controller.launched) == 1
    controller.start()
    assert controller.status()["budget"] == 24 and len(controller.launched) == 2


def test_cheap_gate_cap_is_a_normal_pause_and_resumes_the_same_full_allowance(controller):
    from legowalkers_sim.parallel import ensure_attempt_schema

    controller.start()
    archive = Archive(controller.directory)
    try:
        ensure_attempt_schema(archive.db)
        with archive.db:
            archive.db.execute("INSERT INTO simulation_gate_rounds VALUES (1,12,3,'now','limit')")
        archive.set_status("complete")
    finally:
        archive.db.close()
    controller.launched[0].poll = lambda: 0
    state = controller.status()
    assert not state["running"] and state["error"] is None and state["remaining"] == 12
    assert state["cheapGate"]["pausedAtLimit"] and "cap reached" in state["notice"]
    controller.start()
    assert len(controller.launched) == 2 and controller.status()["budget"] == 12


def test_launch_failure_is_explicit_and_does_not_duplicate_authorized_budget(controller, monkeypatch):
    def fail(*args, **kwargs):
        raise OSError("test executable missing")
    monkeypatch.setattr("legowalkers_sim.dashboard.subprocess.Popen", fail)
    with pytest.raises(AdapterError, match="executable missing"):
        controller.start()
    state = controller.status()
    assert state["remaining"] == 12 and not state["running"]
    assert "executable missing" in state["error"]
    with pytest.raises(AdapterError):
        controller.start()
    assert controller.status()["budget"] == 12


def test_restart_recognizes_an_independent_live_worker(controller, monkeypatch):
    controller.start()
    restarted = EpisodeController(controller.directory, {**protocol(), "grammarVersion": GRAMMAR}, ["test-worker"])
    monkeypatch.setattr("legowalkers_sim.dashboard.os.kill", lambda pid, signal: None)
    assert restarted.status()["running"]
    with pytest.raises(AdapterError, match="already running"):
        restarted.start()
    assert len(controller.launched) == 1


def test_restart_recovers_crash_between_authorization_and_worker_launch(controller):
    with contextlib.closing(controller.connect()) as db, db:
        db.execute("UPDATE run SET budget=12")
        db.execute("INSERT INTO episodes(id,budget,status,started,log) VALUES (1,12,'running','test','episode-0001.log')")
    state = controller.status()
    assert not state["running"] and state["remaining"] == 12 and state["error"]
    controller.start()
    assert controller.status()["budget"] == 12 and len(controller.launched) == 1


def test_http_controls_require_same_origin_and_explicit_token(controller):
    reference = controller.directory.parent / "evolution-pilot"
    reference.mkdir()
    (reference / "preview.html").write_text("Historical snapshot")
    history = controller.directory / "history/0013.html"
    history.parent.mkdir(exist_ok=True)
    ancestry = ('<section class="panel"><h2>Ancestry</h2>'
                '<h3>Actual gene differences from parent</h3><pre>long differences</pre></section>'
                '<section class="panel"><h2>Canonical genome (provenance seed excluded)</h2>'
                '<pre>long genome</pre></section>')
    preview = '<section class="panel"><h2>Saved preview</h2><video controls></video></section>'
    history.write_text("<h1>Birth 13</h1>" + ancestry + preview)
    with ThreadingHTTPServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = controller
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"

        def request(path, data=None, **headers):
            encoded = json.dumps(data).encode() if data is not None else None
            req = Request(url + path, data=encoded, headers={"Content-Type": "application/json", **headers})
            try:
                with urlopen(req, timeout=5) as response:
                    return response.status, response.read()
            except HTTPError as error:
                with contextlib.closing(error):
                    return error.code, error.read()

        try:
            assert request("/")[0] == 200
            assert request("/archives/evolution-pilot/preview.html") == (200, b"Historical snapshot")
            assert request("/archives/unknown/preview.html")[0] == 404
            code, birth_page = request("/history/0013.html")
            assert code == 200 and birth_page.index(b"Saved preview") < birth_page.index(b"Ancestry")
            assert birth_page.count(b'<details class="birth-genes">') == 2
            assert b'<details class="birth-genes" open' not in birth_page
            assert history.read_text() == "<h1>Birth 13</h1>" + ancestry + preview
            assert request("/history/%2e%2e/%2e%2e/outside.html")[0] == 403
            state = json.loads(request("/api/status")[1])
            assert request("/api/episode", {})[0] == 403
            token = {"csrfToken": state["csrfToken"]}
            assert request("/api/episode/pause", {})[0] == 403
            assert request("/api/episode/pause", token, Origin="http://foreign.invalid")[0] == 403
            assert request("/api/episode/pause", token, Origin=url)[0] == 409
            assert request("/versions/v0001/api/episode/pause", token, Origin=url)[0] == 405
            assert request("/api/render/start", {})[0] == 403
            assert request("/api/render/stop", token, Origin="http://foreign.invalid")[0] == 403
            assert request("/api/render/start", token)[0] == 409  # No Blender configured.
            assert request("/api/render/stop", token)[0] == 409
            assert not controller.status()["rendering"]["running"]
            assert request("/api/episode", token, Origin="http://foreign.invalid")[0] == 403
            assert request("/api/status", Host="foreign.invalid")[0] == 403
            assert controller.status()["budget"] == 0
            assert request("/api/episode", token, Origin=url)[0] == 202
            assert request("/api/episode", token, Origin=url)[0] == 409
            assert controller.status()["budget"] == 12
            controller.renderer.blender = Path(sys.executable)
            assert request("/api/render/start", token, Origin=url)[0] == 202
            assert request("/api/render/start", token, Origin=url)[0] == 409
            assert controller.status()["rendering"]["running"]
            assert controller.status()["budget"] == 12 and len(controller.launched) == 2
            monkeypatch = pytest.MonkeyPatch()
            try:
                signals = []
                monkeypatch.setattr("legowalkers_sim.rendering.os.getpgid", lambda pid: pid)
                monkeypatch.setattr("legowalkers_sim.rendering.os.killpg", lambda pid, sig: signals.append(pid))
                assert request("/api/render/stop", token, Origin=url)[0] == 202
                assert signals == [controller.renderer.process.pid]
                assert controller.status()["running"] and controller.status()["budget"] == 12
            finally:
                monkeypatch.undo()
        finally:
            server.shutdown()
            thread.join(timeout=5)


@pytest.mark.parametrize("mode", ["owned", "adopted", "pending-pause", "stubborn"])
def test_pause_stops_owned_children_preserves_allowance_and_recovers_partial_work(controller, monkeypatch, mode):
    import signal
    import time
    from legowalkers_sim import pipeline, user_pool

    controller.start()
    monkeypatch.undo()
    pidfile = controller.directory / "compiler.pid"
    worker = subprocess.Popen([sys.executable, "-c",
        "import subprocess,sys,time,signal; from pathlib import Path; "
        "signal.signal(signal.SIGTERM,signal.SIG_IGN if sys.argv[2]=='stubborn' else signal.SIG_DFL); "
        "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
        "Path(sys.argv[1]).write_text(str(child.pid)); time.sleep(60)", str(pidfile), mode],
        start_new_session=True)
    reaper = threading.Thread(target=worker.wait)
    reaper.start()
    try:
        deadline = time.monotonic() + 10
        while not pidfile.exists():
            assert time.monotonic() < deadline
            time.sleep(.01)
        child_pid = int(pidfile.read_text())
        controller.process = None if mode in ("adopted", "pending-pause") else worker
        with controller.connect() as db:
            db.execute("UPDATE episodes SET pid=? WHERE id=1", (worker.pid,))
        archive = Archive(controller.directory)
        try:
            birth, _ = archive.record_birth(genome(), None, "founder", [], "generated")
            trial = archive.reserve(birth)
            with archive.db:
                archive.db.execute("INSERT INTO trial_workers VALUES (?,?,?)",
                                   (trial, child_pid, "evaluations/0001.log"))
            user_pool.queue(archive)
            entry = {"id": "a"*32, "files": "{}"}
            with archive.db:
                archive.db.execute("INSERT INTO user_pool_snapshots(created,entries) VALUES('test',?)",
                                   (json.dumps([entry]),))
                archive.db.execute("INSERT INTO screening_workers VALUES(?,?,?,'running')",
                                   (birth, child_pid, "construction-workers/000001.log"))
            batch = pipeline.begin_startup(archive)
            pipeline.queue_startup(archive, batch, [(entry["id"], "admission", "Partial contribution")])
            pipeline.update_startup(archive, batch, entry["id"], "running")
            partial = controller.directory / "partial-attempt.json"
            partial.write_text("{}")
            if mode == "pending-pause":
                with controller.connect() as db:
                    db.execute("UPDATE episodes SET status='stopping' WHERE id=1")
                assert controller.status(automatic_previews=False)["pausing"]
            else:
                assert controller.pause()["status"] == "pausing"
            if mode == "stubborn":
                assert controller.status(automatic_previews=False)["pausing"]
                with pytest.raises(AdapterError, match="already pausing"):
                    controller.pause()
                with pytest.raises(AdapterError, match="already running"):
                    controller.start()
            controller.pause_thread.join(timeout=20)
            assert not controller.pause_thread.is_alive()
            state = controller.status(automatic_previews=False)
            assert state["paused"] and not state["running"] and not state["pausing"]
            assert state["error"] is None
            assert state["budget"] == 12 and state["used"] == 1 and state["remaining"] == 11
            assert not state["activeTrials"] and partial.exists()
            assert archive.db.execute("SELECT status FROM evaluations WHERE id=?", (trial,)).fetchone()[0] == "interrupted"
            assert archive.db.execute("SELECT status FROM user_pool_entries").fetchone()[0] == "interrupted"
            assert archive.db.execute("SELECT status FROM screening_workers").fetchone()[0] == "interrupted"
            with pytest.raises(ProcessLookupError):
                os.killpg(worker.pid, 0)
            with pytest.raises(AdapterError, match="No episode"):
                controller.pause()
        finally:
            archive.db.close()
    finally:
        try:
            os.killpg(worker.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        reaper.join(timeout=5)


def test_pause_refuses_unowned_group_and_does_not_change_budget(controller, monkeypatch):
    controller.start()
    monkeypatch.setattr("legowalkers_sim.dashboard.os.getpgid", lambda pid: pid + 1)
    with pytest.raises(AdapterError, match="process-group leader"):
        controller.pause()
    assert controller.status()["running"] and controller.status()["budget"] == 12


def test_restarted_pause_refuses_unowned_group_and_surfaces_failure(controller, monkeypatch):
    controller.start()
    controller.process = None
    with controller.connect() as db:
        db.execute("UPDATE episodes SET status='stopping' WHERE id=1")
    monkeypatch.setattr("legowalkers_sim.dashboard.os.getpgid", lambda pid: pid + 1)
    state = controller.status()
    assert state["pauseFailed"] and not state["pausing"] and state["running"]
    assert "process-group leader" in state["error"]
    assert controller.pause_thread is None
    with pytest.raises(AdapterError, match="already running"):
        controller.start()


def test_unexpected_pause_failure_is_persisted_and_can_be_retried(controller, monkeypatch):
    controller.start()
    with controller.connect() as db:
        db.execute("UPDATE episodes SET status='stopping' WHERE id=1")
        job = dict(db.execute("SELECT * FROM episodes WHERE id=1").fetchone())

    def gone(*args):
        raise ProcessLookupError

    def failed_recovery(archive):
        raise RuntimeError("Recovery fixture failed")

    recover = Archive.recover
    monkeypatch.setattr("legowalkers_sim.dashboard.os.killpg", gone)
    monkeypatch.setattr("legowalkers_sim.dashboard.os.getpgid", lambda pid: pid)
    monkeypatch.setattr(Archive, "recover", failed_recovery)
    with pytest.raises(RuntimeError, match="Recovery fixture"):
        controller._pause_worker(job)
    state = controller.status()
    assert state["pauseFailed"] and state["running"]
    assert "Unexpected pause failure" in state["error"]
    assert controller.pause_thread is None
    monkeypatch.setattr(Archive, "recover", recover)
    assert controller.pause()["status"] == "pausing"
    controller.pause_thread.join(timeout=10)
    assert controller.status()["paused"]


def test_pause_requires_confirmed_exit_after_transient_group_probe_error(controller, monkeypatch):
    controller.start()
    with controller.connect() as db:
        db.execute("UPDATE episodes SET status='stopping' WHERE id=1")
        job = dict(db.execute("SELECT * FROM episodes WHERE id=1").fetchone())
    probes = [PermissionError(), ProcessLookupError()]

    def group_probe(pid, signal):
        assert pid == job["pid"]
        if signal == 0:
            with controller.connect() as db:
                assert db.execute("SELECT status FROM episodes WHERE id=1").fetchone()[0] == "stopping"
            raise probes.pop(0)

    monkeypatch.setattr("legowalkers_sim.dashboard.os.killpg", group_probe)
    controller._pause_worker(job)
    assert not probes and controller.status()["paused"]


def test_real_worker_is_an_independent_process_and_does_not_autorun_more(tmp_path):
    command = [sys.executable, "-c",
               "import os,time; print(os.getpid(),os.getsid(0),flush=True); time.sleep(.25)"]
    controller = EpisodeController(tmp_path, {**protocol(), "grammarVersion": GRAMMAR}, command)
    started = controller.start()
    process = controller.process
    try:
        assert started["pid"] != os.getpid()
        assert controller.status()["running"] and controller.status()["used"] == 0
        assert process.wait(timeout=5) == 0
        state = controller.status()
        assert not state["running"] and state["used"] == 0 and state["budget"] == 12
        assert state["error"]  # A zero-exit worker with no completed run is not success.
        pid, session = map(int, (tmp_path / started["log"]).read_text().split())
        assert pid == session == started["pid"]
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
