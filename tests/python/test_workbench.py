import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
import random
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.request import urlopen
from pathlib import Path
import uuid

import pytest

from legowalkers_sim import discovery, pipeline, screening, user_pool
from legowalkers_sim import workbench as module
from legowalkers_sim.bridge import AdapterError, write_json
from legowalkers_sim.evolution import Archive, Generator, ROOT
from legowalkers_sim.native_episode import native_protocol, stage_native
from legowalkers_sim.workbench import Workbench
from legowalkers_sim.dashboard import DashboardHandler
from test_evolution import protocol, result
from test_native_discovery import Proposals
from test_native_episode import native_fixture


class Manual(Proposals):
    def manual_edits(self, parent):
        return {"dynamicTrials": 0, "edits": [
            {"id": "a"*64, "level": "motion", "operation": "input-phase"},
            {"id": "b"*64, "level": "realization", "operation": "battery-layout"}]}

    def apply_edit(self, parent, output, edit):
        summary = self.propose_native(parent, output, 0, 0, 2)
        summary["selectedEdit"] = edit
        return summary


@pytest.fixture
def environment(tmp_path, monkeypatch):
    source, _ = native_fixture(tmp_path)
    archive = Archive(tmp_path / "archive")
    seed = next(seed for seed in range(100) if random.Random(
        f'{user_pool.POLICY["version"]}:{seed}:0').random() < .2)
    definition = native_protocol(protocol(), source, discover=True)
    definition["nativeDiscovery"] = dict(definition["nativeDiscovery"])
    definition["nativeDiscovery"].pop("motionCarryForward")
    archive.initialize(definition, 4, seed, False)
    stage_native(archive, source)
    archive.finish(archive.reserve(1), result(qualified=True))
    campaign = tmp_path / "campaign"
    write_json(archive.directory / "campaign-version.json", {"campaign": str(campaign), "version": "v0001"})
    store = Workbench(campaign / "user-workbench")
    calls = []

    def screen(batch, output, library, **kwargs):
        assert not Path(output).resolve().is_relative_to(Path(batch).resolve())
        calls.append((batch, kwargs))
        entry = json.loads((Path(batch) / "candidates.json").read_text())["generated"][0]
        return {"results": [{"status": "ready-for-dynamics", "genotypeId": entry["genotypeId"]}],
                "counts": {"dynamicTrials": 0}}

    monkeypatch.setattr(module, "screen_batch", screen)
    monkeypatch.setattr(screening, "screen_batch", screen)
    yield store, archive, Manual(), calls
    archive.db.close()


def fork(store, archive):
    return store.fork(archive.directory, 1, "Interesting branch", "Test contributor")


def test_historical_simulator_contribution_does_not_block_admission_or_duplicate_detection(environment):
    store, archive, generator, calls = environment
    original = fork(store, archive)["node"]
    child = store.apply(original, "a"*64, generator)["children"][0]
    store.contribute(original, "Historical original", "Test", "", Path("."))
    path = store.candidate(original) / "manifest.json"
    historical = json.loads(path.read_text())
    historical["simulator"]["revision"] = "7db4596fe7f970fadc7b53893be32b4b93249d9e"
    write_json(path, historical)
    with pytest.raises(AdapterError, match="different simulator revision"):
        module.load_manifest(path)
    assert store.contribute(child, "Current new seed", "Test", "", Path("."))["dynamicTrials"] == 0
    duplicate = fork(store, archive)["node"]
    with pytest.raises(AdapterError, match="already active as Historical original"):
        store.contribute(duplicate, "Duplicate", "Test", "", Path("."))
    assert len(calls) == 2 and archive.counts() == {"complete": 1}


def test_new_contribution_still_requires_current_simulator_revision(environment):
    store, archive, _, calls = environment
    node = fork(store, archive)["node"]
    path = store.candidate(node) / "manifest.json"
    historical = json.loads(path.read_text())
    historical["simulator"]["revision"] = "7db4596fe7f970fadc7b53893be32b4b93249d9e"
    write_json(path, historical)
    with pytest.raises(AdapterError, match="different simulator revision"):
        store.contribute(node, "Old seed", "Test", "", Path("."))
    assert calls == [] and store.state()["contributions"] == []


@pytest.mark.parametrize("genotype", [None, "invalid", 42])
def test_invalid_saved_contribution_identity_blocks_admission_explicitly(environment, genotype):
    store, archive, generator, calls = environment
    original = fork(store, archive)["node"]
    child = store.apply(original, "a"*64, generator)["children"][0]
    contribution = store.contribute(original, "Corrupt identity", "Test", "", Path("."))["contribution"]
    with store.connect() as db:
        db.execute("UPDATE contributions SET validation=? WHERE id=?",
                   (json.dumps({"results": [{"genotypeId": genotype}]}), contribution))
    with pytest.raises(AdapterError, match="Saved admission identity is invalid for Corrupt identity"):
        store.contribute(child, "New seed", "Test", "", Path("."))
    assert len(calls) == 1


def test_external_design_rebase_preserves_geometry_and_history_then_admits_without_fitness(environment, tmp_path):
    store, archive, generator, calls = environment
    external = Archive(tmp_path / "external-qualification")
    external.initialize(archive.protocol(), 1, 42, False)
    stage_native(external, archive.directory / "native-input")
    external.finish(external.reserve(1), result(qualified=True))
    try:
        original = store.fork(external.directory, 1, "Qualified external design", "Test")["node"]
        selected = store.apply(original, "a"*64, generator)["children"][0]
        before = fingerprint(store.candidate(selected))
        with pytest.raises(AdapterError, match="family is not retained"):
            store.contribute(selected, "External winner", "Test", "", Path("."), target_archive=archive.directory)
        assert calls == [] and store.state()["contributions"] == []
        rebased = store.rebase(selected, archive.directory, 1)
        assert fingerprint(store.candidate(rebased["node"])) == before
        nodes = {node["id"]: node for node in store.state()["nodes"]}
        assert nodes[selected]["source_archive"] == str(external.directory)
        assert nodes[rebased["node"]]["parent"] == selected
        assert nodes[rebased["node"]]["source_archive"] == str(archive.directory)
        assert rebased["summary"]["previousSourceArchive"] == str(external.directory)
        assert rebased["summary"]["dynamicTrials"] == 0 and not rebased["summary"]["fitnessImported"]
        submitted = store.contribute(rebased["node"], "Rebased winner", "Test", "", Path("."),
                                     target_archive=archive.directory)
        user_pool.sync(archive, Path("."))
        entry = archive.db.execute("SELECT * FROM user_pool_entries WHERE contribution=?",
                                   (submitted["contribution"],)).fetchone()
        assert entry["status"] == "eligible" and archive.founder(entry["birth"]) == 1
        assert discovery._pending(archive)["birth"] == entry["birth"]
        assert archive.counts() == {"complete": 1} and len(archive.outcomes()) == 1
        assert external.counts() == {"complete": 1}
    finally:
        external.db.close()


@pytest.mark.parametrize("birth", [0, True, 9999])
def test_rebase_rejects_invalid_anchor_without_creating_a_version(environment, birth):
    store, archive, _, _ = environment
    root = fork(store, archive)["node"]
    before = store.state()
    with pytest.raises(AdapterError, match="birth"):
        store.rebase(root, archive.directory, birth)
    assert store.state() == before


def test_upgrade_retains_active_contribution_family_even_if_missing_from_latest_population(environment, tmp_path):
    from legowalkers_sim.upgrades import _origin, _population_seeds

    store, archive, generator, _ = environment
    root = fork(store, archive)["node"]
    child = store.apply(root, "a"*64, generator)["children"][0]
    store.contribute(child, "Preserve this family", "Test", "", Path("."))
    latest = Archive(tmp_path / "latest")
    source = store.candidate(child).parent
    latest.initialize(native_protocol(protocol(), source, discover=True), 0, 42, False)
    stage_native(latest, source)
    write_json(latest.directory / "campaign-version.json",
               {"campaign": str(store.root.parent), "version": "v0002"})
    try:
        rows = _population_seeds([latest.directory])
        recovered, = [row for row in rows if row["family"] == _origin(archive.directory, 1)]
        assert recovered["sourceArchive"] == str(archive.directory)
        assert recovered["result"] is None and recovered["measured"] is False
        assert archive.counts() == {"complete": 1} and latest.counts() == {}
        store.withdraw(store.state()["contributions"][0]["id"])
        assert all(row["family"] != recovered["family"] for row in _population_seeds([latest.directory]))
    finally:
        latest.db.close()


def test_admission_releases_workspace_for_preview_and_evolution_and_keeps_the_exact_submitted_version(environment, monkeypatch):
    store, archive, generator, _ = environment
    forked = fork(store, archive)["node"]
    root = store.apply(forked, "a"*64, generator)["children"][0]
    started, finish = threading.Event(), threading.Event()
    before = fingerprint(archive.directory)
    from legowalkers_sim import workbench_diagrams
    monkeypatch.setattr(workbench_diagrams, "model_diagram", lambda *args: {"parts": [], "paths": []})

    def screen(*args, **kwargs):
        started.set()
        assert finish.wait(10), "Test did not release the paused static check."
        return {"results": [{"status": "ready-for-dynamics", "gates": [
            {"gate": "simulator-reconstruction", "outcome": "pass", "code": "reconstructed-at-time-zero"},
            {"gate": "dynamic-evaluation", "outcome": "not-run", "code": "no-dynamic-trial-authorized"}]}]}

    monkeypatch.setattr(module, "screen_batch", screen)
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(store.contribute, root, "Submitted original", "Test", "", Path("."))
        try:
            assert started.wait(5)
            check, = Workbench(store.root).checks()
            assert check["phase"] == "checking" and check["status"] == "running"
            assert check["node"] == root and all(gate["source"] == "saved-generation" for gate in check["outcomes"])
            with pytest.raises(AdapterError, match="exact version is being checked"):
                store.select(root, "discard")
            assert store.preview(root, "a"*64, generator, Path("."))["dynamicTrials"] == 0
            child = store.apply(root, "a"*64, generator)["children"][0]
            store.select(child, "keep")
            with pytest.raises(AdapterError, match="check is already running"):
                store.contribute(child, "Another", "Test", "", Path("."))
        finally:
            finish.set()
        result = future.result(timeout=10)
    state = store.state()
    assert state["contributions"][0]["node"] == root and state["workspaces"][0]["head"] == child
    assert result["check"] == state["checks"][0]["id"]
    assert state["checks"][0]["status"] == "complete" and state["checks"][0]["phase"] == "admitted"
    assert state["checks"][0]["outcomes"][-1]["outcome"] == "not-run"
    assert state["checks"][0]["report"].endswith("/contribution-validation.json")
    assert fingerprint(archive.directory) == before and archive.counts() == {"complete": 1}


@pytest.mark.parametrize("change", ["files", "capacity", "discard"])
def test_admission_rechecks_files_capacity_and_node_status_after_static_checks(environment, monkeypatch, change):
    store, archive, _, _ = environment
    root = fork(store, archive)["node"]

    def screen(*args, **kwargs):
        if change == "files":
            with (store.candidate(root) / "program.json").open("a") as file:
                file.write("\n")
        else:
            with store.connect() as db:
                if change == "discard":
                    db.execute("UPDATE nodes SET status='discarded' WHERE id=?", (root,))
                else:
                    for index in range(12):
                        db.execute("INSERT INTO contributions VALUES(?,?,?,?,?,1,?,?,?)",
                                   (uuid.uuid4().hex, uuid.uuid4().hex, str(index), "Test", "", "{}", "{}", "now"))
        return {"results": [{"status": "ready-for-dynamics", "gates": [
            {"gate": "simulator-reconstruction", "outcome": "pass", "code": "loaded-at-time-zero"}]}]}

    monkeypatch.setattr(module, "screen_batch", screen)
    with pytest.raises(AdapterError, match="changed during validation|pool is full|discarded"):
        store.contribute(root, "Checked", "Test", "", Path("."))
    check, = store.checks()
    assert check["status"] == "failed" and check["phase"] == "admitting"
    assert check["outcomes"][0]["outcome"] == "pass" and check["report"]
    assert not any(entry["node"] == root for entry in store.state()["contributions"])
    assert archive.counts() == {"complete": 1}


def test_admission_rejection_and_unexpected_failure_preserve_progress_and_previous_outcomes(environment, monkeypatch):
    store, archive, _, _ = environment
    root = fork(store, archive)["node"]

    def rejected(*args, **kwargs):
        return {"results": [{"status": "rejected", "error": "Initial contact", "gates": [
            {"gate": "simulator-reconstruction", "outcome": "reject", "code": "initial-self-interference"}]}]}

    monkeypatch.setattr(module, "screen_batch", rejected)
    with pytest.raises(AdapterError, match="Initial contact|initial-self-interference"):
        store.contribute(root, "Rejected", "Test", "", Path("."))
    check, = store.checks()
    assert check["status"] == "rejected" and check["outcomes"][0]["outcome"] == "reject"
    elapsed = check["elapsedSeconds"]
    assert store.checks()[0]["elapsedSeconds"] == elapsed

    def failed(*args, **kwargs):
        raise RuntimeError("Checker stopped unexpectedly")

    monkeypatch.setattr(module, "screen_batch", failed)
    with pytest.raises(RuntimeError, match="stopped unexpectedly"):
        store.contribute(root, "Failed", "Test", "", Path("."))
    check = store.checks()[0]
    assert check["status"] == "failed" and check["detail"] == "Checker stopped unexpectedly"
    assert all(gate["source"] == "saved-generation" for gate in check["outcomes"])
    assert store.state()["contributions"] == []


def test_interrupted_admission_is_visible_and_can_be_retried(environment, monkeypatch):
    store, archive, _, _ = environment
    root = fork(store, archive)["node"]
    with store.connect() as db:
        db.execute("INSERT INTO contribution_checks VALUES(?,?,?,?,?,?,?,?,?,?,NULL)",
                   (uuid.uuid4().hex, root, "Interrupted", "checking", "running", "", "[]", None,
                    123456789, module.now()))
    original = module.os.kill

    def kill(pid, signal):
        if pid == 123456789:
            raise ProcessLookupError()
        return original(pid, signal)

    monkeypatch.setattr(module.os, "kill", kill)
    check, = store.checks()
    assert check["status"] == "interrupted" and "Not admitted" in check["detail"]
    assert store.contribute(root, "Retry", "Test", "", Path("."))["dynamicTrials"] == 0
    assert archive.counts() == {"complete": 1}

def test_check_progress_endpoint_does_not_rebuild_catalogs_or_source_lists(environment, monkeypatch):
    store, archive, generator, _ = environment
    root = fork(store, archive)["node"]
    store.contribute(root, "Completed check", "Test", "", Path("."))
    monkeypatch.setattr(DashboardHandler, "workbench", lambda self: (store, generator, None))

    def sources(self):
        raise AssertionError("Progress must not enumerate candidate sources.")

    monkeypatch.setattr(DashboardHandler, "workbench_sources", sources)
    with ThreadingHTTPServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = SimpleNamespace(directory=archive.directory)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/workbench?checks=1", timeout=5) as response:
                value = json.load(response)
            assert value["checks"][0]["status"] == "complete"
            assert value["checks"][0]["label"] == "Completed check"
        finally:
            server.shutdown()
    assert archive.counts() == {"complete": 1}


def test_named_contribution_admissions_show_queue_progress_rejection_and_cached_results(environment, monkeypatch):
    store, archive, generator, _ = environment
    root = fork(store, archive)
    child = store.apply(root["node"], "a"*64, generator)["children"][0]
    store.contribute(root["node"], "4long", "Test", "", Path("."))
    store.contribute(child, "4longer", "Test", "", Path("."))
    seen = []

    def screen(*args, **kwargs):
        admissions = [item for item in pipeline.startup_snapshot(archive.db, True)["items"]
                      if item["kind"] == "admission"]
        seen.append([(item["label"], item["status"]) for item in admissions])
        return {"results": [{"status": "ready-for-dynamics"} if len(seen) == 1 else {
            "status": "rejected", "error": "Initial overlap"}]}

    monkeypatch.setattr(screening, "screen_batch", screen)
    before = archive.counts()
    batch = pipeline.begin_startup(archive)
    with pipeline.startup_step(archive, batch, "contributions"):
        user_pool.sync(archive, Path("."), startup_batch=batch)
    assert seen == [[("4long", "running"), ("4longer", "queued")],
                    [("4long", "complete"), ("4longer", "running")]]
    admissions = [item for item in pipeline.startup_snapshot(archive.db, True)["items"]
                  if item["kind"] == "admission"]
    assert admissions[0]["birth"] == 1 and admissions[0]["history"] == "history/0001.html"
    assert admissions[1]["status"] == "rejected" and admissions[1]["detail"] == "Initial overlap"
    assert admissions[1]["history"] is None
    batch = pipeline.begin_startup(archive)
    user_pool.sync(archive, Path("."), startup_batch=batch)
    assert len(seen) == 2
    admissions = [item for item in pipeline.startup_snapshot(archive.db, True)["items"]
                  if item["kind"] == "admission"]
    assert admissions[0]["detail"] == "Admission already recorded; no repeated reconstruction check."
    assert admissions[1]["status"] == "rejected"
    assert archive.counts() == before


def test_admission_error_is_recorded_and_reraised(environment, monkeypatch):
    store, archive, _, _ = environment
    root = fork(store, archive)
    store.contribute(root["node"], "Broken check", "Test", "", Path("."))

    def fail(*args, **kwargs):
        raise AdapterError("Admission worker failed")

    monkeypatch.setattr(screening, "screen_batch", fail)
    batch = pipeline.begin_startup(archive)
    with pytest.raises(AdapterError, match="Admission worker failed"):
        with pipeline.startup_step(archive, batch, "contributions"):
            user_pool.sync(archive, Path("."), startup_batch=batch)
    items = pipeline.startup_snapshot(archive.db, False)["items"]
    assert items[-1]["status"] == "failed" and items[-1]["detail"] == "Admission worker failed"
    assert next(item for item in items if item["label"] == "Queuing user contributions")["status"] == "failed"


@pytest.mark.parametrize("alive", [False, True])
def test_queued_admission_recovers_dead_worker_without_repeating_partial_work(environment, monkeypatch, alive):
    store, archive, _, _ = environment
    root = fork(store, archive)
    store.contribute(root["node"], "Interrupted contribution", "Test", "", Path("."))
    startup = pipeline.begin_startup(archive)
    task, = user_pool.queue(archive, startup_batch=startup)
    identifier = task["entry"]["id"]
    pipeline.update_startup(archive, startup, identifier, "running")

    def probe(pid, signal):
        if not alive:
            raise ProcessLookupError()

    monkeypatch.setattr(user_pool.os, "kill", probe)
    if alive:
        with pytest.raises(AdapterError, match="still running"):
            user_pool.queue(archive)
        assert archive.db.execute("SELECT count(*) FROM user_pool_entries").fetchone()[0] == 0
    else:
        assert user_pool.queue(archive) == []
        assert archive.db.execute("SELECT status FROM user_pool_entries").fetchone()[0] == "interrupted"
        assert archive.db.execute("SELECT status FROM pipeline_startup WHERE batch=? AND task=?",
                                  (startup, identifier)).fetchone()[0] == "interrupted"
        assert user_pool.queue(archive) == []
    assert archive.counts() == {"complete": 1}


def eligible_admission_fixture(task, directory, generator, library, payload_profile, stage):
    import os
    from legowalkers_sim.native_episode import _freeze_batch, _queue

    source = Path(task["source"])
    _, files = _queue(source)
    _freeze_batch(source, directory / "user-contributions" / task["entry"]["id"], files)
    stage("screening", "Independent admission fixture completed file checks.")
    if not task["release"].wait(timeout=30):
        raise AdapterError("Admission fixture release timed out.")
    return {"status": "eligible", "error": None,
            "entry": {**task["entry"], "admissionFiles": files, "workerPID": os.getpid()}}


def test_spawned_eligible_admission_is_committed_only_by_scheduler(environment):
    import multiprocessing
    import os
    import time
    from legowalkers_sim.prescreen import ScreeningPool
    from test_prescreen import options, reap_until

    store, archive, generator, _ = environment
    root = fork(store, archive)
    child = store.apply(root["node"], "a"*64, generator)["children"][0]
    store.contribute(child, "New candidate", "Test", "", Path("."))
    startup = pipeline.begin_startup(archive)
    task, = user_pool.queue(archive, startup_batch=startup)
    release = multiprocessing.get_context("spawn").Event()
    task["release"] = release
    before = archive.db.execute("SELECT count(*) FROM births").fetchone()[0]
    frozen = archive.directory / "user-contributions" / task["entry"]["id"] / "candidates.json"
    with ScreeningPool(archive, 1, Path("."), options()) as pool:
        try:
            pool.submit_admission(task, startup, None, evaluator=eligible_admission_fixture)
            deadline = time.monotonic() + 30
            while not frozen.exists():
                assert time.monotonic() < deadline, "Admission worker did not freeze its candidate."
                time.sleep(.01)
            assert archive.db.execute("SELECT count(*) FROM births").fetchone()[0] == before
            assert archive.db.execute("SELECT count(*) FROM user_pool_entries").fetchone()[0] == 0
        finally:
            release.set()
        assert reap_until(pool) == []
    entry = archive.db.execute("SELECT * FROM user_pool_entries").fetchone()
    assert entry["status"] == "eligible" and json.loads(entry["provenance"])["workerPID"] != os.getpid()
    assert archive.db.execute("SELECT count(*) FROM births").fetchone()[0] == before + 1
    assert discovery._pending(archive)["birth"] == entry["birth"]
    assert archive.counts() == {"complete": 1}


def test_contribution_timeout_is_visible_and_does_not_block_other_admissions(environment, tmp_path, monkeypatch):
    import subprocess
    import shutil
    from legowalkers_sim import leg_family_migration
    from test_native_discovery import mock_constructor

    store, archive, generator, _ = environment
    root = fork(store, archive)
    child = store.apply(root["node"], "a"*64, generator)["children"][0]
    store.contribute(root["node"], "Slow walker", "Test", "", Path("."))
    store.contribute(child, "Next walker", "Test", "", Path("."))
    selected = dict(archive.protocol())
    selected["nativeDiscovery"] = discovery.POLICY
    mock_constructor(tmp_path, monkeypatch, selected)
    monkeypatch.setattr(archive, "protocol", lambda: selected)
    def convert(source, cache, *args):
        target = cache / Path(source).name
        shutil.copytree(source, target)
        write_json(target / "leg-family-migration.json", {"designs": []})
        return target

    monkeypatch.setattr(leg_family_migration, "convert", convert)
    calls = []
    original = Proposals.build_native

    def build(self, parent, output, **kwargs):
        jobs = [item for item in pipeline.startup_snapshot(archive.db, True)["items"] if item["kind"] == "admission"]
        running = next(item for item in jobs if item["status"] == "running")
        assert "building: Building attempt 1/" in running["detail"]
        calls.append(running["label"])
        if len(calls) == 1:
            output.mkdir()
            raise subprocess.TimeoutExpired(["build-walker"], 600)
        return original(self, parent, output, **kwargs)

    monkeypatch.setattr(Proposals, "build_native", build)
    before = archive.counts()
    batch = pipeline.begin_startup(archive)
    with pipeline.startup_step(archive, batch, "contributions"):
        user_pool.sync(archive, Path("."), startup_batch=batch, generator=generator)
    jobs = [item for item in pipeline.startup_snapshot(archive.db, True)["items"] if item["kind"] == "admission"]
    assert [(item["label"], item["status"]) for item in jobs] == [
        ("Slow walker", "rejected"), ("Next walker", "complete")]
    assert "timed out" in jobs[0]["detail"] and archive.counts() == before
    batch = pipeline.begin_startup(archive)
    user_pool.sync(archive, Path("."), startup_batch=batch, generator=generator)
    assert calls == ["Slow walker", "Next walker"] and archive.counts() == before


def test_historical_contribution_admission_imports_verified_copy_without_changing_submission(environment, tmp_path, monkeypatch):
    import shutil
    from legowalkers_sim.bridge import SIMULATOR_REVISION, load_manifest
    from legowalkers_sim.native_episode import _queue

    store, archive, _, _ = environment
    node = fork(store, archive)["node"]
    candidate = store.candidate(node)
    manifest = json.loads((candidate / "manifest.json").read_text())
    manifest["motion"]["cycleSteps"] = 72
    write_json(candidate / "manifest.json", manifest)
    contribution = store.contribute(node, "Historical L", "Test", "", Path("."))["contribution"]
    manifest["simulator"]["revision"] = "0" * 40
    write_json(candidate / "manifest.json", manifest)
    with store.connect() as db:
        files = json.loads(db.execute("SELECT files FROM contributions WHERE id=?", (contribution,)).fetchone()[0])
        files[f"{candidate.name}/manifest.json"] = hashlib.sha256((candidate / "manifest.json").read_bytes()).hexdigest()
        db.execute("UPDATE contributions SET files=? WHERE id=?", (json.dumps(files), contribution))
    original = fingerprint(candidate.parent)
    compiler = tmp_path / "compiler"
    compiler.mkdir()
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        (compiler / name).write_bytes(b"fixture")
    generated = []

    def invoke(*args):
        generated.append(args)
        source = (Path(args[args.index("--parent") + 1]) if args[0] == "migrate-walker-legs"
                  else Path(args[args.index("--program") + 1]).parent)
        target = Path(args[args.index("--output") + 1]) / "candidate-001"
        shutil.copytree(source, target)
        updated = json.loads((target / "manifest.json").read_text())
        updated["simulator"]["revision"] = SIMULATOR_REVISION
        write_json(target / "manifest.json", updated)
        if args[0] == "migrate-walker-legs":
            write_json(target.parent / "leg-family-conversion.json", {
                "status": "no-matching-legs", "families": [], "unmatched": [
                    {"foot": "f0", "reason": "Shared passive branch."}]})

    screened = []

    def screen(batch, *args, **kwargs):
        summary, _ = _queue(Path(batch))
        path = Path(batch) / summary["generated"][0]["directory"] / "manifest.json"
        assert load_manifest(path)["simulator"]["revision"] == SIMULATOR_REVISION
        screened.append(path)
        return {"results": [{"status": "ready-for-dynamics"}]}

    monkeypatch.setattr(screening, "screen_batch", screen)
    generator = SimpleNamespace(command=["dotnet", str(compiler / "LegoWalkers.Cli.dll")], invoke=invoke)
    before = archive.counts()
    user_pool.sync(archive, Path("."), generator=generator)
    entry = archive.db.execute("SELECT * FROM user_pool_entries WHERE contribution=?", (contribution,)).fetchone()
    assert entry["status"] == "eligible"
    provenance = json.loads(entry["provenance"])
    assert json.loads(provenance["files"]) == files
    assert provenance["admissionFiles"] != files
    assert "export-migration.json" in provenance["admissionFiles"]
    assert fingerprint(candidate.parent) == original
    assert archive.counts() == before
    assert (archive.directory / "user-contribution-inputs" / contribution / candidate.name / "manifest.json").read_bytes() == (candidate / "manifest.json").read_bytes()
    user_pool.sync(archive, Path("."), generator=generator)
    assert len(generated) == 2 and len(screened) == 1
    assert provenance["legFamilyConversion"]["designs"][0]["status"] == "no-matching-legs"
    assert archive.counts() == before


def test_contribution_conversion_admits_shared_copy_and_preserves_saved_node(environment, tmp_path):
    import shutil

    store, archive, _, _ = environment
    node = fork(store, archive)["node"]
    contribution = store.contribute(node, "Shared legacy parent", "Test", "", Path("."))["contribution"]
    original = fingerprint(store.candidate(node).parent)
    binary = tmp_path / "compiler"
    binary.mkdir()
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        (binary / name).write_bytes(b"fixture")
    calls = []
    converted_id = hashlib.sha256(b"shared contribution").hexdigest()

    def invoke(*args):
        calls.append(args)
        destination = Path(args[args.index("--output")+1]) / "candidate-001"
        shutil.copytree(Path(args[args.index("--parent")+1]), destination)
        manifest = json.loads((destination / "manifest.json").read_text())
        manifest["genome"]["legFamilies"] = []
        manifest["motion"]["genotypeId"] = converted_id
        write_json(destination / "manifest.json", manifest)
        write_json(destination / "realization.json", manifest["genome"])
        write_json(destination.parent / "leg-family-conversion.json",
                   {"status": "converted", "families": [], "unmatched": []})

    generator = SimpleNamespace(command=["dotnet", str(binary / "LegoWalkers.Cli.dll")], invoke=invoke)
    before = archive.counts()
    user_pool.sync(archive, Path("."), generator=generator)
    entry = archive.db.execute("SELECT * FROM user_pool_entries WHERE contribution=?", (contribution,)).fetchone()
    assert entry["status"] == "eligible"
    assert json.loads(entry["provenance"])["legFamilyConversion"]["designs"][0]["targetDesign"] == converted_id
    birth = archive.db.execute("SELECT design,parent FROM births WHERE id=?", (entry["birth"],)).fetchone()
    assert birth["design"] == converted_id and birth["parent"] == 1
    assert archive.counts() == before
    assert fingerprint(store.candidate(node).parent) == original
    user_pool.sync(archive, Path("."), generator=generator)
    assert len(calls) == 1


def test_changed_contribution_is_rejected_before_import_or_screening(environment):
    store, archive, _, calls = environment
    node = fork(store, archive)["node"]
    store.contribute(node, "Do not alter", "Test", "", Path("."))
    (store.candidate(node) / "program.json").write_text('{"tampered":true}')
    before = archive.counts(), len(calls)
    with pytest.raises(AdapterError, match="Native batch changed"):
        user_pool.sync(archive, Path("."))
    assert (archive.counts(), len(calls)) == before


def fingerprint(path):
    return {str(file.relative_to(path)): hashlib.sha256(file.read_bytes()).hexdigest()
            for file in path.rglob("*") if file.is_file() and file.suffix != ".sqlite3"}


def test_persistent_fork_edit_keep_undo_branch_and_discard(environment):
    store, archive, generator, calls = environment
    before = fingerprint(archive.directory)
    root = fork(store, archive)
    assert {edit["level"] for edit in store.edits(root["node"], generator)["edits"]} == {"motion", "realization"}
    children = store.apply(root["node"], "a"*64, generator)["children"]
    assert len(children) == 1
    first = children[0]
    store.select(first, "keep")
    next_child = store.apply(first, "b"*64, generator)["children"][0]
    store.select(next_child, "keep")
    store.select(root["node"], "checkout")
    sibling = store.apply(root["node"], "b"*64, generator)["children"][0]
    store.select(sibling, "discard")
    fresh = Workbench(store.root).state()
    assert fresh["workspaces"][0]["head"] == root["node"]
    nodes = {node["id"]: node for node in fresh["nodes"]}
    assert nodes[next_child]["parent"] == first
    assert nodes[sibling]["parent"] == root["node"] and nodes[sibling]["status"] == "discarded"
    assert fingerprint(archive.directory) == before
    assert archive.counts() == {"complete": 1} and calls == []
    with pytest.raises(AdapterError, match="discarded"):
        store.apply(sibling, "a"*64, generator)
    with pytest.raises(AdapterError, match="unavailable"):
        store.apply(root["node"], "c"*64, generator)


def test_rejected_edits_remain_visible_without_a_physical_child(environment):
    store, archive, generator, _ = environment
    root = fork(store, archive)
    generator.actions = ["reject"]
    child = store.apply(root["node"], "a"*64, generator)["children"][0]
    node = next(node for node in store.state()["nodes"] if node["id"] == child)
    assert node["status"] == "rejected" and not node["physical"]
    assert node["summary"]["gates"][0]["outcome"] == "reject"
    with pytest.raises(AdapterError, match="rejected"):
        store.contribute(child, "Invalid", "Test", "", Path("."))


def test_contribution_uses_explicit_intermediate_version_not_head_or_endpoint(environment):
    store, archive, generator, calls = environment
    root = fork(store, archive)
    intermediate = store.apply(root["node"], "a"*64, generator)["children"][0]
    endpoint = store.apply(intermediate, "b"*64, generator)["children"][0]
    store.select(root["node"], "checkout")
    assert store.state()["workspaces"][0]["head"] == root["node"]
    submitted = store.contribute(intermediate, "Intermediate version", "Test", "", Path("."))
    contribution, = store.state()["contributions"]
    assert contribution["id"] == submitted["contribution"]
    assert contribution["node"] == intermediate and contribution["node"] not in (root["node"], endpoint)
    assert Path(calls[-1][0]) == store.candidate(intermediate).parent
    assert store.state()["workspaces"][0]["head"] == root["node"]
    assert archive.counts() == {"complete": 1}


def test_unmeasured_contribution_can_parent_automatic_offspring_and_withdrawal_is_snapshotted(environment):
    store, archive, generator, calls = environment
    root = fork(store, archive)
    child = store.apply(root["node"], "a"*64, generator)["children"][0]
    submitted = store.contribute(child, "Human idea", "Test", "Try this geometry", Path("."))
    assert submitted["status"] == "unmeasured" and submitted["dynamicTrials"] == 0
    assert calls[-1][1]["max_checks"] == 1
    user_pool.sync(archive, Path("."))
    entry = archive.db.execute("SELECT * FROM user_pool_entries").fetchone()
    assert entry["status"] == "eligible" and entry["birth"] != 1
    assert len(archive.outcomes()) == 1
    seed = archive.db.execute("SELECT seed FROM run").fetchone()[0]
    chosen = user_pool.choose(archive, seed, 0)
    assert chosen[0]["birth"] == entry["birth"]
    assert chosen == user_pool.choose(archive, seed, 0)
    samples = sum(user_pool.choose(archive, seed, index) is not None for index in range(10000))
    assert 1800 <= samples <= 2200
    discovery._propose(archive, generator, 1, founder=1)
    proposal = json.loads(archive.db.execute("SELECT summary FROM native_proposals").fetchone()[0])
    assert proposal["parentSelection"]["mode"] == "user-pool"
    assert proposal["parentSelection"]["contribution"] == submitted["contribution"]
    offspring = archive.db.execute("SELECT * FROM births ORDER BY id DESC LIMIT 1").fetchone()
    assert offspring["parent"] == entry["birth"] and archive.founder(offspring["id"]) == 1
    assert discovery._pending(archive)["birth"] == entry["birth"]
    assert discovery._neighborhood(archive) == 1
    assert "queued for evaluation" in user_pool.status(archive.directory)[0]["measurement"]
    assert archive.counts() == {"complete": 1}
    store.withdraw(submitted["contribution"])
    assert user_pool.choose(archive, seed, 0) is not None  # Existing episode snapshot is immutable.
    user_pool.sync(archive, Path("."))
    assert user_pool.choose(archive, seed, 0) is None
    assert archive.db.execute("SELECT count(*) FROM user_pool_snapshots").fetchone()[0] == 2
    assert archive.counts() == {"complete": 1}


def test_failed_admission_is_explicit_and_cannot_enter_pool(environment, monkeypatch):
    store, archive, _, _ = environment
    root = fork(store, archive)
    monkeypatch.setattr(module, "screen_batch", lambda *a, **k: {
        "results": [{"status": "rejected", "error": "Fixture interference", "gates": []}]})
    with pytest.raises(AdapterError, match="static validation"):
        store.contribute(root["node"], "Bad", "Test", "", Path("."))
    assert store.state()["contributions"] == []
    assert (store.root / "validations" / root["node"] / "latest.json").is_file()
    assert archive.counts() == {"complete": 1}


def admitted_child(environment):
    store, archive, _, _ = environment
    root = fork(store, archive)
    child = store.apply(root["node"], "a"*64, environment[2])["children"][0]
    contribution = store.contribute(child, "Evaluate this exact design", "Test", "", Path("."))
    user_pool.sync(archive, Path("."))
    return archive.db.execute("SELECT birth FROM user_pool_entries WHERE contribution=?",
                              (contribution["contribution"],)).fetchone()[0]


@pytest.mark.parametrize("parallel", [False, True])
def test_admitted_design_itself_reaches_evaluation_before_offspring(environment, monkeypatch, parallel):
    from legowalkers_sim import prescreen
    from legowalkers_sim.evaluation import EvaluationOptions
    from types import SimpleNamespace

    _, archive, generator, _ = environment
    birth = admitted_child(environment)
    submitted = []
    budget = archive.db.execute("SELECT budget FROM run").fetchone()[0]

    class Trials:
        jobs = []
        exhausted = False
        full = False
        pending_gates = 0
        generation_step = 1

        def reap(self, **kwargs):
            pass

        def drain(self):
            pass

        def submit(self, selected, checked, options):
            assert selected == birth
            submitted.append(selected)
            archive.finish(archive.reserve(selected), result(qualified=True))

    monkeypatch.setattr(discovery, "_propose", lambda *a, **k: pytest.fail("Mutated before evaluating contribution"))
    if parallel:
        execution = archive.protocol().get("execution", {})
        assert execution["screeningBuffer"] >= 1

        class Screens:
            jobs = []
            constructed = []
            births = set()
            full = False
            failures = 0
            admitting = False

            def __init__(self, *args):
                self.ready = []

            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def reap(self):
                ready, self.ready = self.ready, []
                return ready

            def submit(self, pending, used, maximum):
                assert pending["birth"] == birth
                self.ready.append((birth, SimpleNamespace(report={})))

        monkeypatch.setattr(prescreen, "ScreeningPool", Screens)
        discovery._dispatch_parallel(archive, generator, Path("."), EvaluationOptions(), Trials(), 2, 2)
    else:
        monkeypatch.setattr(discovery, "_screen_pending", lambda *args: Path("exact-contribution/manifest.json"))
        monkeypatch.setattr(discovery, "evaluate_native_candidate",
                            lambda a, selected, path, library, options, pool: pool.submit(selected, None, options) or 0)
        discovery._dispatch(archive, generator, Path("."), EvaluationOptions(), Trials(), 2)
    assert submitted == [birth]
    assert archive.counts() == {"complete": 2}
    assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == budget
    assert discovery._pending(archive) is None
    user_pool.sync(archive, Path("."))
    assert discovery._pending(archive) is None
    assert user_pool.status(archive.directory)[0]["measurement"] == "qualified"


def test_contribution_does_not_repeat_existing_measurement_or_rejected_attempt(environment):
    store, archive, _, _ = environment
    root = fork(store, archive)
    store.contribute(root["node"], "Already measured", "Test", "", Path("."))
    user_pool.sync(archive, Path("."))
    assert discovery._pending(archive) is None
    assert archive.db.execute("SELECT COUNT(*) FROM births").fetchone()[0] == 1
    birth = admitted_child(environment)
    with archive.db:
        archive.db.execute("UPDATE births SET status='simulation-rejected' WHERE id=?", (birth,))
    user_pool.sync(archive, Path("."))
    assert discovery._pending(archive) is None
    assert archive.counts() == {"complete": 1}


def test_previously_admitted_unmeasured_parent_is_queued_once_without_new_admission(environment):
    _, archive, _, calls = environment
    birth = admitted_child(environment)
    with archive.db:
        archive.db.execute("UPDATE births SET status='user-parent' WHERE id=?", (birth,))
    before = len(calls)
    user_pool.sync(archive, Path("."))
    assert len(calls) == before
    assert discovery._pending(archive)["birth"] == birth
    assert discovery._pending(archive, exclude={birth}) is None
    user_pool.sync(archive, Path("."))
    assert discovery._pending(archive)["birth"] == birth
    assert archive.db.execute("SELECT COUNT(*) FROM user_pool_entries").fetchone()[0] == 1
    assert archive.counts() == {"complete": 1}


def test_capacity_is_exactly_twelve_and_ids_cannot_escape_store(environment):
    store, archive, _, _ = environment
    root = fork(store, archive)
    with store.connect() as db:
        for index in range(12):
            db.execute("INSERT INTO contributions VALUES(?,?,?,?,?,1,?,?,?)",
                       (uuid.uuid4().hex, uuid.uuid4().hex, str(index), "Test", "", "{}", "{}", "now"))
    with pytest.raises(AdapterError, match="full"):
        store.contribute(root["node"], "Thirteenth", "Test", "", Path("."))
    for bad in ("../archive", "/etc/passwd", "", "z"*32, 123):
        with pytest.raises(AdapterError):
            store.candidate(bad)
    assert user_pool.POLICY["capacity"] == 12 and user_pool.POLICY["parentProbability"] == .2


def test_duplicate_design_and_active_discard_are_explicit(environment):
    store, archive, _, _ = environment
    first = fork(store, archive)
    store.contribute(first["node"], "Original", "Test", "", Path("."))
    second = fork(store, archive)
    with pytest.raises(AdapterError, match="already active"):
        store.contribute(second["node"], "Duplicate", "Test", "", Path("."))


def test_option_diagrams_do_not_save_nodes_or_change_head_and_clean_temporary_exports(environment, monkeypatch):
    from legowalkers_sim import workbench_diagrams
    store, archive, generator, calls = environment
    root = fork(store, archive)
    paths = []

    def diagram(candidate, library):
        assert (candidate / "manifest.json").is_file()
        paths.append(candidate)
        return {"parts": [], "joints": [], "feet": [], "paths": []}

    monkeypatch.setattr(workbench_diagrams, "model_diagram", diagram)
    before = store.state()
    value = store.preview(root["node"], "a"*64, generator, Path("."))
    assert value["dynamicTrials"] == 0 and len(value["alternatives"]) == 1
    assert value["alternatives"][0]["comparison"]["motionUnchanged"] is True
    assert store.state() == before and not calls
    assert all(not path.exists() for path in paths if path != store.candidate(root["node"]))
    assert archive.counts() == {"complete": 1}
    for edit in ("../escape", "c"*64):
        with pytest.raises(AdapterError):
            store.preview(root["node"], edit, generator, Path("."))


def test_option_rejection_and_geometry_errors_are_explicit_and_never_create_branches(environment, monkeypatch):
    from legowalkers_sim import workbench_diagrams
    store, archive, generator, _ = environment
    root = fork(store, archive)
    before = store.state()
    monkeypatch.setattr(workbench_diagrams, "model_diagram", lambda *args: {"parts": []})
    monkeypatch.setattr(generator, "apply_edit", lambda *args: {
        "generated": [], "gates": [{"outcome": "reject", "code": "fixture"}]})
    value = store.preview(root["node"], "a"*64, generator, Path("."))
    assert value["alternatives"] == [] and value["summary"]["gates"][0]["outcome"] == "reject"

    def broken(*args):
        raise AdapterError("Missing diagram mesh")

    monkeypatch.setattr(workbench_diagrams, "model_diagram", broken)
    with pytest.raises(AdapterError, match="Missing diagram mesh"):
        store.preview(root["node"], "a"*64, generator, Path("."))
    assert store.state() == before


def test_option_preview_exposes_advisory_clearance_and_repair_evidence_without_admitting_a_design(environment, monkeypatch):
    from legowalkers_sim import workbench_diagrams
    store, archive, generator, calls = environment
    root = fork(store, archive)
    before = store.state()
    apply = generator.apply_edit
    report = {"version": "kinematic-clearance-v1", "status": "warning", "suspectPairs": 1,
              "score": 25, "dynamicTrials": 0, "contacts": [{"parts": ["leg", "receiver"]}]}
    repair = {"before": {"score": 50}, "after": report, "motionUnchanged": True}

    def generate(parent, output, edit):
        summary = apply(parent, output, edit)
        for entry in summary["generated"]:
            candidate = output / entry["directory"]
            write_json(candidate / "motion-clearance.json", report)
            write_json(candidate / "motion-clearance-repair.json", repair)
        return summary

    monkeypatch.setattr(generator, "apply_edit", generate)
    monkeypatch.setattr(workbench_diagrams, "model_diagram", lambda *args: {"parts": [], "paths": []})
    value = store.preview(root["node"], "a"*64, generator, Path("."))
    assert value["alternatives"][0]["clearance"] == report
    assert value["alternatives"][0]["clearanceRepair"] == repair
    assert store.state() == before and not calls
    assert archive.counts() == {"complete": 1}


@pytest.mark.skipif(not (ROOT / ".tools/dotnet/dotnet").exists(), reason="Local F# SDK required")
def test_real_cli_lists_and_applies_both_levels_and_preserves_archived_parent_identity(tmp_path):
    generator = Generator(ROOT / ".tools/dotnet/dotnet", ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    generator.invoke("realize-walker", "--feet", 2, "--steps", 72, "--output", tmp_path / "seed")
    parent = tmp_path / "seed/candidate-001"
    path = parent / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["motion"]["genotypeId"] = "e"*64
    path.write_text(json.dumps(manifest))
    catalog = generator.manual_edits(parent)
    assert catalog == generator.manual_edits(parent)
    assert {entry["level"] for entry in catalog["edits"]} == {"motion", "realization", "coupled"}
    for operation, level in (("input-phase", "motion"), ("battery-layout", "realization")):
        edits = [entry for entry in catalog["edits"] if entry["operation"] == operation]
        assert edits and all(entry["level"] == level for entry in edits)
        summaries = []
        for index, edit in enumerate(edits[:2]):
            output = tmp_path / f"{operation}-{index}"
            summary = generator.apply_edit(parent, output, edit["id"])
            summaries.append(summary)
            assert summary["parentGenotypeId"] == "e"*64
            assert summary["selectedEdit"] == edit["id"]
            assert summary["counts"]["dynamicTrials"] == 0
            for entry in summary["generated"]:
                child = output / entry["directory"]
                same_program = json.loads((child / "program.json").read_text()) == json.loads((parent / "program.json").read_text())
                assert same_program is (level == "realization")
        assert any(summary["generated"] for summary in summaries)
