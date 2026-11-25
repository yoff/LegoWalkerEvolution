import multiprocessing
import os
import time
from types import SimpleNamespace

import pytest

from legowalkers_sim import discovery, native_episode, prescreen, screening
from legowalkers_sim.bridge import AdapterError, load_manifest
from legowalkers_sim.evolution import Archive
from test_evolution import protocol, result
from test_native_discovery import Proposals
from test_native_episode import native_fixture


def screened_fixture(batch, output, design, manifest, library, options, stage):
    if getattr(options, "barrier", None) is not None:
        options.barrier.wait(timeout=30)
    stage("preflight")
    if int(output.name) in getattr(options, "hold_births", ()):
        if not options.release.wait(timeout=30):
            raise AdapterError("Test screening release timed out.")
    mode = getattr(options, "mode", "")
    if mode == "error":
        raise AdapterError("Intentional screening failure.")
    if mode == "crash":
        os._exit(7)
    if mode == "rejected":
        return {"status": "mass-layout-rejected", "checks": 1, "error": "Invalid mass layout fixture."}
    return {"status": "invalid" if mode == "invalid-status" else "ready",
            "checks": 2 if mode == "invalid-checks" else 1,
            "checked": SimpleNamespace(manifest=load_manifest(manifest), report={"pid": os.getpid()},
                                       reject=getattr(options, "reject_prefix", False))}


@pytest.fixture
def candidates(tmp_path):
    archive, pending = make_candidates(tmp_path)
    yield archive, pending
    archive.db.close()


def make_candidates(tmp_path, count=2, budget=2):
    source, _ = native_fixture(tmp_path)
    siblings = tmp_path / "siblings"
    Proposals(siblings=count).propose_native(source / "candidate-0", siblings, 42, 0, count)
    archive = Archive(tmp_path / "archive")
    definition = native_episode.native_protocol(protocol(), siblings, discover=True, screening_workers=2)
    definition["nativeDiscovery"] = dict(definition["nativeDiscovery"])
    definition["nativeDiscovery"].pop("motionCarryForward")
    archive.initialize(definition,
                       budget, 42, False)
    native_episode.stage_native(archive, siblings)
    pending = archive.db.execute(
        "SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth ORDER BY birth").fetchall()
    return archive, pending


def options(**values):
    return SimpleNamespace(payload_profile=None, cycle_aware=False, **values)


def reap_until(pool):
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        ready = pool.reap()
        if ready or not pool.jobs:
            return ready
        pool.wait(SimpleNamespace(jobs=[]))
    pytest.fail("Screening workers did not finish within 30 seconds.")


def admission_fixture(task, directory, generator, library, payload_profile, stage):
    stage("building", "Building attempt 1/1.")
    if task.get("barrier") is not None:
        task["barrier"].wait(timeout=30)
    if task.get("release") is not None and not task["release"].wait(timeout=30):
        raise AdapterError("Test admission release timed out.")
    return {"status": "construction-rejected", "error": "Bounded construction rejected fixture.",
            "entry": task["entry"]}


class FixtureConstructor(Proposals):
    def build_native(self, parent, output, **kwargs):
        import subprocess

        if load_manifest(parent / "manifest.json")["motion"]["genotypeId"] == getattr(self, "fail_design", None):
            output.mkdir()
            raise subprocess.TimeoutExpired(["build-walker"], 600)
        return super().build_native(parent, output, **kwargs)


def construction_fixture(task, directory, generator, library, payload_profile, stage):
    from legowalkers_sim import motion_carry

    motion_carry._centering = lambda *args: (None, None)
    return motion_carry.prepare_construction(task, directory, generator, library, payload_profile, stage)


class FixtureScreens(prescreen.ScreeningPool):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, construction_evaluator=construction_fixture, **kwargs)

    def submit_construction(self, task, generator):
        compiler = FixtureConstructor()
        compiler.command = generator.command
        compiler.fail_design = getattr(generator, "fail_design", None)
        return super().submit_construction(task, compiler)


def held_construction_fixture(task, directory, generator, library, payload_profile, stage):
    stage("building", "Bounded constructor fixture is waiting.")
    task["barrier"].wait(timeout=30)
    if not task["release"].wait(timeout=30):
        raise AdapterError("Construction fixture release timed out.")
    return construction_fixture(task, directory, generator, library, payload_profile, stage)


def test_dispatch_collects_admissions_and_refills_workers_while_real_construction_is_blocked(tmp_path, monkeypatch):
    from legowalkers_sim import motion_carry
    from test_native_discovery import setup_discovery

    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=1, screening_workers=2)
    native_episode.stage_native(archive, source)
    context = multiprocessing.get_context("spawn")
    release, barrier = context.Event(), context.Barrier(2)
    startup, tasks = admission_tasks(archive)
    tasks[0]["barrier"] = barrier
    observed = []
    actual = prescreen.ScreeningPool

    class Screens(actual):
        def __init__(self, *args):
            super().__init__(*args, evaluator=screened_fixture, admission_evaluator=admission_fixture,
                             construction_evaluator=held_construction_fixture)

        def submit_construction(self, task, generator):
            compiler = FixtureConstructor()
            compiler.command = generator.command
            super().submit_construction({**task, "barrier": barrier, "release": release}, compiler)

        def wait(self, simulations):
            if archive.db.execute("SELECT count(*) FROM user_pool_entries").fetchone()[0] == 2 and not release.is_set():
                job, = self.jobs
                assert isinstance(job, prescreen.ConstructionJob) and job.process.is_alive()
                assert archive.counts() == {}
                assert archive.db.execute("SELECT count(*) FROM native_checks").fetchone()[0] == 0
                assert archive.db.execute("SELECT stage FROM pipeline_events WHERE birth=1 ORDER BY id DESC").fetchone()[0] == "building"
                observed.append(True)
                release.set()
            super().wait(simulations)

    class Simulations:
        jobs = []
        full = False
        exhausted = False
        pending_gates = 0

        @property
        def generation_step(self):
            return sum(archive.counts().values())

        def reap(self):
            return 0

        def drain(self):
            assert not self.jobs

        def submit(self, birth, checked, settings):
            assert observed, "Admission results were not collected during construction."
            motion_carry.require_construction(archive, birth, checked)
            measured = result(qualified=True)
            measured["metrics"]["logicalFootCount"] = len(checked.manifest["feet"])
            archive.finish(archive.reserve(birth), measured)

    monkeypatch.setattr(prescreen, "ScreeningPool", Screens)
    try:
        discovery._dispatch_parallel(archive, Proposals(), tmp_path, options(), Simulations(), 1, 2,
                                     admissions=tasks, startup=startup)
        assert observed == [True] and archive.counts() == {"complete": 1}
        assert archive.db.execute("SELECT count(*) FROM user_pool_entries").fetchone()[0] == 2
        assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 1
    finally:
        release.set()
        archive.db.close()


def admission_tasks(archive, count=2, **values):
    from legowalkers_sim import pipeline, user_pool

    user_pool.queue(archive)
    startup = pipeline.begin_startup(archive)
    tasks = [{"entry": {"id": f"{index:032x}", "label": f"Contribution {index}"},
              "parent": 1, **values} for index in range(count)]
    pipeline.queue_startup(archive, startup,
                           [(task["entry"]["id"], "admission", task["entry"]["label"]) for task in tasks])
    return startup, tasks


def test_admission_and_native_screen_share_capacity_and_named_live_stages(candidates, tmp_path):
    from legowalkers_sim.pipeline import snapshot
    from legowalkers_sim.previews import PreviewController
    from legowalkers_sim.rendering import RenderController

    archive, pending = candidates
    PreviewController(archive.directory, tmp_path, None)
    RenderController(archive.directory, tmp_path, None)
    context = multiprocessing.get_context("spawn")
    release, barrier = context.Event(), context.Barrier(2)
    startup, tasks = admission_tasks(archive, barrier=barrier, release=release)
    with prescreen.ScreeningPool(archive, 2, tmp_path, options(barrier=barrier),
                                screened_fixture, admission_fixture) as pool:
        try:
            pool.submit_admission(tasks[0], startup, None)
            pool.submit(pending[0], 0, 8)
            assert pool.full and pool.births == {1} and pool.admitting
            with pytest.raises(AdapterError, match="pool is full"):
                pool.submit_admission(tasks[1], startup, None)
            assert [birth for birth, _ in reap_until(pool)] == [1]
            assert len(pool.jobs) == 1 and pool.jobs[0].process.is_alive()
            prescreen.record(archive, 2, "building", "Ordinary candidate construction.")
            live = snapshot(archive.db, archive.directory, running=True, tracked=True)
            building = next(stage for stage in live["stages"] if stage["id"] == "building")
            assert building["count"] == 2 and {item["birth"] for item in building["items"]} == {None, 2}
            item, = [item for item in building["items"] if item["birth"] is None]
            assert item["name"] == "Contribution 0" and item["history"] is None
            assert item["pid"] == pool.jobs[0].process.pid
            assert item["log"].startswith("admission-workers/")
            assert archive.db.execute("SELECT count(*) FROM user_pool_entries").fetchone()[0] == 0
            assert archive.counts() == {}
        finally:
            release.set()
        assert reap_until(pool) == []
    assert archive.db.execute("SELECT status FROM user_pool_entries").fetchone()[0] == "construction-rejected"
    assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 1
    assert archive.counts() == {}


@pytest.mark.parametrize("workers", [1, 2])
@pytest.mark.parametrize("limit", ["budget", "cheap-gate"])
def test_dispatch_does_not_wait_for_all_admissions_or_start_new_ones_after_budget(
        tmp_path, monkeypatch, workers, limit):
    archive, _ = make_candidates(tmp_path, count=1, budget=1)
    context = multiprocessing.get_context("spawn")
    release = context.Event()
    barrier = context.Barrier(2) if workers == 2 else None
    startup, tasks = admission_tasks(archive)
    if workers == 2:
        tasks[0].update(release=release, barrier=barrier)
    actual_pool = prescreen.ScreeningPool
    monkeypatch.setattr(prescreen, "ScreeningPool",
                        lambda *args: actual_pool(*args, evaluator=screened_fixture,
                                                  admission_evaluator=admission_fixture))

    class Simulations:
        jobs = []
        full = False
        exhausted = False
        pending_gates = 0

        @property
        def generation_step(self):
            return sum(archive.counts().values())

        def reap(self):
            return 0

        def drain(self):
            assert not self.jobs

        def submit(self, birth, checked, settings):
            if workers == 2:
                assert archive.db.execute("SELECT status FROM pipeline_startup WHERE batch=? AND task=?",
                                          (startup, tasks[0]["entry"]["id"])).fetchone()[0] == "running"
            release.set()
            if limit == "cheap-gate":
                self.exhausted = True
                return
            measured = result(qualified=True)
            measured["metrics"]["logicalFootCount"] = len(checked.manifest["feet"])
            archive.finish(archive.reserve(birth), measured)

    try:
        assert discovery._dispatch_parallel(archive, Proposals(), tmp_path, options(barrier=barrier),
                                            Simulations(), 1, workers, admissions=tasks, startup=startup) == 0
        assert archive.counts() == ({"complete": 1} if limit == "budget" else {})
        assert archive.db.execute("SELECT status FROM pipeline_startup WHERE batch=? AND task=?",
                                  (startup, tasks[0]["entry"]["id"])).fetchone()[0] == "rejected"
        assert archive.db.execute("SELECT status FROM pipeline_startup WHERE batch=? AND task=?",
                                  (startup, tasks[1]["entry"]["id"])).fetchone()[0] == "queued"
        assert archive.db.execute("SELECT count(*) FROM user_pool_entries").fetchone()[0] == 1
        assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 1
    finally:
        release.set()
        archive.db.close()


def test_two_admissions_overlap_without_consuming_native_check_or_trial_allowance(candidates, tmp_path):
    archive, _ = candidates
    context = multiprocessing.get_context("spawn")
    release = context.Event()
    startup, tasks = admission_tasks(archive, barrier=context.Barrier(2), release=release)
    with prescreen.ScreeningPool(archive, 2, tmp_path, options(), admission_evaluator=admission_fixture) as pool:
        try:
            for task in tasks:
                pool.submit_admission(task, startup, None)
            assert pool.full and not pool.births and pool.admitting
            assert len({job.process.pid for job in pool.jobs}) == 2
            assert all(job.process.is_alive() for job in pool.jobs)
            assert pool.reap() == [] and archive.counts() == {}
        finally:
            release.set()
        assert reap_until(pool) == []
    assert archive.db.execute("SELECT count(*) FROM user_pool_entries").fetchone()[0] == 2
    assert archive.db.execute("SELECT count(*) FROM native_checks").fetchone()[0] == 0
    assert archive.counts() == {}


def admission_with_compiler(task, directory, generator, library, payload_profile, stage):
    import subprocess
    import sys

    partial = directory / "admission-partial"
    partial.mkdir()
    (partial / "attempt.json").write_text("{}")
    stage("building", "Waiting for compiler fixture.")
    subprocess.run([sys.executable, "-c",
                    "import os,sys,time; from pathlib import Path; "
                    "Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(60)",
                    str(partial / "compiler.pid")], check=True, timeout=60)


@pytest.mark.parametrize("kind", ["admission", "construction"])
def test_cancel_reaps_compiler_and_retains_partial_attempt(candidates, tmp_path, kind):
    archive, pending = candidates
    startup, tasks = admission_tasks(archive, count=1)
    with prescreen.ScreeningPool(archive, 1, tmp_path, options(),
                                construction_evaluator=admission_with_compiler) as pool:
        if kind == "admission":
            pool.submit_admission(tasks[0], startup, None, evaluator=admission_with_compiler)
        else:
            pool.submit_construction({"pending": dict(pending[0])}, None)
        pidfile = archive.directory / "admission-partial" / "compiler.pid"
        deadline = time.monotonic() + 30
        while not pidfile.exists():
            assert time.monotonic() < deadline, "Compiler child did not start."
            time.sleep(.01)
        pid = int(pidfile.read_text())
        pool.cancel("Explicit pause; partial construction retained.")
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
    assert (pidfile.parent / "attempt.json").exists()
    if kind == "admission":
        assert archive.db.execute("SELECT status FROM user_pool_entries").fetchone()[0] == "interrupted"
        assert archive.db.execute("SELECT status FROM pipeline_startup WHERE batch=? AND task=?",
                                  (startup, tasks[0]["entry"]["id"])).fetchone()[0] == "interrupted"
    else:
        assert archive.db.execute("SELECT status FROM births WHERE id=1").fetchone()[0] == "construction-interrupted"
        assert archive.db.execute("SELECT status FROM screening_workers WHERE birth=1").fetchone()[0] == "interrupted"
    assert archive.counts() == {}


def test_spawned_screening_overlaps_and_fast_sibling_is_ready_independently(candidates, tmp_path):
    from legowalkers_sim.pipeline import snapshot
    from legowalkers_sim.previews import PreviewController
    from legowalkers_sim.rendering import RenderController

    archive, pending = candidates
    PreviewController(archive.directory, tmp_path, None)
    RenderController(archive.directory, tmp_path, None)
    context = multiprocessing.get_context("spawn")
    release = context.Event()
    settings = options(barrier=context.Barrier(2), release=release, hold_births={2})
    with prescreen.ScreeningPool(archive, 2, tmp_path, settings, screened_fixture) as pool:
        try:
            for row in pending:
                pool.submit(row, 0, 8)
            assert pool.full and pool.births == {1, 2}
            assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 2
            with pytest.raises(AdapterError, match="pool is full"):
                pool.submit(pending[0], 0, 8)
            first = reap_until(pool)
            assert [birth for birth, _ in first] == [1]
            assert pool.births == {2} and pool.jobs[0].process.is_alive()
            deadline = time.monotonic() + 10
            while "Stage: preflight." not in (running_log := pool.jobs[0].log.read_text()):
                assert time.monotonic() < deadline, running_log
                time.sleep(.01)
            assert "Screening worker" in running_log and "Stage: preflight." in running_log
            assert "Worker finished" not in running_log
            assert archive.counts() == {}
            pipeline = snapshot(archive.db, archive.directory, running=True, tracked=True)
            slow = next(item for stage in pipeline["stages"] for item in stage["items"] if item["birth"] == 2)
            assert slow["pid"] == pool.jobs[0].process.pid
            assert slow["log"].startswith("preflight-workers/")
        finally:
            release.set()
        second = reap_until(pool)
    assert [birth for birth, _ in second] == [2]
    for row in archive.db.execute("SELECT log FROM screening_workers"):
        assert "Worker finished" in (archive.directory / row["log"]).read_text()
    pids = [checked.report["pid"] for _, checked in first + second]
    assert len(set(pids)) == 2 and os.getpid() not in pids
    assert archive.counts() == {} and archive.db.execute("SELECT budget FROM run").fetchone()[0] == 2
    assert [row[0] for row in archive.db.execute(
        "SELECT stage FROM pipeline_events WHERE birth=1 ORDER BY id")] == [
            "screening", "screened", "preflight", "ready"]


def test_dispatches_fast_sibling_to_simulation_before_slow_preflight_finishes(candidates, tmp_path, monkeypatch):
    archive, _ = candidates
    context = multiprocessing.get_context("spawn")
    release = context.Event()
    settings = options(barrier=context.Barrier(2), release=release, hold_births={2})
    actual_pool = prescreen.ScreeningPool
    monkeypatch.setattr(prescreen, "ScreeningPool",
                        lambda *args: actual_pool(*args, evaluator=screened_fixture))

    class Simulations:
        jobs = []
        full = False
        exhausted = False
        pending_gates = 0

        @property
        def generation_step(self):
            return sum(archive.counts().values())

        def __init__(self):
            self.births = []

        def reap(self):
            return 0

        def drain(self):
            assert not self.jobs

        def submit(self, birth, checked, options):
            if not self.births:
                assert birth == 1
                assert archive.db.execute(
                    "SELECT status FROM screening_workers WHERE birth=2").fetchone()[0] == "running"
                release.set()
            self.births.append(birth)
            measured = result(qualified=True)
            measured["metrics"]["logicalFootCount"] = len(checked.manifest["feet"])
            archive.finish(archive.reserve(birth), measured)

    simulations = Simulations()
    generator = Proposals()
    try:
        assert discovery._dispatch_parallel(archive, generator, tmp_path, settings, simulations, 2, 2) == 0
        assert simulations.births == [1, 2] and generator.calls == []
        assert archive.counts() == {"complete": 2}
        assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 2
    finally:
        release.set()


@pytest.mark.parametrize("reject", [False, True])
def test_parallel_screening_and_prefix_handoffs_respect_full_budget_and_normal_cap_pause(tmp_path, monkeypatch, reject):
    from legowalkers_sim.parallel import TrialPool
    from test_native_discovery import setup_discovery
    from test_simulation_gate import rejected_or_complete

    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=4, screening_workers=2, replacement=True)
    native_episode.stage_native(archive, source)
    archive.finish(archive.reserve(1), result(qualified=True))
    settings = options(short_gate=2, reject_prefix=reject)
    actual_pool = FixtureScreens
    monkeypatch.setattr(prescreen, "ScreeningPool",
                        lambda *args: actual_pool(*args, evaluator=screened_fixture))
    try:
        with TrialPool(archive, 2, rejected_or_complete) as pool:
            assert discovery._dispatch_parallel(archive, Proposals(), tmp_path, settings, pool, 4, 2) == 0
        assert pool.failures == 0
        assert pool.rejections == (9 if reject else 0)
        assert pool.exhausted is reject
        assert archive.counts() == {"complete": 1 if reject else 4}
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 4
    finally:
        archive.db.close()


def test_cheap_check_reservations_enforce_bound_before_spawn(candidates, tmp_path):
    archive, pending = candidates
    with archive.db:
        archive.db.execute("INSERT INTO native_checks(after_trial,allowance,batch) VALUES (0,8,'prior')")
    with prescreen.ScreeningPool(archive, 2, tmp_path, options(), screened_fixture) as pool:
        with pytest.raises(AdapterError, match="reconstruction limit"):
            pool.submit(pending[0], 0, 8)
        assert not pool.jobs and archive.counts() == {}
    assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 8


@pytest.mark.parametrize("limit", ["reconstruction", "proposal"])
@pytest.mark.parametrize("rejected", [False, True])
def test_dispatch_finishes_inflight_screening_at_limit_before_continuing_or_pausing(
        tmp_path, monkeypatch, limit, rejected):
    archive, _ = make_candidates(tmp_path, count=3, budget=2)
    release = multiprocessing.get_context("spawn").Event()
    settings = options(release=release, hold_births={1}, mode="rejected" if rejected else "")
    actual_pool = prescreen.ScreeningPool
    waits = []

    class Screens(actual_pool):
        def __init__(self, *args):
            super().__init__(*args, evaluator=screened_fixture)

        def wait(self, simulations):
            waits.append(len(self.jobs))
            release.set()
            super().wait(simulations)

    class Simulations:
        jobs = []
        full = False
        exhausted = False
        pending_gates = 0

        @property
        def generation_step(self):
            return sum(archive.counts().values())

        def reap(self):
            return 0

        def drain(self):
            assert not self.jobs

        def submit(self, birth, checked, options):
            measured = result(qualified=True)
            measured["metrics"]["logicalFootCount"] = len(checked.manifest["feet"])
            archive.finish(archive.reserve(birth), measured)

    monkeypatch.setattr(prescreen, "ScreeningPool", Screens)
    if limit == "reconstruction":
        with archive.db:
            archive.db.execute("INSERT INTO native_checks(after_trial,allowance,batch) VALUES (0,7,'prior')")
    else:
        pending = discovery._pending
        monkeypatch.setattr(discovery, "_pending",
                            lambda archive, founder=None, *, exclude=(): None if exclude else pending(archive))
        monkeypatch.setattr(discovery, "_neighborhood", lambda *args, **kwargs: 1)

        def exhausted(*args):
            raise discovery.DiscoveryLimit("Motion proposal limit reached.")

        monkeypatch.setattr(discovery, "_propose", exhausted)
    try:
        if rejected:
            with pytest.raises(discovery.DiscoveryLimit, match="limit reached"):
                discovery._dispatch_parallel(archive, Proposals(), tmp_path, settings, Simulations(), 2, 2)
            assert archive.counts() == {}
        else:
            assert discovery._dispatch_parallel(archive, Proposals(), tmp_path, settings, Simulations(), 2, 2) == 0
            assert archive.counts() == {"complete": 2}
        assert waits and 1 in waits
        workers = archive.db.execute("SELECT birth,status FROM screening_workers ORDER BY birth").fetchall()
        assert workers and all(row["status"] == "complete" for row in workers)
        if limit == "reconstruction":
            assert archive.db.execute(
                "SELECT sum(allowance) FROM native_checks WHERE after_trial=0").fetchone()[0] == 8
            assert len(workers) == (1 if rejected else 2)
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 2
    finally:
        release.set()
        archive.db.close()


@pytest.mark.parametrize("limited", [False, True])
def test_serial_dispatch_waits_for_simulations_only_for_expected_discovery_limits(monkeypatch, limited):
    archive = SimpleNamespace(counts=lambda: {})
    steps = []

    class Simulations:
        jobs = [object()]
        full = False
        exhausted = False
        generation_step = 0

        def reap(self, block=False):
            if block:
                self.jobs = []
                self.generation_step += 1

    def propose(archive, generator, step, founder):
        steps.append(step)
        if limited and step == 0:
            raise discovery.DiscoveryLimit("Fixture reconstruction limit.")
        raise AdapterError("Fixture generator failure.")

    monkeypatch.setattr(discovery, "_neighborhood", lambda *args, **kwargs: 1)
    monkeypatch.setattr(discovery, "_pending", lambda *args: None)
    monkeypatch.setattr(discovery, "_propose", propose)
    pool = Simulations()
    with pytest.raises(AdapterError, match="generator failure"):
        discovery._dispatch(archive, None, None, options(), pool, 1)
    assert steps == ([0, 1] if limited else [0])
    assert bool(pool.jobs) is not limited


@pytest.mark.parametrize("rejection", [None, "mass-layout-rejected", "evaluation-envelope-rejected"])
def test_candidate_worker_selects_one_realization_and_propagates_preflight(candidates, tmp_path, monkeypatch, capsys, rejection):
    archive, pending = candidates
    selected = pending[0]
    checked, layout = SimpleNamespace(report={}), {"gate": {"outcome": "pass"}}
    stages, calls = [], []
    settings = options()
    manifest = archive.directory / "designs" / selected["design"] / "candidate/manifest.json"

    def screen(batch, output, library, **kwargs):
        assert kwargs["candidate_ids"] == {selected["design"]} and kwargs["max_checks"] == 1
        calls.append("screen")
        return {"counts": {"reconstructionAttempts": 1}, "results": [{"status": "ready-for-dynamics"}]}

    def preflight(path, library, selected_options):
        assert path == manifest and selected_options is settings
        calls.append("preflight")
        if rejection:
            raise native_episode.PreflightRejection(rejection, "Fixture rejection.", layout)
        return checked, layout

    monkeypatch.setattr(screening, "screen_batch", screen)
    monkeypatch.setattr(native_episode, "prepare_native_candidate", preflight)
    outcome = prescreen.screen_candidate(archive.directory / selected["batch"], tmp_path / "isolated",
                                        selected["design"], manifest, tmp_path, settings, stages.append)
    assert calls == ["screen", "preflight"] and stages == ["preflight"]
    log = capsys.readouterr().out
    assert "Load-only screening started" in log and "Load-only screening finished" in log
    assert "ready-for-dynamics; reconstruction checks: 1." in log
    assert "Final preflight started" in log
    assert outcome["status"] == (rejection or "ready") and outcome["checks"] == 1
    assert outcome["layout"] == layout
    if rejection:
        assert outcome["error"] == "Fixture rejection."
    else:
        assert outcome["checked"] is checked
    assert archive.counts() == {}


@pytest.mark.parametrize("budget,expected", [(3, 1), (4, 2), (8, 4)])
def test_ready_buffer_is_bounded_by_four_and_remaining_authorization(tmp_path, monkeypatch, budget, expected):
    archive, pending = make_candidates(tmp_path, count=6, budget=budget)
    archive.reserve(pending[0]["birth"])
    archive.reserve(pending[1]["birth"])
    submitted = []

    class Screens:
        def __init__(self, *args):
            self.jobs = []
            self.constructed = []
            self.admitting = False
            self.births = set()
            self.failures = 0
            self.full = False

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def submit(self, candidate, used, maximum):
            submitted.append(candidate["birth"])
            self.jobs.append((candidate["birth"], SimpleNamespace(report={})))
            self.births.add(candidate["birth"])

        def reap(self):
            ready, self.jobs = self.jobs, []
            self.births.clear()
            return ready

        def wait(self, simulations):
            assert len(submitted) == expected
            assert len(set(submitted)) == expected
            raise AdapterError("Reached bounded ready queue.")

    monkeypatch.setattr(prescreen, "ScreeningPool", Screens)
    simulations = SimpleNamespace(full=True, jobs=[object()], reap=lambda: 0,
                                  exhausted=False, pending_gates=0, generation_step=2)
    generator = Proposals()
    try:
        with pytest.raises(AdapterError, match="bounded ready queue"):
            discovery._dispatch_parallel(archive, generator, tmp_path, options(), simulations, budget, 2)
        assert archive.counts() == {"running": 2} and generator.calls == []
    finally:
        archive.db.close()


@pytest.mark.parametrize("mode,diagnostic", [
    ("error", "Intentional screening failure"),
    ("crash", "without a result"),
    ("invalid-status", "invalid outcome"),
    ("invalid-checks", "single reserved reconstruction"),
])
def test_failed_screening_is_visible_without_spending_physics(candidates, tmp_path, mode, diagnostic):
    archive, pending = candidates
    with prescreen.ScreeningPool(archive, 2, tmp_path, options(mode=mode), screened_fixture) as pool:
        pool.submit(pending[0], 0, 8)
        with pytest.raises(AdapterError, match=diagnostic):
            reap_until(pool)
    assert archive.counts() == {}
    assert archive.db.execute("SELECT status FROM screening_workers").fetchone()[0] == "failed"
    assert archive.db.execute("SELECT stage FROM pipeline_events ORDER BY id DESC LIMIT 1").fetchone()[0] == "paused"
    assert archive.db.execute("SELECT sum(allowance) FROM native_checks").fetchone()[0] == 1


def test_cancellation_stops_only_owned_children_without_spending_physics(candidates, tmp_path):
    archive, pending = candidates
    context = multiprocessing.get_context("spawn")
    with prescreen.ScreeningPool(archive, 2, tmp_path,
                                options(release=context.Event(), hold_births={1, 2}), screened_fixture) as pool:
        for row in pending:
            pool.submit(row, 0, 8)
        pids = [job.process.pid for job in pool.jobs]
    assert archive.counts() == {}
    assert [row[0] for row in archive.db.execute("SELECT status FROM screening_workers")] == ["interrupted"] * 2
    for pid in pids:
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)


def test_resume_refuses_live_orphan_screening_process(candidates, tmp_path):
    archive, pending = candidates
    with archive.db:
        archive.db.execute("INSERT INTO screening_workers VALUES (?,?,?,'running')",
                           (pending[0]["birth"], os.getpid(), "orphan.log"))
    with pytest.raises(AdapterError, match="still running"):
        prescreen.ScreeningPool(archive, 2, tmp_path, options(), screened_fixture)


@pytest.mark.parametrize("value", [0, -1, 1.5, True, "2"])
def test_screening_capacity_requires_positive_integer(value):
    with pytest.raises(AdapterError, match="positive integer"):
        prescreen.validate_workers(value)
