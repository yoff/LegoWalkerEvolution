import json
import multiprocessing
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from legowalkers_sim.bridge import AdapterError, load_manifest, write_json
from legowalkers_sim.evolution import Archive
from legowalkers_sim.parallel import TrialPool, worker_count
from test_evolution import genome, protocol, result


def simulated_fixture(checked, output, options):
    if getattr(checked, "barrier", None) is not None:
        checked.barrier.wait(timeout=30)
    if getattr(checked, "release", None) is not None and not checked.release.wait(timeout=30):
        raise AdapterError("Test simulation release timed out.")
    if getattr(checked, "mode", "") == "error":
        raise AdapterError("Intentional simulated failure.")
    if getattr(checked, "mode", "") == "crash":
        os._exit(7)
    value = result(qualified=False)
    value["testPid"] = os.getpid()
    output.mkdir(parents=True)
    write_json(output / "result.json", value)
    return value


@pytest.fixture
def archive(tmp_path):
    value = Archive(tmp_path / "archive")
    value.initialize(protocol(), 3, 42, False)
    yield value
    value.db.close()


def birth(archive):
    return archive.record_birth(genome(), None, "founder", [], "generated")[0]


def test_simulations_overlap_and_fast_result_is_recorded_without_waiting_for_slow_one(archive):
    context = multiprocessing.get_context("spawn")
    barrier, release = context.Barrier(2), context.Event()
    slow, fast = birth(archive), birth(archive)
    with TrialPool(archive, 2, simulated_fixture) as pool:
        try:
            first = pool.submit(slow, SimpleNamespace(barrier=barrier, release=release), None)
            second = pool.submit(fast, SimpleNamespace(barrier=barrier), None)
            assert pool.full and archive.counts() == {"running": 2}
            assert pool.reap(block=True) == 1
            statuses = dict(archive.db.execute("SELECT id,status FROM evaluations"))
            assert statuses == {first: "running", second: "complete"}
            assert len(pool.jobs) == 1
            assert not pool.full
            with pytest.raises(AdapterError, match="still running"):
                archive.recover()
        finally:
            release.set()
    assert archive.counts() == {"complete": 2}
    pids = [json.loads(row[0])["testPid"] for row in archive.db.execute("SELECT result FROM evaluations")]
    assert len(set(pids)) == 2 and os.getpid() not in pids
    assert archive.db.execute("SELECT count(*) FROM trial_workers").fetchone()[0] == 2


def test_four_simulations_overlap_within_a_four_trial_allowance(tmp_path):
    archive = Archive(tmp_path)
    archive.initialize(protocol(), 4, 42, False)
    try:
        barrier = multiprocessing.get_context("spawn").Barrier(4)
        with TrialPool(archive, 4, simulated_fixture) as pool:
            for _ in range(4):
                pool.submit(birth(archive), SimpleNamespace(barrier=barrier), None)
            assert pool.full and archive.counts() == {"running": 4}
            pool.drain()
        assert archive.counts() == {"complete": 4}
        pids = [json.loads(row[0])["testPid"] for row in archive.db.execute("SELECT result FROM evaluations")]
        assert len(set(pids)) == 4 and os.getpid() not in pids
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 4
    finally:
        archive.db.close()


def test_pool_and_hard_budget_limits_cannot_be_exceeded(archive):
    context = multiprocessing.get_context("spawn")
    release = context.Event()
    births = [birth(archive) for _ in range(5)]
    with TrialPool(archive, 2, simulated_fixture) as pool:
        try:
            pool.submit(births[0], SimpleNamespace(release=release), None)
            pool.submit(births[1], SimpleNamespace(release=release), None)
            with pytest.raises(AdapterError, match="pool is full"):
                pool.submit(births[2], SimpleNamespace(), None)
        finally:
            release.set()
        pool.reap(block=True)
        pool.submit(births[2], SimpleNamespace(), None)
        pool.drain()
        with pytest.raises(AdapterError, match="budget exhausted"):
            pool.submit(births[3], SimpleNamespace(), None)
    assert archive.counts() == {"complete": 3}
    assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 3


@pytest.mark.parametrize("mode", ["error", "crash"])
def test_child_failure_is_explicit_spent_and_does_not_discard_other_results(archive, mode):
    failed, healthy = birth(archive), birth(archive)
    with TrialPool(archive, 2, simulated_fixture) as pool:
        pool.submit(failed, SimpleNamespace(mode=mode), None)
        pool.submit(healthy, SimpleNamespace(), None)
    assert pool.failures == 1 and archive.counts() == {"complete": 1, "failed": 1}
    error = archive.db.execute("SELECT error FROM evaluations WHERE birth=?", (failed,)).fetchone()[0]
    assert ("Intentional simulated failure" if mode == "error" else "code 7") in error
    assert (archive.directory / "evaluations/0001/error.json").is_file()
    assert (archive.directory / "evaluations/0001.log").is_file()


def test_dispatcher_interruption_stops_children_and_keeps_reserved_slots_spent(archive):
    release = multiprocessing.get_context("spawn").Event()
    children = []
    with pytest.raises(KeyboardInterrupt):
        with TrialPool(archive, 2, simulated_fixture) as pool:
            pool.submit(birth(archive), SimpleNamespace(release=release), None)
            pool.submit(birth(archive), SimpleNamespace(release=release), None)
            children = [job.process.pid for job in pool.jobs]
            raise KeyboardInterrupt
    assert archive.counts() == {"interrupted": 2}
    for pid in children:
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
    archive.recover()
    assert sum(archive.counts().values()) == 2
    assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 3


def test_native_discovery_fills_two_slots_with_sibling_realizations(tmp_path, monkeypatch):
    from legowalkers_sim import discovery, native_episode
    from test_native_discovery import Proposals, setup_discovery

    source, data, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=3)
    barrier = multiprocessing.get_context("spawn").Barrier(2)

    def checked(path, library):
        manifest = load_manifest(path)
        return SimpleNamespace(manifest=manifest, report={},
                               barrier=barrier if manifest["motion"]["genotypeId"] != data["motion"]["genotypeId"] else None)

    monkeypatch.setattr(native_episode, "check", checked)
    monkeypatch.setattr(native_episode, "evaluate", simulated_fixture)
    generator = Proposals(siblings=2)
    try:
        measured = discovery.search_discovery(archive, generator, source, tmp_path, SimpleNamespace(payload_profile=None, cycle_aware=False),
                                              workers=2)
        assert measured["evaluations"] == {"complete": 3}
        assert measured["failures"] == 0
        assert len(generator.calls) == 1
        assert archive.db.execute("SELECT count(*) FROM trial_workers").fetchone()[0] == 3
    finally:
        archive.db.close()


def test_dispatch_bound_pauses_new_work_but_commits_already_running_results(tmp_path, monkeypatch):
    from legowalkers_sim import discovery, native_episode
    from test_native_discovery import Proposals, setup_discovery

    source, data, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=4)
    context = multiprocessing.get_context("spawn")
    barrier, release = context.Barrier(2), context.Event()
    original = discovery._propose
    proposals = []

    def checked(path, library):
        manifest = load_manifest(path)
        is_child = manifest["motion"]["genotypeId"] != data["motion"]["genotypeId"]
        slow = manifest["genome"]["speed"] == data["genome"]["speed"] + 2
        return SimpleNamespace(manifest=manifest, report={}, barrier=barrier if is_child else None,
                               release=release if slow else None)

    def bounded(*args):
        if not proposals:
            proposals.append(True)
            return original(*args)
        assert archive.counts()["running"] == 1
        release.set()
        raise AdapterError("Intentional dispatcher proposal bound.")

    monkeypatch.setattr(native_episode, "check", checked)
    monkeypatch.setattr(native_episode, "evaluate", simulated_fixture)
    monkeypatch.setattr(discovery, "_propose", bounded)
    try:
        with pytest.raises(AdapterError, match="dispatcher proposal bound"):
            discovery.search_discovery(archive, Proposals(siblings=2), source, tmp_path,
                                       SimpleNamespace(payload_profile=None, cycle_aware=False), workers=2)
        assert archive.counts() == {"complete": 3}
        assert archive.db.execute("SELECT status,budget FROM run").fetchone()[:] == ("paused", 4)
    finally:
        release.set()
        archive.db.close()


def test_finite_native_queue_runs_independent_candidates_concurrently(tmp_path, monkeypatch):
    from legowalkers_sim import native_episode
    from test_native_discovery import Proposals
    from test_native_episode import mock_native, native_fixture

    source, data = native_fixture(tmp_path)
    batch = tmp_path / "finite"
    Proposals(siblings=2).propose_native(source / "candidate-0", batch, 42, 0, 2)
    archive = Archive(tmp_path / "finite-archive")
    archive.initialize(native_episode.native_protocol(protocol(), batch, simulation_workers=2), 2, 42, False)
    mock_native(monkeypatch, archive, data)
    monkeypatch.setattr(native_episode, "screen_batch", lambda source, *args, **kwargs: {
        "results": [{"candidate": str(source / item["directory"]), "status": "ready-for-dynamics"}
                    for item in json.loads((source / "candidates.json").read_text())["generated"]]
    })
    barrier = multiprocessing.get_context("spawn").Barrier(2)
    monkeypatch.setattr(native_episode, "check", lambda path, library: SimpleNamespace(
        manifest=load_manifest(path), report={}, barrier=barrier))
    monkeypatch.setattr(native_episode, "evaluate", simulated_fixture)
    try:
        measured = native_episode.search_native(
            archive, batch, tmp_path, SimpleNamespace(payload_profile=None, cycle_aware=False), workers=2)
        assert measured["evaluations"] == {"complete": 2} and measured["failures"] == 0
        assert archive.db.execute("SELECT count(DISTINCT pid) FROM trial_workers").fetchone()[0] == 2
    finally:
        archive.db.close()


@pytest.mark.parametrize("value", [0, -1, True, 1.5, "2"])
def test_worker_count_rejects_invalid_settings(value):
    with pytest.raises(AdapterError, match="positive integer"):
        worker_count(value)
