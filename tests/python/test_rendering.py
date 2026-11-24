import contextlib
import os
import signal
import sqlite3
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from legowalkers_sim import rendering
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.evolution import Archive
from test_evolution import genome, protocol, result


@pytest.fixture
def campaign(tmp_path):
    archive = Archive(tmp_path)
    archive.initialize(protocol(), 4, 42, False)
    archive.db.close()
    return rendering.RenderController(tmp_path, tmp_path / "library", Path(sys.executable))


def add_trial(directory, *, seed=42, poses=True):
    archive = Archive(directory)
    try:
        birth, design = archive.record_birth({**genome(), "seed": seed, "phaseQuarterTurns": seed},
                                            None, "test", [], "generated")
        trial = archive.reserve(birth)
        archive.finish(trial, result())
        candidate = directory / "designs" / design / "candidate"
        candidate.mkdir(parents=True, exist_ok=True)
        evaluation = directory / "evaluations" / f"{trial:04d}"
        evaluation.mkdir(parents=True, exist_ok=True)
        if poses:
            (evaluation / "poses.json").write_text("{}")
        return trial, design
    finally:
        archive.db.close()


def snapshot(directory):
    with contextlib.closing(sqlite3.connect(directory / "archive.sqlite3")) as db:
        return (db.execute("SELECT * FROM run").fetchall(),
                db.execute("SELECT * FROM evaluations").fetchall())


def test_rendering_is_explicit_independent_and_never_adds_authorization(campaign, monkeypatch):
    add_trial(campaign.directory)
    before = snapshot(campaign.directory)
    launched = []
    process = SimpleNamespace(pid=123456, returncode=None, poll=lambda: None)
    monkeypatch.setattr(rendering.subprocess, "Popen",
                        lambda command, **options: launched.append((command, options)) or process)
    assert campaign.status()["queued"] == 1 and not campaign.status()["running"]
    assert launched == []
    assert campaign.start() == {"pid": 123456}
    command, options = launched[0]
    assert "legowalkers_sim.rendering" in command and "--budget" not in command
    assert options["start_new_session"] and options["close_fds"]
    assert campaign.status()["running"] and snapshot(campaign.directory) == before
    with pytest.raises(AdapterError, match="already running"):
        campaign.start()
    signaled = []
    monkeypatch.setattr(rendering.os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(rendering.os, "killpg", lambda pid, sig: signaled.append((pid, sig)))
    assert campaign.stop()["status"] == "stopping"
    assert signaled == [(123456, signal.SIGTERM)]
    process.returncode = -signal.SIGTERM
    process.poll = lambda: process.returncode
    assert campaign.status()["status"] == "stopped"
    assert campaign.status()["queued"] == 1 and snapshot(campaign.directory) == before


def test_busy_means_unfinished_jobs_not_a_live_idle_watcher(campaign, monkeypatch):
    with contextlib.closing(rendering.connect(campaign.directory)) as db, db:
        db.execute("UPDATE render_worker SET pid=123456,status='running'")
    monkeypatch.setattr(rendering.os, "kill", lambda *args: None)
    state = campaign.status()
    assert state["running"] and not state["busy"]
    trial, _ = add_trial(campaign.directory)
    assert campaign.status()["busy"]
    with contextlib.closing(rendering.connect(campaign.directory)) as db, db:
        db.execute("UPDATE render_jobs SET status='rendering' WHERE evaluation=?", (trial,))
    assert campaign.status()["busy"]
    with contextlib.closing(rendering.connect(campaign.directory)) as db, db:
        db.execute("UPDATE render_jobs SET status='complete' WHERE evaluation=?", (trial,))
    assert not campaign.status()["busy"]


def test_idle_watcher_can_stop_for_update_without_stopping_active_rendering(campaign, monkeypatch):
    stopped = []
    monkeypatch.setattr(campaign, "stop", lambda: stopped.append(True))
    monkeypatch.setattr(campaign, "status", lambda: {"running": True, "busy": True})
    assert not campaign.stop_if_idle() and stopped == []
    states = iter([{"running": True, "busy": False}, {"running": True, "busy": False},
                   {"running": False, "busy": False}])
    monkeypatch.setattr(campaign, "status", lambda: next(states))
    assert campaign.stop_if_idle() and stopped == [True]
    monkeypatch.setattr(campaign, "status", lambda: {"running": True, "busy": False})
    assert not campaign.stop_if_idle(timeout=0)


def test_missing_blender_and_failed_launch_are_visible(campaign, monkeypatch):
    missing = rendering.RenderController(campaign.directory, campaign.library, None)
    assert not missing.status()["available"]
    with pytest.raises(AdapterError, match="Blender is unavailable"):
        missing.start()
    def fail(*args, **kwargs):
        raise OSError("launch denied")
    monkeypatch.setattr(rendering.subprocess, "Popen", fail)
    with pytest.raises(AdapterError, match="launch denied"):
        campaign.start()
    state = campaign.status()
    assert state["status"] == "failed" and "launch denied" in state["error"]


def test_only_completed_local_evaluations_are_queued(campaign):
    trial, _ = add_trial(campaign.directory)
    archive = Archive(campaign.directory)
    try:
        birth, _ = archive.record_birth({**genome(), "phaseQuarterTurns": 99}, None, "pending", [], "generated")
        archive.reserve(birth)
    finally:
        archive.db.close()
    state = campaign.status()
    assert state["queued"] == 1
    with contextlib.closing(rendering.connect(campaign.directory)) as db:
        assert [row[0] for row in db.execute("SELECT evaluation FROM render_jobs")] == [trial]


def test_restart_observes_existing_worker_and_recovers_interrupted_jobs(campaign, monkeypatch):
    trial, design = add_trial(campaign.directory)
    campaign.status()
    with contextlib.closing(rendering.connect(campaign.directory)) as db, db:
        db.execute("UPDATE render_worker SET pid=123456,status='running'")
        db.execute("UPDATE render_jobs SET status='rendering'")
    restarted = rendering.RenderController(campaign.directory, campaign.library, campaign.blender)
    monkeypatch.setattr(rendering.os, "kill", lambda pid, sig: None)
    assert restarted.status()["running"]
    with pytest.raises(AdapterError, match="already running"):
        restarted.start()
    def gone(*args):
        raise ProcessLookupError()
    monkeypatch.setattr(rendering.os, "kill", gone)
    state = restarted.status()
    assert state["status"] == "failed" and "unexpectedly" in state["error"]
    assert state["queued"] == 1


def test_stop_refuses_to_signal_an_unrelated_process_group(campaign, monkeypatch):
    with contextlib.closing(rendering.connect(campaign.directory)) as db, db:
        db.execute("UPDATE render_worker SET pid=123456,status='running'")
    monkeypatch.setattr(rendering.os, "kill", lambda *args: None)
    monkeypatch.setattr(rendering.os, "getpgid", lambda pid: pid + 1)
    monkeypatch.setattr(rendering.os, "killpg", lambda *args: pytest.fail("Must not signal an unrelated group"))
    with pytest.raises(AdapterError, match="refusing to signal"):
        campaign.stop()


def test_trial_without_saved_poses_never_calls_renderer_or_physics(campaign, monkeypatch):
    trial, _ = add_trial(campaign.directory, poses=False)
    monkeypatch.setattr(rendering, "render_representative",
                        lambda *args: pytest.fail("A missing trace must not fall back to physics"))
    before = snapshot(campaign.directory)
    with pytest.raises(AdapterError, match="will not replay physics"):
        rendering.render_trial(campaign.directory, campaign.library, campaign.blender, trial)
    assert snapshot(campaign.directory) == before


def test_trial_requires_real_outputs_and_passes_the_saved_evaluation(campaign, monkeypatch):
    trial, design = add_trial(campaign.directory)
    seen = []
    def render(archive, row, blender):
        seen.append((row["id"], row["design"], blender))
        target = archive.directory / "designs" / design / "render"
        target.mkdir()
        (target / "walker-v2.png").write_bytes(b"image")
        (target / "walker-v2-anchored.mp4").write_bytes(b"video")
    monkeypatch.setattr(rendering, "render_representative", render)
    rendering.render_trial(campaign.directory, campaign.library, campaign.blender, trial)
    assert seen == [(trial, design, campaign.blender)]
    (campaign.directory / "designs" / design / "render/walker-v2-anchored.mp4").unlink()
    monkeypatch.setattr(rendering, "render_representative", lambda *args: None)
    with pytest.raises(AdapterError, match="no complete animation"):
        rendering.render_trial(campaign.directory, campaign.library, campaign.blender, trial)


def test_worker_watches_for_new_trials_publishes_and_keeps_budget_unchanged(campaign, monkeypatch):
    first, _ = add_trial(campaign.directory)
    stop = threading.Event()
    commands = []
    class Child:
        def __init__(self, command, **options):
            commands.append(command)
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def wait(self, **options):
            return 0
    monkeypatch.setattr(rendering.subprocess, "Popen", Child)
    published = []
    def publish(archive):
        published.append(archive.db.execute("SELECT count(*) FROM render_jobs WHERE status='complete'").fetchone()[0])
        if len(published) == 1:
            add_trial(campaign.directory, seed=43)
        else:
            stop.set()
    monkeypatch.setattr(rendering, "publish", publish)
    rendering.run_worker(campaign.directory, campaign.library, campaign.blender, stop=stop, poll_interval=.01)
    assert published == [1, 2] and len(commands) == 2
    assert all("--trial" in command and "--budget" not in command for command in commands)
    state = campaign.status()
    assert state["complete"] == 2 and state["failed"] == state["queued"] == 0
    assert [item["trial"] for item in state["recent"]] == [2, 1]
    assert state["recent"][1]["history"] == "history/0001.html"
    assert state["recent"][1]["video"].endswith("/render/walker-v2-anchored.mp4")
    with contextlib.closing(rendering.connect(campaign.directory)) as db:
        assert db.execute("SELECT budget FROM run").fetchone()[0] == 4
        assert db.execute("SELECT count(*) FROM evaluations").fetchone()[0] == 2


def test_failed_render_is_not_retried_in_a_busy_loop(campaign, monkeypatch):
    add_trial(campaign.directory)
    stop = threading.Event()
    class Child:
        def __init__(self, *args, **kwargs):
            pass
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def wait(self, **kwargs):
            return 2
    monkeypatch.setattr(rendering.subprocess, "Popen", Child)
    monkeypatch.setattr(rendering, "publish", lambda archive: stop.set())
    rendering.run_worker(campaign.directory, campaign.library, campaign.blender, stop=stop)
    state = campaign.status()
    assert state["failed"] == 1 and state["queued"] == 0 and "code 2" in state["error"]


def test_cancellation_requeues_job_and_removes_only_its_temporary_assets(campaign, monkeypatch):
    trial, design = add_trial(campaign.directory)
    parent = campaign.directory / "designs" / design
    old = parent / "render-pending-existing"
    new = parent / "render-pending-new"
    old.mkdir()
    stop = threading.Event()
    class Child:
        def __init__(self, *args, **kwargs):
            new.mkdir()
        def __enter__(self):
            return self
        def __exit__(self, *args):
            pass
        def wait(self, **kwargs):
            stop.set()
            return -signal.SIGTERM
    monkeypatch.setattr(rendering.subprocess, "Popen", Child)
    before = snapshot(campaign.directory)
    rendering.run_worker(campaign.directory, campaign.library, campaign.blender, stop=stop)
    assert old.exists() and not new.exists()
    assert campaign.status()["queued"] == 1 and campaign.status()["failed"] == 0
    assert snapshot(campaign.directory) == before


def test_real_idle_worker_can_be_stopped_and_restarted_without_trials(campaign):
    before = snapshot(campaign.directory)
    for _ in range(2):
        started = campaign.start()
        child = campaign.process
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                text = (campaign.directory / "rendering.log").read_text()
                if f"Rendering worker {started['pid']} started" in text:
                    break
                assert child.poll() is None
                time.sleep(.05)
            else:
                pytest.fail("Rendering worker did not become ready")
            assert os.getpgid(child.pid) == child.pid and campaign.status()["running"]
            campaign.stop()
            assert child.wait(timeout=10) == 0
            assert campaign.status()["status"] == "stopped"
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=10)
    assert snapshot(campaign.directory) == before
