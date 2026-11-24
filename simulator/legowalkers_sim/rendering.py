"""Independent, explicitly started animation queue. Never reserves or replays trials."""

import argparse
import contextlib
import fcntl
import os
import signal
import shutil
import sqlite3
import subprocess
import sys
import threading
import time
from pathlib import Path

from .bridge import AdapterError
from .evolution import Archive, ROOT, render_representative
from .gallery import publish


def connect(directory):
    db = sqlite3.connect(directory / "archive.sqlite3")
    db.row_factory = sqlite3.Row
    return db


def sync_jobs(db):
    with db:
        db.execute("""INSERT OR IGNORE INTO render_jobs(evaluation,design,status)
            SELECT e.id,b.design,'queued' FROM evaluations e JOIN births b ON b.id=e.birth
            WHERE e.status='complete' AND e.result IS NOT NULL""")


class RenderController:
    def __init__(self, directory, library, blender):
        self.directory = Path(directory).resolve()
        self.library = Path(library).resolve()
        self.blender = Path(blender).resolve() if blender else None
        self.process = None
        self.mutex = threading.Lock()
        with contextlib.closing(connect(self.directory)) as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS render_worker(
                    id INTEGER PRIMARY KEY CHECK(id=1),pid INTEGER,status TEXT NOT NULL,error TEXT);
                INSERT OR IGNORE INTO render_worker VALUES(1,NULL,'stopped',NULL);
                CREATE TABLE IF NOT EXISTS render_jobs(
                    evaluation INTEGER PRIMARY KEY,design TEXT NOT NULL,status TEXT NOT NULL,error TEXT);
            """)

    def observe(self, db):
        if self.process is not None and self.process.poll() is not None:
            code = self.process.returncode
            self.process = None
        else:
            code = None
        row = db.execute("SELECT * FROM render_worker WHERE id=1").fetchone()
        if row["status"] not in ("running", "stopping"):
            return False
        if self.process is not None:
            return True
        if code is None:
            if row["pid"] is not None:
                try:
                    os.kill(row["pid"], 0)
                except ProcessLookupError:
                    pass
                else:
                    return True
        stopped = row["status"] == "stopping"
        message = None if stopped else f"Rendering worker exited unexpectedly ({code}); see rendering.log. Start to retry unfinished jobs."
        with db:
            db.execute("UPDATE render_worker SET status=?,error=? WHERE id=1",
                       ("stopped" if stopped else "failed", message))
            db.execute("UPDATE render_jobs SET status='queued' WHERE status='rendering'")
        return False

    def status(self):
        with self.mutex, contextlib.closing(connect(self.directory)) as db:
            running = self.observe(db)
            sync_jobs(db)
            row = dict(db.execute("SELECT * FROM render_worker WHERE id=1").fetchone())
            counts = dict(db.execute("SELECT status,count(*) FROM render_jobs GROUP BY status"))
            current = db.execute("SELECT evaluation FROM render_jobs WHERE status='rendering'").fetchone()
            failures = db.execute("SELECT error FROM render_jobs WHERE status='failed' ORDER BY evaluation DESC LIMIT 1").fetchone()
            recent = [{"trial": job["evaluation"], "video": f"designs/{job['design']}/render/walker-v2-anchored.mp4",
                       "history": f"history/{job['birth']:04d}.html"} for job in db.execute("""
                SELECT j.evaluation,j.design,e.birth FROM render_jobs j
                JOIN evaluations e ON e.id=j.evaluation WHERE j.status='complete'
                ORDER BY j.evaluation DESC LIMIT 5""")]
            available = self.blender is not None and self.blender.is_file() and os.access(self.blender, os.X_OK)
            return {**row, "running": running, "busy": current is not None or counts.get("queued", 0) > 0,
                    "available": available,
                    "queued": counts.get("queued", 0), "complete": counts.get("complete", 0),
                    "failed": counts.get("failed", 0), "current": current[0] if current else None,
                    "error": row["error"] or (failures[0] if failures else None),
                    "recent": recent,
                    "log": "rendering.log" if (self.directory / "rendering.log").exists() else None}

    def stop_if_idle(self, *, timeout=5):
        state = self.status()
        if not state["running"]:
            return True
        if state["busy"]:
            return False
        self.stop()
        deadline = time.monotonic()+timeout
        while self.status()["running"]:
            if time.monotonic() >= deadline:
                return False
            time.sleep(.05)
        return True

    def start(self):
        with self.mutex, contextlib.closing(connect(self.directory)) as db:
            if self.observe(db):
                raise AdapterError("The rendering worker is already running.")
            if self.blender is None or not self.blender.is_file() or not os.access(self.blender, os.X_OK):
                raise AdapterError("Blender is unavailable. Restart the dashboard with --blender PATH.")
            sync_jobs(db)
            with db:
                db.execute("UPDATE render_jobs SET status='queued',error=NULL WHERE status IN ('failed','rendering')")
                db.execute("UPDATE render_worker SET pid=NULL,status='running',error=NULL WHERE id=1")
            command = [sys.executable, "-m", "legowalkers_sim.rendering", "--output", str(self.directory),
                       "--library", str(self.library), "--blender", str(self.blender)]
            try:
                with (self.directory / "rendering.log").open("ab") as log:
                    self.process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                                    stdout=log, stderr=subprocess.STDOUT,
                                                    start_new_session=True, close_fds=True)
            except OSError as error:
                with db:
                    db.execute("UPDATE render_worker SET status='failed',error=? WHERE id=1", (str(error),))
                raise AdapterError(f"Could not launch rendering worker: {error}") from error
            with db:
                db.execute("UPDATE render_worker SET pid=? WHERE id=1", (self.process.pid,))
            return {"pid": self.process.pid}

    def stop(self):
        with self.mutex, contextlib.closing(connect(self.directory)) as db:
            if not self.observe(db):
                raise AdapterError("The rendering worker is not running.")
            row = db.execute("SELECT pid FROM render_worker WHERE id=1").fetchone()
            try:
                if os.getpgid(row["pid"]) != row["pid"]:
                    raise AdapterError("Rendering PID is not its process-group leader; refusing to signal it.")
                os.killpg(row["pid"], signal.SIGTERM)
            except ProcessLookupError:
                self.observe(db)
                return {"status": "stopped"}
            with db:
                db.execute("UPDATE render_worker SET status='stopping' WHERE id=1")
            return {"status": "stopping"}


class RenderStopped(Exception):
    pass


def run_worker(directory, library, blender, *, stop=None, poll_interval=2):
    stop = stop or threading.Event()
    archive = Archive(directory)
    archive.library = library
    owns_lock = False

    def render(job):
        if stop.is_set():
            raise RenderStopped()
        design = directory / "designs" / job["design"]
        existing = set(design.glob("render-pending-*"))
        command = [sys.executable, "-m", "legowalkers_sim.rendering", "--output", str(directory),
                   "--library", str(library), "--blender", str(blender), "--trial", str(job["evaluation"])]
        try:
            with subprocess.Popen(command, stdin=subprocess.DEVNULL) as child:
                while True:
                    if stop.is_set():
                        child.terminate()
                        try:
                            child.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            child.kill()
                            child.wait()
                        raise RenderStopped()
                    try:
                        code = child.wait(timeout=.2)
                    except subprocess.TimeoutExpired:
                        continue
                    if stop.is_set():
                        raise RenderStopped()
                    if code:
                        raise AdapterError(f"Trial {job['evaluation']} render exited with code {code}; see rendering.log and designs/{job['design']}/render.log.")
                    return
        finally:
            for temporary in set(design.glob("render-pending-*")) - existing:
                if temporary.is_dir():
                    shutil.rmtree(temporary)

    try:
        with (directory / ".render-worker.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise AdapterError("Another animation worker owns this archive.") from error
            owns_lock = True
            print(f"Rendering worker {os.getpid()} started; saved poses only.", flush=True)
            while not stop.is_set():
                sync_jobs(archive.db)
                job = archive.db.execute("SELECT * FROM render_jobs WHERE status='queued' ORDER BY evaluation LIMIT 1").fetchone()
                if job is None:
                    stop.wait(poll_interval)
                    continue
                with archive.db:
                    archive.db.execute("UPDATE render_jobs SET status='rendering',error=NULL WHERE evaluation=?", (job["evaluation"],))
                print(f"Rendering trial {job['evaluation']}.", flush=True)
                try:
                    render(job)
                    success, error = True, None
                except AdapterError as failure:
                    success, error = False, str(failure)
                    print(f"ERROR: {error}", file=sys.stderr, flush=True)
                with archive.db:
                    archive.db.execute("UPDATE render_jobs SET status=?,error=? WHERE evaluation=?",
                                       ("complete" if success else "failed", error, job["evaluation"]))
                publish(archive)
    except RenderStopped:
        pass
    finally:
        if owns_lock:
            with archive.db:
                archive.db.execute("UPDATE render_jobs SET status='queued' WHERE status='rendering'")
        archive.db.close()


def render_trial(directory, library, blender, trial):
    archive = Archive(directory)
    archive.library = library
    try:
        row = next((row for row in archive.outcomes() if row["id"] == trial and not row.get("reused")), None)
        if row is None:
            raise AdapterError(f"Trial {trial} is not a completed local evaluation.")
        if not (directory / "evaluations" / f"{trial:04d}" / "poses.json").is_file():
            raise AdapterError(f"Trial {trial} has no recorded poses; rendering will not replay physics.")
        render_representative(archive, row, blender)
        target = directory / "designs" / row["design"] / "render"
        if not all((target / name).is_file() for name in ("walker-v2.png", "walker-v2-anchored.mp4")):
            raise AdapterError(f"Trial {trial} produced no complete animation; see {target.parent / 'render.log'}.")
    finally:
        archive.db.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--blender", type=Path, required=True)
    parser.add_argument("--trial", type=int, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    directory = args.output.resolve()
    if args.trial is not None:
        try:
            render_trial(directory, args.library.resolve(), args.blender.resolve(), args.trial)
            return 0
        except (AdapterError, OSError, sqlite3.Error, ValueError) as failure:
            print(f"ERROR: {failure}", file=sys.stderr, flush=True)
            return 2
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    status, error = "failed", "Rendering worker exited unexpectedly; see rendering.log."
    try:
        run_worker(directory, args.library.resolve(), args.blender.resolve(), stop=stop)
        status, error = "stopped", None
        return 0
    except (AdapterError, OSError, sqlite3.Error, ValueError) as failure:
        status, error = "failed", str(failure)
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 2
    finally:
        with contextlib.closing(connect(directory)) as db, db:
            db.execute("UPDATE render_worker SET status=?,error=? WHERE id=1 AND pid=?",
                       (status, error, os.getpid()))


if __name__ == "__main__":
    raise SystemExit(main())
