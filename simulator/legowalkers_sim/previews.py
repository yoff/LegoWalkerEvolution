"""Automatic, single-worker assembly stills; never loads or advances physics."""

import argparse
import contextlib
import fcntl
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time

from .bridge import AdapterError
from .evolution import ROOT


def connect(directory):
    db = sqlite3.connect(directory / "archive.sqlite3", timeout=30)
    db.row_factory = sqlite3.Row
    return db


def ensure_schema(db):
    db.executescript("""
        CREATE TABLE IF NOT EXISTS assembly_previews(
            design TEXT PRIMARY KEY REFERENCES designs(id),status TEXT NOT NULL,
            error TEXT,elapsed REAL);
        CREATE TABLE IF NOT EXISTS assembly_preview_worker(
            id INTEGER PRIMARY KEY CHECK(id=1),pid INTEGER,status TEXT NOT NULL,error TEXT);
        INSERT OR IGNORE INTO assembly_preview_worker VALUES(1,NULL,'idle',NULL);
    """)


def sync_jobs(db, directory):
    from .gallery import media_directory

    for row in db.execute("SELECT DISTINCT b.design FROM births b WHERE b.status!='duplicate'").fetchall():
        design = directory / "designs" / row["design"]
        if not (design / "candidate/manifest.json").is_file():
            continue
        complete = (media_directory(design) / "walker-v2.png").is_file()
        with db:
            db.execute("INSERT OR IGNORE INTO assembly_previews(design,status) VALUES (?,?)",
                       (row["design"], "complete" if complete else "queued"))
            if complete:
                db.execute("UPDATE assembly_previews SET status='complete',error=NULL WHERE design=?",
                           (row["design"],))


class PreviewController:
    def __init__(self, directory, library, blender):
        self.directory, self.library = Path(directory), Path(library)
        self.blender = Path(blender) if blender else None
        self.process = None
        self.mutex = threading.Lock()
        with contextlib.closing(connect(self.directory)) as db:
            ensure_schema(db)

    def status(self, *, watch=False, automatic=True):
        with self.mutex, contextlib.closing(connect(self.directory)) as db:
            row = db.execute("SELECT * FROM assembly_preview_worker WHERE id=1").fetchone()
            alive = False
            if row["status"] == "running" and row["pid"] is not None:
                if self.process is not None:
                    alive = self.process.poll() is None
                else:
                    try:
                        os.kill(row["pid"], 0)
                        alive = True
                    except ProcessLookupError:
                        pass
                if not alive:
                    with db:
                        db.execute("""UPDATE assembly_previews SET status='failed',
                            error='Preview worker interrupted; see assembly-previews.log.'
                            WHERE status='rendering'""")
                        db.execute("UPDATE assembly_preview_worker SET status='idle',pid=NULL WHERE id=1")
            sync_jobs(db, self.directory)
            available = self.blender is not None and self.blender.is_file() and os.access(self.blender, os.X_OK)
            error = row["error"]
            queued = db.execute("SELECT 1 FROM assembly_previews WHERE status='queued' LIMIT 1").fetchone()
            if automatic and not alive and available and row["status"] != "failed" and (watch or queued):
                command = [sys.executable, "-m", "legowalkers_sim.previews", "--output", str(self.directory),
                           "--library", str(self.library), "--blender", str(self.blender)]
                try:
                    with (self.directory / "assembly-previews.log").open("ab") as log:
                        self.process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                                        stdout=log, stderr=subprocess.STDOUT,
                                                        start_new_session=True, close_fds=True)
                    with db:
                        db.execute("UPDATE assembly_preview_worker SET status='running',pid=?,error=NULL WHERE id=1",
                                   (self.process.pid,))
                    error = None
                    alive = True
                except OSError as failure:
                    error = str(failure)
                    with db:
                        db.execute("UPDATE assembly_preview_worker SET status='failed',error=? WHERE id=1", (error,))
            counts = dict(db.execute("SELECT status,count(*) FROM assembly_previews GROUP BY status"))
            return {"running": alive, "available": available, "counts": counts,
                    "error": error,
                    "log": "assembly-previews.log" if (self.directory / "assembly-previews.log").exists() else None}

    def retry(self):
        with self.mutex, contextlib.closing(connect(self.directory)) as db, db:
            db.execute("UPDATE assembly_previews SET status='queued',error=NULL WHERE status='failed'")
            db.execute("UPDATE assembly_preview_worker SET status='idle',error=NULL WHERE status='failed'")
        return self.status()


def render(directory, design, library, blender):
    root = directory / "designs" / design
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="assembly-preview-", dir=root) as temporary:
        assets = Path(temporary) / "assets"
        commands = [
            [sys.executable, str(ROOT / "scripts/preview.py"), "prepare-still", "--assets", str(assets),
             "--manifest", str(root / "candidate/manifest.json"), "--library", str(library), "--preset", "thumbnail"],
            [str(blender), "--background", "--factory-startup", "--threads", "2", "--python-exit-code", "1",
             "--python", str(ROOT / "scripts/render_preview.py"), "--", "--assets", str(assets),
             "--mode", "still", "--preset", "thumbnail"],
        ]
        with (root / "assembly-preview.log").open("w") as log:
            for command in commands:
                subprocess.run(command, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                               stderr=subprocess.STDOUT, check=True, timeout=180)
        image = assets / "walker-v2.png"
        if not image.is_file():
            raise AdapterError(f"No assembly image produced; see designs/{design}/assembly-preview.log.")
        target = root / "assembly-render"
        target.mkdir(exist_ok=True)
        (assets / "metadata.json").replace(target / "metadata.json")
        image.replace(target / "walker-v2.png")
        elapsed = time.monotonic() - started
        (target / "preview-info.json").write_text(json.dumps({
            "preset": "thumbnail", "width": 480, "height": 300,
            "elapsedSeconds": elapsed, "simulationSteps": 0,
        }) + "\n")
        return elapsed


def run_worker(directory, library, blender):
    from .campaign import require_active
    from .evolution import Archive
    from .gallery import publish

    with contextlib.closing(connect(directory)) as db:
        ensure_schema(db)
        # Serialize preview rendering across experiment upgrades, not just within one archive.
        lock_path = ROOT / "output/.assembly-preview.lock"
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        with lock_path.open("a") as lock:
            with db:
                db.execute("""UPDATE assembly_previews SET status='failed',
                    error='Previous preview worker was interrupted; see assembly-previews.log.'
                    WHERE status='rendering'""")
            while True:
                require_active(directory)
                sync_jobs(db, directory)
                job = db.execute("""SELECT j.design FROM assembly_previews j WHERE j.status='queued'
                    ORDER BY
                    (SELECT max(b.id) FROM births b WHERE b.design=j.design) DESC LIMIT 1""").fetchone()
                if job is None:
                    run = db.execute("SELECT status FROM run").fetchone()
                    waiting = run["status"] == "running"
                    if not waiting and run["status"] == "ready" and db.execute(
                            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='episodes'").fetchone():
                        episode = db.execute("SELECT status,pid FROM episodes ORDER BY id DESC LIMIT 1").fetchone()
                        if episode and episode["status"] == "running" and episode["pid"] is not None:
                            try:
                                os.kill(episode["pid"], 0)
                                waiting = True
                            except ProcessLookupError:
                                pass
                    if not waiting:
                        break
                    time.sleep(.5)
                    continue
                fcntl.flock(lock, fcntl.LOCK_EX)
                try:
                    require_active(directory)
                    with db:
                        db.execute("UPDATE assembly_previews SET status='rendering',error=NULL WHERE design=?", (job["design"],))
                    try:
                        elapsed = render(directory, job["design"], library, blender)
                        status, error = "complete", None
                    except (AdapterError, OSError, subprocess.SubprocessError) as failure:
                        elapsed, status = None, "failed"
                        error = f"{failure}; see designs/{job['design']}/assembly-preview.log"
                        print(f"ERROR: {error}", file=sys.stderr, flush=True)
                finally:
                    fcntl.flock(lock, fcntl.LOCK_UN)
                with db:
                    db.execute("UPDATE assembly_previews SET status=?,error=?,elapsed=? WHERE design=?",
                               (status, error, elapsed, job["design"]))
                if status == "complete":
                    require_active(directory)
                    archive = Archive(directory)
                    try:
                        publish(archive)
                    finally:
                        archive.db.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--blender", type=Path, required=True)
    args = parser.parse_args(argv)
    directory = args.output.resolve()
    status, error = "idle", None
    try:
        run_worker(directory, args.library.resolve(), args.blender.resolve())
        return 0
    except (AdapterError, OSError, sqlite3.Error) as failure:
        status, error = "failed", str(failure)
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 2
    finally:
        with contextlib.closing(connect(directory)) as db, db:
            db.execute("UPDATE assembly_preview_worker SET status=?,error=? WHERE id=1 AND pid=?",
                       (status, error, os.getpid()))


if __name__ == "__main__":
    raise SystemExit(main())
