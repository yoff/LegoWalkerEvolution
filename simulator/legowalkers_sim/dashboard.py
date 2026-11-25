"""Loopback-only dashboard; explicit episodes authorize a configurable trial ceiling."""

import argparse
import contextlib
import json
import logging
import os
import re
import secrets
import shutil
import signal
import sqlite3
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

from .bridge import AdapterError
from .evaluation import EvaluationOptions
from .evolution import Archive, Generator, ROOT, evidence_invalidation, locked, now, protocol_definition
from .gallery import birth_previews, category_anchors, collapsible_genes, document, media_directory, preview_first, publish, relative
from .proposal_forest import campaign_snapshot, insert, snapshot
from .parallel import worker_count
from .payload import load_profile
from .rendering import RenderController
from .pinned import verify_snapshot
from .campaign import CampaignController, definition, require_active, version_html
from .pipeline import ensure_schema as ensure_pipeline, snapshot as pipeline_snapshot
from .screening_reports import birth_page
from .previews import PreviewController


class EpisodeController:
    def __init__(self, directory, protocol, worker_args, *, seed=42, episode_trials=12,
                 library=None, blender=None, initial_budget=0):
        if type(episode_trials) is not int or episode_trials <= 0:
            raise AdapterError("Episode trial allowance must be a positive integer.")
        self.episode_trials = episode_trials
        self.directory = Path(directory).resolve()
        self.worker_args = list(map(str, worker_args))
        self.seed = seed
        self.process = None
        self.pause_thread = None
        self.mutex = threading.Lock()
        self.token = secrets.token_urlsafe(32)
        self.native_candidates = None
        self.requested_protocol = protocol
        self.pinned = False
        self.resume_error = None
        existing = self.directory / "protocol.json"
        if existing.is_file() and (self.directory / "protocol-source").is_dir():
            stored = json.loads(existing.read_text())
            stored.pop("id", None)
            try:
                verify_snapshot(self.directory)
                self.pinned = True
            except (OSError, ValueError) as error:
                self.resume_error = str(error)
            protocol = stored
        if protocol.get("grammarVersion") == "technic-motion-v3" and not protocol.get("nativeDiscovery"):
            candidates = protocol.get("nativeCandidates")
            if not isinstance(candidates, list) or not candidates or len(set(candidates)) != len(candidates):
                raise AdapterError("A native dashboard needs a finite list of unique prepared candidates.")
            self.native_candidates = set(candidates)
        self.simulation_workers = worker_count(protocol.get("execution", {}).get("simulationWorkers", 1))
        self.screening_workers = protocol.get("execution", {}).get("screeningWorkers", 1)
        archive = Archive(self.directory)
        try:
            run = archive.db.execute("SELECT * FROM run").fetchone()
            archive.initialize(protocol, run["budget"] if run else initial_budget, seed, run is not None)
            ensure_pipeline(archive.db)
            with archive.db:
                archive.db.execute("""CREATE TABLE IF NOT EXISTS episodes(
                    id INTEGER PRIMARY KEY, budget INTEGER NOT NULL, status TEXT NOT NULL,
                    pid INTEGER, started TEXT NOT NULL, completed TEXT, error TEXT, log TEXT NOT NULL)""")
            publish(archive)
        finally:
            archive.db.close()
        self.renderer = RenderController(self.directory, library or ROOT / ".tools/ldraw", blender)
        self.previews = PreviewController(self.directory, library or ROOT / ".tools/ldraw", blender)

    def connect(self):
        connection = sqlite3.connect(self.directory / "archive.sqlite3")
        connection.row_factory = sqlite3.Row
        return connection

    def native_available(self, db):
        if self.native_candidates is None:
            return None
        terminal = {row[0] for row in db.execute("""
            SELECT DISTINCT b.design FROM births b
            WHERE b.status!='generated' OR EXISTS(SELECT 1 FROM evaluations e WHERE e.birth=b.id)
        """)}
        return len(self.native_candidates - terminal)

    def observe(self, db):
        job = db.execute("SELECT * FROM episodes ORDER BY id DESC LIMIT 1").fetchone()
        if job is not None and job["status"] in ("stopping", "pause-failed"):
            if job["status"] == "stopping" and (self.pause_thread is None or not self.pause_thread.is_alive()):
                try:
                    self._launch_pause(db, job)
                except (AdapterError, OSError) as error:
                    logging.exception("Could not recover episode pause request")
                    with db:
                        db.execute("UPDATE episodes SET status='pause-failed',error=? WHERE id=?",
                                   (f"Could not recover pause: {error}", job["id"]))
            return True
        if job is None or job["status"] != "running":
            return False
        if self.process is not None:
            code = self.process.poll()
            if code is None:
                return True
            self.process = None
        else:
            # A restarted dashboard must not start a second worker.
            if job["pid"] is None:
                code = None
            else:
                try:
                    os.kill(job["pid"], 0)
                except ProcessLookupError:
                    code = None
                else:
                    return True
        run = db.execute("SELECT * FROM run").fetchone()
        complete = run["status"] == "complete" and code in (0, None)
        message = None
        if not complete:
            message = f"Episode worker exited with code {code}; remaining authorized slots can be resumed. See {job['log']}."
            logfile = self.directory / job["log"]
            if logfile.is_file():
                with logfile.open("rb") as stream:
                    stream.seek(max(0, logfile.stat().st_size - 8192))
                    errors = [line for line in stream.read().decode("utf-8", errors="replace").splitlines()
                              if line.startswith("ERROR")]
                if errors:
                    message = errors[-1] + f" See {job['log']}."
            if self.native_candidates is not None:
                archive = Archive(self.directory)
                try:
                    archive.recover()
                    publish(archive)
                finally:
                    archive.db.close()
        with db:
            db.execute("UPDATE episodes SET status=?,completed=?,error=? WHERE id=?",
                       ("complete" if complete else "failed", now(), message, job["id"]))
        return False

    def status(self, *, automatic_previews=True):
        with self.mutex, contextlib.closing(self.connect()) as db:
            running = self.observe(db)
            run = db.execute("SELECT * FROM run").fetchone()
            used = db.execute("SELECT count(*) FROM evaluations").fetchone()[0]
            job = db.execute("SELECT * FROM episodes ORDER BY id DESC LIMIT 1").fetchone()
            page = (self.directory / "index.html").stat()
            available = self.native_available(db)
            active_trials = [dict(row) for row in db.execute("""
                SELECT e.id AS trial,e.birth,b.design,d.category,w.pid,w.log FROM evaluations e
                JOIN births b ON b.id=e.birth JOIN designs d ON d.id=b.design
                LEFT JOIN trial_workers w ON w.trial=e.id WHERE e.status='running' ORDER BY e.id
            """)]
            historical = None
            try:
                require_active(self.directory)
            except AdapterError as error:
                historical = str(error)
            previews = self.previews.status(watch=running, automatic=automatic_previews and historical is None)
            pipeline = pipeline_snapshot(db, self.directory, running=running,
                                         tracked=bool(self.requested_protocol.get("pipelineTracking")))
            for trial in active_trials:
                design = self.directory / "designs" / trial["design"]
                trial["preview"] = next(
                    ({"url": relative(path, self.directory / "index.html"), "kind": kind}
                     for path, kind in ((media_directory(design) / "walker-v2.png", "assembly"),
                                        (design / "candidate/cycle.svg", "motion-diagram"))
                     if path.is_file()), None)
            from .parallel import attempt_summary
            cheap = attempt_summary(db)
            return {"running": running, "pid": job["pid"] if running else None,
                    "pausing": bool(job and job["status"] == "stopping"),
                    "paused": bool(job and job["status"] == "paused"),
                    "pauseFailed": bool(job and job["status"] == "pause-failed"),
                    "used": used, "budget": run["budget"], "remaining": run["budget"] - used,
                    "finiteQueue": available is not None,
                    "canStart": not historical and self.resume_error is None and (available is None or available > 0),
                    "nextEpisodeTrials": min(run["budget"]-used or self.episode_trials, available) if available is not None else run["budget"]-used or self.episode_trials,
                    "error": historical or (job["error"] if job and job["status"] == "pause-failed" else None)
                             or self.resume_error or (job["error"] if job else None), "log": job["log"] if job else None,
                    "cheapGate": cheap,
                    "notice": ("Cheap gate attempt cap reached. Unused full-simulation allowance is retained; "
                               "another explicit episode authorizes up to three new attempts per remaining slot."
                               if cheap and cheap["pausedAtLimit"] else None),
                    "pinnedRuntime": self.pinned,
                    "simulationWorkers": self.simulation_workers, "activeTrials": active_trials,
                    "screeningWorkers": self.screening_workers,
                    "pipeline": pipeline, "assemblyPreviews": previews,
                    "revision": f"{page.st_mtime_ns}:{page.st_size}", "csrfToken": self.token,
                    "rendering": self.renderer.status()}

    def start(self):
        with self.mutex, contextlib.closing(self.connect()) as db:
            require_active(self.directory)
            if self.observe(db):
                raise AdapterError("An episode is already running.")
            if self.resume_error:
                raise AdapterError(self.resume_error)
            if self.pinned:
                try:
                    verify_snapshot(self.directory)
                except (OSError, ValueError) as error:
                    raise AdapterError(f"Cannot resume pinned experiment: {error}") from error
            with locked(self.directory):
                with db:
                    db.execute("BEGIN IMMEDIATE")
                    run = db.execute("SELECT * FROM run").fetchone()
                    used = db.execute("SELECT count(*) FROM evaluations").fetchone()[0]
                    available = self.native_available(db)
                    if available == 0:
                        raise AdapterError("All prepared native candidates have finished or spent their trial; no repeat is authorized.")
                    budget = run["budget"] if used < run["budget"] else run["budget"] + self.episode_trials
                    if available is not None:
                        budget = min(budget, used + available)
                    db.execute("UPDATE run SET budget=?,status='ready',updated=? WHERE id=1", (budget, now()))
                    number = db.execute("SELECT coalesce(max(id),0)+1 FROM episodes").fetchone()[0]
                    log = f"episode-{number:04d}.log"
                    db.execute("INSERT INTO episodes(id,budget,status,started,log) VALUES (?,?,'running',?,?)",
                               (number, budget, now(), log))
            command = [*self.worker_args, "--output", str(self.directory), "--budget", str(budget),
                       "--seed", str(self.seed), "--resume"]
            if self.pinned:
                command = [sys.executable, str(Path(__file__).with_name("pinned.py")), *self.worker_args[3:],
                           "--output", str(self.directory), "--budget", str(budget),
                           "--seed", str(self.seed), "--resume"]
            try:
                archive = Archive(self.directory)
                try:
                    publish(archive)
                finally:
                    archive.db.close()
                with (self.directory / log).open("wb") as stream:
                    self.process = subprocess.Popen(
                        command, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=stream,
                        stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
            except (AdapterError, OSError, sqlite3.Error) as error:
                with db:
                    db.execute("UPDATE episodes SET status='failed',completed=?,error=? WHERE id=?",
                               (now(), f"Could not launch episode worker: {error}", number))
                raise AdapterError(f"Could not launch episode worker: {error}") from error
            with db:
                db.execute("UPDATE episodes SET pid=? WHERE id=?", (self.process.pid, number))
            self.previews.status(watch=True)
            return {"episode": number, "pid": self.process.pid, "budget": budget, "log": log}

    def upgrade(self):
        from .upgrades import launch
        with self.mutex, contextlib.closing(self.connect()) as db:
            if self.observe(db):
                raise AdapterError("Wait for the current episode to finish before preparing an upgrade.")
            return launch(self)

    def pause(self):
        with self.mutex, contextlib.closing(self.connect()) as db:
            require_active(self.directory)
            job = db.execute("SELECT * FROM episodes ORDER BY id DESC LIMIT 1").fetchone()
            if job is not None and job["status"] == "stopping":
                raise AdapterError("The episode is already pausing.")
            if not self.observe(db):
                raise AdapterError("No episode is running.")
            job = db.execute("SELECT * FROM episodes ORDER BY id DESC LIMIT 1").fetchone()
            try:
                if os.getpgid(job["pid"]) != job["pid"]:
                    raise AdapterError("Episode PID is not its process-group leader; refusing to signal it.")
            except ProcessLookupError:
                if job["status"] != "pause-failed":
                    self.observe(db)
                    raise AdapterError("The episode has already stopped.")
            self._launch_pause(db, job)
            return {"status": "pausing", "episode": job["id"]}

    def _launch_pause(self, db, job):
        try:
            if os.getpgid(job["pid"]) != job["pid"]:
                raise AdapterError("Episode PID is not its process-group leader; refusing to signal it.")
        except ProcessLookupError:
            pass
        with db:
            db.execute("UPDATE episodes SET status='stopping',error=NULL WHERE id=?", (job["id"],))
        self.pause_thread = threading.Thread(target=self._pause_worker, args=(dict(job),),
                                             name="episode-pause", daemon=True)
        try:
            self.pause_thread.start()
        except RuntimeError as error:
            with db:
                db.execute("UPDATE episodes SET status='pause-failed',error=? WHERE id=?",
                           (f"Could not start pause: {error}", job["id"]))
            raise AdapterError(f"Could not start pause: {error}") from error

    def _pause_worker(self, job):
        completed = False
        try:
            try:
                os.killpg(job["pid"], signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 10
            escalated = False
            while True:
                if self.process is not None:
                    self.process.poll()
                try:
                    os.killpg(job["pid"], 0)
                except ProcessLookupError:
                    break
                except PermissionError:
                    # macOS can briefly return EPERM while the last group members exit.
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.1)
                    continue
                if time.monotonic() >= deadline:
                    if escalated:
                        raise AdapterError("Episode workers have not exited; do not close the computer yet.")
                    try:
                        os.killpg(job["pid"], signal.SIGKILL)
                    except ProcessLookupError:
                        break
                    escalated = True
                    deadline = time.monotonic() + 5
                time.sleep(.1)
            with self.mutex, locked(self.directory):
                archive = Archive(self.directory)
                try:
                    archive.recover()
                    admissions = archive.db.execute("""
                        SELECT batch,task FROM pipeline_startup WHERE kind='admission' AND status='running'
                    """).fetchall()
                    if admissions:
                        from .user_pool import interrupt_admission
                        snapshot = archive.db.execute(
                            "SELECT entries FROM user_pool_snapshots ORDER BY id DESC LIMIT 1").fetchone()
                        entries = {entry["id"]: entry for entry in json.loads(snapshot["entries"])} if snapshot else {}
                        for admission in admissions:
                            if admission["task"] not in entries:
                                raise AdapterError("Interrupted admission has no immutable input snapshot.")
                            interrupt_admission(archive, {"entry": entries[admission["task"]]},
                                                "Episode paused; partial admission output is retained.")
                    with archive.db:
                        archive.db.execute("""UPDATE births SET status='construction-interrupted',
                            error='Episode paused; partial construction output is retained.'
                            WHERE id IN (SELECT birth FROM screening_workers
                                WHERE status='running' AND log LIKE 'construction-workers/%')""")
                        archive.db.execute("UPDATE screening_workers SET status='interrupted' WHERE status='running'")
                        if archive.db.execute("SELECT 1 FROM sqlite_master WHERE name='native_proposals'").fetchone():
                            archive.db.execute("""UPDATE native_proposals SET status='interrupted',
                                error='Episode paused; partial proposal output is retained.' WHERE status='running'""")
                        archive.db.execute("""UPDATE pipeline_startup SET status='interrupted',
                            detail='Episode paused; partial output is retained.',completed=? WHERE status='running'""",
                            (now(),))
                        archive.db.execute("UPDATE run SET status='paused',updated=? WHERE id=1", (now(),))
                        archive.db.execute("UPDATE episodes SET status='paused',completed=?,error=NULL WHERE id=?",
                                           (now(), job["id"]))
                    publish(archive)
                    self.process = None
                    completed = True
                finally:
                    archive.db.close()
        except (AdapterError, OSError, sqlite3.Error, ValueError):
            logging.exception("Episode pause failed")
            with self.mutex, contextlib.closing(self.connect()) as db, db:
                db.execute("UPDATE episodes SET status='pause-failed',error=? WHERE id=?",
                           (f"Could not finish pausing: {sys.exc_info()[1]}", job["id"]))
        finally:
            if not completed:
                with self.mutex, contextlib.closing(self.connect()) as db, db:
                    db.execute("""UPDATE episodes SET status='pause-failed',
                        error=coalesce(error,'Unexpected pause failure; see the dashboard process log.')
                        WHERE id=? AND status IN ('stopping','paused')""", (job["id"],))


class DashboardServer(ThreadingHTTPServer):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.forest_mutex = threading.Lock()
        self.forest_cache = None

    def campaign_forest(self, versions, active):
        directory = Path(next(version["archive"] for version in versions if version["id"] == active))
        page = (directory / "index.html").stat()
        key = (tuple((version["id"], version["archive"]) for version in versions),
               active, page.st_mtime_ns, page.st_size)
        with self.forest_mutex:
            if self.forest_cache is not None:
                previous, generated, content = self.forest_cache
                if previous == key and time.monotonic() - generated < 10:
                    return content
            content = campaign_snapshot(versions, active)
            self.forest_cache = (key, time.monotonic(), content)
            return content


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, request, client_address, server):
        super().__init__(request, client_address, server, directory=str(server.controller.directory))

    def allowed_host(self):
        host = self.headers.get("Host")
        allowed = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
        if host not in allowed:
            self.json_response(403, {"error": "Dashboard is available only through its loopback address."})
            return False
        return True

    def json_response(self, status, value):
        body = json.dumps(value).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def html_response(self, value, *, head=False):
        from urllib.parse import urlencode
        from .gallery import motion_assets
        value = motion_assets(value)
        path = urlsplit(self.path).path
        if path != "/workbench.html":
            link, label = "/workbench.html", "Breeding workbench"
            match = re.fullmatch(r"(?:/versions/([^/]+))?/history/(\d+)\.html", path)
            if match:
                controller = self.server.controller
                version = match[1] or (controller.state["active"] if isinstance(controller, CampaignController)
                                       else controller.directory.name)
                link += "?" + urlencode({"version": version, "birth": int(match[2])})
                label = "Fork this candidate in the breeding workbench"
            value = value.replace("<body>", f'<body><nav><a href="{link}">{label}</a></nav>', 1)
        body = value.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if not head:
            self.wfile.write(body)

    def do_GET(self):
        try:
            self.get()
        except (AdapterError, OSError, sqlite3.Error, ValueError) as error:
            self.log_error("Dashboard request failed: %s", error)
            self.json_response(500, {"error": str(error)})

    def get(self):
        if not self.allowed_host():
            return
        path = urlsplit(self.path)
        if path.path == "/workbench.html":
            self.html_response(self.workbench_page())
        elif path.path == "/workbench.js":
            body = Path(__file__).with_name("workbench.js").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/javascript; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path.path == "/api/workbench":
            store, generator, active = self.workbench()
            query = parse_qs(path.query)
            if "checks" in query:
                self.json_response(200, {"checks": store.checks()})
            elif "edits" in query:
                self.json_response(200, store.edits(query["edits"][0], generator))
            else:
                from .user_pool import status as user_status
                self.json_response(200, {**store.state(), "sources": self.workbench_sources(),
                                         "activeVersion": (self.server.controller.state["active"]
                                                           if isinstance(self.server.controller, CampaignController)
                                                           else active.directory.name),
                                         "admissions": user_status(active.directory)})
        elif path.path.startswith("/workbench-assets/"):
            self.workbench_asset(path.path)
        elif path.path == "/api/status":
            try:
                self.json_response(200, self.server.controller.status())
            except (OSError, sqlite3.Error) as error:
                self.log_error("Status failed: %s", error)
                self.json_response(500, {"error": str(error)})
        elif path.path == "/migration.log" and isinstance(self.server.controller, CampaignController):
            log = self.server.controller.root / "migration.log"
            if not log.is_file():
                self.send_error(404, "No preparation log is available.")
                return
            body = log.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif isinstance(self.server.controller, CampaignController) and urlsplit(self.path).path == "/api/history":
            self.json_response(200, self.server.controller.history())
        elif isinstance(self.server.controller, CampaignController) and urlsplit(self.path).path == "/history.html":
            self.html_response(self.server.controller.history_page())
        elif urlsplit(self.path).path == "/api/forest":
            controller = self.server.controller
            if isinstance(controller, CampaignController):
                with controller.mutex:
                    versions = [dict(version) for version in controller.state["versions"]]
                    active = controller.state["active"]
                forest = self.server.campaign_forest(versions, active)
            else:
                forest = snapshot(controller.directory)
            self.json_response(200, {"html": forest})
        elif urlsplit(self.path).path == "/api/motif-opportunities":
            self.json_response(200, self.motif_opportunities())
        elif urlsplit(self.path).path == "/motif-opportunities.html":
            from .motif_opportunities import page
            self.html_response(page(self.motif_opportunities()))
        elif urlsplit(self.path).path in ("/mutation-gallery.html", "/motif-gallery.html"):
            from .gallery import motif_gallery, mutation_gallery
            self.html_response(motif_gallery() if urlsplit(self.path).path == "/motif-gallery.html" else mutation_gallery())
        else:
            self.serve_file()

    def do_HEAD(self):
        if self.allowed_host():
            if urlsplit(self.path).path == "/workbench.html":
                self.html_response(self.workbench_page(), head=True)
            elif urlsplit(self.path).path.startswith("/workbench-assets/"):
                self.workbench_asset(urlsplit(self.path).path, head=True)
            elif isinstance(self.server.controller, CampaignController) and urlsplit(self.path).path == "/history.html":
                self.html_response(self.server.controller.history_page(), head=True)
            elif urlsplit(self.path).path == "/motif-opportunities.html":
                from .motif_opportunities import page
                self.html_response(page(self.motif_opportunities()), head=True)
            elif urlsplit(self.path).path in ("/mutation-gallery.html", "/motif-gallery.html"):
                from .gallery import motif_gallery, mutation_gallery
                self.html_response(motif_gallery() if urlsplit(self.path).path == "/motif-gallery.html" else mutation_gallery(), head=True)
            else:
                self.serve_file(head=True)

    def motif_opportunities(self):
        from .motif_opportunities import report
        controller = self.server.controller
        if isinstance(controller, CampaignController):
            with controller.mutex:
                versions = [dict(version) for version in controller.state["versions"]]
        else:
            versions = [{"id": controller.directory.name, "archive": str(controller.directory)}]
        return report(versions)

    def workbench(self):
        from .workbench import Workbench
        controller = self.server.controller
        active = controller.active if isinstance(controller, CampaignController) else controller
        root = controller.root if isinstance(controller, CampaignController) else controller.directory
        store = Workbench(root / "user-workbench")
        args = active.worker_args
        dotnet = args[args.index("--dotnet")+1] if "--dotnet" in args else ROOT / ".tools/dotnet/dotnet"
        dll = active.directory / "protocol-source/generator/LegoWalkers.Cli.dll"
        if not dll.is_file():
            dll = Path(args[args.index("--generator")+1]) if "--generator" in args else ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll"
        return store, Generator(dotnet, dll), active

    def workbench_sources(self):
        from .campaign import database
        controller = self.server.controller
        versions = (list(reversed(controller.state["versions"])) if isinstance(controller, CampaignController)
                    else [{"id": controller.directory.name, "archive": str(controller.directory)}])
        sources = []
        for version in versions:
            with database(version["archive"]) as db:
                for row in db.execute("""SELECT b.id,b.design,b.operation,d.category FROM births b
                    JOIN designs d ON d.id=b.design WHERE b.status!='duplicate' ORDER BY b.id DESC"""):
                    candidate = Path(version["archive"]) / "designs" / row["design"] / "candidate"
                    if (candidate / "program.json").is_file():
                        sources.append({"version": version["id"], "birth": row["id"], "design": row["design"],
                                        "label": f'{version["id"]} birth {row["id"]} - {row["category"]} - {row["operation"]}'})
        return sources

    def workbench_page(self):
        body = Path(__file__).with_name("workbench.html").read_text()
        return document("Walker breeding workbench", body)

    def workbench_asset(self, path, *, head=False):
        store, generator, _ = self.workbench()
        segments = path.split("/")
        if len(segments) != 4:
            raise AdapterError("Invalid workbench asset.")
        node, name = segments[2:]
        with store.connect() as db:
            store.node(db, node)
        candidate = store.candidate(node)
        if name == "motion.json":
            self.motion_response(candidate, generator, head=head)
            return
        allowed = {name: candidate / name for name in
                   ("manifest.json", "model.ldr", "cycle.svg", "program.json", "realization.json", "gates.json", "construction.json",
                    "motion-clearance.json", "motion-clearance-repair.json")}
        allowed.update({"walker-v2.png": candidate.parent / "assembly-render/walker-v2.png",
                        "assembly-preview.log": candidate.parent / "assembly-preview.log",
                        "contribution-validation.json": store.root / "validations" / node / "latest.json"})
        if name not in allowed:
            raise AdapterError("Unknown workbench asset.")
        target = allowed[name]
        if not target.is_file():
            self.send_error(404, "Asset is not available.")
            return
        content = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", self.guess_type(str(target)))
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        if not head:
            self.wfile.write(content)

    def motion_response(self, candidate, generator, *, head=False):
        from .motion_diagrams import solved_motion
        content = json.dumps(solved_motion(candidate, generator.command), allow_nan=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        if not head:
            self.wfile.write(content)

    def workbench_action(self, value):
        store, generator, active = self.workbench()
        action = value.get("action")
        fields = {"fork": {"version", "birth", "label", "author"}, "apply": {"node", "edit"},
                  "rebase": {"node", "version", "birth"},
                  "preview": {"node", "edit"},
                  "keep": {"node"}, "checkout": {"node"}, "discard": {"node"}, "render": {"node"},
                  "contribute": {"node", "label", "author", "note"}, "withdraw": {"contribution"}}
        if not isinstance(action, str) or action not in fields or set(value) != fields[action] | {"csrfToken", "action"}:
            raise AdapterError("Invalid workbench action fields.")
        if action == "fork":
            controller = self.server.controller
            if isinstance(controller, CampaignController):
                directory = controller.version_directory(value["version"])
            elif value["version"] == controller.directory.name:
                directory = controller.directory
            else:
                raise AdapterError("Unknown source version.")
            return store.fork(directory, value["birth"], value["label"], value["author"])
        if action == "apply":
            return store.apply(value["node"], value["edit"], generator)
        if action == "rebase":
            controller = self.server.controller
            version = controller.state["active"] if isinstance(controller, CampaignController) else active.directory.name
            if value["version"] != version:
                raise AdapterError("Rebase onto a candidate from the active campaign version.")
            return store.rebase(value["node"], active.directory, value["birth"])
        if action == "preview":
            return store.preview(value["node"], value["edit"], generator, active.renderer.library)
        if action in ("keep", "checkout", "discard"):
            return store.select(value["node"], action)
        if action == "render":
            return store.render(value["node"], active.renderer.library, active.renderer.blender)
        if action == "withdraw":
            return store.withdraw(value["contribution"])
        profile = active.directory / "protocol-source/selected-payloads.json"
        return store.contribute(value["node"], value["label"], value["author"], value["note"],
                                active.renderer.library, profile if profile.is_file() else None,
                                target_archive=active.directory)

    def serve_file(self, *, head=False):
        path = urlsplit(self.path)
        original_path, original_directory = self.path, self.directory
        campaign = self.server.controller if isinstance(self.server.controller, CampaignController) else None
        motion = re.fullmatch(r"/(?:versions/(v\d{4})/)?designs/([a-f0-9]{64})/candidate/motion.json",
                              unquote(path.path))
        if motion:
            version, design = motion.groups()
            if version and not campaign:
                self.send_error(404, "Unknown experiment version.")
                return
            try:
                directory = campaign.version_directory(version) if version else self.server.controller.directory
            except AdapterError as error:
                self.send_error(404, str(error))
                return
            candidate = (directory / "designs" / design / "candidate").resolve()
            if not candidate.is_relative_to(directory.resolve()):
                self.send_error(403, "Motion programs must remain inside their archive.")
                return
            if not (candidate / "program.json").is_file():
                self.send_error(404, "Saved motion program is unavailable.")
                return
            _, generator, _ = self.workbench()
            self.motion_response(candidate, generator, head=head)
            return
        if path.path == "/forest.html":
            controller = self.server.controller
            if campaign:
                with campaign.mutex:
                    versions = [dict(version) for version in campaign.state["versions"]]
                    active = campaign.state["active"]
                forest = self.server.campaign_forest(versions, active)
            else:
                forest = snapshot(controller.directory, details=True)
            self.html_response(document("Proposal ancestry forest", '<a href="/">Back to dashboard</a>'
                                        + forest), head=head)
            return
        if campaign and path.path in ("/", "/index.html", "/preview.html"):
            with campaign.mutex:
                text = campaign.dashboard_page()
                versions = [dict(version) for version in campaign.state["versions"]]
                active = campaign.state["active"]
            forest = self.server.campaign_forest(versions, active)
            text = insert(text, forest)
            self.html_response(category_anchors(text), head=head)
            return
        if campaign and path.path.startswith("/versions/"):
            segments = path.path.split("/", 3)
            if len(segments) != 4:
                self.send_error(404, "Unknown experiment version.")
                return
            try:
                directory = campaign.version_directory(segments[2])
            except AdapterError as error:
                self.send_error(404, str(error))
                return
            relative = unquote(segments[3]) or "index.html"
            target = (directory / relative).resolve()
            if not target.is_relative_to(directory.resolve()):
                self.send_error(403, "Version files must remain inside their archive.")
                return
            if relative == "forest.html":
                prefix = f'/versions/{segments[2]}/'
                text = document("Proposal ancestry forest", '<a href="/">Current experiment</a>'
                                + snapshot(directory, prefix, details=True))
                self.html_response(version_html(text, prefix + "forest.html", readonly=True), head=head)
                return
            if target.is_file() and target.suffix == ".html":
                text = birth_previews(birth_page(target.read_text(), directory, relative), directory, relative)
                if relative in ("index.html", "preview.html"):
                    text = insert(text, snapshot(directory, f"/versions/{segments[2]}/"))
                self.html_response(version_html(category_anchors(collapsible_genes(preview_first(text))), f"/versions/{segments[2]}/{segments[3]}",
                                                readonly=True,
                                                invalidation=evidence_invalidation(definition(directory))), head=head)
                return
            self.directory, self.path = str(directory), "/" + segments[3]
        elif path.path in ("/", "/index.html", "/preview.html"):
            name = "preview.html" if path.path == "/preview.html" else "index.html"
            text = (self.server.controller.directory / name).read_text()
            forest = snapshot(self.server.controller.directory)
            text = insert(text, forest)
            self.html_response(category_anchors(text), head=head)
            return
        elif path.path.startswith("/history/") and path.path.endswith(".html"):
            directory = self.server.controller.directory
            target = (directory / unquote(path.path).lstrip("/")).resolve()
            if not target.is_relative_to(directory.resolve()):
                self.send_error(403, "Birth pages must remain inside their archive.")
                return
            if target.is_file():
                relative_page = target.relative_to(directory.resolve()).as_posix()
                text = birth_previews(birth_page(target.read_text(), directory, relative_page), directory, relative_page)
                self.html_response(collapsible_genes(preview_first(text)), head=head)
                return
        elif path.path.startswith("/motif-illustrations/"):
            self.directory = str(ROOT / "tests/visualizations/motifs")
            self.path = "/" + path.path.removeprefix("/motif-illustrations/")
        elif path.path.startswith("/mutation-animations/"):
            self.directory = str(ROOT / "tests/visualizations")
            self.path = "/" + path.path.removeprefix("/mutation-animations/")
        elif path.path.startswith("/archives/"):
            parts = path.path.split("/", 3)
            if len(parts) < 4 or parts[2] not in ("evolution-pilot", "evolution-feet-wagon-pilot", "render"):
                self.send_error(404, "Unknown reference archive.")
                return
            self.directory = str(self.server.controller.directory.parent / parts[2])
            self.path = "/" + parts[3] + ("?" + path.query if path.query else "")
        try:
            if head:
                super().do_HEAD()
            else:
                super().do_GET()
        finally:
            self.path, self.directory = original_path, original_directory

    def do_POST(self):
        if not self.allowed_host():
            return
        if urlsplit(self.path).path.startswith("/versions/"):
            self.json_response(405, {"error": "Historical versions are read-only."})
            return
        controller = self.server.controller
        actions = {"/api/episode": controller.start,
                   "/api/episode/pause": controller.pause,
                   "/api/upgrade": self.server.controller.upgrade,
                   "/api/render/start": controller.render_start if isinstance(controller, CampaignController) else controller.renderer.start,
                   "/api/render/stop": controller.render_stop if isinstance(controller, CampaignController) else controller.renderer.stop,
                   "/api/previews/retry": lambda: (controller.active if isinstance(controller, CampaignController)
                                                   else controller).previews.retry(),
                   "/api/workbench": None}
        if isinstance(controller, CampaignController):
            actions["/api/requalify"] = None
        if self.path not in actions:
            self.json_response(404, {"error": "Unknown action."})
            return
        origin = self.headers.get("Origin")
        if origin is not None and origin != f"http://{self.headers.get('Host')}":
            self.json_response(403, {"error": "Cross-origin worker requests are not allowed."})
            return
        if self.headers.get("Content-Type") != "application/json":
            self.json_response(415, {"error": "Expected application/json."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= (4096 if self.path == "/api/workbench" else 1024):
                raise ValueError("Request must contain a small JSON object.")
            value = json.loads(self.rfile.read(length))
            expected = {"csrfToken", "trials"} if self.path == "/api/requalify" else {"csrfToken"}
            if (not isinstance(value, dict) or (self.path != "/api/workbench" and set(value) != expected)
                    or not isinstance(value["csrfToken"], str)
                    or not secrets.compare_digest(value["csrfToken"], self.server.controller.token)):
                self.json_response(403, {"error": "Invalid worker authorization token."})
                return
            if self.path == "/api/requalify":
                response = controller.requalify(value["trials"])
            else:
                response = self.workbench_action(value) if self.path == "/api/workbench" else actions[self.path]()
            self.json_response(202, response)
        except AdapterError as error:
            self.log_error("Worker action refused: %s", error)
            self.json_response(409, {"error": str(error)})
        except (ValueError, UnicodeDecodeError) as error:
            self.json_response(400, {"error": str(error)})
        except (OSError, sqlite3.Error, subprocess.SubprocessError) as error:
            self.log_error("Worker action failed: %s", error)
            self.json_response(500, {"error": str(error)})

    def list_directory(self, path):
        self.send_error(403, "Directory listings are disabled.")
        return None

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "output/evolution-interactive")
    parser.add_argument("--campaign", type=Path, help="Stable experiment registry; adopt this archive and its upgrades")
    parser.add_argument("--defer-upgrade", action="store_true",
                        help="Compatibility flag: campaigns always defer rebuilding until explicit preparation")
    parser.add_argument("--describe-protocol", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--prepare-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--defer-construction", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--initial-budget", type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--compatible-archive", type=Path, action="append", default=[], help=argparse.SUPPRESS)
    parser.add_argument("--ldraw-library", type=Path, default=ROOT / ".tools/ldraw")
    parser.add_argument("--dotnet", type=Path, default=ROOT / ".tools/dotnet/dotnet")
    parser.add_argument("--generator", type=Path, default=ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    parser.add_argument("--blender", type=Path, help="Blender executable for the separately controlled animation worker")
    parser.add_argument("--payload-profile", type=Path)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--candidate-batch", type=Path,
                        help="Stage a finite native walker batch without starting any episode")
    parser.add_argument("--discover", action="store_true", help="Enable native reproduction for explicitly authorized episodes")
    parser.add_argument("--episode-trials", type=int, default=12, help="Maximum new trials authorized per button press")
    parser.add_argument("--cycle-aware", action="store_true", help="Use bounded two-cycle gait evaluation for a new experiment")
    parser.add_argument("--short-gate", type=float, default=2.0,
                        help="Early safety gate in driven seconds after settling (default: 2; 0 disables)")
    parser.add_argument("--reuse-archive", type=Path, action="append", default=[], help="Reuse compatible founder measurements")
    parser.add_argument("--simulation-workers", type=int,
                        help="Concurrent native simulations (default: 2 for native dashboards, 1 for legacy)")
    parser.add_argument("--screening-workers", type=int, default=1,
                        help="Concurrent screening/preflight processes for native discovery")
    args = parser.parse_args(argv)
    try:
        args.simulation_workers = worker_count(args.simulation_workers if args.simulation_workers is not None
                                              else 2 if args.candidate_batch is not None else 1)
        from .prescreen import validate_workers
        validate_workers(args.screening_workers)
        if args.screening_workers > 1 and not args.discover:
            raise AdapterError("Parallel screening requires native --discover.")
        if args.simulation_workers > 1 and args.candidate_batch is None:
            raise AdapterError("Parallel simulations require a native --candidate-batch.")
        if not 0 <= args.seed < 2147483647:
            raise AdapterError("Seed must be between 0 and 2147483646.")
        if args.discover and args.candidate_batch is None:
            raise AdapterError("--discover requires a complete native --candidate-batch.")
        if args.reuse_archive and not args.discover:
            raise AdapterError("--reuse-archive requires native --discover.")
        if args.campaign and not args.discover:
            raise AdapterError("--campaign requires native --discover.")
        if args.initial_budget < 0 or ((args.initial_budget or args.compatible_archive) and not args.prepare_only):
            raise AdapterError("Transferred allowance and compatible-archive selection require version preparation.")
        if args.prepare_only and (not args.discover or args.campaign):
            raise AdapterError("Version preparation requires native discovery without a nested experiment registry.")
        if args.campaign:
            args.cycle_aware = True
            args.defer_construction = True
        options = EvaluationOptions(duration=20, timestep=.002, capture_poses=True,
                                    cycle_aware=args.cycle_aware, short_gate=args.short_gate)
        generator = Generator(args.dotnet.resolve(), args.generator.resolve())
        protocol = protocol_definition(generator, args.ldraw_library.resolve(), options, load_profile(args.payload_profile))
        def current_inputs(source):
            descriptor = Path(source) / "seeds.json"
            rechecks = descriptor.exists() and json.loads(descriptor.read_text()).get("requalificationBatches")
            if args.defer_construction and not rechecks:
                return source
            from .native_episode import refresh_seed_exports
            return refresh_seed_exports(source, args.campaign / "input-exports", generator)
        frozen_campaign = False
        if args.campaign and args.candidate_batch is not None:
            registry = args.campaign / "campaign.json"
            if registry.exists():
                state = json.loads(registry.read_text())
                version = next(v for v in state["versions"] if v["id"] == state["active"])
                args.candidate_batch = Path(version["archive"]) / "native-input"
                frozen_campaign = not args.describe_protocol and not args.prepare_only
                if frozen_campaign:
                    protocol = definition(Path(version["archive"]))
            if not frozen_campaign:
                args.candidate_batch = current_inputs(args.candidate_batch)
        if args.candidate_batch is not None and not frozen_campaign:
            from .native_episode import native_protocol
            protocol = native_protocol(protocol, args.candidate_batch.resolve(), discover=args.discover,
                                       reuse_archives=args.reuse_archive, simulation_workers=args.simulation_workers,
                                       screening_workers=args.screening_workers, defer_construction=args.defer_construction)
        if args.compatible_archive:
            from .campaign import observation_sources
            compatible = observation_sources(args.compatible_archive, protocol, target=args.output)
            if compatible:
                protocol["reuseArchives"] = compatible
        if args.describe_protocol:
            print(json.dumps(protocol))
            return 0
        worker = [sys.executable, "-m", "legowalkers_sim.evolution",
                  "--ldraw-library", str(args.ldraw_library.resolve()),
                  "--dotnet", str(args.dotnet.resolve()), "--generator", str(args.generator.resolve())]
        if args.candidate_batch is not None:
            worker += ["--candidate-batch", str(args.candidate_batch.resolve())]
        if args.discover:
            worker += ["--discover"]
        if args.cycle_aware:
            worker += ["--cycle-aware"]
        worker += ["--short-gate", str(args.short_gate)]
        for path in args.reuse_archive:
            worker += ["--reuse-archive", str(path.resolve())]
        for flag, value in (("--payload-profile", args.payload_profile),):
            if value:
                worker += [flag, str(value.resolve())]
        # One server per archive; its worker has a separate simulation lock.
        lock_directory = args.campaign or args.output
        lock_directory.mkdir(parents=True, exist_ok=True)
        import fcntl
        with (lock_directory / ".dashboard.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise AdapterError("Another dashboard already controls this archive.") from error
            blender = args.blender or shutil.which("blender")
            if blender is None:
                installed = Path("/Applications/Blender.app/Contents/MacOS/Blender")
                if installed.is_file():
                    blender = installed
            def configured_worker(source, selected):
                command = []
                skip = False
                for item in worker:
                    if skip:
                        skip = False
                    elif item in ("--reuse-archive", "--short-gate"):
                        skip = True
                    else:
                        command.append(item)
                if source is not None:
                    command[command.index("--candidate-batch")+1] = str(source)
                for source_archive in selected.get("reuseArchives", []):
                    command += ["--reuse-archive", source_archive]
                gate = selected.get("evaluation", {}).get("short_gate")
                if not selected:
                    gate = args.short_gate
                if gate is not None:
                    command += ["--short-gate", str(gate)]
                count = selected.get("execution", {}).get("simulationWorkers")
                if not selected:
                    count = args.simulation_workers
                if count is not None:
                    command += ["--simulation-workers", str(count)]
                screening = selected.get("execution", {}).get("screeningWorkers")
                if not selected:
                    screening = args.screening_workers
                if screening is not None:
                    command += ["--screening-workers", str(screening)]
                if selected.get("deferredConstruction") or (not selected and args.defer_construction):
                    command += ["--defer-construction"]
                return command

            def load(directory):
                selected = definition(directory)
                with sqlite3.connect(Path(directory) / "archive.sqlite3") as db:
                    seed = db.execute("SELECT seed FROM run").fetchone()[0]
                source = Path(directory) / "native-input"
                command = configured_worker(source, selected)
                profile = Path(directory) / "protocol-source/selected-payloads.json"
                if profile.exists():
                    if "--payload-profile" in command:
                        command[command.index("--payload-profile")+1] = str(profile)
                    else:
                        command += ["--payload-profile", str(profile)]
                return EpisodeController(directory, selected, command,
                                         seed=seed, episode_trials=args.episode_trials,
                                         library=args.ldraw_library, blender=blender)

            def build(directory, source, selected, allowance=0):
                value = EpisodeController(directory, selected, configured_worker(source, selected),
                                          seed=args.seed, episode_trials=args.episode_trials,
                                          library=args.ldraw_library, blender=blender, initial_budget=allowance)
                archive = Archive(directory)
                try:
                    if source is not None:
                        from .native_episode import stage_native
                        from .observations import reuse_observations
                        stage_native(archive, source)
                        if not selected.get("deferredConstruction"):
                            reuse_observations(archive, args.ldraw_library)
                    if args.campaign or args.prepare_only:
                        from .evolution import freeze_runtime
                        from .bridge import write_json
                        freeze_runtime(archive, selected, generator)
                        profile = Path(directory) / "protocol-source/selected-payloads.json"
                        write_json(profile, selected["payloadProfile"])
                        if "--payload-profile" in value.worker_args:
                            value.worker_args[value.worker_args.index("--payload-profile")+1] = str(profile)
                        else:
                            value.worker_args += ["--payload-profile", str(profile)]
                        verify_snapshot(Path(directory))
                        value.pinned = True
                    publish(archive)
                finally:
                    archive.db.close()
                return value

            if args.campaign:
                def describe(source):
                    source = current_inputs(source)
                    command = configured_worker(source, {})
                    command[2] = "legowalkers_sim.dashboard"
                    result = subprocess.run([*command, "--describe-protocol"], cwd=ROOT, capture_output=True, text=True)
                    if result.returncode:
                        raise AdapterError(f"Could not prepare current implementation: {result.stderr.strip()}")
                    return json.loads(result.stdout)
                def prepare_version(directory, source, selected, allowance):
                    source = current_inputs(source)
                    command = configured_worker(source, {})
                    command[2] = "legowalkers_sim.dashboard"
                    command += ["--prepare-only", "--output", str(directory),
                                "--initial-budget", str(allowance), "--seed", str(args.seed)]
                    for version in controller.state["versions"]:
                        command += ["--compatible-archive", version["archive"]]
                    with (args.campaign / "migration.log").open("a") as log:
                        result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, text=True)
                    if result.returncode:
                        raise AdapterError(f"Frozen version preparation failed (exit code {result.returncode}); see migration.log.")
                    return load(directory)
                controller = CampaignController(args.campaign, args.output, load=load, build=prepare_version,
                                                describe=describe, generator=generator)
                print("Rebuilding/migration requires explicit preparation; Resume never prepares or migrates.", flush=True)
            else:
                controller = build(args.output, args.candidate_batch.resolve() if args.candidate_batch else None,
                                   protocol, args.initial_budget)
                if args.prepare_only:
                    print(json.dumps({"prepared": str(controller.directory),
                                      "budget": controller.status(automatic_previews=False)["budget"]}))
                    return 0
            with DashboardServer(("127.0.0.1", args.port), DashboardHandler) as server:
                server.controller = controller
                print(f"Live dashboard: http://127.0.0.1:{server.server_port}/ ; no episode starts until you press the button.", flush=True)
                server.serve_forever()
        return 0
    except (AdapterError, OSError, sqlite3.Error, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 2
    except KeyboardInterrupt:
        print("Dashboard stopped. Any independent episode worker continues; restart with the same --output.", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
