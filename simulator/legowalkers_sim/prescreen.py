"""Isolated screening/preflight children; the scheduler owns archive accounting."""

import contextlib
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import multiprocessing
from multiprocessing.connection import Connection, wait
import os
from pathlib import Path
import sys
import time
import traceback

from .bridge import AdapterError, write_json
from .pipeline import record
from .screening_reports import rejection_detail


def validate_workers(value):
    if type(value) is not int or value < 1:
        raise AdapterError("Screening worker count must be a positive integer.")
    return value


def _progress(message):
    print(f"[{datetime.now(timezone.utc).isoformat(timespec='seconds')}] {message}", flush=True)


def screen_candidate(batch, output, design, manifest, library, options, stage):
    from .native_episode import PreflightRejection, prepare_native_candidate
    from .screening import cache_directory, screen_batch

    started = time.monotonic()
    _progress(f"Load-only screening started for {design}: reconstruction, mass checks and time-zero collision/closure checks.")
    screened = screen_batch(batch, cache_directory(output, payload_profile=options.payload_profile),
                            library, max_checks=1, payload_profile=options.payload_profile,
                            candidate_ids={design})
    if len(screened["results"]) != 1:
        raise AdapterError("Isolated screening must return exactly one candidate report.")
    report = screened["results"][0]
    status = report.get("previousStatus", report["status"])
    checks = screened["counts"]["reconstructionAttempts"]
    _progress(f"Load-only screening finished after {time.monotonic()-started:.1f}s: {status}; reconstruction checks: {checks}.")
    for gate in report.get("gates", []):
        _progress(f"Reported gate (saved generation or screening): {gate['gate']}: {gate['outcome']}; {gate.get('code', '')}")
    if status != "ready-for-dynamics":
        return {"status": "screening-rejected", "checks": checks,
                "error": rejection_detail(report)}
    stage("preflight")
    _progress("Final preflight started: search/contact policy, mass layout and gait envelope.")
    try:
        checked, layout = prepare_native_candidate(manifest, library, options)
    except PreflightRejection as error:
        return {"status": error.status, "checks": checks, "error": str(error), "layout": error.layout}
    return {"status": "ready", "checks": checks, "checked": checked, "layout": layout}


def _run(evaluator, arguments, sender, log):
    with Path(log).open("w") as stream, contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
        started = time.monotonic()
        _progress(f"Screening worker {os.getpid()} started. This worker reserves no walking trials.")

        def stage(name):
            _progress(f"Stage: {name}.")
            sender.send({"stage": name})

        try:
            result = evaluator(*arguments, stage)
            if isinstance(result, dict):
                _progress(f"Worker finished after {time.monotonic()-started:.1f}s: {result.get('status', 'missing outcome')}.")
                if result.get("error"):
                    _progress(f"Diagnostic: {result['error']}")
            else:
                _progress(f"Worker returned an invalid result after {time.monotonic()-started:.1f}s.")
            sender.send({"result": result})
        except (AdapterError, OSError, ValueError) as error:
            traceback.print_exc()
            _progress(f"Worker failed after {time.monotonic()-started:.1f}s: {error}")
            sender.send({"error": str(error)})
        finally:
            if sys.exc_info()[0] is not None:
                traceback.print_exc()
            sender.close()


@dataclass
class Job:
    birth: int
    check: int
    manifest: Path
    process: multiprocessing.Process
    receiver: Connection
    log: Path


class ScreeningPool:
    def __init__(self, archive, workers, library, options, evaluator=screen_candidate):
        self.archive, self.library, self.options = archive, library, options
        self.workers, self.evaluator = validate_workers(workers), evaluator
        self.context = multiprocessing.get_context("spawn")
        self.jobs = []
        self.failures = 0
        for row in archive.db.execute("SELECT birth,pid FROM screening_workers WHERE status='running'").fetchall():
            try:
                os.kill(row["pid"], 0)
            except ProcessLookupError:
                with archive.db:
                    archive.db.execute("UPDATE screening_workers SET status='interrupted' WHERE birth=?", (row["birth"],))
                record(archive, row["birth"], "paused", "Previous screening worker exited; no simulation trial was spent.")
            else:
                raise AdapterError(f"Screening process {row['pid']} is still running; wait before resuming.")

    @property
    def full(self):
        return len(self.jobs) >= self.workers

    @property
    def births(self):
        return {job.birth for job in self.jobs}

    def __enter__(self):
        return self

    def __exit__(self, kind, error, trace):
        if self.jobs:
            self.cancel("Screening dispatcher stopped; no simulation trial was reserved.")

    def submit(self, pending, used, maximum):
        from .discovery import DiscoveryLimit
        from .native_episode import _queue

        if self.full:
            raise AdapterError("Screening pool is full.")
        if pending["birth"] in self.births:
            raise AdapterError("Candidate already has an active screening worker.")
        batch = self.archive.directory / pending["batch"]
        _, files = _queue(batch)
        if files != json.loads(pending["files"]):
            raise AdapterError("Archived offspring batch changed after its immutable birth.")
        with self.archive.db:
            spent = self.archive.db.execute(
                "SELECT coalesce(sum(allowance),0) FROM native_checks WHERE after_trial=?", (used,)).fetchone()[0]
            if spent >= maximum:
                raise DiscoveryLimit("Native discovery paused: load-only reconstruction limit reached for the next trial.")
            check = self.archive.db.execute(
                "INSERT INTO native_checks(after_trial,allowance,batch) VALUES (?,1,?)", (used, pending["batch"])).lastrowid
        root = self.archive.directory
        log = root / "preflight-workers" / f"{check:06d}.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        manifest = batch / pending["directory"] / "manifest.json"
        arguments = (batch, root / "preflight-isolated" / f"{pending['birth']:06d}",
                     pending["design"], manifest, self.library, self.options)
        receiver, sender = self.context.Pipe(duplex=False)
        process = self.context.Process(target=_run, args=(self.evaluator, arguments, sender, log), daemon=True)
        registered = False
        try:
            process.start()
            with self.archive.db:
                self.archive.db.execute("""INSERT INTO screening_workers(birth,pid,log,status)
                    VALUES (?,?,?,'running') ON CONFLICT(birth) DO UPDATE SET
                    pid=excluded.pid,log=excluded.log,status='running'""",
                    (pending["birth"], process.pid, str(log.relative_to(root))))
            record(self.archive, pending["birth"], "screening", "Independent load-only screening worker.")
            self.jobs.append(Job(pending["birth"], check, manifest, process, receiver, log))
            registered = True
        finally:
            sender.close()
            if not registered:
                if process.pid is not None:
                    process.terminate()
                    process.join()
                receiver.close()
                process.close()

    def reap(self):
        ready = []
        for job in list(self.jobs):
            while job.receiver.poll() or job.process.exitcode is not None:
                try:
                    message = job.receiver.recv()
                except (EOFError, OSError):
                    message = {"error": f"Screening worker exited without a result; see {job.log}."}
                if isinstance(message, dict) and "stage" in message:
                    if message["stage"] != "preflight":
                        raise AdapterError("Screening worker returned an invalid pipeline stage.")
                    record(self.archive, job.birth, "screened", "Independent load-only screening passed.")
                    record(self.archive, job.birth, message["stage"], "Independent final preflight worker.")
                    continue
                job.process.join()
                self.jobs.remove(job)
                job.receiver.close()
                code = job.process.exitcode
                job.process.close()
                result = message.get("result") if isinstance(message, dict) else None
                if code != 0 or not isinstance(result, dict):
                    error = message.get("error") if isinstance(message, dict) else None
                    self._failed(job, error or f"Screening worker exited with code {code}; see {job.log}.")
                if result.get("status") not in ("ready", "screening-rejected", "mass-layout-rejected",
                                                "evaluation-envelope-rejected"):
                    self._failed(job, "Screening worker returned an invalid outcome.")
                checks = result.get("checks")
                if type(checks) is not int or not 0 <= checks <= 1:
                    self._failed(job, "Screening worker exceeded its single reserved reconstruction check.")
                if result["status"] == "ready":
                    if not isinstance(getattr(result.get("checked"), "report", None), dict):
                        self._failed(job, "Screening worker omitted the checked model and compatibility report.")
                elif not isinstance(result.get("error"), str) or not result["error"]:
                    self._failed(job, "Rejected screening outcome omitted its diagnostic.")
                with self.archive.db:
                    self.archive.db.execute("UPDATE native_checks SET allowance=? WHERE id=?", (checks, job.check))
                    self.archive.db.execute("UPDATE screening_workers SET status='complete' WHERE birth=?", (job.birth,))
                if result.get("layout") is not None:
                    design = self.archive.db.execute("SELECT design FROM births WHERE id=?", (job.birth,)).fetchone()[0]
                    write_json(self.archive.directory / "designs" / design / "mass-layout.json", result["layout"])
                if result["status"] == "ready":
                    checked = result["checked"]
                    design = self.archive.db.execute("SELECT design FROM births WHERE id=?", (job.birth,)).fetchone()[0]
                    write_json(self.archive.directory / "designs" / design / "compatibility.json", checked.report)
                    record(self.archive, job.birth, "ready", "Preflight passed; awaiting a simulation slot.")
                    ready.append((job.birth, checked))
                else:
                    error = result["error"]
                    with self.archive.db:
                        self.archive.db.execute("UPDATE births SET status=?,error=? WHERE id=?",
                                                (result["status"], error, job.birth))
                    record(self.archive, job.birth, "rejected", error)
                    self.failures += result["status"] != "screening-rejected"
                    print(f"ERROR: Birth {job.birth}: {error}", file=sys.stderr, flush=True)
                break
        return ready

    def _failed(self, job, detail):
        with self.archive.db:
            self.archive.db.execute("UPDATE screening_workers SET status='failed' WHERE birth=?", (job.birth,))
        record(self.archive, job.birth, "paused", detail)
        raise AdapterError(detail)

    def wait(self, simulations):
        handles = [handle for job in self.jobs for handle in (job.receiver, job.process.sentinel)]
        handles += [handle for job in simulations.jobs for handle in (job.receiver, job.process.sentinel)]
        if handles:
            wait(handles, timeout=.2)

    def cancel(self, detail):
        jobs, self.jobs = self.jobs, []
        for job in jobs:
            if job.process.is_alive():
                job.process.terminate()
        for job in jobs:
            job.process.join()
            job.receiver.close()
            job.process.close()
            with self.archive.db:
                self.archive.db.execute("UPDATE screening_workers SET status='interrupted' WHERE birth=?", (job.birth,))
            record(self.archive, job.birth, "paused", detail)
