"""Bounded simulation processes; only the dispatcher writes trial bookkeeping."""

import contextlib
from dataclasses import dataclass
import multiprocessing
import os
from multiprocessing.connection import Connection, wait
from pathlib import Path
import sys
import traceback

from .bridge import AdapterError, write_json
from .simulation_gate import GateAttemptLimit, SimulationGateRejected


def ensure_attempt_schema(db):
    db.executescript("""
        CREATE TABLE IF NOT EXISTS simulation_gate_rounds(
            id INTEGER PRIMARY KEY, full_budget INTEGER NOT NULL,
            attempt_limit INTEGER NOT NULL, created TEXT NOT NULL,
            status TEXT NOT NULL DEFAULT 'running');
        CREATE TABLE IF NOT EXISTS simulation_attempts(
            id INTEGER PRIMARY KEY, birth INTEGER NOT NULL UNIQUE REFERENCES births(id),
            round INTEGER NOT NULL REFERENCES simulation_gate_rounds(id), slot INTEGER NOT NULL,
            trial INTEGER UNIQUE REFERENCES evaluations(id), status TEXT NOT NULL,
            pid INTEGER, log TEXT, error TEXT, started TEXT NOT NULL, completed TEXT);
    """)


def attempt_summary(db):
    if not db.execute("SELECT 1 FROM sqlite_master WHERE name='simulation_attempts'").fetchone():
        return None
    counts = dict(db.execute("SELECT status,COUNT(*) FROM simulation_attempts GROUP BY status"))
    latest = db.execute("SELECT id,attempt_limit,status FROM simulation_gate_rounds ORDER BY id DESC LIMIT 1").fetchone()
    return {"attempts": sum(counts.values()), "counts": counts,
            "maxAttemptsPerFullSimulation": latest[1] if latest else 3,
            "pausedAtLimit": bool(latest and latest[2] == "limit")}


def worker_count(value):
    if type(value) is not int or value < 1:
        raise AdapterError("Simulation worker count must be a positive integer.")
    return value


def _simulate(evaluator, checked, output, options, sender, log, replacement=False):
    with Path(log).open("w") as stream, contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
        try:
            try:
                def promote():
                    sender.send({"gatePassed": True})
                    approval = sender.recv()
                    if not isinstance(approval, dict) or type(approval.get("trial")) is not int:
                        raise AdapterError("Dispatcher did not authorize continuation into a full simulation.")

                result = (evaluator(checked, output, options, on_gate_passed=promote) if replacement
                          else evaluator(checked, output, options))
            except SimulationGateRejected as error:
                print(f"REJECTED: {error}", flush=True)
                sender.send({"rejection": str(error)})
            except (AdapterError, OSError) as error:
                traceback.print_exc()
                sender.send({"error": str(error)})
            else:
                sender.send({"result": result})
        finally:
            if sys.exc_info()[0] is not None:
                traceback.print_exc()
            sender.close()


@dataclass
class Job:
    trial: int | None
    birth: int
    process: multiprocessing.Process | None
    receiver: Connection | None
    log: Path
    attempt: int | None = None


class TrialPool:
    def __init__(self, archive, workers, evaluator, blender=None, renderer=None):
        self.archive = archive
        self.workers = worker_count(workers)
        self.evaluator = evaluator
        self.blender = blender
        self.renderer = renderer
        self.context = multiprocessing.get_context("spawn")
        self.jobs = []
        self.failures = 0
        self.rejections = 0
        policy = archive.protocol().get("execution", {}).get("simulationGateAccounting")
        self.replacement = policy is not None
        self.round = None
        if self.replacement:
            if policy.get("version") != "prefix-before-trial-v1":
                raise AdapterError("Unsupported simulation gate accounting policy.")
            self.attempt_limit = policy["maxAttemptsPerFullSimulation"]
            if type(self.attempt_limit) is not int or self.attempt_limit < 1:
                raise AdapterError("Cheap gate attempt cap must be a positive integer.")
            ensure_attempt_schema(archive.db)
            from .evolution import now
            with archive.db:
                budget = archive.db.execute("SELECT budget FROM run").fetchone()[0]
                self.round = archive.db.execute(
                    "INSERT INTO simulation_gate_rounds(full_budget,attempt_limit,created) VALUES (?,?,?)",
                    (budget, self.attempt_limit, now())).lastrowid

    def _slots(self):
        budget = self.archive.db.execute("SELECT full_budget FROM simulation_gate_rounds WHERE id=?",
                                         (self.round,)).fetchone()[0]
        occupied = {row[0] for row in self.archive.db.execute("""
            SELECT slot FROM simulation_attempts WHERE trial IS NOT NULL OR status='running'
        """)}
        occupied.update(row[0] for row in self.archive.db.execute("""
            SELECT e.id FROM evaluations e WHERE NOT EXISTS(
                SELECT 1 FROM simulation_attempts a WHERE a.trial=e.id)
        """))
        counts = dict(self.archive.db.execute(
            "SELECT slot,COUNT(*) FROM simulation_attempts WHERE round=? GROUP BY slot", (self.round,)))
        return [slot for slot in range(1, budget + 1)
                if slot not in occupied and counts.get(slot, 0) < self.attempt_limit]

    @property
    def pending_gates(self):
        return sum(job.trial is None for job in self.jobs)

    @property
    def generation_step(self):
        used = sum(self.archive.counts().values())
        return used + (self.archive.db.execute("SELECT COUNT(*) FROM simulation_attempts").fetchone()[0]
                       if self.replacement else 0)

    @property
    def exhausted(self):
        if not self.replacement or self.pending_gates:
            return False
        used = sum(self.archive.counts().values())
        budget = self.archive.db.execute("SELECT budget FROM run").fetchone()[0]
        return used < budget and not self._slots()

    @property
    def full(self):
        return len(self.jobs) >= self.workers or (self.replacement and not self._slots())

    def __enter__(self):
        return self

    def __exit__(self, kind, error, trace):
        if kind is None:
            try:
                self.drain()
            finally:
                if self.jobs:
                    self.cancel("Dispatcher failed while collecting simulation results.")
            if self.replacement:
                with self.archive.db:
                    self.archive.db.execute("UPDATE simulation_gate_rounds SET status=? WHERE id=?",
                                            ("limit" if self.exhausted else "complete", self.round))
        else:
            self.cancel(f"Dispatcher interrupted: {error or kind.__name__}.")
            if self.replacement:
                with self.archive.db:
                    self.archive.db.execute("UPDATE simulation_gate_rounds SET status='interrupted' WHERE id=?", (self.round,))

    def _finish(self, trial, birth, result=None, error=None, status="failed", *, attempt=None):
        from .gallery import publish
        from .evolution import render_representative

        from .evolution import now
        output = self.archive.directory / ("simulation-attempts" if attempt is not None else "evaluations") / f"{attempt if attempt is not None else trial:04d}"
        if trial is None and error is None:
            error = "Simulation returned without passing its gate and reserving a full-simulation slot."
        if error is not None:
            name = "rejection.json" if status == "rejected" else "error.json"
            write_json(output / name, {"status": status, "error": error})
            if status == "rejected":
                self.rejections += 1
                print(f"REJECTED: Simulation attempt {attempt or trial}: {error}", flush=True)
            else:
                self.failures += 1
                print(f"ERROR: Simulation attempt {attempt or trial}: {error}", file=sys.stderr, flush=True)
        if attempt is not None:
            if trial is not None:
                target = self.archive.directory / "evaluations" / f"{trial:04d}"
                if output.exists():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    output.rename(target)
                log = output.with_suffix(".log")
                if log.exists():
                    destination = target.with_suffix(".log")
                    log.rename(destination)
                    with self.archive.db:
                        self.archive.db.execute("UPDATE trial_workers SET log=? WHERE trial=?",
                                                (str(destination.relative_to(self.archive.directory)), trial))
                        self.archive.db.execute("UPDATE simulation_attempts SET log=? WHERE id=?",
                                                (str(destination.relative_to(self.archive.directory)), attempt))
        if trial is not None:
            self.archive.finish(trial, result if error is None else None, error=error,
                                status="complete" if error is None else status)
        if attempt is not None:
            with self.archive.db:
                self.archive.db.execute("UPDATE simulation_attempts SET status=?,error=?,completed=? WHERE id=?",
                                        ("complete" if error is None else status, error, now(), attempt))
                if trial is None:
                    self.archive.db.execute("UPDATE births SET status=?,error=? WHERE id=?",
                                            ("simulation-" + status, error, birth))
        publish(self.archive)
        if self.blender is not None and error is None:
            category = self.archive.db.execute(
                "SELECT d.category FROM births b JOIN designs d ON d.id=b.design WHERE b.id=?", (birth,)
            ).fetchone()[0]
            representative = self.archive.best(category)
            if representative is not None:
                (self.renderer or render_representative)(self.archive, representative, self.blender)
                publish(self.archive)

    def _promote(self, job):
        from .evolution import now
        from .gallery import publish
        if job.trial is not None:
            raise AdapterError("Simulation attempted to reserve its full trial more than once.")
        job.trial = self.archive.reserve(job.birth, attempt=job.attempt)
        with self.archive.db:
            if job.process is not None:
                self.archive.db.execute("INSERT INTO trial_workers(trial,pid,log) VALUES (?,?,?)",
                                        (job.trial, job.process.pid, str(job.log.relative_to(self.archive.directory))))
        print(f"Full trial {job.trial}: attempt {job.attempt} passed its cheap gate.", flush=True)
        publish(self.archive)

    def submit(self, birth, checked, options):
        from .gallery import publish
        from .motion_carry import require_construction

        require_construction(self.archive, birth, checked)
        if self.exhausted:
            raise GateAttemptLimit("Cheap simulation gate attempt cap reached; unused full-simulation allowance is retained.")
        if self.full:
            raise AdapterError("Simulation pool is full; dispatch must wait for a completion.")
        trial, attempt = None, None
        if self.replacement:
            from .evolution import now
            if options.short_gate <= 0:
                raise AdapterError("Replacement accounting requires an enabled short simulation gate.")
            with self.archive.db:
                self.archive.db.execute("BEGIN IMMEDIATE")
                slots = self._slots()
                if not slots:
                    raise GateAttemptLimit("Cheap simulation gate attempt cap reached; unused full-simulation allowance is retained.")
                attempt = self.archive.db.execute("""
                    INSERT INTO simulation_attempts(birth,round,slot,status,started,pid) VALUES (?,?,?,'running',?,?)
                """, (birth, self.round, slots[0], now(), os.getpid() if self.workers == 1 else None)).lastrowid
                self.archive.db.execute("UPDATE births SET status='gating' WHERE id=?", (birth,))
            output = self.archive.directory / "simulation-attempts" / f"{attempt:04d}"
        else:
            trial = self.archive.reserve(birth)
            output = self.archive.directory / "evaluations" / f"{trial:04d}"
        publish(self.archive)
        print(f"Simulation {'attempt ' + str(attempt) if attempt else 'trial ' + str(trial)}: birth {birth}; workers {self.workers}.", flush=True)
        job = Job(trial, birth, None, None, output.with_suffix(".log"), attempt)
        if self.workers == 1:
            try:
                result = (self.evaluator(checked, output, options, on_gate_passed=lambda: self._promote(job))
                          if self.replacement else self.evaluator(checked, output, options))
                if not isinstance(result, dict):
                    raise AdapterError("Simulation did not return a result object.")
            except SimulationGateRejected as error:
                self._finish(job.trial, birth, error=str(error), status="rejected", attempt=attempt)
            except (AdapterError, OSError) as error:
                self._finish(job.trial, birth, error=str(error), attempt=attempt)
            else:
                self._finish(job.trial, birth, result=result, attempt=attempt)
            finally:
                if job.trial is not None:
                    status = self.archive.db.execute("SELECT status FROM evaluations WHERE id=?", (job.trial,)).fetchone()[0]
                else:
                    status = self.archive.db.execute("SELECT status FROM simulation_attempts WHERE id=?", (attempt,)).fetchone()[0]
                if status in ("running", "promoted"):
                    self._finish(job.trial, birth, error="Simulation interrupted before returning a result.",
                                 status="interrupted", attempt=attempt)
            return job.trial
        receiver, sender = self.context.Pipe(duplex=True)
        log = output.with_suffix(".log")
        log.parent.mkdir(parents=True, exist_ok=True)
        process = self.context.Process(target=_simulate,
                                       args=(self.evaluator, checked, output, options, sender, log, self.replacement),
                                       daemon=True)
        started = False
        registered = False
        try:
            process.start()
            started = True
            with self.archive.db:
                if attempt is None:
                    self.archive.db.execute("INSERT INTO trial_workers(trial,pid,log) VALUES (?,?,?)",
                                            (trial, process.pid, str(log.relative_to(self.archive.directory))))
                else:
                    self.archive.db.execute("UPDATE simulation_attempts SET pid=?,log=? WHERE id=?",
                                            (process.pid, str(log.relative_to(self.archive.directory)), attempt))
            self.jobs.append(Job(trial, birth, process, receiver, log, attempt))
            registered = True
        finally:
            sender.close()
            if not registered:
                if started:
                    process.terminate()
                    process.join()
                receiver.close()
                self._finish(trial, birth, error="Simulation process could not be dispatched; see episode log.", attempt=attempt)
                process.close()
        return trial

    def reap(self, *, block=False):
        if not self.jobs:
            return 0
        wait([handle for job in self.jobs for handle in (job.receiver, job.process.sentinel)],
             timeout=None if block else 0)
        completed = 0
        for job in list(self.jobs):
            if not job.receiver.poll() and job.process.exitcode is None:
                continue
            try:
                message = job.receiver.recv()
            except (EOFError, OSError):
                message = {}
            if isinstance(message, dict) and message.get("gatePassed") is True:
                if job.attempt is None:
                    raise AdapterError("A legacy trial sent an unexpected gate handoff.")
                self._promote(job)
                job.receiver.send({"trial": job.trial})
                continue
            job.process.join()
            self.jobs.remove(job)
            job.receiver.close()
            if (job.process.exitcode == 0 and isinstance(message, dict)
                    and isinstance(message.get("result"), dict)):
                self._finish(job.trial, job.birth, result=message["result"], attempt=job.attempt)
            elif (job.process.exitcode == 0 and isinstance(message, dict)
                  and isinstance(message.get("rejection"), str)):
                self._finish(job.trial, job.birth, error=message["rejection"], status="rejected", attempt=job.attempt)
            else:
                error = message.get("error") if isinstance(message, dict) else None
                error = error or f"Simulation process exited with code {job.process.exitcode}; see {job.log.name}."
                log = (self.archive.directory / "evaluations" / f"{job.trial:04d}.log"
                       if job.attempt is not None and job.trial is not None else job.log)
                self._finish(job.trial, job.birth, error=error + f" Log: {log.relative_to(self.archive.directory)}",
                             attempt=job.attempt)
            job.process.close()
            completed += 1
        return completed

    def drain(self):
        while self.jobs:
            self.reap(block=True)

    def cancel(self, message):
        self.blender = None
        try:
            self.reap()
        finally:
            remaining, self.jobs = self.jobs, []
            for job in remaining:
                if job.process.is_alive():
                    job.process.terminate()
            for job in remaining:
                job.process.join()
                job.receiver.close()
                job.process.close()
            for job in remaining:
                accounting = "Trial remains spent." if job.trial is not None else "No full-simulation slot was consumed."
                self._finish(job.trial, job.birth, error=f"{message} {accounting}",
                             status="interrupted", attempt=job.attempt)
