"""Budgeted, resumable evolutionary trials; F# owns every genome proposal."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import importlib.metadata
import json
import os
import platform
import shutil
import sqlite3
import subprocess
import sys
import uuid
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from .bridge import AdapterError, SIMULATOR_REVISION, check, library_fingerprint, load_manifest, logical_foot_count, write_json
from .evaluation import EvaluationOptions, evaluate
from .payload import load_profile
from .schema import MIN_WALKER_FEET, MAX_FEET

ROOT = Path(__file__).resolve().parents[2]
GRAMMAR = "technic-fourbar-v5"
LEGACY_CATEGORIES = [(pairs, remote, towing) for pairs in (1, 2, 3)
                     for remote in (False, True) for towing in (False, True)]
CATEGORIES = [(pairs, True, False) for pairs in range(1, MAX_FEET//2+1)]
SEARCH_POLICY = {"categoryDescriptor": "logical-foot-count", "minimumFeet": MIN_WALKER_FEET, "maximumFeet": MAX_FEET,
                 "power": "remote", "towing": False, "lineage": "cross-category"}


def categories_for(protocol):
    if "categories" not in protocol:
        return [(pairs, remote, None) for remote in (False, True) for pairs in (1, 2, 3)]
    return [(item["pairs"], item["remote"], item["towWagon"]) for item in protocol["categories"]]

def category_descriptors(protocol):
    if protocol.get("grammarVersion") == "technic-motion-v3":
        return [(f"{item['footCount']}-feet", item["footCount"], True, False)
                for item in protocol["categories"]]
    return [(category(category_genome(pairs, remote, towing), foot_only=foot_count_categories(protocol)),
             pairs*2, remote, towing) for pairs, remote, towing in categories_for(protocol)]


def category_genome(pairs, remote, towing):
    return {"pairs": pairs, "remote": remote, **({"towWagon": towing} if towing is not None else {})}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def identity(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def design_identity(genome):
    genes = {key: value for key, value in genome.items() if key != "seed"}
    return identity({"grammarVersion": GRAMMAR, "genome": genes})


def category(genome, *, foot_only=False):
    if foot_only:
        return f"{genome['pairs'] * 2}-feet"
    name = f"four-bar-{genome['pairs'] * 2}-{'remote' if genome['remote'] else 'wired'}"
    return name + ("-towing" if genome["towWagon"] else "-solo") if "towWagon" in genome else name


def foot_count_categories(protocol):
    return protocol.get("searchPolicy", {}).get("categoryDescriptor") == "logical-foot-count"


def validate_search_candidate(checked, expected_feet):
    count = validate_search_manifest(checked.manifest, expected_feet)
    checked.report["logicalFootCount"] = count


def validate_search_manifest(manifest, expected_feet):
    count = logical_foot_count(manifest)
    genome = manifest["genome"]
    if (not MIN_WALKER_FEET <= count <= MAX_FEET or count != expected_feet or genome.get("remote") is not True
            or genome.get("towWagon") is not False or manifest.get("groundWheels")):
        raise AdapterError("Realized design violates the remote/solo inferred-foot-count search policy.")
    return count


def now():
    return datetime.now(timezone.utc).isoformat()


def evidence_invalidation(protocol):
    if protocol.get("simulatorRevision") == "7db4596fe7f970fadc7b53893be32b4b93249d9e":
        return ("Invalidated mass-property evidence: reflected LDraw winding could produce invalid "
                "centres of mass and enormous rotational inertia. Corrected simulation is required.")
    return None


def trusted_result(result, protocol):
    reason = evidence_invalidation(protocol)
    if reason is None:
        return result
    metrics = result["metrics"]
    return {**result, "evidenceInvalidation": reason,
            "metrics": {**metrics, "eligibleForRanking": False,
                        "rankingRejectionReasons": list(dict.fromkeys([
                            *metrics.get("rankingRejectionReasons", []), "invalidated-mass-properties"]))}}


def is_qualified(result):
    if result.get("evidenceInvalidation"):
        return False
    ground_ok = "groundContact" not in result or not result["groundContact"]["forbiddenContactDetected"]
    metrics = result["metrics"]
    feet_ok = ("logicalFootCount" not in metrics
               or (type(metrics["logicalFootCount"]) is int
                   and MIN_WALKER_FEET <= metrics["logicalFootCount"] <= MAX_FEET))
    gait = result.get("gait")
    cycles_ok = gait is None or (gait["completeCycles"] >= gait["requiredCycles"]
                                and gait["repeatableForwardMotion"] is True)
    return result["status"] == "evaluated" and metrics["eligibleForRanking"] and ground_ok and feet_ok and cycles_ok


def fitness(result):
    if result.get("evidenceInvalidation"):
        return (-1,)
    m = result["metrics"]
    if is_qualified(result):
        return (1, m["speedMetersPerSecond"], -m["absoluteMechanicalWorkJoules"])
    return (0, result["status"] != "interference", m["closureWithinTolerance"],
            not m["fallen"], "groundContact" not in result or not result["groundContact"]["forbiddenContactDetected"],
            result["collision"]["fullMotorCycle"],
            result.get("gait", {}).get("cycleCoverage", min(abs(turns) for turns in m["motorTurns"])), m["speedMetersPerSecond"],
            -result["collision"]["maxSelfPenetrationMeters"])


class Archive:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.directory / "archive.sqlite3")
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA foreign_keys=ON")
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS protocols(
                id TEXT PRIMARY KEY, definition TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS run(
                id INTEGER PRIMARY KEY CHECK(id=1), protocol TEXT NOT NULL REFERENCES protocols(id),
                budget INTEGER NOT NULL, seed INTEGER NOT NULL, status TEXT NOT NULL,
                created TEXT NOT NULL, updated TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS designs(
                id TEXT PRIMARY KEY, category TEXT NOT NULL, genome TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS births(
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                design TEXT NOT NULL REFERENCES designs(id),
                parent INTEGER REFERENCES births(id), operation TEXT NOT NULL,
                changes TEXT NOT NULL, seed INTEGER NOT NULL,
                status TEXT NOT NULL, error TEXT, created TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS evaluations(
                id INTEGER PRIMARY KEY AUTOINCREMENT, birth INTEGER NOT NULL UNIQUE REFERENCES births(id),
                protocol TEXT NOT NULL REFERENCES protocols(id), status TEXT NOT NULL,
                result TEXT, error TEXT, started TEXT NOT NULL, completed TEXT);
            CREATE TABLE IF NOT EXISTS trial_workers(
                trial INTEGER PRIMARY KEY REFERENCES evaluations(id), pid INTEGER NOT NULL, log TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS pipeline_events(
                id INTEGER PRIMARY KEY, birth INTEGER NOT NULL REFERENCES births(id),
                stage TEXT NOT NULL, detail TEXT NOT NULL, created TEXT NOT NULL);
            CREATE INDEX IF NOT EXISTS pipeline_birth ON pipeline_events(birth,id);
            CREATE TABLE IF NOT EXISTS screening_workers(
                birth INTEGER PRIMARY KEY REFERENCES births(id),pid INTEGER NOT NULL,log TEXT NOT NULL,status TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS media_errors(
                design TEXT PRIMARY KEY REFERENCES designs(id), error TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS native_proposals(
                id INTEGER PRIMARY KEY AUTOINCREMENT, parent INTEGER NOT NULL REFERENCES births(id),
                seed INTEGER NOT NULL, attempt INTEGER NOT NULL, after_trial INTEGER NOT NULL,
                status TEXT NOT NULL, summary TEXT, error TEXT, created TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS native_assemblies(
                assembly TEXT PRIMARY KEY, birth INTEGER NOT NULL REFERENCES births(id));
            CREATE TABLE IF NOT EXISTS native_actuations(
                assembly TEXT NOT NULL, controller TEXT NOT NULL, birth INTEGER NOT NULL REFERENCES births(id),
                PRIMARY KEY(assembly,controller));
            CREATE TABLE IF NOT EXISTS native_candidates(
                birth INTEGER PRIMARY KEY REFERENCES births(id), batch TEXT NOT NULL,
                directory TEXT NOT NULL, files TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS native_checks(
                id INTEGER PRIMARY KEY AUTOINCREMENT, after_trial INTEGER NOT NULL,
                allowance INTEGER NOT NULL, batch TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS reused_evaluations(
                birth INTEGER PRIMARY KEY REFERENCES births(id), source_archive TEXT NOT NULL,
                source_trial INTEGER NOT NULL, source_protocol TEXT NOT NULL, signature TEXT NOT NULL,
                result TEXT NOT NULL, completed TEXT);
            CREATE TRIGGER IF NOT EXISTS immutable_reused_evaluation BEFORE UPDATE ON reused_evaluations
            BEGIN SELECT RAISE(ABORT, 'Reused observation provenance is immutable'); END;
            CREATE TRIGGER IF NOT EXISTS chronological_parent AFTER INSERT ON births
            WHEN NEW.parent IS NOT NULL AND NEW.parent >= NEW.id
            BEGIN SELECT RAISE(ABORT, 'Birth ancestry must point backwards'); END;
            CREATE TRIGGER IF NOT EXISTS immutable_birth
            BEFORE UPDATE OF design,parent,operation,changes,seed,created ON births
            BEGIN SELECT RAISE(ABORT, 'Birth provenance is immutable'); END;
            CREATE TRIGGER IF NOT EXISTS immutable_design BEFORE UPDATE ON designs
            BEGIN SELECT RAISE(ABORT, 'Design identity is immutable'); END;
            CREATE TRIGGER IF NOT EXISTS immutable_protocol BEFORE UPDATE ON protocols
            BEGIN SELECT RAISE(ABORT, 'Evaluation protocol is immutable'); END;
            CREATE TRIGGER IF NOT EXISTS immutable_native_candidate BEFORE UPDATE ON native_candidates
            BEGIN SELECT RAISE(ABORT, 'Native candidate inputs are immutable'); END;
            CREATE TRIGGER IF NOT EXISTS immutable_native_assembly BEFORE UPDATE ON native_assemblies
            BEGIN SELECT RAISE(ABORT, 'Native physical identity is immutable'); END;
            CREATE TRIGGER IF NOT EXISTS immutable_native_actuation BEFORE UPDATE ON native_actuations
            BEGIN SELECT RAISE(ABORT, 'Native controller identity is immutable'); END;
        """)
        with self.db:
            for row in self.db.execute("""
                SELECT a.assembly,a.birth,d.genome FROM native_assemblies a
                JOIN births b ON b.id=a.birth JOIN designs d ON d.id=b.design
            """).fetchall():
                self.db.execute("INSERT OR IGNORE INTO native_actuations VALUES (?,?,?)",
                                (row["assembly"], self._native_controller(json.loads(row["genome"])), row["birth"]))

    @staticmethod
    def _native_controller(genome):
        return canonical({"polarity": 1 if genome["speed"] > 0 else -1, "maxTorque": genome["maxTorque"]})

    def initialize(self, protocol, budget, seed, resume):
        identifier = identity(protocol)
        run = self.db.execute("SELECT * FROM run").fetchone()
        if run is not None:
            if not resume:
                raise AdapterError("Archive already exists; use --resume or a different output directory.")
            if (run["protocol"], run["budget"], run["seed"]) != (identifier, budget, seed):
                raise AdapterError("Resume protocol/budget/seed differs. Use a new archive for a different experiment.")
        else:
            if resume:
                raise AdapterError("Cannot resume an archive with no initialized run.")
            with self.db:
                self.db.execute("INSERT INTO protocols VALUES (?,?)", (identifier, canonical(protocol)))
                self.db.execute("INSERT INTO run VALUES (1,?,?,?,?,?,?)",
                                (identifier, budget, seed, "ready", now(), now()))
            write_json(self.directory / "protocol.json", {"id": identifier, **protocol})
        if protocol.get("execution", {}).get("simulationGateAccounting"):
            from .parallel import ensure_attempt_schema
            ensure_attempt_schema(self.db)
        return identifier

    def outcomes(self):
        if evidence_invalidation(self.protocol()):
            return []
        rows = [dict(row) for row in self.db.execute("""
            SELECT e.*, b.design, b.parent, b.operation, d.category, d.genome
            FROM evaluations e JOIN births b ON b.id=e.birth JOIN designs d ON d.id=b.design
            JOIN run r ON r.protocol=e.protocol WHERE e.status='complete' ORDER BY e.id
        """)]
        rows.extend(dict(row) for row in self.db.execute("""
            SELECT 0 AS id,r.birth,r.source_protocol AS protocol,'complete' AS status,r.result,
                   NULL AS error,NULL AS started,r.completed,b.design,b.parent,b.operation,d.category,d.genome,1 AS reused
            FROM reused_evaluations r JOIN births b ON b.id=r.birth JOIN designs d ON d.id=b.design
            ORDER BY r.birth
        """))
        return rows

    def founder(self, birth):
        while True:
            row = self.db.execute("SELECT id,parent FROM births WHERE id=?", (birth,)).fetchone()
            if row is None:
                raise AdapterError("Unknown birth while resolving its founder neighborhood.")
            if row["parent"] is None:
                return row["id"]
            birth = row["parent"]

    def protocol(self):
        row = self.db.execute("SELECT p.definition FROM protocols p JOIN run r ON r.protocol=p.id").fetchone()
        if row is None:
            raise AdapterError("Archive has no initialized protocol.")
        return json.loads(row[0])

    def best(self, kind=None):
        rows = [row for row in self.outcomes() if kind is None or row["category"] == kind]
        return max(rows, key=lambda row: fitness(json.loads(row["result"]))) if rows else None

    def counts(self):
        return {row["status"]: row["n"] for row in self.db.execute(
            "SELECT status, count(*) n FROM evaluations GROUP BY status")}

    def record_birth(self, genome, parent, operation, changed, status, error=None):
        foot_only = foot_count_categories(self.protocol())
        if foot_only and (genome["remote"] is not True or genome.get("towWagon", False)
                          or type(genome["pairs"]) is not int or not 1 <= genome["pairs"] <= MAX_FEET//2):
            raise AdapterError("Active search requires remote, non-towing designs with two to six logical feet.")
        identifier = design_identity(genome)
        return self._record_birth(genome, identifier, category(genome, foot_only=foot_only),
                                  parent, operation, changed, genome["seed"], status, error)

    def record_native_birth(self, manifest, *, status="generated", error=None, parent=None,
                            operation="import-motion-walker", changed=None, seed=None, native=None):
        if (self.protocol().get("grammarVersion") != "technic-motion-v3"
                or manifest["grammarVersion"] != "technic-motion-v3"):
            raise AdapterError("Native walker births require an isolated motion-v3 protocol.")
        count = logical_foot_count(manifest)
        validate_search_manifest(manifest, count)
        if seed is None:
            seed = self.db.execute("SELECT seed FROM run WHERE id=1").fetchone()[0]
        return self._record_birth(manifest["genome"], manifest["motion"]["genotypeId"], f"{count}-feet",
                                  parent, operation, changed or [], seed, status, error, native=native)

    def _record_birth(self, genome, identifier, kind, parent, operation, changed, seed, status, error, *, native=None):
        with self.db:
            if native is not None:
                controller = self._native_controller(genome)
                prior = self.db.execute("SELECT birth FROM native_actuations WHERE assembly=? AND controller=?",
                                        (native["assembly"], controller)).fetchone()
                if prior is not None:
                    status, error = "duplicate", f"Physical assembly and controller already recorded at birth {prior['birth']}; no trial spent."
            self.db.execute("INSERT OR IGNORE INTO designs VALUES (?,?,?)",
                            (identifier, kind, canonical(genome)))
            cursor = self.db.execute("""
                INSERT INTO births(design,parent,operation,changes,seed,status,error,created)
                VALUES (?,?,?,?,?,?,?,?)
            """, (identifier, parent, operation, canonical(changed), seed, status, error, now()))
            if native is not None:
                birth = cursor.lastrowid
                self.db.execute("INSERT OR IGNORE INTO native_assemblies VALUES (?,?)", (native["assembly"], birth))
                self.db.execute("INSERT OR IGNORE INTO native_actuations VALUES (?,?,?)",
                                (native["assembly"], controller, birth))
                self.db.execute("INSERT INTO native_candidates VALUES (?,?,?,?)",
                                (birth, native["batch"], native["directory"], canonical(native["files"])))
        return cursor.lastrowid, identifier

    def reserve(self, birth, *, attempt=None):
        with self.db:
            self.db.execute("BEGIN IMMEDIATE")
            run = self.db.execute("SELECT * FROM run").fetchone()
            used = self.db.execute("SELECT count(*) FROM evaluations").fetchone()[0]
            if used >= run["budget"]:
                raise AdapterError("Evaluation budget exhausted; refusing another simulation.")
            cursor = self.db.execute("""
                INSERT INTO evaluations(birth,protocol,status,started) VALUES (?,?,'running',?)
            """, (birth, run["protocol"], now()))
            if attempt is not None:
                updated = self.db.execute("""
                    UPDATE simulation_attempts SET trial=?,status='promoted'
                    WHERE id=? AND birth=? AND status='running' AND trial IS NULL
                """, (cursor.lastrowid, attempt, birth))
                if updated.rowcount != 1:
                    raise AdapterError("Only a running cheap attempt may reserve a full-simulation slot.")
                self.db.execute("UPDATE births SET status='generated' WHERE id=?", (birth,))
        return cursor.lastrowid

    def finish(self, evaluation, result=None, error=None, status="complete"):
        with self.db:
            cursor = self.db.execute("""
                UPDATE evaluations SET status=?,result=?,error=?,completed=? WHERE id=? AND status='running'
            """, (status, canonical(result) if result is not None else None, error, now(), evaluation))
            if cursor.rowcount != 1:
                raise AdapterError("Only a running trial can be finalized; existing outcomes are immutable.")

    def recover(self):
        attempts = self.db.execute("SELECT 1 FROM sqlite_master WHERE name='simulation_attempts'").fetchone()
        if attempts:
            for attempt in self.db.execute(
                    "SELECT * FROM simulation_attempts WHERE status='running' AND trial IS NULL").fetchall():
                if attempt["pid"] is not None:
                    try:
                        os.kill(attempt["pid"], 0)
                    except ProcessLookupError:
                        pass
                    else:
                        raise AdapterError(f"Cheap simulation attempt {attempt['id']} is still running; wait before resuming.")
                message = "Cheap simulation attempt interrupted; no full-simulation slot was consumed."
                with self.db:
                    self.db.execute("UPDATE simulation_attempts SET status='interrupted',error=?,completed=? WHERE id=?",
                                    (message, now(), attempt["id"]))
                    self.db.execute("UPDATE births SET status='simulation-interrupted',error=? WHERE id=?",
                                    (message, attempt["birth"]))
                write_json(self.directory / "simulation-attempts" / f"{attempt['id']:04d}" / "error.json",
                           {"status": "interrupted", "error": message})
        for row in self.db.execute("SELECT id,birth FROM evaluations WHERE status='running'").fetchall():
            child = self.db.execute("SELECT pid FROM trial_workers WHERE trial=?", (row["id"],)).fetchone()
            if child is None and attempts:
                child = self.db.execute("SELECT pid FROM simulation_attempts WHERE trial=?", (row["id"],)).fetchone()
            if child is not None and child["pid"] is not None:
                try:
                    os.kill(child["pid"], 0)
                except ProcessLookupError:
                    pass
                else:
                    raise AdapterError(f"Simulation process {child['pid']} for trial {row['id']} is still running; "
                                       "wait for it to exit before resuming the dispatcher.")
            output = self.directory / "evaluations" / f"{row['id']:04d}"
            if attempts:
                attempt = self.db.execute("SELECT id FROM simulation_attempts WHERE trial=?", (row["id"],)).fetchone()
                if attempt:
                    source = self.directory / "simulation-attempts" / f"{attempt['id']:04d}"
                    if source.exists():
                        output.parent.mkdir(parents=True, exist_ok=True)
                        source.rename(output)
                    if source.with_suffix(".log").exists():
                        source.with_suffix(".log").rename(output.with_suffix(".log"))
                        with self.db:
                            log = str(output.with_suffix(".log").relative_to(self.directory))
                            self.db.execute("UPDATE simulation_attempts SET log=? WHERE id=?", (log, attempt["id"]))
                            self.db.execute("UPDATE trial_workers SET log=? WHERE trial=?", (log, row["id"]))
            result_file = output / "result.json"
            if result_file.exists():
                try:
                    result = json.loads(result_file.read_text())
                except json.JSONDecodeError as error:
                    message = f"Interrupted trial left incomplete result JSON: {error}"
                    self.finish(row["id"], error=message, status="interrupted")
                    result_file.rename(output / "result-partial.json")
                    write_json(output / "error.json", {"status": "interrupted", "error": message})
                    print(f"ERROR: {message}; trial remains spent.", file=sys.stderr)
                    continue
                birth = self.db.execute("""
                    SELECT b.design,d.genome FROM births b JOIN designs d ON d.id=b.design WHERE b.id=?
                """, (row["birth"],)).fetchone()
                manifest = json.loads((self.directory / "designs" / birth["design"] / "candidate" / "manifest.json").read_text())
                if result["compatibility"]["ldrawSha256"] != manifest["ldrawSha256"]:
                    raise AdapterError("Interrupted evaluation has a mismatched result checksum.")
                self.finish(row["id"], result)
            else:
                error = "Interrupted after budget reservation; trial remains spent and is not simulated twice."
                self.finish(row["id"], error=error, status="interrupted")
                write_json(output / "error.json", {"status": "interrupted", "error": error})
        if attempts:
            with self.db:
                self.db.execute("""
                    UPDATE simulation_attempts SET status=(SELECT status FROM evaluations WHERE id=trial),
                    completed=(SELECT completed FROM evaluations WHERE id=trial),
                    error=(SELECT error FROM evaluations WHERE id=trial)
                    WHERE trial IS NOT NULL AND status='promoted'
                """)
                self.db.execute("""
                    UPDATE simulation_gate_rounds SET status='interrupted' WHERE status='running'
                    AND NOT EXISTS(SELECT 1 FROM simulation_attempts
                                   WHERE round=simulation_gate_rounds.id AND status IN ('running','promoted'))
                """)

    def set_status(self, status):
        with self.db:
            self.db.execute("UPDATE run SET status=?,updated=? WHERE id=1", (status, now()))


@contextlib.contextmanager
def locked(directory):
    import fcntl

    directory.mkdir(parents=True, exist_ok=True)
    with (directory / ".runner.lock").open("a") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise AdapterError("Another evolution runner owns this archive.") from error
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


class Generator:
    def __init__(self, dotnet, dll):
        self.command = [str(dotnet), str(dll)]

    def invoke(self, *arguments):
        result = subprocess.run([*self.command, *map(str, arguments)], capture_output=True, text=True)
        if result.returncode:
            raise AdapterError(f"F# generator failed ({result.returncode}): {result.stderr.strip()}")
        return result.stdout

    def manual_edits(self, parent):
        value = json.loads(self.invoke("walker-edits", "--parent", parent))
        if value.get("dynamicTrials") != 0 or not isinstance(value.get("edits"), list):
            raise AdapterError("Native edit catalog is invalid.")
        return value

    def apply_edit(self, parent, output, edit, limit=2):
        if limit not in (1, 2):
            raise AdapterError("Manual edits allow one or two physical realizations.")
        result = subprocess.run([*self.command, "propose-walker", "--parent", str(parent),
                                 "--output", str(output), "--seed", "0", "--edit", edit, "--limit", str(limit)],
                                capture_output=True, text=True, timeout=180)
        path = output / "candidates.json"
        if result.returncode not in (0, 1) or not path.is_file():
            raise AdapterError(f"Native manual edit failed ({result.returncode}): {result.stderr.strip()}")
        summary = json.loads(path.read_text())
        manifest = load_manifest(parent / "manifest.json")
        if (summary.get("selectedEdit") != edit or summary.get("counts", {}).get("dynamicTrials") != 0
                or summary.get("parentGenotypeId") != manifest["motion"]["genotypeId"]
                or not isinstance(summary.get("generated"), list)
                or len(summary["generated"]) > limit or bool(summary["generated"]) != (result.returncode == 0)):
            raise AdapterError("Manual edit returned inconsistent provenance or results.")
        summary["manualRuntime"] = {
            "generator": str(Path(self.command[1]).resolve()),
            "binaries": {name: hashlib.sha256(Path(self.command[1]).with_name(name).read_bytes()).hexdigest()
                         for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll")}}
        return summary

    def propose(self, pairs, remote, towing, seed, attempt, parent):
        args = ["propose", "--pairs", pairs, "--variant", "remote" if remote else "wired",
                "--towing", "yes" if towing else "no",
                "--seed", seed, "--attempt", attempt]
        if parent is not None:
            args += ["--parent", parent]
        return json.loads(self.invoke(*args))

    def propose_native(self, parent, output, seed, attempt, limit):
        from .discovery import POLICY
        command = ["propose-walker", "--parent", parent, "--output", output, "--seed", seed,
                   "--attempt", attempt, "--limit", limit]
        result = subprocess.run([*self.command, *map(str, command)], capture_output=True, text=True)
        summary_file = output / "candidates.json"
        if result.returncode not in (0, 1) or not summary_file.exists():
            raise AdapterError(f"Native F# proposal failed ({result.returncode}): {result.stderr.strip()}")
        try:
            summary = json.loads(summary_file.read_text())
        except json.JSONDecodeError as error:
            raise AdapterError(f"Native F# proposal left invalid summary JSON: {error}") from error
        if (not isinstance(summary, dict) or not isinstance(summary.get("counts"), dict)
                or not isinstance(summary.get("generated"), list) or not isinstance(summary.get("rejected"), list)
                or not isinstance(summary.get("gates"), list)
                or any(not isinstance(gate, dict) for gate in summary["gates"])
                or summary["counts"].get("exported") != len(summary["generated"])
                or len(summary["generated"]) > limit):
            raise AdapterError("Native F# proposal returned an invalid bounded realization batch.")
        if (not isinstance(summary, dict) or summary.get("mutationVersion") != POLICY["mutationVersion"]
                or summary.get("seed") != seed or summary.get("attempt") != attempt
                or not isinstance(summary.get("changed"), list) or not summary["changed"]
                or not isinstance(summary.get("generated"), list)
                or summary.get("counts", {}).get("dynamicTrials") != 0
                or bool(summary["generated"]) != (result.returncode == 0)
                or (result.returncode == 1 and not (summary.get("rejected") or any(
                    gate.get("outcome") == "reject" for gate in summary.get("gates", []))))):
            raise AdapterError("Native F# proposal returned an inconsistent mutation/rejection summary.")
        return summary

    def build_native(self, parent, output, *, layers=1, placements=None, seen_requests=()):
        from .motion_carry import POLICY

        command = ["build-walker", "--parent", str(parent), "--output", str(output),
                   "--limit", "1", "--steps", str(POLICY["samples"]),
                   "--foot-policy", "paired-plates", "--foot-layers", str(layers),
                   "--ground-clearance-ldu", str(POLICY["groundClearanceLdu"])]
        if placements is not None:
            command += ["--target-placements", str(placements)]
        if seen_requests:
            command += ["--seen-requests", json.dumps(list(seen_requests))]
        result = subprocess.run([*self.command, *command], capture_output=True, text=True, timeout=600)
        summary_file = output / "candidates.json"
        if result.returncode not in (0, 1) or not summary_file.is_file():
            raise AdapterError(f"Motion-first construction failed ({result.returncode}): {result.stderr.strip()}")
        summary = json.loads(summary_file.read_text())
        original = load_manifest(parent / "manifest.json")
        if (summary.get("parentGenotypeId") != original["motion"]["genotypeId"]
                or summary.get("counts", {}).get("dynamicTrials") != 0
                or summary.get("counts", {}).get("exported") != len(summary.get("generated", []))
                or len(summary.get("generated", [])) > 1
                or bool(summary.get("generated")) != (result.returncode == 0)
                or (result.returncode == 1 and not summary.get("rejected"))
                or summary.get("motionFirst", {}).get("packagingVersion") != POLICY["packagingVersion"]
                or summary.get("motionFirst", {}).get("theoryVersion") != POLICY["theoryVersion"]):
            raise AdapterError("Motion-first construction returned inconsistent provenance, bounds or results.")
        request = json.loads((output / "request.json").read_text())
        program = json.loads((parent / "program.json").read_text())["program"]
        if (request["reference"] != program or request["drive"]["motorClockScale"] != 1
                or request["groundClearance"]["minimumNonFootClearanceLdu"] != POLICY["groundClearanceLdu"]):
            raise AdapterError("Motion-first construction changed its reference intent, input clock or ground requirement.")
        source = json.loads((parent / "realization.json").read_text())
        for entry in summary["generated"]:
            candidate = output / entry["directory"]
            genome = json.loads((candidate / "realization.json").read_text())
            if any(genome.get(key) != source.get(key) for key in ("speed", "maxTorque", "remote", "towWagon")):
                raise AdapterError("Motion-first construction changed electrical control or payload mode.")
            if not all((candidate / name).is_file() for name in ("ground-plan.json", "construction-proof.json")):
                raise AdapterError("Motion-first construction omitted its ground plan or construction proof.")
        return summary

    def repair_native(self, parent, output, feedback, limit):
        from .discovery import POLICY
        hints = json.loads(feedback.read_text())
        if hints.get("kind") == "stance-support-mass-layout":
            from .stance_support import repair_mass_layout
            return repair_mass_layout(self, parent, output, feedback, hints, limit)
        conflicts = None
        if hints.get("kind") in ("initial-self-interference", "motion-self-interference") and "pairs" in hints:
            pairs = hints["pairs"]
            if (not isinstance(pairs, list) or not pairs
                    or any(not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1]
                           or any(not isinstance(part, str) or not part.strip() for part in pair)
                           for pair in pairs)):
                raise AdapterError("Connection grammar repair requires nonempty pairs of distinct part IDs.")
            conflicts = feedback.with_name(feedback.stem + "-conflicts.json")
            write_json(conflicts, {"pairs": pairs})
            selected = feedback.with_name(feedback.stem + "-targets.json")
            write_json(selected, {key: value for key, value in hints.items() if key != "pairs"})
            feedback = selected
        command = ["construct-walker", "--parent", parent, "--output", output,
                   "--feedback", feedback, "--limit", limit]
        if conflicts is not None:
            command += ["--conflicts", conflicts]
        result = subprocess.run([*self.command, *map(str, command)], capture_output=True, text=True)
        summary_file = output / "candidates.json"
        if result.returncode not in (0, 1) or not summary_file.is_file():
            raise AdapterError(f"Native F# construction repair failed ({result.returncode}): {result.stderr.strip()}")
        try:
            summary = json.loads(summary_file.read_text())
        except json.JSONDecodeError as error:
            raise AdapterError(f"Native F# repair left invalid summary JSON: {error}") from error
        manifest = load_manifest(parent / "manifest.json")
        if (not isinstance(summary, dict) or not isinstance(summary.get("counts"), dict)
                or summary.get("constructionVersion") != POLICY["construction"]["version"]
                or (conflicts is not None
                    and summary.get("grammarVersion") != POLICY["construction"]["grammar"]["version"])
                or summary.get("parentGenotypeId") != manifest["motion"]["genotypeId"]
                or summary.get("mechanismId") != manifest["motion"]["mechanismId"]
                or summary.get("operation") != "repair-realization"
                or not isinstance(summary.get("changed"), list) or not summary["changed"]
                or not isinstance(summary.get("generated"), list) or len(summary["generated"]) > limit
                or summary.get("counts", {}).get("dynamicTrials") != 0
                or type(summary["counts"].get("realizationAttempts")) is not int
                or not 0 <= summary["counts"]["realizationAttempts"] <= limit
                or summary["counts"].get("exported") != len(summary["generated"])
                or bool(summary["generated"]) != (result.returncode == 0)
                or (not summary["generated"] and not any(gate.get("outcome") == "reject"
                                                       for gate in summary.get("gates", [])))):
            raise AdapterError("Native F# repair returned an inconsistent bounded construction summary.")
        return summary


def protocol_definition(generator, library, options, profile):
    from .motor import definition as motor_definition
    options.validate()
    catalog = [line.split()[0] for line in generator.invoke("catalog").splitlines()]
    sources = [*sorted((ROOT / "src/LegoWalkers.Core").glob("*.fs")),
               *[Path(__file__).with_name(name) for name in
                 ("bridge.py", "collision.py", "evaluation.py", "joint_tree.py", "spatial_gears.py", "stance_support.py", "motion_carry.py", "motor.py", "payload.py", "ground.py", "schema.py", "evolution.py",
                  "native_episode.py", "discovery.py", "leg_family_migration.py", "population.py", "parallel.py", "prescreen.py", "pipeline.py", "screening.py", "screening_reports.py", "observations.py", "cycles.py", "mass_layout.py", "phase_clearance.py", "campaign.py", "upgrades.py", "simulation_gate.py", "support_surfaces.py", "user_pool.py")],
               ROOT / "src/LegoWalkers.Cli/Program.fs"]
    settings = asdict(options)
    settings["payload_profile"] = None
    return {
        "schemaVersion": 1, "grammarVersion": GRAMMAR, "simulatorRevision": SIMULATOR_REVISION,
        "categories": [category_genome(*item) for item in CATEGORIES],
        "searchPolicy": SEARCH_POLICY,
        "partsDependencySha256": library_fingerprint(library, catalog),
        "sourceSha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in sources},
        "generatorBinarySha256": {
            name: hashlib.sha256(Path(generator.command[1]).with_name(name).read_bytes()).hexdigest()
            for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll")
        },
        "versions": {name: importlib.metadata.version(name) for name in
                     ("mujoco", "numpy", "lego-technic-sim", "legowalkers-sim")},
        "evaluation": settings, "payloadProfile": profile,
        "qualification": {"maxTiltDegrees": 60, "maxClosureErrorMeters": .002,
                          "minimumLogicalFeet": MIN_WALKER_FEET, "maximumLogicalFeet": MAX_FEET,
                          "maxSelfPenetrationMeters": .0001, "minimumMotorTurns": 1,
                          "positiveForwardMotion": True, "selfCollision": True, "freeChassis": True,
                          "allowedGroundContacts": "designated-foot-regions-only",
                          "groundContactDistanceThresholdMeters": 0.0, "maximumFootRegionRadiusMeters": .004},
        "selection": "Qualified: speed, then lower work. Prototypes: collision/closure/upright/allowed-ground/cycle gates, turns, speed.",
        "controller": motor_definition(options.motor_model, options.speed, options.torque),
        "power": "Remote only: onboard 8881 AA + 8884 with actual payload masses and ideal electrical supply.",
        "schedule": "Balanced foot-count bins; alternate within-bin and global elite parents. Foot count does not restrict ancestry.",
        "wagon": "Excluded from active search; historical towing results and export recipes are preserved.",
    }


def freeze_runtime(archive, protocol, generator):
    destination = archive.directory / "protocol-source"
    sources = {ROOT / name for name in protocol["sourceSha256"]}
    sources.update(Path(__file__).parent.glob("*.py"))
    sources.update((ROOT / "src").glob("*/*.fsproj"))
    sources.update((ROOT / "src").glob("*/*.html"))
    sources.update(ROOT / name for name in ("global.json", "pyproject.toml", "scripts/preview.py",
                                           "scripts/render_preview.py", "simulator/legowalkers_sim/payloads.json"))
    for path in sources:
        target = destination / path.relative_to(ROOT)
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    binary = Path(generator.command[1]).parent
    for path in [*binary.glob("*.dll"), *binary.glob("*.json")]:
        target = destination / "generator" / path.name
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    for name, expected in protocol["sourceSha256"].items():
        if hashlib.sha256((destination / name).read_bytes()).hexdigest() != expected:
            raise AdapterError(f"Frozen protocol source checksum mismatch: {name}")
    for name, expected in protocol["generatorBinarySha256"].items():
        if hashlib.sha256((destination / "generator" / name).read_bytes()).hexdigest() != expected:
            raise AdapterError(f"Frozen compiler checksum mismatch: {name}")
    provenance = archive.directory / "provenance.json"
    if not provenance.exists():
        write_json(provenance, {"python": sys.version, "platform": platform.platform(),
                               "protocol": identity(protocol), "sourceSnapshot": "protocol-source",
                               "note": "Original hash-verified implementation and compiler; do not edit."})
    generator.command[1] = str(destination / "generator" / Path(generator.command[1]).name)


def render_representative(archive, row, blender):
    if row.get("reused"):
        return
    design = archive.directory / "designs" / row["design"]
    target = design / "render"
    if (target / "walker-v2-anchored.mp4").exists():
        return
    assets = design / f"render-pending-{uuid.uuid4().hex[:8]}"
    evaluation = archive.directory / "evaluations" / f"{row['id']:04d}"
    commands = [
        [sys.executable, str(ROOT / "scripts/preview.py"), "prepare", "--assets", str(assets),
         "--manifest", str(design / "candidate/manifest.json"), "--evaluation", str(evaluation),
         "--library", str(archive.library), "--preset", "draft"],
        [str(blender), "--background", "--factory-startup", "--python-exit-code", "1",
         "--python", str(ROOT / "scripts/render_preview.py"), "--", "--assets", str(assets), "--mode", "still"],
        [str(blender), "--background", "--factory-startup", "--python-exit-code", "1",
         "--python", str(ROOT / "scripts/render_preview.py"), "--", "--assets", str(assets),
         "--mode", "animation", "--preset", "draft"],
    ]
    log = design / "render.log"
    try:
        with log.open("w") as handle:
            for command in commands:
                subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, check=True)
        assets.rename(target)
        with archive.db:
            archive.db.execute("DELETE FROM media_errors WHERE design=?", (row["design"],))
    except (OSError, subprocess.CalledProcessError) as error:
        message = f"Preview failed: {error}; see {log.name}."
        print(f"ERROR: {message}", file=sys.stderr, flush=True)
        with archive.db:
            archive.db.execute("INSERT OR REPLACE INTO media_errors VALUES (?,?)", (row["design"], message))


def search(archive, generator, library, options, blender=None):
    from .gallery import publish

    archive.library = library
    archive.recover()
    archive.set_status("running")
    publish(archive)
    run = archive.db.execute("SELECT * FROM run").fetchone()
    protocol = archive.protocol()
    categories = categories_for(protocol)
    foot_only = foot_count_categories(protocol)
    per_category = run["budget"] // len(categories)
    try:
        for round_index in range(per_category):
            for index, (pairs, remote, towing) in enumerate(categories):
                kind = category(category_genome(pairs, remote, towing), foot_only=foot_only)
                count = archive.db.execute("""
                    SELECT count(*) FROM evaluations e JOIN births b ON b.id=e.birth
                    JOIN designs d ON d.id=b.design WHERE d.category=?
                """, (kind,)).fetchone()[0]
                if count > round_index:
                    continue
                if sum(archive.counts().values()) >= run["budget"]:
                    break
                best = archive.best(kind)
                if foot_only and (best is None or round_index % 2 == 0):
                    best = archive.best()
                if best is None:
                    parent_row = archive.db.execute("""
                        SELECT b.id,b.design FROM births b JOIN designs d ON b.design=d.id
                        WHERE d.category=? AND b.status='generated' ORDER BY b.id DESC LIMIT 1
                    """, (kind,)).fetchone()
                    if parent_row is None and foot_only:
                        parent_row = archive.db.execute("""
                            SELECT b.id,b.design FROM births b WHERE b.status='generated'
                            ORDER BY b.id DESC LIMIT 1
                        """).fetchone()
                    if parent_row is None and not foot_only and (remote or towing):
                        parent_row = archive.db.execute("""
                            SELECT b.id,b.design FROM births b JOIN designs d ON b.design=d.id
                            WHERE d.category=? AND b.status='generated' ORDER BY b.id LIMIT 1
                        """, (category(category_genome(pairs, remote if towing else False, False)),)).fetchone()
                    parent = parent_row["id"] if parent_row else None
                    parent_design = parent_row["design"] if parent_row else None
                else:
                    parent, parent_design = best["birth"], best["design"]
                parent_file = (archive.directory / "designs" / parent_design / "genome.json"
                               if parent_design else None)
                proposal_number = archive.db.execute("""
                    SELECT count(*) FROM births b JOIN designs d ON d.id=b.design WHERE d.category=?
                """, (kind,)).fetchone()[0]
                seed = (run["seed"] + round_index * 1009 + index * 7919) % 2147483647
                for attempt in range(proposal_number, proposal_number + 64):
                    proposal = generator.propose(pairs, remote, towing, seed, attempt, parent_file)
                    genome = proposal["genome"]
                    identifier = design_identity(genome)
                    duplicate = archive.db.execute("SELECT 1 FROM designs WHERE id=?", (identifier,)).fetchone()
                    status = "duplicate" if duplicate else ("generated" if proposal["accepted"] else "generation-rejected")
                    errors = "; ".join(proposal["errors"]) or None
                    birth, identifier = archive.record_birth(
                        genome, parent, proposal["operation"], proposal["changed"], status, errors)
                    design = archive.directory / "designs" / identifier
                    if not duplicate:
                        write_json(design / "genome.json", genome)
                    if duplicate or not proposal["accepted"]:
                        publish(archive)
                        continue
                    candidate = design / "candidate"
                    try:
                        generator.invoke("generate", "--genome", design / "genome.json", "--output", candidate)
                        checked = check(candidate / "manifest.json", library)
                        if foot_only:
                            validate_search_candidate(checked, pairs*2)
                        write_json(design / "compatibility.json", checked.report)
                    except (AdapterError, OSError) as error:
                        print(f"ERROR [{kind} proposal {birth}]: {error}", file=sys.stderr, flush=True)
                        with archive.db:
                            archive.db.execute("UPDATE births SET status='compatibility-rejected',error=? WHERE id=?",
                                               (str(error), birth))
                        write_json(design / "error.json", {"status": "compatibility-rejected", "error": str(error)})
                        publish(archive)
                        continue
                    trial = archive.reserve(birth)
                    output = archive.directory / "evaluations" / f"{trial:04d}"
                    publish(archive)
                    print(f"Trial {trial}/{run['budget']}: {kind}, design {identifier[:10]}, parent {parent}", flush=True)
                    try:
                        result = evaluate(checked, output, options)
                        archive.finish(trial, result)
                        print(f"  {result['status']}; {result['metrics']['rankingRejectionReasons']}", flush=True)
                    except (AdapterError, OSError) as error:
                        print(f"ERROR [trial {trial}]: {error}", file=sys.stderr, flush=True)
                        archive.finish(trial, error=str(error), status="failed")
                        write_json(output / "error.json", {"status": "failed", "error": str(error)})
                    publish(archive)
                    representative = archive.best(kind)
                    if blender is not None and representative is not None:
                        render_representative(archive, representative, blender)
                        publish(archive)
                    break
                else:
                    raise AdapterError(f"Exhausted 64 proposals for {kind}; archive preserved. Expand grammar or start a new run.")
        archive.set_status("complete")
    except (AdapterError, OSError, KeyboardInterrupt):
        archive.set_status("paused")
        publish(archive)
        raise
    publish(archive)
    return {"status": "complete", "evaluations": archive.counts(), "page": str(archive.directory / "preview.html")}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ldraw-library", type=Path, default=os.environ.get("LDRAW_LIBRARY"))
    parser.add_argument("--dotnet", type=Path, default=ROOT / ".tools/dotnet/dotnet")
    parser.add_argument("--generator", type=Path, default=ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    parser.add_argument("--budget", type=int, default=12)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--payload-profile", type=Path)
    parser.add_argument("--blender", type=Path)
    parser.add_argument("--publish-only", action="store_true")
    parser.add_argument("--candidate-batch", type=Path,
                        help="Evaluate an immutable whole-walker realization batch instead of legacy evolution")
    parser.add_argument("--discover", action="store_true", help="Evolve native motion programs after evaluating founders")
    parser.add_argument("--defer-construction", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--cycle-aware", action="store_true", help="Bound runtime from gearing and require repeatable complete gait cycles")
    parser.add_argument("--short-gate", type=float, default=2.0,
                        help="Early safety gate after settling, in driven seconds (default: 2; 0 disables)")
    parser.add_argument("--reuse-archive", type=Path, action="append", default=[], help="Reuse compatible founder measurements with original provenance")
    parser.add_argument("--simulation-workers", type=int, default=1,
                        help="Maximum concurrent native simulations; legacy paired evolution remains serial")
    parser.add_argument("--screening-workers", type=int, default=1,
                        help="Concurrent screening/preflight processes for native discovery")
    args = parser.parse_args(argv)
    try:
        if args.publish_only:
            from .gallery import publish
            if not (args.output / "archive.sqlite3").exists():
                raise AdapterError("No existing archive to publish.")
            archive = Archive(args.output)
            publish(archive)
            archive.db.close()
            return 0
        if args.discover and args.candidate_batch is None:
            raise AdapterError("--discover requires a complete native --candidate-batch.")
        from .parallel import worker_count
        worker_count(args.simulation_workers)
        from .prescreen import validate_workers
        validate_workers(args.screening_workers)
        if args.screening_workers > 1 and not args.discover:
            raise AdapterError("Parallel screening requires native --discover.")
        if args.simulation_workers > 1 and args.candidate_batch is None:
            raise AdapterError("Parallel simulations require a native --candidate-batch.")
        if args.reuse_archive and not args.discover:
            raise AdapterError("--reuse-archive requires native --discover.")
        if args.budget <= 0 or (args.candidate_batch is None and args.budget % len(CATEGORIES)):
            raise AdapterError(f"Budget must be a positive multiple of {len(CATEGORIES)}, balanced across the remote-only foot-count bins.")
        if args.ldraw_library is None:
            raise AdapterError("Supply --ldraw-library or set LDRAW_LIBRARY.")
        args.ldraw_library = args.ldraw_library.expanduser().resolve()
        if not 0 <= args.seed < 2147483647:
            raise AdapterError("Seed must be between 0 and 2147483646.")
        options = EvaluationOptions(duration=20, timestep=.002, capture_poses=True,
                                    payload_profile=str(args.payload_profile.resolve()) if args.payload_profile else None,
                                    cycle_aware=args.cycle_aware, short_gate=args.short_gate)
        profile = load_profile(args.payload_profile)
        generator = Generator(args.dotnet, args.generator)
        protocol = protocol_definition(generator, args.ldraw_library, options, profile)
        if args.candidate_batch is not None:
            from .native_episode import native_protocol
            protocol = native_protocol(protocol, args.candidate_batch.resolve(), discover=args.discover,
                                       reuse_archives=args.reuse_archive, simulation_workers=args.simulation_workers,
                                       screening_workers=args.screening_workers, defer_construction=args.defer_construction)
        from .campaign import require_active
        require_active(args.output.resolve())
        with locked(args.output.resolve()):
            require_active(args.output.resolve())
            archive = Archive(args.output)
            archive.initialize(protocol, args.budget, args.seed, args.resume)
            freeze_runtime(archive, protocol, generator)
            # Freeze the actual mass inputs as well as their protocol identity.
            profile_file = archive.directory / "payload-profile.json"
            if not profile_file.exists():
                write_json(profile_file, profile)
            if load_profile(profile_file) != profile:
                raise AdapterError("Archived payload profile changed; refusing to mix mass assumptions.")
            options = EvaluationOptions(duration=20, timestep=.002, capture_poses=True,
                                        payload_profile=str(profile_file), cycle_aware=args.cycle_aware,
                                        short_gate=args.short_gate)
            try:
                if args.discover:
                    from .discovery import search_discovery
                    result = search_discovery(archive, generator, args.candidate_batch.resolve(),
                                              args.ldraw_library, options, args.blender, workers=args.simulation_workers)
                elif args.candidate_batch is not None:
                    from .native_episode import search_native
                    result = search_native(archive, args.candidate_batch.resolve(), args.ldraw_library, options,
                                           args.blender, workers=args.simulation_workers)
                else:
                    result = search(archive, generator, args.ldraw_library, options, args.blender)
            finally:
                archive.db.close()
        print(json.dumps(result, indent=2))
        return 1 if result.get("failures", 0) else 0
    except (AdapterError, OSError, json.JSONDecodeError, sqlite3.Error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Evolution paused; resume uses the remaining budget, not repeated interrupted trials.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
