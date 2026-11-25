"""Recorded user-parent admissions, separate from measured fitness retention."""

import contextlib
import json
import os
from pathlib import Path
import random
import re
import sqlite3

from .bridge import AdapterError, load_manifest
from .evolution import canonical, evidence_invalidation, now

POLICY = {"version": "user-population-v5", "capacity": 12, "parentProbability": .2,
          "selection": "Uniform active admitted contributions, before measured-parent tournaments; "
                       "diagnostic repairs retain priority. Contributions never acquire invented fitness.",
          "admission": "Snapshot motion/control intent; current bounded paired-foot, ground-reserved construction and exact validation per immutable version.",
          "evaluation": "Contribution admission shares screening capacity with ordinary candidates; ready unmeasured contributions queue before offspring. Normal gates and existing authorized slots apply. "
                        "Existing measurements and attempted designs are not repeated.",
          "budget": "No dynamic authorization from edits, contributions or admissions."}


def status(directory):
    with contextlib.closing(sqlite3.connect(
            f'{(Path(directory).resolve() / "archive.sqlite3").as_uri()}?mode=ro', uri=True)) as db:
        db.row_factory = sqlite3.Row
        if not db.execute("SELECT 1 FROM sqlite_master WHERE name='user_pool_entries'").fetchone():
            return []
        invalidated = evidence_invalidation(json.loads(db.execute(
            "SELECT p.definition FROM protocols p JOIN run r ON r.protocol=p.id").fetchone()[0]))
        values = []
        for row in db.execute("""SELECT u.*,coalesce(e.result,r.result) AS result,b.status AS birth_status,
                EXISTS(SELECT 1 FROM evaluations attempted WHERE attempted.birth=u.birth) AS attempted
                FROM user_pool_entries u
                LEFT JOIN births b ON b.id=u.birth
                LEFT JOIN evaluations e ON e.birth=u.birth AND e.status='complete'
                LEFT JOIN reused_evaluations r ON r.birth=u.birth ORDER BY u.contribution"""):
            measured = row["result"] is not None
            from .evolution import is_qualified
            values.append({"contribution": row["contribution"], "birth": row["birth"], "admission": row["status"],
                           "error": row["error"], "measurement": (
                               "invalidated; corrected simulation required" if invalidated else
                               "qualified" if is_qualified(json.loads(row["result"])) else "evaluated, not qualified")
                           if measured else (
                               "unmeasured; queued for evaluation" if row["birth_status"] == "generated"
                               and not row["attempted"] else "unmeasured")})
        return values


def founder_origins(archive):
    from .upgrades import _origin

    roots = archive.db.execute("SELECT id FROM births WHERE parent IS NULL AND status!='duplicate'").fetchall()
    return {_origin(archive.directory, row["id"]): row["id"] for row in roots}


def queue(archive, *, startup_batch=None):
    from .native_episode import _freeze_batch
    from .upgrades import _origin

    archive.db.executescript("""
        CREATE TABLE IF NOT EXISTS user_pool_entries(
            contribution TEXT PRIMARY KEY,birth INTEGER,status TEXT NOT NULL,
            error TEXT,provenance TEXT NOT NULL);
        CREATE TABLE IF NOT EXISTS user_pool_snapshots(
            id INTEGER PRIMARY KEY,created TEXT NOT NULL,entries TEXT NOT NULL);
    """)
    marker = archive.directory / "campaign-version.json"
    entries = []
    if marker.exists():
        campaign = Path(json.loads(marker.read_text())["campaign"]).resolve()
        workbench = campaign / "user-workbench"
        registry = workbench / "workbench.sqlite3"
        if registry.exists():
            with contextlib.closing(sqlite3.connect(f'{registry.as_uri()}?mode=ro', uri=True)) as db:
                db.row_factory = sqlite3.Row
                db.execute("BEGIN")
                entries = [dict(row) for row in db.execute("""
                    SELECT c.*,n.source_archive,n.source_birth FROM contributions c
                    JOIN nodes n ON n.id=c.node WHERE c.active=1 ORDER BY c.created,c.id
                """)]
                for entry in entries:
                    lineage, node, seen = [], entry["node"], set()
                    while node is not None:
                        if node in seen:
                            raise AdapterError("Workbench contribution ancestry contains a cycle.")
                        seen.add(node)
                        row = db.execute("SELECT id,parent,summary FROM nodes WHERE id=?", (node,)).fetchone()
                        if row is None:
                            raise AdapterError("Workbench contribution ancestry is incomplete.")
                        lineage.append({"id": row["id"], "parent": row["parent"], "summary": json.loads(row["summary"])})
                        node = row["parent"]
                    entry["lineage"] = list(reversed(lineage))
    if len(entries) > POLICY["capacity"]:
        raise AdapterError("User contribution registry exceeds the frozen population capacity.")
    with archive.db:
        archive.db.execute("INSERT INTO user_pool_snapshots(created,entries) VALUES(?,?)",
                           (now(), canonical(entries)))
    origins = founder_origins(archive) if entries else {}
    from .pipeline import ensure_schema, queue_startup, startup_step
    ensure_schema(archive.db)
    if startup_batch is not None:
        queue_startup(archive, startup_batch, [
            (entry["id"], "admission", entry["label"]) for entry in entries])
    tasks = []
    for entry in entries:
        if not all(isinstance(entry[key], str) and re.fullmatch("[a-f0-9]{32}", entry[key])
                   for key in ("id", "node")):
            raise AdapterError("User contribution contains an invalid immutable identifier.")
        source = workbench / "designs" / entry["node"]
        saved = archive.directory / "user-contribution-inputs" / entry["id"]
        _freeze_batch(source, saved, json.loads(entry["files"]))
        existing = archive.db.execute("SELECT * FROM user_pool_entries WHERE contribution=?", (entry["id"],)).fetchone()
        if existing is None:
            previous = archive.db.execute("""
                SELECT batch,task,pid FROM pipeline_startup
                WHERE kind='admission' AND task=? AND status='running' ORDER BY id DESC LIMIT 1
            """, (entry["id"],)).fetchone()
            if previous is not None:
                try:
                    os.kill(previous["pid"], 0)
                except ProcessLookupError:
                    from .pipeline import update_startup
                    error = "Previous admission worker exited; partial work is retained and not automatically repeated."
                    interrupt_admission(archive, {"entry": entry}, error)
                    update_startup(archive, previous["batch"], previous["task"], "interrupted", error)
                    existing = archive.db.execute(
                        "SELECT * FROM user_pool_entries WHERE contribution=?", (entry["id"],)).fetchone()
                else:
                    raise AdapterError(f"Admission worker {previous['pid']} is still running; wait before resuming.")
        if existing is not None:
            tracking = (startup_step(archive, startup_batch, entry["id"]) if startup_batch is not None
                        else contextlib.nullcontext())
            with tracking as progress:
                if json.loads(existing["provenance"])["files"] != entry["files"]:
                    raise AdapterError("Previously admitted user contribution inputs changed.")
                if existing["status"] == "eligible":
                    with archive.db:
                        archive.db.execute("""
                            UPDATE births SET status='generated' WHERE id=? AND status='user-parent'
                            AND NOT EXISTS(SELECT 1 FROM evaluations WHERE birth=births.id)
                            AND NOT EXISTS(SELECT 1 FROM reused_evaluations WHERE birth=births.id)
                        """, (existing["birth"],))
                if progress is not None:
                    progress.update(status="complete" if existing["status"] == "eligible" else "rejected",
                                    birth=existing["birth"], detail=existing["error"] or
                                    "Admission already recorded; no repeated reconstruction check.")
            continue
        from .motion_carry import POLICY as motion_policy
        tasks.append({"entry": entry, "source": str(saved),
                      "parent": origins.get(_origin(entry["source_archive"], entry["source_birth"])),
                      "motionFirst": archive.protocol().get("nativeDiscovery", {}).get("motionCarryForward") == motion_policy})
    return tasks


def prepare_admission(task, directory, generator, library, payload_profile, observe):
    from .native_episode import _queue, _freeze_batch, refresh_seed_exports
    from .screening import screen_batch
    from .screening_reports import rejection_detail

    entry = json.loads(canonical(task["entry"]))
    saved, directory = Path(task["source"]), Path(directory)
    original_files = json.loads(entry["files"])
    _freeze_batch(saved, saved, original_files)
    observe("planning", "Importing immutable contribution inputs with the frozen generator.")
    source = refresh_seed_exports(saved, directory / "input-exports" / entry["id"], generator)
    summary, files = _queue(source)
    migrated = source != saved.resolve()
    unchanged = {name: digest for name, digest in files.items()
                 if not migrated or (name != "export-migration.json" and not name.endswith("/manifest.json"))}
    expected = {name: digest for name, digest in original_files.items()
                if not migrated or (name != "export-migration.json" and not name.endswith("/manifest.json"))}
    if unchanged != expected:
        raise AdapterError("User contribution changed after static validation.")
    if generator is not None:
        from .leg_family_migration import convert
        source = convert(source, directory / "leg-family-inputs" / entry["id"], generator)
        summary, files = _queue(source)
        entry["legFamilyConversion"] = json.loads((source / "leg-family-migration.json").read_text())
    if task["motionFirst"]:
        from .motion_carry import rebuild
        if generator is None:
            raise AdapterError("Motion-first user admission requires the frozen current constructor.")
        rebuilt = rebuild(source / summary["generated"][0]["directory"],
                          directory / "user-motion-builds" / entry["id"],
                          generator, library, payload_profile, observer=observe)
        entry["motionReconstruction"] = rebuilt
        if rebuilt["status"] != "generated":
            return {"status": "construction-rejected", "error": rebuilt["error"], "entry": entry}
        source = Path(rebuilt["batch"])
        summary, files = _queue(source)
    entry["admissionFiles"] = files
    frozen = directory / "user-contributions" / entry["id"]
    _freeze_batch(source, frozen, files)
    observe("screening", "Exact load-only admission; no walking trial reserved.")
    screening = screen_batch(frozen, directory / "user-admissions" / entry["id"],
                             library, max_checks=1, payload_profile=payload_profile)
    report, = screening["results"]
    ready = report["status"] == "ready-for-dynamics" or (
        report["status"] == "duplicate" and report.get("previousStatus") == "ready-for-dynamics")
    error = None if ready else rejection_detail(report)
    if task["parent"] is None:
        ready, error = False, "Source founder family is not retained; fork a current candidate to rebase."
    return {"status": "eligible" if ready else "rejected", "error": error, "entry": entry}


def finish_admission(archive, task, result):
    from .native_episode import _queue, _copy_candidate

    if (not isinstance(result, dict) or result.get("status") not in ("eligible", "rejected", "construction-rejected")
            or not isinstance(result.get("entry"), dict)):
        raise AdapterError("Admission worker returned an invalid outcome.")
    entry, status, error = result["entry"], result["status"], result.get("error")
    if any(entry.get(key) != value for key, value in task["entry"].items()):
        raise AdapterError("Admission worker changed immutable contribution provenance.")
    if (status == "eligible" and error is not None) or (status != "eligible" and (not isinstance(error, str) or not error)):
        raise AdapterError("Admission outcome omitted or contradicted its diagnostic.")
    birth = None
    if status == "eligible":
        frozen = archive.directory / "user-contributions" / entry["id"]
        summary, files = _queue(frozen)
        if files != entry.get("admissionFiles"):
            raise AdapterError("Admission inputs changed after independent checking.")
        candidate, = summary["generated"]
        prior = archive.db.execute(
            "SELECT id FROM births WHERE design=? AND status!='duplicate' ORDER BY id LIMIT 1",
            (candidate["genotypeId"],)).fetchone()
        if prior:
            birth = prior[0]
            if "motionReconstruction" in entry:
                from .motion_carry import bind_candidate
                bind_candidate(archive, birth, {"batch": str(frozen.relative_to(archive.directory)),
                                              "directory": candidate["directory"], "files": files})
        else:
            manifest, _ = _copy_candidate(archive, frozen, candidate, files)
            birth, _ = archive.record_native_birth(
                manifest, status="generated", parent=task["parent"], operation="user-contribution",
                changed=[{"path": "user-contribution", "before": [], "after": [entry["id"]]}],
                native={"assembly": candidate["assemblyId"],
                        "batch": str(frozen.relative_to(archive.directory)), "directory": candidate["directory"],
                        "files": files})
    with archive.db:
        archive.db.execute("INSERT INTO user_pool_entries VALUES(?,?,?,?,?)",
                           (entry["id"], birth, status, error, canonical(entry)))
    return birth


def interrupt_admission(archive, task, error):
    with archive.db:
        archive.db.execute("""INSERT INTO user_pool_entries VALUES(?,NULL,'interrupted',?,?)
            ON CONFLICT(contribution) DO NOTHING""",
                           (task["entry"]["id"], error, canonical(task["entry"])))


def sync(archive, library, payload_profile=None, *, startup_batch=None, generator=None):
    from .pipeline import startup_detail, startup_step

    for task in queue(archive, startup_batch=startup_batch):
        tracking = (startup_step(archive, startup_batch, task["entry"]["id"]) if startup_batch is not None
                    else contextlib.nullcontext())
        with tracking as progress:
            def observe(stage, detail):
                if startup_batch is not None:
                    startup_detail(archive, startup_batch, task["entry"]["id"], f"{stage}: {detail}")

            result = prepare_admission(task, archive.directory, generator, library, payload_profile, observe)
            birth = finish_admission(archive, task, result)
            if progress is not None:
                progress.update(status="complete" if result["status"] == "eligible" else "rejected", birth=birth,
                                detail=result["error"] or "Load-only admission passed; walking qualification is separate.")


def choose(archive, seed, proposal):
    if not archive.db.execute("SELECT 1 FROM sqlite_master WHERE name='user_pool_snapshots'").fetchone():
        return None
    snapshot = archive.db.execute("SELECT id,entries FROM user_pool_snapshots ORDER BY id DESC LIMIT 1").fetchone()
    if snapshot is None:
        return None
    active = {entry["id"] for entry in json.loads(snapshot["entries"])}
    rows = [dict(row) for row in archive.db.execute("""
        SELECT u.contribution,u.provenance,u.birth,b.design FROM user_pool_entries u
        JOIN births b ON b.id=u.birth WHERE u.status='eligible' ORDER BY u.contribution
    """) if row["contribution"] in active]
    if not rows:
        return None
    rng = random.Random(f'{POLICY["version"]}:{seed}:{proposal}')
    if rng.random() >= POLICY["parentProbability"]:
        return None
    parent = rng.choice(rows)
    return parent, {"policy": POLICY["version"], "mode": "user-pool", "probability": POLICY["parentProbability"],
                    "snapshot": snapshot["id"], "contribution": parent["contribution"],
                    "selectedBirth": parent["birth"], "pool": [row["contribution"] for row in rows],
                    "provenance": json.loads(parent["provenance"])}
