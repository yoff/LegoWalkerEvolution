"""Recorded user-parent admissions, separate from measured fitness retention."""

import contextlib
import json
from pathlib import Path
import random
import re
import sqlite3

from .bridge import AdapterError, load_manifest
from .evolution import canonical, evidence_invalidation, now

POLICY = {"version": "user-population-v4", "capacity": 12, "parentProbability": .2,
          "selection": "Uniform active admitted contributions, before measured-parent tournaments; "
                       "diagnostic repairs retain priority. Contributions never acquire invented fitness.",
          "admission": "Snapshot motion/control intent; current bounded paired-foot, ground-reserved construction and exact validation per immutable version.",
          "evaluation": "Admitted unmeasured designs queue before offspring; normal gates and existing authorized slots apply. "
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


def sync(archive, library, payload_profile=None, *, startup_batch=None, generator=None):
    from .native_episode import _queue, _freeze_batch, _copy_candidate, refresh_seed_exports
    from .screening import screen_batch
    from .screening_reports import rejection_detail
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
    from .pipeline import queue_startup, startup_step
    if startup_batch is not None:
        queue_startup(archive, startup_batch, [
            (entry["id"], "admission", entry["label"]) for entry in entries])
    for entry in entries:
        tracking = (startup_step(archive, startup_batch, entry["id"]) if startup_batch is not None
                    else contextlib.nullcontext())
        with tracking as progress:
            if not all(isinstance(entry[key], str) and re.fullmatch("[a-f0-9]{32}", entry[key])
                       for key in ("id", "node")):
                raise AdapterError("User contribution contains an invalid immutable identifier.")
            source = workbench / "designs" / entry["node"]
            original_files = json.loads(entry["files"])
            saved = archive.directory / "user-contribution-inputs" / entry["id"]
            _freeze_batch(source, saved, original_files)
            existing = archive.db.execute("SELECT * FROM user_pool_entries WHERE contribution=?", (entry["id"],)).fetchone()
            if existing is not None:
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
            source = refresh_seed_exports(saved, archive.directory / "input-exports", generator)
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
                source = convert(source, archive.directory / "leg-family-inputs", generator)
                summary, files = _queue(source)
                entry["legFamilyConversion"] = json.loads((source / "leg-family-migration.json").read_text())
            from .motion_carry import POLICY as motion_policy, rebuild
            if archive.protocol().get("nativeDiscovery", {}).get("motionCarryForward") == motion_policy:
                if generator is None:
                    raise AdapterError("Motion-first user admission requires the frozen current constructor.")
                rebuilt = rebuild(source / summary["generated"][0]["directory"],
                                  archive.directory / "user-motion-builds" / entry["id"],
                                  generator, library, payload_profile)
                entry["motionReconstruction"] = rebuilt
                if rebuilt["status"] != "generated":
                    with archive.db:
                        archive.db.execute("INSERT INTO user_pool_entries VALUES(?,NULL,?,?,?)",
                                           (entry["id"], "construction-rejected", rebuilt["error"], canonical(entry)))
                    if progress is not None:
                        progress.update(status="rejected", detail=rebuilt["error"])
                    continue
                source = Path(rebuilt["batch"])
                summary, files = _queue(source)
            entry["admissionFiles"] = files
            origin = _origin(entry["source_archive"], entry["source_birth"])
            parent = origins.get(origin)
            frozen = archive.directory / "user-contributions" / entry["id"]
            _freeze_batch(source, frozen, files)
            screening = screen_batch(frozen, archive.directory / "user-admissions" / entry["id"],
                                     library, max_checks=1, payload_profile=payload_profile)
            report, = screening["results"]
            ready = report["status"] == "ready-for-dynamics" or (
                report["status"] == "duplicate" and report.get("previousStatus") == "ready-for-dynamics")
            error = None if ready else rejection_detail(report)
            if parent is None:
                ready, error = False, "Source founder family is not retained; fork a current candidate to rebase."
            birth = None
            if ready:
                candidate = summary["generated"][0]
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
                        manifest, status="generated", parent=parent, operation="user-contribution",
                        changed=[{"path": "user-contribution", "before": [], "after": [entry["id"]]}],
                        native={"assembly": candidate["assemblyId"],
                                "batch": str(frozen.relative_to(archive.directory)), "directory": candidate["directory"],
                                "files": files})
            with archive.db:
                archive.db.execute("INSERT INTO user_pool_entries VALUES(?,?,?,?,?)",
                                   (entry["id"], birth, "eligible" if ready else "rejected", error, canonical(entry)))
            if progress is not None:
                progress.update(status="complete" if ready else "rejected", birth=birth,
                                detail=error or "Load-only admission passed; walking qualification is separate.")


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
