"""Presentation-only candidate stages and bounded live pipeline snapshots."""

from collections import Counter
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
import sys

from .bridge import AdapterError
from .evolution import is_qualified
from .screening_reports import rejection_detail, rejections, screening_progress

STAGES = {
    "planning": "Planning / selecting edits",
    "building": "Building / realization attempts",
    "generating": "Generating",
    "queued": "Pending / awaiting screening",
    "screening": "Screening",
    "screened": "Screened / awaiting preflight",
    "preflight": "Final preflight",
    "ready": "Ready for simulation",
    "gating": "Cheap simulation gate",
    "simulating": "Simulating",
    "rendering": "Rendering animation",
    "paused": "Paused",
    "user-parent": "User-contributed parent (unmeasured)",
}

def construction_progress(directory, proposal, summary):
    path = directory / "proposals" / f"{proposal:06d}" / "construction-progress.json"
    events = []
    if path.is_file():
        try:
            document = json.loads(path.read_text())
        except json.JSONDecodeError as error:
            raise AdapterError(f"Invalid construction progress for proposal {proposal}: {error}") from error
        if not isinstance(document, dict):
            raise AdapterError(f"Invalid construction progress for proposal {proposal}.")
        events = document.get("events")
        if document.get("schemaVersion") != 1 or not isinstance(events, list) or not events:
            raise AdapterError(f"Invalid construction progress for proposal {proposal}.")
        for event in events:
            if (not isinstance(event, dict) or event.get("stage") not in ("planning", "building")
                    or event.get("status") not in ("running", "complete", "rejected", "duplicate")
                    or type(event.get("attempt")) is not int or event["attempt"] < 0
                    or not isinstance(event.get("detail"), str)):
                raise AdapterError(f"Invalid construction event for proposal {proposal}.")
    report = directory / "proposals" / f"{proposal:06d}" / "candidates.json"
    tracked = bool(events)
    if not tracked:
        events = [{"stage": "saved realization", "status": "rejected", "attempt": rejected["attempt"],
                   "detail": json.dumps(rejected["decision"])} for rejected in summary.get("rejected", [])
                  if "attempt" in rejected and "decision" in rejected]
    return {"tracked": tracked, "events": events[-24:], "operation": summary.get("operation"),
            "counts": summary.get("counts"), "report": str(report.relative_to(directory)) if report.is_file()
            else str(path.relative_to(directory)) if tracked else None}


def ensure_schema(db):
    db.execute("""CREATE TABLE IF NOT EXISTS pipeline_events(
        id INTEGER PRIMARY KEY, birth INTEGER NOT NULL REFERENCES births(id),
        stage TEXT NOT NULL, detail TEXT NOT NULL, created TEXT NOT NULL)""")
    db.execute("CREATE INDEX IF NOT EXISTS pipeline_birth ON pipeline_events(birth,id)")
    db.execute("""CREATE TABLE IF NOT EXISTS screening_workers(
        birth INTEGER PRIMARY KEY REFERENCES births(id),pid INTEGER NOT NULL,log TEXT NOT NULL,status TEXT NOT NULL)""")
    db.execute("""CREATE TABLE IF NOT EXISTS pipeline_startup(
        id INTEGER PRIMARY KEY, batch INTEGER NOT NULL, task TEXT NOT NULL,
        kind TEXT NOT NULL, label TEXT NOT NULL, status TEXT NOT NULL, detail TEXT NOT NULL,
        birth INTEGER REFERENCES births(id), pid INTEGER NOT NULL, log TEXT,
        started TEXT, completed TEXT, UNIQUE(batch,task))""")
    db.commit()


def begin_startup(archive):
    ensure_schema(archive.db)
    batch = archive.db.execute("SELECT coalesce(max(batch),0)+1 FROM pipeline_startup").fetchone()[0]
    queue_startup(archive, batch, [
        ("candidates", "startup", "Preparing immutable candidates"),
        ("observations", "startup", "Restoring retained measurements"),
        ("contributions", "startup", "Admitting user contributions"),
        ("recovery", "startup", "Recovering interrupted candidate work"),
    ])
    return batch


def queue_startup(archive, batch, tasks):
    log = None
    if archive.db.execute("SELECT 1 FROM sqlite_master WHERE name='episodes'").fetchone():
        episode = archive.db.execute("SELECT log FROM episodes ORDER BY id DESC LIMIT 1").fetchone()
        log = episode["log"] if episode else None
    with archive.db:
        archive.db.executemany("""
            INSERT INTO pipeline_startup(batch,task,kind,label,status,detail,pid,log)
            VALUES (?,?,?,?,'queued','',?,?)
        """, [(batch, task, kind, label, os.getpid(), log) for task, kind, label in tasks])


def update_startup(archive, batch, task, status, detail="", birth=None):
    if status not in ("running", "complete", "rejected", "failed"):
        raise AdapterError(f"Invalid startup outcome: {status}.")
    timestamp = datetime.now(timezone.utc).isoformat()
    with archive.db:
        row = archive.db.execute("SELECT status FROM pipeline_startup WHERE batch=? AND task=?",
                                 (batch, task)).fetchone()
        expected = "queued" if status == "running" else "running"
        if row is None or row["status"] != expected:
            raise AdapterError(f"Startup task {task} cannot transition to {status}.")
        archive.db.execute("""
            UPDATE pipeline_startup SET status=?,detail=?,birth=?,
                started=coalesce(started,?),completed=? WHERE batch=? AND task=?
        """, (status, detail, birth, timestamp, None if status == "running" else timestamp, batch, task))


@contextmanager
def startup_step(archive, batch, task):
    update_startup(archive, batch, task, "running")
    outcome = {"status": "complete", "detail": "", "birth": None}
    completed = False
    try:
        yield outcome
        completed = True
    finally:
        error = None if completed else sys.exc_info()[1]
        update_startup(archive, batch, task, "failed" if error is not None else outcome["status"],
                       str(error) if error is not None else outcome["detail"], outcome["birth"])


def startup_snapshot(db, running, directory=None):
    if not db.execute("SELECT 1 FROM sqlite_master WHERE name='pipeline_startup'").fetchone():
        return {"tracked": False, "items": [], "counts": {}}
    rows = db.execute("""
        SELECT * FROM pipeline_startup WHERE batch=(SELECT max(batch) FROM pipeline_startup) ORDER BY id
    """).fetchall()
    current = datetime.now(timezone.utc)
    items, counts = [], Counter()
    for row in rows:
        status = row["status"]
        if not running and status in ("queued", "running"):
            status = "interrupted"
        elapsed = None
        if row["started"] and (row["completed"] or status == "running"):
            end = datetime.fromisoformat(row["completed"]) if row["completed"] else current
            elapsed = max(0, (end-datetime.fromisoformat(row["started"])).total_seconds())
        counts[status] += 1
        items.append({
            "key": f"startup-{row['batch']}-{row['task']}", "label": row["label"], "kind": row["kind"],
            "status": status, "detail": row["detail"], "birth": row["birth"],
            "history": f"history/{row['birth']:04d}.html" if row["birth"] is not None else None,
            "pid": row["pid"], "log": row["log"], "started": row["started"],
            "completed": row["completed"], "elapsedSeconds": elapsed,
        })
        if directory is not None and row["kind"] == "admission":
            items[-1]["screening"] = screening_progress(
                directory / "user-admissions" / row["task"], running=running and status == "running")
            birth = db.execute("SELECT design FROM births WHERE id=?", (row["birth"],)).fetchone()
            items[-1]["visuals"] = visuals(directory, birth["design"]) if birth else {}
    return {"tracked": bool(rows), "items": items, "counts": dict(counts)}


def record(archive, birth, stage, detail=""):
    if stage not in {*STAGES, "rejected"}:
        raise AdapterError(f"Unknown pipeline stage: {stage}.")
    with archive.db:
        archive.db.execute("INSERT INTO pipeline_events(birth,stage,detail,created) VALUES (?,?,?,?)",
                           (birth, stage, detail, datetime.now(timezone.utc).isoformat()))


def screening_observer(archive):
    def observe(design, report=None):
        row = archive.db.execute("""
            SELECT b.id FROM births b WHERE b.design=? AND b.status='generated'
            AND NOT EXISTS(SELECT 1 FROM evaluations e WHERE e.birth=b.id)
            AND NOT EXISTS(SELECT 1 FROM reused_evaluations r WHERE r.birth=b.id)
            ORDER BY b.id DESC LIMIT 1""", (design,)).fetchone()
        if row is None:
            return
        if report is None:
            record(archive, row["id"], "screening", "Load-only reconstruction and initial-contact checks.")
        else:
            status = report.get("previousStatus", report["status"])
            outcomes = {"ready-for-dynamics": "screened", "deferred": "queued",
                        "reconstructed-not-loaded": "queued", "rejected": "rejected"}
            if status not in outcomes:
                raise AdapterError(f"Unknown screening outcome: {status}.")
            stage = outcomes[status]
            record(archive, row["id"], stage, rejection_detail(report) if stage == "rejected"
                   else report.get("error", status))
    return observe


def visuals(directory, design):
    from .gallery import media_directory, relative

    root = directory / "designs" / design
    image = media_directory(root) / "walker-v2.png"
    diagram = root / "candidate/cycle.svg"
    page = directory / "index.html"
    return {
        "assembly": relative(image, page) if image.is_file() else None,
        "diagram": relative(diagram, page) if diagram.is_file() else None,
    }


def qualified_walkers(db, directory, *, prefix="", version=None):
    from .gallery import display_result, relative

    items = []
    for row in db.execute("""
        SELECT e.id AS trial,e.birth,e.result,e.completed,b.design,d.category,d.genome,p.definition
        FROM evaluations e JOIN births b ON b.id=e.birth JOIN designs d ON d.id=b.design
        JOIN protocols p ON p.id=e.protocol JOIN run r ON r.protocol=e.protocol
        WHERE e.status='complete' ORDER BY e.id DESC
    """):
        result = display_result(row, json.loads(row["definition"]))
        if not is_qualified(result):
            continue
        video = directory / "designs" / row["design"] / "render/walker-v2-anchored.mp4"
        items.append({
            "key": f"qualified-{version or 'local'}-{row['birth']}", "birth": row["birth"],
            "trial": row["trial"], "design": row["design"], "category": row["category"],
            "stage": "qualified", "history": f"history/{row['birth']:04d}.html",
            "visuals": visuals(directory, row["design"]), "prefix": prefix, "version": version,
            "updated": row["completed"],
            "qualification": {"qualified": True, "rejectionReasons": []},
            "speedMmPerSecond": 1000 * result["metrics"]["speedMetersPerSecond"],
            "forwardMeters": result["metrics"]["forwardMeters"],
            "video": relative(video, directory / "index.html") if video.is_file() else None,
        })
    return items


def recent_qualified(items, limit=12):
    by_design = {}
    for item in sorted(items, key=lambda item: (item["updated"] or "", item["birth"]), reverse=True):
        by_design.setdefault(item["design"], item)
    return {"count": len(by_design), "items": list(by_design.values())[:limit]}


def compact_walker(item, names):
    return {
        **{key: item[key] for key in ("key", "birth", "history", "stage", "visuals")},
        "name": (f"Proposal {item['proposal']} (parent B{item['birth']})" if item.get("proposal")
                 else names.get(item["birth"], f"Birth {item['birth']}")),
        "parentPreview": bool(item.get("parentPreview")),
        "qualification": item.get("qualification"),
    }


def snapshot(db, directory, *, running, tracked, limit=8):
    from .gallery import display_result

    rejected = rejections(directory)
    protocol = json.loads(db.execute("""
        SELECT p.definition FROM protocols p JOIN run r ON r.protocol=p.id WHERE r.id=1""").fetchone()[0])
    attempts = ("simulation_attempts" if db.execute(
        "SELECT 1 FROM sqlite_master WHERE name='simulation_attempts'").fetchone()
        else "(SELECT NULL AS id,NULL AS birth,NULL AS status,NULL AS error,NULL AS pid,NULL AS log WHERE 0)")
    rows = db.execute(f"""
        SELECT b.id,b.design,b.status,b.parent,b.error,b.created,d.category,d.genome,
               e.id AS trial,e.status AS trial_status,e.error AS trial_error,e.result AS trial_result,
               coalesce(w.pid,a.pid,CASE WHEN s.status='running' THEN s.pid END) AS pid,
               coalesce(w.log,a.log,s.log) AS log,p.stage AS pipeline_stage,p.detail AS pipeline_detail,
               a.id AS gate_attempt,a.status AS gate_status,a.error AS gate_error,
               j.status AS preview_status,j.error AS preview_error,
               r.status AS animation_status,e.completed AS measured_at,p.created AS stage_at
        FROM births b JOIN designs d ON d.id=b.design
        LEFT JOIN evaluations e ON e.birth=b.id
        LEFT JOIN {attempts} a ON a.birth=b.id
        LEFT JOIN trial_workers w ON w.trial=e.id
        LEFT JOIN screening_workers s ON s.birth=b.id
        LEFT JOIN pipeline_events p ON p.id=(SELECT max(id) FROM pipeline_events WHERE birth=b.id)
        LEFT JOIN assembly_previews j ON j.design=b.design
        LEFT JOIN render_jobs r ON r.evaluation=e.id
        WHERE NOT EXISTS(SELECT 1 FROM reused_evaluations o WHERE o.birth=b.id)
        ORDER BY b.id DESC""").fetchall()
    groups = {stage: [] for stage in STAGES}
    recent, counts = [], Counter()
    names = {}
    if db.execute("SELECT 1 FROM sqlite_master WHERE name='user_pool_entries'").fetchone():
        for row in db.execute("SELECT birth,provenance FROM user_pool_entries WHERE birth IS NOT NULL ORDER BY contribution"):
            names.setdefault(row["birth"], json.loads(row["provenance"])["label"])
    repairs = {}
    for child in db.execute("""
        SELECT id,parent,status FROM births
        WHERE operation='repair-realization' AND parent IS NOT NULL ORDER BY id
    """):
        repairs.setdefault(child["parent"], []).append({
            "birth": child["id"], "status": child["status"],
            "history": f"history/{child['id']:04d}.html",
        })
    proposals, by_design = {}, {}
    for proposal in db.execute("SELECT id,parent,status,summary,error FROM native_proposals ORDER BY id DESC LIMIT 64"):
        summary = json.loads(proposal["summary"]) if proposal["summary"] else {}
        progress = construction_progress(directory, proposal["id"], summary)
        proposals[proposal["id"]] = (proposal, progress)
        for generated in summary.get("generated", []):
            by_design.setdefault((proposal["parent"], generated["genotypeId"]), progress)
    for row in rows:
        stage = row["pipeline_stage"] or "queued"
        if row["trial_status"] == "running":
            stage = "simulating"
        elif row["animation_status"] == "rendering":
            stage = "rendering"
        elif row["trial_status"] is not None:
            stage = row["trial_status"]
        elif row["gate_status"] is not None:
            stage = "gating" if row["gate_status"] == "running" else row["gate_status"]
        elif row["status"] != "generated":
            stage = row["status"]
        elif stage in ("screening", "screened", "preflight", "ready") and not running:
            stage = "paused"
        detail = row["pipeline_detail"] if stage == row["pipeline_stage"] else {
            "simulating": "Simulation running.",
            "gating": "Cheap prefix running; no full-simulation slot reserved.",
            "complete": "Simulation completed.",
            "rendering": "Rendering simulation animation.",
            "paused": "Pipeline paused.",
        }.get(stage, "")
        item = {
            "key": f"birth-{row['id']}", "birth": row["id"], "design": row["design"],
            "category": row["category"], "stage": stage, "trial": row["trial"],
            "pid": row["pid"], "log": row["log"],
            "detail": row["trial_error"] or row["gate_error"] or row["error"] or detail or "",
            "gateAttempt": row["gate_attempt"],
            "visuals": visuals(directory, row["design"]),
            "previewStatus": row["preview_status"],
            "previewError": row["preview_error"],
            "history": f"history/{row['id']:04d}.html",
            "updated": row["measured_at"] or row["stage_at"] or row["created"],
            "qualification": None,
            "construction": by_design.get((row["parent"], row["design"])),
            "repairs": repairs.get(row["id"], []),
        }
        for folder in (directory / "preflight-isolated" / f"{row['id']:06d}", directory / "preflight"):
            progress = screening_progress(folder, genotype=row["design"],
                                          running=running and stage in ("screening", "preflight"))
            if progress["checks"]:
                item["screening"] = progress
                break
        if row["trial_status"] == "complete":
            if row["trial_result"] is None:
                raise AdapterError(f"Completed trial {row['trial']} has no recorded result.")
            result = display_result({"result": row["trial_result"], "genome": row["genome"]}, protocol)
            item["qualification"] = {
                "qualified": is_qualified(result),
                "rejectionReasons": result["metrics"]["rankingRejectionReasons"],
            }
        elif row["gate_status"] == "rejected":
            path = directory / "simulation-attempts" / f"{row['gate_attempt']:04d}" / "simulation-gate.json"
            report = json.loads(path.read_text())
            reasons = report.get("reasons")
            if report.get("status") != "rejected" or not isinstance(reasons, list) or not reasons:
                raise AdapterError(f"Cheap attempt {row['gate_attempt']} has no valid rejection evidence.")
            item["qualification"] = {"qualified": False, "rejectionReasons": reasons}
            item["report"] = str(path.relative_to(directory))
        if row["status"] == "screening-rejected" or (stage == "rejected" and row["gate_status"] != "rejected"):
            if info := rejected.get(row["design"]):
                item["detail"] = info["detail"]
                item["report"] = info["report"]
        counts[stage] += 1
        if stage in groups:
            groups[stage].append(item)
        else:
            recent.append(item)
    for row in db.execute("""
        SELECT p.id,p.parent,p.summary,p.status,p.error,p.created,d.id AS design,d.category FROM native_proposals p
        JOIN births b ON b.id=p.parent JOIN designs d ON d.id=b.design
        ORDER BY p.id DESC LIMIT 64"""):
        summary = json.loads(row["summary"]) if row["summary"] else {}
        if row["status"] != "running" and summary.get("generated"):
            continue
        progress = proposals[row["id"]][1]
        stage = (progress["events"][-1]["stage"] if progress["tracked"] else "generating") if running else "paused"
        if row["status"] != "running":
            stage = row["status"]
        counts[stage] += 1
        item = {
            "key": f"proposal-{row['id']}", "proposal": row["id"], "birth": row["parent"],
            "design": row["design"], "category": row["category"], "stage": stage,
            "detail": row["error"] or (progress["events"][-1]["detail"] if progress["tracked"]
                                      else summary.get("operation", "Selecting and realizing a mutation.")),
            "visuals": visuals(directory, row["design"]), "parentPreview": True,
            "history": f"forest.html#proposal-{row['id']}", "log": None, "pid": None,
            "previewStatus": None, "previewError": None,
            "construction": progress, "updated": row["created"],
        }
        if row["status"] == "running":
            groups[stage].append(item)
        else:
            recent.append(item)
    recent.sort(key=lambda item: item["updated"], reverse=True)
    return {
        "tracked": tracked,
        "qualified": recent_qualified(qualified_walkers(db, directory)),
        "startup": startup_snapshot(db, running, directory),
        "stages": [{"id": stage, "label": label, "count": counts[stage],
                    "items": sorted(groups[stage], key=lambda item: item["birth"])[:limit],
                    "walkers": [compact_walker(item, names) for item in
                                sorted(groups[stage], key=lambda item: item["birth"])]}
                   for stage, label in STAGES.items()],
        "recent": recent[:limit],
        "finished": {"count": len(recent), "walkers": [compact_walker(item, names) for item in recent]},
    }
