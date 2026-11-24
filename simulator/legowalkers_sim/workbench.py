"""Persistent human-guided branches; no dynamic evaluation or trial authorization."""

import contextlib
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import uuid

from .bridge import AdapterError, load_manifest, write_json
from .evolution import Archive, canonical, locked, now, validate_search_manifest
from .bridge import logical_foot_count
from .native_episode import _queue
from .screening import GATE_ORDER, screen_batch
from .screening_reports import screening_progress
from .user_pool import POLICY


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch("[a-f0-9]{32}", value):
        raise AdapterError("Unknown workspace or node identifier.")
    return value


def text(value, name, limit):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise AdapterError(f"{name} must contain 1-{limit} characters.")
    return value.strip()


def batch(candidate):
    manifest = load_manifest(candidate / "manifest.json")
    motion = manifest["motion"]
    write_json(candidate.parent / "candidates.json", {
        "schemaVersion": 1, "gateVersion": "motion-gates-v1", "mechanismId": motion["mechanismId"],
        "gates": json.loads((candidate / "gates.json").read_text()), "rejected": [],
        "generated": [{"directory": "candidate", "genotypeId": motion["genotypeId"],
                       "assemblyId": motion["assemblyId"]}],
        "counts": {"proposals": 1, "realizationAttempts": 1, "exported": 1, "duplicates": 0,
                   "rejected": 0, "dynamicTrials": 0}})
    return _queue(candidate.parent)


def comparison(parent, child):
    before, after = (load_manifest(path / "manifest.json") for path in (parent, child))
    old, new = ({part["id"]: part for part in manifest["parts"]} for manifest in (before, after))
    return {
        "motionUnchanged": json.loads((parent / "program.json").read_text()) == json.loads((child / "program.json").read_text()),
        "parts": [{"id": name, "before": old.get(name), "after": new.get(name)}
                  for name in sorted(old.keys() | new.keys()) if old.get(name) != new.get(name)],
    }


class Workbench:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        with self.connect() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS workspaces(
                    id TEXT PRIMARY KEY,label TEXT NOT NULL,author TEXT NOT NULL,
                    created TEXT NOT NULL,head TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS nodes(
                    id TEXT PRIMARY KEY,workspace TEXT NOT NULL,parent TEXT,
                    status TEXT NOT NULL,summary TEXT NOT NULL,created TEXT NOT NULL,
                    source_archive TEXT NOT NULL,source_birth INTEGER NOT NULL);
                CREATE TABLE IF NOT EXISTS contributions(
                    id TEXT PRIMARY KEY,node TEXT UNIQUE NOT NULL,label TEXT NOT NULL,
                    author TEXT NOT NULL,note TEXT NOT NULL,active INTEGER NOT NULL,
                    validation TEXT NOT NULL,files TEXT NOT NULL,created TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS contribution_checks(
                    id TEXT PRIMARY KEY,node TEXT NOT NULL,label TEXT NOT NULL,
                    phase TEXT NOT NULL,status TEXT NOT NULL,detail TEXT NOT NULL,
                    outcomes TEXT NOT NULL,report TEXT,pid INTEGER NOT NULL,
                    started TEXT NOT NULL,completed TEXT);
            """)

    @contextlib.contextmanager
    def connect(self):
        with contextlib.closing(sqlite3.connect(self.root / "workbench.sqlite3", timeout=30)) as db:
            db.row_factory = sqlite3.Row
            with db:
                yield db

    def candidate(self, node):
        return self.root / "designs" / identifier(node) / "candidate"

    def node(self, db, node, *, editable=False):
        row = db.execute("SELECT * FROM nodes WHERE id=?", (identifier(node),)).fetchone()
        if row is None:
            raise AdapterError("Unknown workbench node.")
        if editable and row["status"] in ("rejected", "discarded"):
            raise AdapterError("Select a retained or draft physical candidate, not a discarded/rejected edit.")
        return dict(row)

    def state(self):
        with self.connect() as db:
            nodes = []
            for row in db.execute("SELECT * FROM nodes ORDER BY created,id"):
                node = dict(row)
                node["summary"] = json.loads(node["summary"])
                path = self.candidate(node["id"])
                node["physical"] = (path / "manifest.json").is_file()
                node["preview"] = (path.parent / "assembly-render/walker-v2.png").is_file()
                node["clearance"] = (path / "motion-clearance.json").is_file()
                nodes.append(node)
            return {"workspaces": [dict(row) for row in db.execute("SELECT * FROM workspaces ORDER BY created DESC")],
                    "nodes": nodes, "contributions": [dict(row) for row in db.execute(
                        "SELECT id,node,label,author,note,active,created FROM contributions ORDER BY created")],
                    "policy": POLICY, "dynamicTrials": 0, "checks": self.checks()}

    def _recover_checks(self, db):
        for row in db.execute("SELECT id,pid FROM contribution_checks WHERE status='running'").fetchall():
            try:
                os.kill(row["pid"], 0)
            except ProcessLookupError:
                db.execute("""UPDATE contribution_checks SET status='interrupted',completed=?,detail=?
                    WHERE id=? AND status='running'""",
                           (now(), "Admission worker stopped before completion. Not admitted; submit again to retry.",
                            row["id"]))

    def checks(self):
        with self.connect() as db:
            self._recover_checks(db)
            rows = db.execute("""SELECT * FROM contribution_checks
                ORDER BY (status='running') DESC,started DESC LIMIT 20""").fetchall()
            result = []
            for row in rows:
                item = dict(row)
                item["outcomes"] = json.loads(item["outcomes"])
                end = datetime.fromisoformat(item["completed"]) if item["completed"] else datetime.now(timezone.utc)
                item["elapsedSeconds"] = max(0, (end-datetime.fromisoformat(item["started"])).total_seconds())
                validation = self.root / "validations" / item["node"]
                exact_check = validation / item["id"]
                item["screening"] = screening_progress(
                    exact_check if exact_check.exists() or item["status"] == "running" else validation,
                    running=item["status"] == "running")
                result.append(item)
            return result

    def _check_progress(self, check, phase, detail, *, status="running", outcomes=None, report=None):
        with self.connect() as db:
            changed = db.execute("""UPDATE contribution_checks SET phase=?,status=?,detail=?,
                outcomes=COALESCE(?,outcomes),report=COALESCE(?,report),completed=?
                WHERE id=? AND status='running'""",
                (phase, status, detail, canonical(outcomes) if outcomes is not None else None, report,
                 now() if status != "running" else None, check))
            if changed.rowcount != 1:
                raise AdapterError("The contribution check is no longer running.")

    def fork(self, archive, birth, label, author):
        label, author = text(label, "Workspace name", 100), text(author, "Contributor", 80)
        if type(birth) is not int or birth <= 0:
            raise AdapterError("A positive birth number is required.")
        archive = Path(archive).resolve()
        with contextlib.closing(sqlite3.connect(f'{(archive / "archive.sqlite3").as_uri()}?mode=ro', uri=True)) as db:
            row = db.execute("SELECT design FROM births WHERE id=?", (birth,)).fetchone()
        if row is None:
            raise AdapterError("The source birth does not exist.")
        source = archive / "designs" / row[0] / "candidate"
        manifest = load_manifest(source / "manifest.json")
        validate_search_manifest(manifest, logical_foot_count(manifest))
        workspace, node = uuid.uuid4().hex, uuid.uuid4().hex
        with locked(self.root):
            shutil.copytree(source, self.candidate(node))
            batch(self.candidate(node))
            from .gallery import media_directory
            image = media_directory(source.parent) / "walker-v2.png"
            if image.is_file():
                target = self.candidate(node).parent / "assembly-render"
                target.mkdir()
                shutil.copy2(image, target / image.name)
            summary = {"operation": "fork", "sourceArchive": str(archive), "sourceBirth": birth,
                       "genotypeId": manifest["motion"]["genotypeId"], "dynamicTrials": 0}
            with self.connect() as db:
                db.execute("INSERT INTO workspaces VALUES(?,?,?,?,?)", (workspace, label, author, now(), node))
                db.execute("INSERT INTO nodes VALUES(?,?,NULL,'kept',?,?,?,?)",
                           (node, workspace, canonical(summary), now(), str(archive), birth))
        return {"workspace": workspace, "node": node}

    def rebase(self, node, archive, birth):
        if type(birth) is not int or birth <= 0:
            raise AdapterError("A positive birth number is required.")
        archive = Path(archive).resolve()
        with contextlib.closing(sqlite3.connect(f'{(archive / "archive.sqlite3").as_uri()}?mode=ro', uri=True)) as db:
            target = db.execute("SELECT design FROM births WHERE id=? AND status!='duplicate'", (birth,)).fetchone()
        if target is None:
            raise AdapterError("The rebase source birth does not exist or is a duplicate.")
        anchor = archive / "designs" / target[0] / "candidate"
        manifest = load_manifest(anchor / "manifest.json")
        validate_search_manifest(manifest, logical_foot_count(manifest))
        with locked(self.root), self.connect() as db:
            parent = self.node(db, node, editable=True)
            source = self.candidate(node)
            manifest = load_manifest(source / "manifest.json")
            validate_search_manifest(manifest, logical_foot_count(manifest))
            child = uuid.uuid4().hex
            shutil.copytree(source, self.candidate(child))
            batch(self.candidate(child))
            preview = source.parent / "assembly-render"
            if preview.is_dir():
                shutil.copytree(preview, self.candidate(child).parent / "assembly-render")
            summary = {"operation": "rebase", "sourceArchive": str(archive), "sourceBirth": birth,
                       "previousSourceArchive": parent["source_archive"], "previousSourceBirth": parent["source_birth"],
                       "genotypeId": manifest["motion"]["genotypeId"], "ldrawSha256": manifest["ldrawSha256"],
                       "dynamicTrials": 0, "fitnessImported": False}
            db.execute("INSERT INTO nodes VALUES(?,?,?,'kept',?,?,?,?)",
                       (child, parent["workspace"], node, canonical(summary), now(), str(archive), birth))
            db.execute("UPDATE workspaces SET head=? WHERE id=?", (child, parent["workspace"]))
        return {"workspace": parent["workspace"], "node": child, "summary": summary}

    def edits(self, node, generator):
        with self.connect() as db:
            self.node(db, node, editable=True)
        return generator.manual_edits(self.candidate(node))

    def preview(self, node, edit, generator, library):
        from .workbench_diagrams import model_diagram
        if not isinstance(edit, str) or not re.fullmatch("[a-f0-9]{64}", edit):
            raise AdapterError("Select an edit from the current catalog.")
        with locked(self.root):
            with self.connect() as db:
                self.node(db, node, editable=True)
            parent = self.candidate(node)
            available = {entry["id"] for entry in generator.manual_edits(parent)["edits"]}
            if edit not in available:
                raise AdapterError("This edit is unavailable; refresh the catalog.")
            with tempfile.TemporaryDirectory(prefix="walker-option-") as temporary:
                output = Path(temporary) / "proposal"
                summary = generator.apply_edit(parent, output, edit)
                alternatives = []
                for entry in summary["generated"]:
                    candidate = (output / entry["directory"]).resolve()
                    if candidate.parent != output.resolve():
                        raise AdapterError("Manual edit returned an unsafe candidate directory.")
                    evidence = {}
                    for key, name in (("clearance", "motion-clearance.json"), ("clearanceRepair", "motion-clearance-repair.json")):
                        report = candidate / name
                        if report.is_file():
                            evidence[key] = json.loads(report.read_text())
                    alternatives.append({"diagram": model_diagram(candidate, library),
                                         "comparison": comparison(parent, candidate), **evidence})
                return {"parent": model_diagram(parent, library), "alternatives": alternatives,
                        "summary": summary, "dynamicTrials": 0}

    def apply(self, node, edit, generator):
        if not isinstance(edit, str) or not re.fullmatch("[a-f0-9]{64}", edit):
            raise AdapterError("Select an edit from the current catalog.")
        with locked(self.root), self.connect() as db:
            parent = self.node(db, node, editable=True)
            available = {entry["id"] for entry in generator.manual_edits(self.candidate(node))["edits"]}
            if edit not in available:
                raise AdapterError("This edit is unavailable; refresh the catalog.")
            output = self.root / "requests" / uuid.uuid4().hex
            summary = generator.apply_edit(self.candidate(node), output, edit)
            children = []
            for entry in summary["generated"] or [None]:
                child = uuid.uuid4().hex
                status = "rejected"
                if entry is not None:
                    source = (output / entry["directory"]).resolve()
                    if source.parent != output.resolve():
                        raise AdapterError("Manual edit returned an unsafe candidate directory.")
                    shutil.copytree(source, self.candidate(child))
                    batch(self.candidate(child))
                    status = "draft"
                evidence = {**summary, "comparison": comparison(self.candidate(node), self.candidate(child))} if entry else summary
                db.execute("INSERT INTO nodes VALUES(?,?,?,?,?,?,?,?)",
                           (child, parent["workspace"], node, status, canonical(evidence), now(),
                            parent["source_archive"], parent["source_birth"]))
                children.append(child)
        return {"children": children, "summary": summary}

    def select(self, node, action):
        with locked(self.root), self.connect() as db:
            row = self.node(db, node, editable=True)
            if action == "discard":
                if row["parent"] is None:
                    raise AdapterError("The fork root cannot be discarded.")
                if db.execute("SELECT 1 FROM contributions WHERE node=? AND active=1", (node,)).fetchone():
                    raise AdapterError("Withdraw the active contribution before discarding its node.")
                if db.execute("SELECT 1 FROM contribution_checks WHERE node=? AND status='running'", (node,)).fetchone():
                    raise AdapterError("This exact version is being checked; wait before discarding it. Other versions remain editable.")
                db.execute("UPDATE nodes SET status='discarded' WHERE id=?", (node,))
                db.execute("UPDATE workspaces SET head=? WHERE id=? AND head=?",
                           (row["parent"], row["workspace"], node))
            elif action in ("keep", "checkout"):
                if action == "keep":
                    db.execute("UPDATE nodes SET status='kept' WHERE id=?", (node,))
                db.execute("UPDATE workspaces SET head=? WHERE id=?", (node, row["workspace"]))
            else:
                raise AdapterError("Unknown branch action.")
        return {"node": node, "action": action}

    def contribute(self, node, label, author, note, library, payload_profile=None, *, target_archive=None):
        label, author = text(label, "Contribution name", 100), text(author, "Contributor", 80)
        if not isinstance(note, str) or len(note) > 1000:
            raise AdapterError("Contribution note must be at most 1000 characters.")
        with locked(self.root), self.connect() as db:
            self._recover_checks(db)
            if db.execute("SELECT 1 FROM contribution_checks WHERE status='running'").fetchone():
                raise AdapterError("A contribution check is already running. Continue editing; wait before submitting another.")
            self._contribution_eligibility(db, node, target_archive)
            candidate = self.candidate(node)
            _, files = _queue(candidate.parent)
            stored = [{**gate, "source": "saved-generation"}
                      for gate in json.loads((candidate / "gates.json").read_text())]
            check = uuid.uuid4().hex
            db.execute("INSERT INTO contribution_checks VALUES(?,?,?,?,?,?,?,?,?,?,NULL)",
                       (check, node, label, "inputs", "running",
                        "Saved version, identities and admission eligibility checked. Reading saved construction outcomes.",
                        canonical(stored), None, os.getpid(), now()))
        finished = False
        phase = "checking"
        try:
            self._check_progress(check, phase,
                "Reconstructing LEGO parts, connectors and rigid bodies; checking mass layout, initial collisions "
                "and hinge closure at time zero. These are combined static gates, not a walking simulation.")
            validation_root = self.root / "validations" / node
            validation = screen_batch(candidate.parent, validation_root / check,
                                      library, max_checks=1, payload_profile=payload_profile)
            report, = validation["results"]
            outcomes = [{**gate, "source": "saved-generation" if gate["gate"] in GATE_ORDER else "admission-check"}
                        for gate in report.get("gates", stored)]
            ready = report["status"] == "ready-for-dynamics" or (
                report["status"] == "duplicate" and report.get("previousStatus") == "ready-for-dynamics")
            self._check_progress(check, phase, "Static gate results received; saving validation evidence.",
                                 outcomes=outcomes)
            write_json(validation_root / "latest.json", validation)
            report_url = f"/workbench-assets/{node}/contribution-validation.json"
            if not ready:
                from .screening_reports import rejection_detail
                detail = "Contribution failed static validation: " + rejection_detail(report)
                self._check_progress(check, phase, detail, status="rejected", outcomes=outcomes, report=report_url)
                finished = True
                raise AdapterError(detail)
            phase = "admitting"
            self._check_progress(check, phase,
                "Static checks passed. Waiting for any workspace edit, then rechecking the unchanged submitted "
                "version and pool eligibility before admission.", outcomes=outcomes, report=report_url)
            # Only the final commit waits for the workspace lock; reconstruction never owns it.
            with (self.root / ".runner.lock").open("a") as handle:
                fcntl.flock(handle, fcntl.LOCK_EX)
                try:
                    with self.connect() as db:
                        prior = self._contribution_eligibility(db, node, target_archive)
                        if _queue(candidate.parent)[1] != files:
                            raise AdapterError("Submitted design files changed during validation; not admitted. Submit again.")
                        contribution = prior["id"] if prior else uuid.uuid4().hex
                        db.execute("""INSERT INTO contributions VALUES(?,?,?,?,?,1,?,?,?)
                            ON CONFLICT(node) DO UPDATE SET label=excluded.label,author=excluded.author,note=excluded.note,
                            active=1,validation=excluded.validation,files=excluded.files""",
                            (contribution, node, label, author, note, canonical(validation), canonical(files), now()))
                        db.execute("UPDATE nodes SET status='kept' WHERE id=?", (node,))
                        db.execute("""UPDATE contribution_checks SET phase='admitted',status='complete',detail=?,
                            completed=? WHERE id=?""",
                            ("Admitted without fitness. Evaluation requires an explicitly resumed episode; no trial was spent.",
                             now(), check))
                finally:
                    fcntl.flock(handle, fcntl.LOCK_UN)
            finished = True
            return {"contribution": contribution, "status": "unmeasured", "dynamicTrials": 0, "check": check}
        finally:
            if not finished:
                error = sys.exc_info()[1]
                self._check_progress(check, phase,
                    str(error) if error is not None else "Admission interrupted before completion; not admitted.",
                    status="failed")

    def _contribution_eligibility(self, db, node, target_archive=None):
        source = self.node(db, node, editable=True)
        if target_archive is not None:
            from .upgrades import _origin
            from .user_pool import founder_origins
            archive = Archive(target_archive)
            try:
                if _origin(source["source_archive"], source["source_birth"]) not in founder_origins(archive):
                    raise AdapterError("Source founder family is not retained; rebase this saved design onto a current campaign candidate.")
            finally:
                archive.db.close()
        prior = db.execute("SELECT * FROM contributions WHERE node=?", (node,)).fetchone()
        if prior and prior["active"]:
            raise AdapterError("This node is already an active contribution.")
        if db.execute("SELECT count(*) FROM contributions WHERE active=1").fetchone()[0] >= POLICY["capacity"]:
            raise AdapterError("User pool is full; withdraw an entry before contributing another.")
        manifest = load_manifest(self.candidate(node) / "manifest.json")
        validate_search_manifest(manifest, logical_foot_count(manifest))
        for active in db.execute("SELECT label,validation FROM contributions WHERE active=1"):
            # Compare the admitted identity without requiring historical simulator compatibility.
            results = json.loads(active["validation"]).get("results")
            if not isinstance(results, list) or len(results) != 1 or not isinstance(results[0], dict):
                raise AdapterError(f"Saved admission identity is invalid for {active['label']}.")
            genotype = results[0].get("genotypeId")
            if not isinstance(genotype, str) or not re.fullmatch("[a-f0-9]{64}", genotype):
                raise AdapterError(f"Saved admission identity is invalid for {active['label']}.")
            if genotype == manifest["motion"]["genotypeId"]:
                raise AdapterError(f"This design is already active as {active['label']}.")
        return prior

    def withdraw(self, contribution):
        with locked(self.root), self.connect() as db:
            result = db.execute("UPDATE contributions SET active=0 WHERE id=? AND active=1",
                                (identifier(contribution),))
            if result.rowcount != 1:
                raise AdapterError("Unknown or already withdrawn contribution.")
        return {"withdrawn": contribution}

    def render(self, node, library, blender):
        with self.connect() as db:
            self.node(db, node, editable=True)
        if blender is None:
            raise AdapterError("Blender is unavailable; motion diagrams and model downloads remain available.")
        from .previews import render
        from .evolution import ROOT
        import fcntl
        lock_path = ROOT / "output/.assembly-preview.lock"
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with lock_path.open("a") as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as error:
                    raise AdapterError("Another assembly preview is rendering; retry when it finishes.") from error
                try:
                    render(self.root, identifier(node), library, blender)
                finally:
                    fcntl.flock(lock, fcntl.LOCK_UN)
        except subprocess.SubprocessError as error:
            raise AdapterError(f"Assembly preview failed: {error}; see assembly-preview.log.") from error
        return {"node": node, "simulationSteps": 0}
