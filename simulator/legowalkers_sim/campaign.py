"""A stable experiment identity over immutable, independently reproducible versions."""

import contextlib
import json
import logging
import os
from pathlib import Path
import re
import secrets
import sqlite3
import subprocess
import threading
from html.parser import HTMLParser
from urllib.parse import quote, urljoin
import uuid

from .bridge import AdapterError, load_manifest
from .evolution import canonical, evidence_invalidation, is_qualified, locked, now
from .gallery import atomic_text, document, esc
from .observations import PHYSICS_SOURCES, compatible_evaluation
from .pipeline import qualified_walkers, recent_qualified
from .upgrades import prepare


@contextlib.contextmanager
def database(directory):
    with contextlib.closing(sqlite3.connect(f"file:{Path(directory) / 'archive.sqlite3'}?mode=ro", uri=True)) as db:
        db.row_factory = sqlite3.Row
        yield db


def definition(directory):
    with database(directory) as db:
        return json.loads(db.execute("SELECT p.definition FROM protocols p JOIN run r ON r.protocol=p.id").fetchone()[0])


def require_active(directory):
    marker = Path(directory) / "campaign-version.json"
    if marker.exists():
        link = json.loads(marker.read_text())
        state = json.loads((Path(link["campaign"]) / "campaign.json").read_text())
        if state["active"] != link["version"]:
            raise AdapterError("This is a read-only historical version. Continue from the stable experiment dashboard.")
        version = next(v for v in state["versions"] if v["id"] == link["version"])
        if Path(version["archive"]).resolve() != Path(directory).resolve():
            raise AdapterError("Experiment version registration does not match this archive.")


def idle(directory):
    with database(directory) as db:
        tables = {row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        for table in ("episodes", "render_worker"):
            if table not in tables:
                continue
            job = db.execute(f"SELECT status,pid FROM {table} ORDER BY id DESC LIMIT 1").fetchone()
            if job and job["status"] in ("running", "stopping") and job["pid"]:
                try:
                    os.kill(job["pid"], 0)
                except ProcessLookupError:
                    continue
                raise AdapterError(f"Stop the {table.replace('_', ' ')} worker before updating {Path(directory).name}.")


def runtime_identity(protocol):
    return canonical({key: value for key, value in protocol.items()
                      if key not in ("id", "candidateBatch", "nativeCandidates", "reuseArchives",
                                     "requalificationCandidates", "deferredConstruction")})


def reusable(source, target, *, designs=None, target_source=None):
    original = definition(source)
    required = ("simulatorRevision", "versions", "evaluation",
                "payloadProfile", "qualification", "controller")
    names = [f"simulator/legowalkers_sim/{name}" for name in PHYSICS_SOURCES]
    if original.get("evaluation", {}).get("cycle_aware"):
        names.append("simulator/legowalkers_sim/cycles.py")
    if any(key not in original for key in required) or any(name not in original.get("sourceSha256", {}) for name in names):
        return False
    if original["simulatorRevision"] != target.get("simulatorRevision"):
        return False
    if compatible_evaluation(original, target, Path(source), target_source=target_source):
        return True
    if "simulator/legowalkers_sim/support_surfaces.py" not in target.get("sourceSha256", {}):
        return False
    with database(source) as db:
        measured = [row[0] for row in db.execute("""
            SELECT DISTINCT b.design FROM births b
            WHERE EXISTS(SELECT 1 FROM evaluations e WHERE e.birth=b.id AND e.status='complete')
            OR EXISTS(SELECT 1 FROM reused_evaluations r WHERE r.birth=b.id)
        """) if designs is None or row[0] in designs]
    return bool(measured) and all(
        compatible_evaluation(original, target, Path(source),
                              manifest=load_manifest(Path(source) / "designs" / design / "candidate/manifest.json"),
                              target_source=target_source)
        for design in measured)


def observation_sources(paths, protocol, *, target=None):
    from .upgrades import design_aliases
    aliases = design_aliases(target) if target is not None else {}
    pending = set(protocol["nativeCandidates"])
    pending -= set(protocol.get("requalificationCandidates", []))
    selected = []
    for path in reversed(paths):
        with database(path) as db:
            measured = {row[0] for row in db.execute(
                "SELECT b.design FROM births b JOIN evaluations e ON e.birth=b.id WHERE e.status='complete'")}
        matching = {design for design in measured if aliases.get(design, design) in pending}
        if matching and reusable(path, protocol, designs=matching):
            selected.append(str(path))
            pending -= {aliases.get(design, design) for design in matching}
    return selected


class VersionHTML(HTMLParser):
    def __init__(self, prefix, *, readonly=False):
        super().__init__(convert_charrefs=False)
        self.prefix = prefix
        self.readonly = readonly
        self.skip = None
        self.depth = 0
        self.output = []

    def handle_starttag(self, tag, attrs):
        if self.skip:
            if tag == self.skip:
                self.depth += 1
            return
        values = dict(attrs)
        if tag == "script" or (self.readonly and tag == "section" and (
                values.get("id") == "live-controls" or values.get("aria-label") == "Animation rendering")):
            self.skip, self.depth = tag, 1
            return
        changed = False
        rewritten = []
        for name, value in attrs:
            if name in ("href", "src") and value:
                if name == "href" and "data-archive-link" in values:
                    value = "/archives/" + values["data-archive-link"]
                else:
                    value = urljoin(self.prefix, value)
                changed = True
            rewritten.append(name if value is None else f'{name}="{esc(value)}"')
        self.output.append(f"<{tag} {' '.join(rewritten)}>" if changed else self.get_starttag_text())

    def handle_startendtag(self, tag, attrs):
        self.handle_starttag(tag, attrs)

    def handle_endtag(self, tag):
        if self.skip:
            if tag == self.skip:
                self.depth -= 1
                if self.depth == 0:
                    self.skip = None
            return
        self.output.append(f"</{tag}>")

    def handle_data(self, data):
        if not self.skip:
            self.output.append(data)

    def handle_entityref(self, name):
        self.handle_data(f"&{name};")

    def handle_charref(self, name):
        self.handle_data(f"&#{name};")

    def handle_decl(self, decl):
        self.output.append(f"<!{decl}>")

    def handle_comment(self, data):
        if not self.skip:
            self.output.append(f"<!--{data}-->")


def version_html(text, prefix, *, readonly=False, invalidation=None):
    parser = VersionHTML(prefix, readonly=readonly)
    parser.feed(text)
    parser.close()
    result = "".join(parser.output)
    if readonly:
        banner = '<aside class="panel"><strong>Read-only version history.</strong> ' \
                 '<a href="/">Continue the current experiment</a> · <a href="/history.html">All updates</a></aside>'
        result = result.replace("<body>", "<body>" + banner, 1)
        script = Path(__file__).with_name("dashboard.js").read_text()
        result = result.replace("</body>", f"<script>{script}</script></body>", 1)
    if invalidation:
        result = re.sub(r'<span class="pill good">[^<]*[Qq]ualified[^<]*</span>',
                        '<span class="pill warn">Invalidated historical qualification</span>', result)
        result = result.replace("<td>qualified</td>", "<td>invalidated</td>")
        banner = ('<aside class="panel bad" data-evidence-invalidation>'
                  f'<strong>Simulation evidence invalidated.</strong> {esc(invalidation)} '
                  'Saved results and movies are retained only as historical records; '
                  'their speeds, stability and rejection diagnoses are not current evidence.</aside>')
        result = result.replace("<body>", "<body>" + banner, 1)
    return result


class CampaignController:
    def __init__(self, directory, initial, *, load, build, describe, generator=None):
        self.root = Path(directory).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.path = self.root / "campaign.json"
        self.mutex = threading.RLock()
        self.preparation_lock = threading.Lock()
        self.preparation_thread = None
        self.token = secrets.token_urlsafe(32)
        self.load, self.build, self.describe = load, build, describe
        self.generator = generator
        with locked(self.root):
            if self.path.exists():
                self.state = json.loads(self.path.read_text())
                if not isinstance(self.state, dict) or self.state.get("schemaVersion") != 1:
                    raise AdapterError("Unsupported experiment history schema.")
            else:
                paths = [Path(initial).resolve()]
                pending = sorted(paths[0].parent.glob(paths[0].name + "-upgrade-*"))
                while pending:
                    matched = [p for p in pending if (p / "archive.sqlite3").exists() and (p / "upgrade.json").exists()
                               and Path(json.loads((p / "upgrade.json").read_text())["sourceArchive"]).resolve() in paths]
                    if not matched:
                        break
                    paths.extend(matched)
                    pending = [p for p in pending if p not in matched]
                with contextlib.ExitStack() as stack:
                    for path in paths:
                        stack.enter_context(locked(path))
                        idle(path)
                    versions = [self._record(f"v{index+1:04d}", path) for index, path in enumerate(paths)]
                    for version in versions:
                        self._check_owner(version["archive"], version["id"])
                    self.state = {"schemaVersion": 1, "id": str(uuid.uuid4()), "created": now(),
                                  "active": versions[-1]["id"], "versions": versions, "lastUpdateError": None}
                    self._save()
                    for version in versions:
                        self._mark(version["archive"], version["id"])
            versions = self.state.get("versions")
            if (not isinstance(versions, list) or not versions
                    or any(not isinstance(v, dict) or not isinstance(v.get("id"), str)
                           or not re.fullmatch(r"v[0-9]{4,}", v["id"])
                           or not isinstance(v.get("archive"), str) or not Path(v["archive"]).is_absolute()
                           or type(v.get("inheritedAllowance")) is not int or v["inheritedAllowance"] < 0
                           for v in versions)
                    or len({v["id"] for v in versions}) != len(versions)
                    or len({v["archive"] for v in versions}) != len(versions)
                    or self.state.get("active") not in {v["id"] for v in versions}):
                raise AdapterError("Experiment history has invalid version registrations.")
            for version in self.state["versions"]:
                self._mark(version["archive"], version["id"])
        self.active = self.load(self.directory)
        self.stamp = None
        if self.state.get("migration", {}).get("status") == "running":
            self._finish_migration("failed", "Preparation was interrupted; the active version and partial outputs were retained.")

    def _record(self, identifier, path, inherited=0):
        with database(path) as db:
            created = db.execute("SELECT created FROM run").fetchone()[0]
        protocol = definition(path)
        return {"id": identifier, "archive": str(Path(path).resolve()), "created": created,
                "mutationVersion": protocol.get("nativeDiscovery", {}).get("mutationVersion"),
                "inheritedAllowance": inherited}

    def _save(self):
        atomic_text(self.path, json.dumps(self.state, indent=2) + "\n")

    def _mark(self, directory, identifier):
        self._check_owner(directory, identifier)
        atomic_text(Path(directory) / "campaign-version.json",
                    json.dumps({"campaign": str(self.root), "version": identifier}) + "\n")

    def _check_owner(self, directory, identifier):
        marker = Path(directory) / "campaign-version.json"
        if marker.exists():
            link = json.loads(marker.read_text())
            if Path(link["campaign"]).resolve() != self.root or link["version"] != identifier:
                raise AdapterError(f"Archive already belongs to another experiment registration: {directory}")

    @property
    def directory(self):
        return Path(next(v["archive"] for v in self.state["versions"] if v["id"] == self.state["active"]))

    @property
    def renderer(self):
        return self.active.renderer

    def _stamp(self, directory=None, controller=None):
        from .evolution import ROOT
        protocol = definition(directory or self.directory)
        paths = [ROOT / name for name in protocol.get("sourceSha256", {})]
        worker = (controller or self.active).worker_args
        if "--generator" in worker:
            binary = Path(worker[worker.index("--generator")+1]).parent
            paths.extend(binary / name for name in protocol.get("generatorBinarySha256", {}))
        values = []
        for path in paths:
            try:
                stat = path.stat()
            except FileNotFoundError:
                values.append((str(path), None, None))
            else:
                values.append((str(path), stat.st_mtime_ns, stat.st_size))
        return tuple(values)

    def history(self):
        with self.mutex:
            versions = []
            for version in self.state["versions"]:
                with database(version["archive"]) as db:
                    budget = db.execute("SELECT budget FROM run").fetchone()[0]
                    used = db.execute("SELECT count(*) FROM evaluations").fetchone()[0]
                versions.append({key: value for key, value in version.items() if key != "archive"} |
                                {"used": used, "budget": budget, "url": f'/versions/{version["id"]}/',
                                 "active": version["id"] == self.state["active"]})
            return {"id": self.state["id"], "active": self.state["active"], "versions": versions,
                    "used": sum(v["used"] for v in versions),
                    "budget": sum(v["budget"]-v["inheritedAllowance"] for v in versions)}

    def status(self):
        with self.mutex:
            migrating = self.state.get("migration", {}).get("status") == "running"
            state = self.active.status(automatic_previews=not migrating)
            qualified = []
            for version in self.state["versions"]:
                with database(version["archive"]) as db:
                    qualified.extend(qualified_walkers(
                        db, Path(version["archive"]), prefix=f'/versions/{version["id"]}/',
                        version=version["id"]))
            state["pipeline"]["qualified"] = recent_qualified(qualified)
            history = self.history()
            remaining = history["budget"]-history["used"]
            update_available = self.stamp != self._stamp()
            migration = dict(self.state.get("migration", {
                "status": "required", "stage": "Prepare the current implementation before resuming.",
                "error": self.state.get("lastUpdateError")}))
            if migration["status"] == "ready" and update_available:
                migration.update(status="required", stage="Implementation changed; explicit preparation is required.")
            state.update(campaign=history["id"], version=self.state["active"],
                         versionUsed=state["used"], versionBudget=state["budget"],
                         used=history["used"], budget=history["budget"], remaining=remaining,
                         revision=f'{self.state["active"]}:{state["revision"]}',
                         csrfToken=self.token, historyUrl="/history.html",
                         updateAvailable=update_available,
                         migration=migration,
                         canStart=state["canStart"] and not migrating and not update_available,
                         updateError=self.state.get("lastUpdateError"))
            return state

    def _migration_phase(self, stage, target=None):
        with self.mutex:
            migration = self.state.get("migration")
            if migration is None or migration["status"] != "running":
                return
            migration["stage"] = stage
            if target is not None:
                migration["targetVersion"] = target
            self._save()
            with (self.root / "migration.log").open("a") as log:
                log.write(f"{now()} {stage}\n")

    def _finish_migration(self, status, error=None):
        with self.mutex:
            migration = self.state["migration"]
            migration.update(status=status, completed=now(), error=error,
                             stage="Ready to resume; no episode started." if status == "ready" else "Preparation failed; previous version retained.")
            if error is not None:
                self.state["lastUpdateError"] = error
            self._save()
            with (self.root / "migration.log").open("a") as log:
                log.write(f"{now()} {migration['stage']}{': '+error if error else ''}\n")

    def _run_migration(self):
        try:
            self.ensure_latest()
        except (AdapterError, OSError, sqlite3.Error, ValueError, subprocess.SubprocessError) as error:
            logging.exception("Campaign preparation failed")
            self._finish_migration("failed", str(error))
        else:
            self._finish_migration("ready")
        finally:
            if self.state["migration"]["status"] == "running":
                logging.error("Unexpected campaign preparation failure", exc_info=True)
                self._finish_migration("failed", "Unexpected preparation failure; see the dashboard process log.")

    def ensure_latest(self, *, additional_parents=(), authorization=0):
        with self.preparation_lock, locked(self.root):
            with self.mutex:
                source = self.directory
                active = self.active
                paths = [Path(v["archive"]) for v in self.state["versions"]]
            self._migration_phase("Checking the requested implementation and input snapshots.")
            try:
                requested_stamp = self._stamp(source, active)
                requested = self.describe(source / "native-input")
                if requested_stamp != self._stamp(source, active):
                    raise AdapterError("Implementation changed while checking preparation; retry the explicit preparation.")
            except (AdapterError, OSError, ValueError) as error:
                with self.mutex:
                    self.state["lastUpdateError"] = str(error)
                    self._save()
                raise
            if not additional_parents and runtime_identity(requested) == runtime_identity(definition(source)):
                with self.mutex:
                    self.stamp = self._stamp()
                    if self.state.get("lastUpdateError"):
                        self.state["lastUpdateError"] = None
                        self._save()
                return {"updated": False, "version": self.state["active"], "url": "/"}
            state = active.status(automatic_previews=False)
            if state["running"]:
                raise AdapterError("Stop the episode and rendering workers before updating this experiment.")
            if state["rendering"]["running"] and not active.renderer.stop_if_idle():
                raise AdapterError("Stop the episode and rendering workers before updating this experiment.")
            number = len(self.state["versions"])+1
            while (self.root / "versions" / f"v{number:04d}").exists():
                number += 1
            identifier = f"v{number:04d}"
            target = self.root / "versions" / identifier
            try:
                self._migration_phase("Collecting retained motion plans and preparing version inputs.", identifier)
                with contextlib.ExitStack() as stack:
                    for path in paths:
                        stack.enter_context(locked(path))
                        idle(path)
                    options = {"additional_parents": additional_parents} if additional_parents else {}
                    _, seeds = prepare(source, target=target, sources=paths,
                                       generator=self.generator if additional_parents else None, **options)
                    self._mark(target, identifier)
                    protocol = self.describe(seeds)
                    if runtime_identity(protocol) != runtime_identity(requested):
                        raise AdapterError("Implementation changed during update preparation; retry the update.")
                    compatible = observation_sources(paths, protocol, target=target)
                    if compatible:
                        protocol["reuseArchives"] = compatible
                    history = self.history()
                    allowance = history["budget"]-history["used"]
                    if allowance < 0:
                        raise AdapterError("Experiment authorization accounting is inconsistent.")
                    self._migration_phase("Freezing the implementation and queuing immutable motion inputs; construction runs after Resume.")
                    controller = self.build(target, seeds, protocol, allowance + authorization)
                    self._migration_phase("Verifying the frozen version and unchanged trial accounting.")
                    if runtime_identity(definition(target)) != runtime_identity(protocol):
                        raise AdapterError("Implementation changed while preparing the frozen runtime; retry the update.")
                    ready = controller.status()
                    if not ready["pinnedRuntime"]:
                        raise AdapterError("Prepared version has no verified frozen implementation.")
                    if ready["running"] or ready["used"] != 0 or ready["budget"] != allowance + authorization:
                        raise AdapterError("Prepared version changed trial accounting or started a worker.")
                    version = self._record(identifier, target, allowance)
                    if additional_parents:
                        version["requalificationSources"] = [row["design"] for row in additional_parents]
                        version["newAuthorization"] = authorization
                    stamp = self._stamp(target, controller)
                    with self.mutex:
                        previous = self.state
                        self.state = {**previous, "active": identifier, "versions": [*previous["versions"], version],
                                      "lastUpdateError": None}
                        try:
                            self._save()
                        except OSError:
                            self.state = previous
                            raise
                        self.active = controller
                        self.stamp = stamp
                return {"updated": True, "version": identifier, "url": "/",
                        "remaining": allowance + authorization}
            except (AdapterError, OSError, sqlite3.Error, ValueError) as error:
                with self.mutex:
                    self.state["lastUpdateError"] = str(error)
                    self._save()
                raise

    def start(self):
        with self.mutex:
            if self.state.get("migration", {}).get("status") == "running":
                raise AdapterError("Preparation is running; wait for Ready before resuming.")
            if self.stamp != self._stamp():
                raise AdapterError("Explicitly prepare the current implementation before resuming; Resume never rebuilds or migrates.")
            return {**self.active.start(), "version": self.state["active"]}

    def pause(self):
        with self.mutex:
            active = self.active
        return active.pause()

    def requalify(self, trials):
        from .upgrades import _origin
        if type(trials) is not int or not 1 <= trials <= 32:
            raise AdapterError("Requalification requires an explicit allowance of one to 32 trials.")
        with self.mutex:
            if self.state.get("migration", {}).get("status") == "running":
                raise AdapterError("Wait for preparation to finish before requalification.")
            candidates = {}
            for version in self.state["versions"]:
                source = Path(version["archive"])
                if not evidence_invalidation(definition(source)):
                    continue
                with database(source) as db:
                    for row in db.execute("""
                        SELECT b.id AS birth,b.design,d.category,d.genome,e.result FROM evaluations e
                        JOIN births b ON b.id=e.birth JOIN designs d ON d.id=b.design
                        WHERE e.status='complete' ORDER BY e.id
                    """):
                        if is_qualified(json.loads(row["result"])):
                            candidates.setdefault(row["design"], {
                                **dict(row), "sourceArchive": str(source), "result": None,
                                "founder": list(_origin(source, row["birth"])), "measured": False})
            if len(candidates) != trials:
                raise AdapterError(f"Found {len(candidates)} invalidated qualified walkers; "
                                   f"the explicit allowance was {trials}. No authorization changed.")
            if any(set(version.get("requalificationSources", [])) == set(candidates)
                   for version in self.state["versions"]):
                raise AdapterError("These walkers already have a registered requalification batch; "
                                   "resume that batch instead of authorizing duplicates.")
            update = self.ensure_latest(additional_parents=list(candidates.values()), authorization=trials)
            job = self.active.start()
            return {**update, **job, "authorizedAdditionalTrials": trials, "scheduledWalkers": len(candidates)}

    def upgrade(self):
        with self.mutex:
            if self.state.get("migration", {}).get("status") == "running":
                raise AdapterError("Preparation is already running.")
            if self.active.status(automatic_previews=False)["running"]:
                raise AdapterError("Wait for the episode to finish before preparing a migration.")
            self.state["migration"] = {
                "status": "running", "stage": "Preparation queued.", "started": now(),
                "completed": None, "error": None, "sourceVersion": self.state["active"],
                "targetVersion": None, "log": "/migration.log"}
            self.state["lastUpdateError"] = None
            atomic_text(self.root / "migration.log", f"{now()} Explicit preparation requested; no trials authorized.\n")
            self._save()
            self.preparation_thread = threading.Thread(target=self._run_migration, name="campaign-preparation", daemon=True)
            try:
                self.preparation_thread.start()
            except RuntimeError as error:
                self._finish_migration("failed", str(error))
                raise AdapterError(f"Could not start preparation: {error}") from error
            return {"preparing": True, "version": self.state["active"], "url": "/",
                    "migration": dict(self.state["migration"])}

    def render_start(self):
        with self.mutex:
            if self.state.get("migration", {}).get("status") == "running":
                raise AdapterError("Wait for preparation to finish before starting rendering.")
            return self.renderer.start()

    def render_stop(self):
        with self.mutex:
            return self.renderer.stop()

    def version_directory(self, identifier):
        with self.mutex:
            version = next((v for v in self.state["versions"] if v["id"] == identifier), None)
            if version is None:
                raise AdapterError("Unknown experiment version.")
            return Path(version["archive"])

    def history_page(self):
        with self.mutex:
            history = self.history()
            rows = "".join(
                f'<tr><td><a href="{v["url"]}">{esc(v["id"])}</a>{" · current" if v["active"] else ""}</td>'
                f'<td>{esc(v["created"])}</td><td>{esc(v["mutationVersion"])}</td><td>{v["used"]}</td>'
                f'<td>{v["inheritedAllowance"]}</td></tr>' for v in history["versions"])
            return document("Experiment update history",
                            '<a href="/">Back to current experiment</a><h1>Updates &amp; history</h1>'
                            f'<p>One experiment: {history["used"]} simulations spent, {history["budget"]} authorized. '
                            'Updates transfer unused allowance; they do not authorize simulations.</p>'
                            '<p>Each version preserves its original rules, results, animations and robot ancestry. '
                            'Historical pages are read-only; results are not silently reranked under newer rules.</p>'
                            '<table><thead><tr><th>Version</th><th>Created</th><th>Mutation implementation</th>'
                            f'<th>Simulations</th><th>Transferred allowance</th></tr></thead><tbody>{rows}</tbody></table>')

    def dashboard_page(self):
        with self.mutex:
            text = (self.directory / "index.html").read_text()
            _, separator, rest = text.partition('<main id="dashboard-content">')
            content, end, _ = rest.rpartition("</main>")
            if not separator or not end:
                raise AdapterError("Active dashboard has no generated content region.")
            prefix = f'/versions/{self.state["active"]}/'
            content = version_html(content, prefix)
            history = self.history()
            banner = f'<section class="panel"><strong>One evolving experiment · {esc(self.state["active"])}</strong>' \
                     f'<p>{history["used"]} simulations spent · {history["budget"]} authorized · ' \
                     f'{history["budget"]-history["used"]} remaining. <a href="/history.html">Updates &amp; history</a></p></section>'
            carried = self._carried()
            return document("LegoWalkers evolution", banner + carried +
                            '<section aria-label="Current-version results"><h2>Results under current rules</h2>'
                            + content + "</section>", dashboard=True)

    def _carried(self):
        metadata = self.directory / "upgrade.json"
        if not metadata.exists():
            return ""
        provenance = json.loads(metadata.read_text())
        parents = provenance["parents"]
        cards = []
        with database(self.directory) as db:
            measured = {r[0] for r in db.execute(
                "SELECT b.design FROM births b JOIN evaluations e ON e.birth=b.id WHERE e.status='complete' "
                "UNION SELECT b.design FROM births b JOIN reused_evaluations r ON r.birth=b.id")}
        for parent in parents:
            source = parent.get("sourceArchive", provenance["sourceArchive"])
            version = next(v for v in self.state["versions"] if v["archive"] == source)
            prefix = f'/versions/{version["id"]}/'
            design = Path(source) / "designs" / parent["sourceDesign"]
            media_version = version
            clips = []
            for candidate in reversed(self.state["versions"]):
                found = sorted((Path(candidate["archive"]) / "designs" / parent["sourceDesign"] / "render").glob("walker-v2*.mp4"))
                if found:
                    media_version, clips = candidate, found
                    break
            image = next((p for p in (design / "render/walker-v2.png", design / "assembly-render/walker-v2.png") if p.exists()), None)
            if clips:
                src = f'/versions/{media_version["id"]}/' + quote(str(clips[0].relative_to(Path(media_version["archive"]))))
                media = f'<video controls muted loop preload="none"><source src="{src}" type="video/mp4"></video>'
            elif image:
                src = prefix + quote(str(image.relative_to(Path(source))))
                media = f'<img src="{src}" alt="Carried-forward robot">'
            else:
                media = '<div class="empty">Inherited physical design</div>'
            label = ("Current measurement available" if parent.get("targetDesign", parent["sourceDesign"]) in measured else
                     "Historical measurement — needs evaluation under current rules" if parent.get("historicalMeasurement", True) else
                     "Carried forward — not yet measured")
            conversion = ('<p class="muted">Shared leg templates bound; geometry and gait unchanged.</p>'
                          if parent.get("targetDesign", parent["sourceDesign"]) != parent["sourceDesign"] else "")
            manifest = json.loads((design / "candidate/manifest.json").read_text())
            cards.append(f'<article class="card"><h3>{esc(parent["seed"])} · {manifest["motion"]["footCount"]} feet</h3><p>{label}</p>{conversion}{media}'
                         f'<p class="muted">Media from {esc(media_version["id"])}; earlier ancestry through {esc(version["id"])}.</p>'
                         f'<a href="{prefix}history/{parent["sourceBirth"]:04d}.html">Earlier robot and ancestry</a></article>')
        return '<section><h2>Carried-forward population</h2><div class="grid">' + "".join(cards) + "</div></section>"
