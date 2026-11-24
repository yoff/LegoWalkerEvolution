"""Readable screening diagnostics, including immutable reports from older workers."""

import ast
from contextlib import contextmanager
from datetime import datetime, timezone
from functools import lru_cache
from html import escape
import json
import logging
import math
from pathlib import Path
import re
import sqlite3
import sys

from .bridge import AdapterError

logger = logging.getLogger(__name__)

CHECK_LABELS = {
    "inputs": "Saved inputs and construction gates",
    "dependencies": "LEGO part dependencies",
    "reconstruction": "Parts, rigid bodies and moving joints",
    "collision": "Connector geometry and collision preparation",
    "mass": "Payloads and mass layout",
    "export": "Simulation geometry and model export",
    "load": "MuJoCo model loading at time zero",
    "closure": "Initial hinge closure",
    "contacts": "Initial self-interference",
    "phases": "Complete-assembly prescribed gait clearance",
}


class ScreeningProgress:
    """Observational checkpoints only; never authorize or advance physics."""

    def __init__(self, output, genotype):
        if not isinstance(genotype, str) or re.fullmatch("[a-f0-9]{64}", genotype) is None:
            raise AdapterError("Screening progress requires a valid genotype identifier.")
        self.path = Path(output) / "progress" / f"{genotype}.json"
        self.document = {"schemaVersion": 1, "genotypeId": genotype, "status": "running",
                         "dynamicTrials": 0, "checks": [
                             {"id": key, "label": label, "status": "pending", "detail": "",
                              "started": None, "completed": None}
                             for key, label in CHECK_LABELS.items()]}
        self.current = None
        self.save()

    def save(self):
        from .gallery import atomic_text
        atomic_text(self.path, json.dumps(self.document, allow_nan=False))

    def start(self, key):
        if self.current is not None:
            raise AdapterError("A screening check is already running.")
        check = next(check for check in self.document["checks"] if check["id"] == key)
        if check["status"] != "pending":
            raise AdapterError(f"Screening check {key} has already run.")
        check.update(status="running", started=datetime.now(timezone.utc).isoformat())
        self.current = check
        self.save()
        print(f"Screening {self.document['genotypeId'][:10]}: {check['label']} - running",
              file=sys.stderr, flush=True)

    def complete(self, status="pass", detail=""):
        if self.current is None:
            raise AdapterError("No screening check is running.")
        check = self.current
        check.update(status=status, detail=detail, completed=datetime.now(timezone.utc).isoformat())
        self.current = None
        self.save()
        print(f"Screening {self.document['genotypeId'][:10]}: {check['label']} - {status}"
              + (f": {detail}" if detail else ""), file=sys.stderr, flush=True)

    @contextmanager
    def step(self, key):
        self.start(key)
        try:
            yield
        finally:
            error = sys.exc_info()[1]
            self.complete("failed" if error is not None else "pass", str(error) if error is not None else "")

    def finish(self, status, detail=""):
        if self.current is not None:
            self.complete("failed", detail)
        self.document["status"] = status
        self.document["detail"] = detail
        for check in self.document["checks"]:
            if check["status"] == "pending":
                check.update(status="not-run", detail="Not executed in this check.")
        self.save()


def screening_progress(directory, *, genotype=None, running=False):
    """Read live checkpoints or actual legacy outcomes, never infer completed checks."""
    directory = Path(directory)
    paths = [*directory.glob("progress/*.json"), *directory.glob("*/progress/*.json")]
    if genotype is not None:
        paths = [path for path in paths if path.stem == genotype]
    if paths:
        path = max(paths, key=lambda path: path.stat().st_mtime_ns)
        document = _load(path)
        if (not isinstance(document, dict) or document.get("schemaVersion") != 1
                or not isinstance(document.get("checks"), list)
                or any(not isinstance(check, dict) or check.get("id") not in CHECK_LABELS
                       or check.get("status") not in ("pending", "running", "pass", "failed", "rejected", "not-run")
                       or not isinstance(check.get("label"), str) or not isinstance(check.get("detail"), str)
                       for check in document["checks"])):
            raise AdapterError(f"Invalid screening progress: {path}")
        checks = [dict(check) for check in document["checks"]]
        if not running:
            for check in checks:
                if check["status"] in ("running", "pending"):
                    check["status"] = "interrupted"
        return {"tracked": True, "checks": checks,
                "detail": (document.get("detail", "") + " Load-only checks; no walking trial spent.").strip()}
    reports = sorted([*directory.glob("screen-*.json"), *directory.glob("*/screen-*.json")],
                     key=lambda path: path.stat().st_mtime_ns, reverse=True)
    unreadable = []
    for path in reports:
        document = _load(path)
        if document is None:
            unreadable.append(path.name)
            continue
        for report in document.get("results", []):
            if genotype is None or report.get("genotypeId") == genotype:
                return {"tracked": False, "checks": [
                    {"id": gate["gate"], "label": gate["gate"], "status": gate["outcome"],
                     "detail": "; ".join(gate.get("diagnostics", []))}
                    for gate in report.get("gates", [])],
                    "detail": "Saved gate outcomes; this worker did not record individual check timings."}
    return {"tracked": False, "checks": [],
            "detail": ("Cannot read saved screening reports: " + ", ".join(unreadable) if unreadable else
                       "Detailed checkpoints are unavailable for this worker; no completed checks are inferred.")}


def repair_hints(report):
    phase = report.get("phaseClearance", {})
    if isinstance(phase, dict) and phase.get("version") == "prescribed-phase-clearance-v1" and phase.get("status") == "collision":
        contacts = phase.get("contacts")
        if (not isinstance(contacts, list) or not contacts
                or any(not isinstance(c, dict) or not isinstance(c.get("parts"), list)
                       or len(c["parts"]) != 2 or any(not isinstance(p, str) or not p.strip() for p in c["parts"])
                       or c["parts"][0] == c["parts"][1] for c in contacts)):
            logger.warning("Prescribed phase contacts lack valid pairs; no repair targets extracted.")
            return None
        pairs = sorted({tuple(sorted(c["parts"])) for c in contacts})
        return {"kind": "motion-self-interference",
                "parts": sorted({p for pair in pairs for p in pair}), "bodies": [],
                "pairs": [list(pair) for pair in pairs]}
    hints = report.get("repairHints")
    if hints is not None and hints.get("kind") != "initial-self-interference":
        return hints
    overlaps = report.get("loadOnly", {}).get("initialSelfInterference", [])
    if overlaps:
        return {"kind": "initial-self-interference",
                "parts": sorted({part for item in overlaps for part in item["parts"]}), "bodies": [],
                "pairs": [item["parts"] for item in overlaps]}
    if hints is not None:
        return hints
    error = report.get("error") or report.get("rejectionDetail", "")
    if not error.startswith("Rigid-body partition mismatch for "):
        return None
    match = re.fullmatch(r"Rigid-body partition mismatch for (.+?): expected (\[.*\]), detected (\{.*\})", error)
    if match is None or len(error) > 65536:
        logger.warning("Unsupported rigid-partition diagnostic; no repair targets extracted.")
        return None
    try:
        expected, detected = ast.literal_eval(match[2]), ast.literal_eval(match[3])
    except (ValueError, SyntaxError) as failure:
        logger.warning("Cannot decode rigid-partition diagnostic: %s", failure)
        return None
    if (not isinstance(expected, list) or not all(isinstance(part, str) for part in expected)
            or not isinstance(detected, dict)
            or any(not isinstance(parts, list) or not all(isinstance(part, str) for part in parts)
                   for parts in detected.values())):
        logger.warning("Unsupported rigid-partition membership fields; no repair targets extracted.")
        return None
    extras = {part for parts in detected.values() for part in parts} - set(expected)
    components = [set(parts) & set(expected) for parts in detected.values()]
    components = [parts for parts in components if parts]
    islands = set()
    if len(components) > 1:
        largest = max(map(len, components))
        main = [parts for parts in components if len(parts) == largest]
        islands = set(expected) - main[0] if len(main) == 1 else set(expected)
    missing = set(expected) - {part for parts in detected.values() for part in parts}
    return {"kind": "rigid-body-partition", "parts": sorted(extras | islands | missing), "bodies": [match[1]]}


def rejection_detail(report):
    if report.get("rejectionDetail") or report.get("error"):
        return report.get("rejectionDetail") or report["error"]
    overlaps = report.get("loadOnly", {}).get("initialSelfInterference", [])
    if overlaps:
        pairs = [f"{' / '.join(item['parts'])}: {1000 * item['depthMeters']:.3f} mm overlap"
                 for item in overlaps]
        return "Initial self-interference: " + "; ".join(pairs) + " (collision model)."
    gates = [gate for gate in report.get("gates", []) if gate.get("outcome") == "reject"]
    if gates:
        return "; ".join(f"{gate['code']}: {'; '.join(gate.get('diagnostics', []))}".rstrip(": ")
                         for gate in gates)
    return "Load-only screening rejected this realization; no detailed diagnostic was recorded."


@lru_cache(maxsize=256)
def _read_report(path, modified, size):
    return json.loads(path.read_text())


def _load(path):
    try:
        stat = path.stat()
        return _read_report(path, stat.st_mtime_ns, stat.st_size)
    except (OSError, json.JSONDecodeError) as error:
        logger.warning("Cannot read screening diagnostics from %s: %s", path, error)
        return None


def rejections(directory):
    directory = Path(directory).resolve()
    found = {}
    for folder in ("preflight", "preflight-isolated"):
        for path in sorted((directory / folder).glob("**/screen-*.json")):
            summary = _load(path)
            if summary is None:
                continue
            for report in summary.get("results", []):
                if report.get("previousStatus", report.get("status")) != "rejected":
                    continue
                source, detail = path, report
                identifier = report.get("screeningId")
                if isinstance(identifier, str) and re.fullmatch("[0-9a-f]{64}", identifier):
                    canonical = (path.parent / "reports" / f"{identifier}.json").resolve()
                    if not canonical.is_relative_to(directory):
                        raise AdapterError("Screening diagnostics must remain inside the archive.")
                    if canonical.is_file():
                        saved = _load(canonical)
                        if saved is not None:
                            source, detail = canonical, saved
                found[report["genotypeId"]] = {
                    "detail": rejection_detail(detail),
                    "report": source.relative_to(directory).as_posix(),
                    "repairHints": repair_hints(detail),
                }
    return found


def motion_repair_hints(report):
    if not isinstance(report, dict):
        logger.warning("Moving-interference evidence must be a JSON object; no repair targets extracted.")
        return None
    if report.get("version") == "simulation-prefix-gate-v1":
        if report.get("status") != "rejected" or "self-interference" not in report.get("reasons", []):
            return initial_ground_repair_hints(report) or stance_repair_hints(report)
        collision = report.get("diagnostics", {})
    else:
        if "self-interference" not in report.get("metrics", {}).get("rankingRejectionReasons", []):
            return stance_repair_hints(report)
        collision = report.get("collision", {})
    if not isinstance(collision, dict):
        logger.warning("Moving-interference collision diagnostics must be an object; no repair targets extracted.")
        return None
    contacts = collision.get("selfContacts")
    if not isinstance(contacts, list):
        logger.warning("Moving self-interference report has no measured contact list; no repair targets extracted.")
        return None
    pairs = set()
    for contact in contacts:
        if not isinstance(contact, dict):
            logger.warning("Unsupported moving-contact diagnostic; no repair targets extracted.")
            return None
        parts, depth = contact.get("parts"), contact.get("depthMeters")
        if (not isinstance(parts, list) or len(parts) != 2 or parts[0] == parts[1]
                or any(not isinstance(part, str) or not part.strip() for part in parts)
                or isinstance(depth, bool) or not isinstance(depth, (int, float))
                or not math.isfinite(depth) or depth < 0):
            logger.warning("Invalid moving-contact identities or penetration; no repair targets extracted.")
            return None
        if depth > 0.0001:
            pairs.add(tuple(sorted(parts)))
    if not pairs:
        logger.warning("Moving self-interference report has no contacts above the rejection tolerance.")
        return None
    return {"kind": "motion-self-interference", "parts": sorted({part for pair in pairs for part in pair}),
            "bodies": [], "pairs": [list(pair) for pair in sorted(pairs)]}


def initial_ground_repair_hints(report):
    if (report.get("version") != "simulation-prefix-gate-v1" or report.get("status") != "rejected"
            or "forbidden-ground-contact" not in report.get("reasons", [])
            or report.get("measurementSeconds") != 0):
        return None
    evidence = report.get("diagnostics")
    if not isinstance(evidence, dict):
        logger.warning("Initial ground-contact diagnostics must be an object.")
        return None
    ground = evidence.get("groundContact")
    if (not isinstance(ground, dict) or not ground.get("forbiddenContactDetected")
            or type(ground.get("footContactSamples")) is not int or ground["footContactSamples"] != 0
            or type(ground.get("wheelContactSamples")) is not int or ground["wheelContactSamples"] != 0):
        return None
    contacts = ground.get("forbiddenContacts")
    if (not isinstance(contacts, list) or not contacts
            or any(not isinstance(contact, dict) or not isinstance(contact.get("part"), str)
                   or not contact["part"].strip() for contact in contacts)):
        logger.warning("Initial forbidden-ground evidence lacks valid part identities.")
        return None
    return {"kind": "initial-foot-clearance", "parts": sorted({c["part"] for c in contacts}), "bodies": []}


def stance_repair_hints(report):
    from .stance_support import VERSION
    prefix = report.get("version") == "simulation-prefix-gate-v1"
    if prefix and report.get("status") != "rejected":
        return None
    evidence = report.get("diagnostics", {}) if prefix else report
    if not isinstance(evidence, dict):
        logger.warning("Stance diagnostics must be an object; no mass-layout targets extracted.")
        return None
    stance = evidence.get("stanceSupport", {})
    ground = evidence.get("groundContact", {})
    if not isinstance(stance, dict) or not isinstance(ground, dict):
        logger.warning("Stance and ground-contact evidence must be objects; no mass-layout targets extracted.")
        return None
    if not ground.get("forbiddenContactDetected"):
        return None
    if stance.get("version") != VERSION or stance.get("status") != "measured":
        return None
    fraction = stance.get("unsupportedFraction")
    if (isinstance(fraction, bool) or not isinstance(fraction, (int, float))
            or not math.isfinite(fraction) or not 0 <= fraction <= 1):
        logger.warning("Invalid measured unsupported fraction; no mass-layout targets extracted.")
        return None
    if fraction == 0:
        return None
    return {"kind": "stance-support-mass-layout", "parts": ["payload-battery", "payload-receiver"], "bodies": []}


def _measured_hints(report, path, directory):
    hints = motion_repair_hints(report)
    if hints and hints["kind"] == "stance-support-mass-layout":
        trace = path.with_name("stance-support.json")
        if not trace.is_file() or not trace.with_name("mass-layout.json").is_file():
            logger.warning("Stance repair evidence lacks its recorded trace or mass layout: %s", path)
            return None
        hints["supportTrace"] = trace.relative_to(directory).as_posix()
    return hints


def motion_rejections(archive):
    from .evolution import evidence_invalidation
    if evidence_invalidation(archive.protocol()):
        return {}
    found = {}
    if archive.db.execute("SELECT 1 FROM sqlite_master WHERE name='simulation_attempts'").fetchone():
        for row in archive.db.execute("""
            SELECT a.id,b.design FROM simulation_attempts a JOIN births b ON b.id=a.birth
            WHERE a.status='rejected' AND b.status='simulation-rejected' ORDER BY a.id
        """):
            path = archive.directory / "simulation-attempts" / f"{row['id']:04d}" / "simulation-gate.json"
            report = _load(path)
            if report is None:
                continue
            hints = _measured_hints(report, path, archive.directory)
            if hints:
                found[row["design"]] = {"report": path.relative_to(archive.directory).as_posix(),
                                        "repairHints": hints}
    for row in archive.db.execute("""
        SELECT e.id,e.result,b.design FROM evaluations e JOIN births b ON b.id=e.birth
        WHERE e.status='complete' AND b.status='evaluated' AND e.result IS NOT NULL ORDER BY e.id
    """):
        report = json.loads(row["result"])
        hints = _measured_hints(report, archive.directory / f"evaluations/{row['id']:04d}/result.json", archive.directory)
        if hints:
            found[row["design"]] = {"report": f"evaluations/{row['id']:04d}/result.json", "repairHints": hints}
    return found


def rejection_html(detail, report=None, prefix=""):
    link = (f'<p><a href="{escape(prefix + report, quote=True)}">Detailed screening report (JSON)</a></p>'
            if report else "")
    return ('<section class="panel bad" data-screening-rejection>'
            f'<h2>Screening rejection</h2><p>{escape(detail)}</p>{link}</section>')


def birth_page(text, directory, relative):
    match = re.fullmatch(r"history/(\d+)\.html", relative)
    if match is None or "data-screening-rejection" in text:
        return text
    directory = Path(directory)
    with sqlite3.connect(f"{(directory / 'archive.sqlite3').resolve().as_uri()}?mode=ro", uri=True) as db:
        row = db.execute("SELECT design,error FROM births WHERE id=? AND status='screening-rejected'",
                         (int(match[1]),)).fetchone()
    if row is None:
        return text
    info = rejections(directory).get(row[0])
    if info is None:
        return text
    section = rejection_html(info["detail"], info["report"], "../")
    old = f'<section class="panel bad">{escape(row[1] or "")}</section>'
    if old in text:
        return text.replace(old, section, 1)
    return text.replace("</body>", section + "</body>", 1)
