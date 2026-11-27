"""Carry motion/control intent through current bounded construction, never legacy hardware fallback."""

import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from .bridge import AdapterError, check, load_manifest, write_json
from .mass_layout import mass_layout_gate
from .payload import load_profile

POLICY = {
    "version": "motion-first-carry-forward-v5",
    "samples": 96, "groundClearanceLdu": 8,
    "packagingVersion": "packaging-realization-v27",
    "theoryVersion": "walker-construction-theory-v30",
    "scaffold": "Bounded backbone relocations with stock routes to retained walking interfaces; original frame remains an alternative. Exact whole-chassis connectivity remains mandatory.",
    "search": "Choose smallest support domains first and propagate connector compatibility; factor gait/bearing caches and minimize local conflicts with bounded joint axle counterfactuals. Truncated route domains do not prove physical impossibility.",
    "maximumBuildAttempts": 4, "plateLayers": [1, 2],
    "preserves": "Reference walking motion and electrical control; old physical stock is realization advice.",
    "placement": "Try estimated CoM centering at fixed leg mounts, then inherited placement preferences; all new routes remain mandatory.",
    "motorPlacement": "Try housing-centered and bounded inward/outward axial positions with compatible input-mesh planes; retain the nominal placement as a search alternative.",
    "balance": "Rank joint hardware layouts by normalized estimated whole-robot horizontal COM offset before routing. Correct battery placement for each motor/receiver alternative; source mass and swept-foot bounds are heuristic, never qualification.",
    "deduplication": "Effective request identities include the full request and frozen theory/oracle scope; an identical request is never searched twice within an obligation.",
    "inertia": "Estimated diagnostics, not a universal smaller/larger reward.",
    "repairs": "Existing exact-screening feedback, repair depth and proposal/reconstruction limits remain in force.",
    "failure": "Explicit construction rejection; no fallback to simulating the inherited assembly.",
    "scope": "Ordinary campaign parents, contributions and offspring; immutable targeted/requalification comparisons remain exact.",
}


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _binaries(generator):
    return {name: _hash(Path(generator.command[1]).with_name(name))
            for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll")}


def _centering(parent, library, profile):
    checked = check(parent / "manifest.json", library)
    layout = mass_layout_gate(checked, profile)
    program = json.loads((parent / "program.json").read_text())["program"]
    genome = json.loads((parent / "realization.json").read_text())
    ground, = [body for body in program["bodies"] if body["grounded"]]
    driven = {program["motor"]["body"]}
    driven.update(edge[side] for edge in program["transmissions"] for side in ("input", "output"))
    anchors = []
    for hinge in program["hinges"]:
        for side, other in (("a", "b"), ("b", "a")):
            port = hinge[side]
            if port["body"] == ground["id"] and hinge[other]["body"] not in driven:
                site = next(site for site in ground["sites"] if site["id"] == port["site"])
                anchors.append(site["position"])
    if not anchors:
        raise AdapterError("Mass centering requires fixed passive-leg mount sites.")
    origin = ground["initial"]["origin"]
    angle = ground["initial"]["angle"]
    points = np.array([[point["x"], point["y"]] for point in anchors])
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    center = (points @ rotation.T).mean(axis=0) + [origin["x"], origin["y"]]
    center_z = float(np.mean([foot["depth"] for foot in genome["feet"]]))
    battery = next((component for component in layout["components"]
                    if component["partInstance"] == "payload-battery"), None)
    if battery is None:
        return None, layout
    parts = {part["id"]: part for body in genome["bodies"] for part in body["parts"]}
    position = dict(parts["payload-battery"]["position"])
    com = layout["centerOfMassLdu"]
    for axis, target, index in (("x", center[0], 0), ("z", center_z, 2)):
        desired = position[axis] + (target - com[index]) * layout["totalMassKg"] / battery["massKg"]
        position[axis] = float(round(desired / 20) * 20)
    return {"payload-battery": position}, layout


def _balance_model(layout, motor_part="motor", scaffold_parts=()):
    try:
        total = layout["totalMassKg"]
        center = np.asarray(layout["centerOfMassLdu"], dtype=float)
        hull = np.asarray(layout["sweptFootHullXZLdu"], dtype=float)
        components = layout["components"]
        if (not np.isfinite(total) or total <= 0 or center.shape != (3,)
                or not np.isfinite(center).all() or hull.ndim != 2 or hull.shape[1] != 2
                or not len(hull) or not np.isfinite(hull).all()):
            raise ValueError("non-finite mass/center or invalid swept-foot bounds")
        lower, upper = hull.min(axis=0), hull.max(axis=0)
        span = (upper-lower)/2
        if (span <= 0).any():
            raise ValueError("degenerate horizontal support bounds")
        required = {motor_part, "payload-battery", "payload-receiver"}
        selected = [component for component in components if component["partInstance"] in required]
        if len(selected) != 3 or {c["partInstance"] for c in selected} != required:
            raise ValueError("missing or duplicate hardware masses")
        for component in selected:
            point = np.asarray(component["centerLdu"], dtype=float)
            if (not np.isfinite(component["massKg"]) or component["massKg"] <= 0
                    or point.shape != (3,) or not np.isfinite(point).all()):
                raise ValueError("invalid component mass or center")
        if sum(c["massKg"] for c in selected) > total + 1e-9:
            raise ValueError("hardware masses exceed total mass")
        scaffold = [c for c in layout.get("partMasses", []) if c["partInstance"] in scaffold_parts]
        if (len(scaffold) != len(set(scaffold_parts)) or len({c["partInstance"] for c in scaffold}) != len(scaffold)
                or any(not np.isfinite(c["massKg"]) or c["massKg"] <= 0
                       or np.asarray(c["centerLdu"]).shape != (3,) or not np.isfinite(c["centerLdu"]).all()
                       for c in scaffold)
                or sum(c["massKg"] for c in selected+scaffold) > total+1e-9):
            raise ValueError("missing, duplicate or invalid scaffold masses")
        point = lambda values: dict(zip(("x", "y", "z"), map(float, values)))
        support = (lower+upper)/2
        return {"totalMassKg": float(total), "centerOfMassLdu": point(center),
                "supportCenterLdu": point([support[0], center[1], support[1]]),
                "supportHalfSpanLdu": point([span[0], 0, span[1]]),
                "components": [{"part": c["partInstance"], "massKg": c["massKg"],
                                "centerLdu": point(c["centerLdu"])} for c in selected],
                "scaffoldComponents": [{"part": c["partInstance"], "massKg": c["massKg"],
                                        "centerLdu": point(c["centerLdu"])} for c in scaffold] if scaffold_parts else None}
    except (KeyError, TypeError, ValueError) as error:
        raise AdapterError(f"Invalid source balance estimate: {error}") from error


def rebuild(parent, output, generator, library, payload_profile=None, *, preserve_layout=False, observer=None):
    parent, output = Path(parent).resolve(), Path(output).resolve()
    manifest = load_manifest(parent / "manifest.json")
    source = {"genotypeId": manifest["motion"]["genotypeId"],
              "programSha256": _hash(parent / "program.json"),
              "realizationSha256": _hash(parent / "realization.json"),
              "preserveLayout": preserve_layout,
              "payloadProfileSha256": hashlib.sha256(json.dumps(
                  load_profile(payload_profile), sort_keys=True, separators=(",", ":")).encode()).hexdigest()}
    binaries = _binaries(generator)
    if output.exists():
        result_file = output / "result.json"
        if not result_file.is_file():
            raise AdapterError("Interrupted motion construction is preserved; a fresh obligation is required.")
        previous = json.loads(result_file.read_text())
        receipt = previous.get("receipt", previous)
        if (receipt.get("source") != source or receipt.get("builderBinaries") != binaries
                or receipt.get("policy") != POLICY):
            raise AdapterError("Existing motion construction belongs to a different source or constructor.")
        return previous
    output.mkdir(parents=True)
    if observer is not None:
        observer("planning", "Estimating source mass layout and current construction preferences.")
    hints, layout, diagnostics = None, None, []
    if not preserve_layout:
        try:
            hints, layout = _centering(parent, library, payload_profile)
        except AdapterError as error:
            diagnostics.append(f"Source mass-centering estimate unavailable: {error}. Construction still requires fresh exact validation.")
    if layout is not None:
        write_json(output / "source-mass-layout.json", layout)
    balance_file = None
    if layout is not None:
        balance_file = output / "balance-model.json"
        genome = json.loads((parent / "realization.json").read_text())
        manifest = load_manifest(parent / "manifest.json")
        ground = manifest["chassis"]
        hardware = {genome["motorPart"], "payload-battery", "payload-receiver"}
        scaffold_parts = [p["id"] for b in genome["bodies"] if b["body"] == ground
                          for p in b["parts"] if p["id"] not in hardware]
        scaffold_parts += [p["id"] for p in genome["mounts"]]
        write_json(balance_file, _balance_model(layout, genome["motorPart"], scaffold_parts))
    hint_file = output / "target-placements.json"
    if hints is not None:
        write_json(hint_file, hints)
    attempts = []
    seen_requests = []
    preferences = [hint_file, None] if hints is not None else [None]
    for placements in preferences:
        for layers in POLICY["plateLayers"]:
            folder = output / f"attempt-{len(attempts)+1:03d}"
            if observer is not None:
                observer("building", f"Building alternative {len(attempts)+1}/{len(preferences)*len(POLICY['plateLayers'])}: "
                         f"{'estimated CoM-centered' if placements else 'inherited'} placement preferences, {layers} plate layer(s). "
                         "Identical effective requests are skipped.")
            try:
                summary = generator.build_native(parent, folder, layers=layers, placements=placements,
                                                 seen_requests=seen_requests, balance_model=balance_file)
            except subprocess.TimeoutExpired as error:
                message = "Bounded motion construction timed out; partial output is preserved and inherited hardware is not substituted."
                attempts.append({"directory": folder.name, "layers": layers,
                                 "status": "construction-timeout", "error": str(error)})
                result = {"status": "construction-timeout", "attempts": attempts, "source": source,
                          "builderBinaries": binaries, "policy": POLICY, "diagnostics": diagnostics,
                          "error": message, "dynamicTrials": 0}
                write_json(output / "result.json", result)
                return result
            except AdapterError as error:
                attempts.append({"directory": folder.name, "layers": layers, "status": "construction-rejected",
                                 "error": str(error)})
                continue
            request_id = summary.get("motionFirst", {}).get("requestId")
            duplicate = summary.get("motionFirst", {}).get("status") == "duplicate-request"
            if request_id is not None and not duplicate:
                seen_requests.append(request_id)
            attempts.append({"directory": folder.name, "layers": layers,
                             "placement": "estimated-CoM" if placements else "inherited",
                             "requestId": request_id,
                             "status": "duplicate-request" if duplicate else "generated" if summary["generated"] else "construction-rejected",
                             "rejected": summary.get("rejected", []), "gates": summary["gates"]})
            if not summary["generated"]:
                continue
            receipt = {"policy": POLICY, "source": source, "builderBinaries": binaries,
                       "plateLayers": layers, "centeringPreferred": placements is not None,
                       "generated": summary["generated"], "diagnostics": diagnostics,
                       "motionValidatedBy": "Reference-body phase equivalence and unit input-clock check in current F# packaging.",
                       "dynamicTrials": 0}
            write_json(folder / "motion-carry-forward.json", receipt)
            result = {"status": "generated", "batch": str(folder), "attempts": attempts,
                      "receipt": receipt, "diagnostics": diagnostics, "dynamicTrials": 0}
            write_json(output / "result.json", result)
            return result
    result = {"status": "construction-rejected", "attempts": attempts, "source": source,
              "builderBinaries": binaries, "policy": POLICY,
              "diagnostics": diagnostics, "error": "No bounded current-builder realization; inherited hardware was not substituted.",
              "dynamicTrials": 0}
    write_json(output / "result.json", result)
    return result


def bind_candidate(archive, birth, native):
    archive.db.execute("""CREATE TABLE IF NOT EXISTS motion_candidates(
        birth INTEGER PRIMARY KEY REFERENCES births(id),batch TEXT NOT NULL,
        directory TEXT NOT NULL,files TEXT NOT NULL)""")
    archive.db.executescript("""
        CREATE TRIGGER IF NOT EXISTS immutable_motion_candidates_update BEFORE UPDATE ON motion_candidates
        BEGIN SELECT RAISE(ABORT,'Motion construction bindings are immutable'); END;
        CREATE TRIGGER IF NOT EXISTS immutable_motion_candidates_delete BEFORE DELETE ON motion_candidates
        BEGIN SELECT RAISE(ABORT,'Motion construction bindings are immutable'); END;
    """)
    existing = archive.db.execute(
        "SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth WHERE c.birth=?",
        (birth,)).fetchone()
    if archive.db.execute("SELECT 1 FROM motion_candidates WHERE birth=?", (birth,)).fetchone():
        if existing is None or _receipt(archive, existing) is None:
            raise AdapterError("Existing immutable construction binding is not current.")
        return
    with archive.db:
        archive.db.execute("INSERT INTO motion_candidates VALUES (?,?,?,?)",
                           (birth, native["batch"], native["directory"], json.dumps(native["files"])))


def current_pending(archive, pending):
    if archive.db.execute("SELECT 1 FROM sqlite_master WHERE name='motion_candidates'").fetchone():
        constructed = archive.db.execute("SELECT * FROM motion_candidates WHERE birth=?",
                                        (pending["birth"],)).fetchone()
        if constructed is not None:
            return {**dict(pending), **dict(constructed)}
    return pending


def _receipt(archive, pending, generator=None):
    from .native_episode import DEFERRED_CONSTRUCTION, _queue

    pending = current_pending(archive, pending)
    batch = archive.directory / pending["batch"]
    path = batch / "motion-carry-forward.json"
    deferred = archive.protocol().get("deferredConstruction") == DEFERRED_CONSTRUCTION
    summary, files = _queue(batch, motion_source=deferred and not path.is_file())
    if files != json.loads(pending["files"]):
        raise AdapterError("Motion realization batch changed after its immutable birth.")
    if not path.is_file():
        return None
    receipt = json.loads(path.read_text())
    recorded = archive.protocol().get("generatorBinarySha256", {})
    expected = (_binaries(generator) if generator is not None else
                {name: recorded.get(name) for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll")})
    if (receipt.get("policy") != POLICY or receipt.get("builderBinaries") != expected
            or receipt.get("dynamicTrials") != 0
            or receipt.get("generated") != summary["generated"]
            or not any(entry["genotypeId"] == pending["design"] for entry in summary["generated"])):
        raise AdapterError("Motion realization has a foreign or incomplete current-construction contract.")
    return receipt


def source_candidate(archive, pending, generator):
    from .native_episode import DEFERRED_CONSTRUCTION

    pending = current_pending(archive, pending)
    return _source_candidate(archive.directory, pending, generator,
                             archive.protocol().get("deferredConstruction") == DEFERRED_CONSTRUCTION)


def _source_candidate(directory, pending, generator, deferred, *, cache=None):
    from .native_episode import _queue, refresh_seed_exports

    batch = directory / pending["batch"]
    if deferred:
        _, files = _queue(batch, motion_source=True)
        if files != json.loads(pending["files"]):
            raise AdapterError("Queued motion inputs changed after version preparation.")
        batch = refresh_seed_exports(batch, cache or directory / "input-exports", generator)
    return batch / pending["directory"]


def construction_task(archive, pending, generator):
    from .native_episode import DEFERRED_CONSTRUCTION

    if archive.protocol().get("nativeDiscovery", {}).get("motionCarryForward") != POLICY:
        return None
    pending = current_pending(archive, pending)
    if _receipt(archive, pending, generator) is not None:
        return None
    birth = archive.db.execute("SELECT operation,status FROM births WHERE id=?", (pending["birth"],)).fetchone()
    if birth["status"] != "generated":
        raise AdapterError("Only a pending generated candidate can start construction.")
    return {"pending": dict(pending), "operation": birth["operation"],
            "builderBinaries": _binaries(generator),
            "deferred": archive.protocol().get("deferredConstruction") == DEFERRED_CONSTRUCTION}


def prepare_construction(task, directory, generator, library, payload_profile, observe):
    directory = Path(directory)
    pending = task["pending"]
    cache = directory / "construction-inputs" / f"{pending['birth']:06d}"
    observe("planning", "Importing source intent for current ground-aware construction.")
    source = _source_candidate(directory, pending, generator, task["deferred"], cache=cache / "exports")
    if task["deferred"]:
        from .leg_family_migration import convert
        batch = convert(source.parent, cache / "legs", generator)
        source = batch / source.name
    return rebuild(source, directory / "motion-builds" / f"{pending['birth']:06d}", generator, library, payload_profile,
                   preserve_layout=task["operation"] in (
                       "repair-realization", "battery-layout", "receiver-layout", "motor-layout", "motor-direction"),
                   observer=observe)


def finish_construction(archive, task, result):
    from .native_episode import _copy_candidate, _queue
    from .pipeline import record

    pending = task["pending"]
    if (not isinstance(result, dict) or result.get("status") not in (
            "generated", "construction-rejected", "construction-timeout")):
        raise AdapterError("Construction worker returned an invalid outcome.")
    output = archive.directory / "motion-builds" / f"{pending['birth']:06d}"
    if json.loads((output / "result.json").read_text()) != result:
        raise AdapterError("Construction worker result differs from its saved immutable outcome.")
    archive.db.execute("""CREATE TABLE IF NOT EXISTS motion_realizations(
        source_birth INTEGER PRIMARY KEY,target_birth INTEGER,status TEXT NOT NULL,receipt TEXT NOT NULL)""")
    if result["status"] != "generated":
        with archive.db:
            archive.db.execute("UPDATE births SET status='construction-rejected',error=? WHERE id=?",
                               (result["error"], pending["birth"]))
            archive.db.execute("INSERT INTO motion_realizations VALUES (?,NULL,?,?)",
                               (pending["birth"], result["status"], json.dumps(result)))
        record(archive, pending["birth"], "building", result["error"])
        return None
    batch = Path(result["batch"])
    if not batch.resolve().is_relative_to(output.resolve()):
        raise AdapterError("Construction worker returned a batch outside its isolated output.")
    summary, files = _queue(batch)
    receipt = result.get("receipt", {})
    if (receipt.get("policy") != POLICY or receipt.get("builderBinaries") != task["builderBinaries"]
            or receipt.get("dynamicTrials") != 0 or receipt.get("generated") != summary["generated"]):
        raise AdapterError("Construction worker returned a foreign or incomplete receipt.")
    entry, = summary["generated"]
    native = {"assembly": entry["assemblyId"], "batch": str(batch.relative_to(archive.directory)),
              "directory": entry["directory"], "files": files}
    if entry["genotypeId"] == pending["design"]:
        birth = pending["birth"]
        bind_candidate(archive, birth, native)
    else:
        candidate = batch / entry["directory"]
        manifest = load_manifest(candidate / "manifest.json")
        if not archive.db.execute("SELECT 1 FROM designs WHERE id=?", (entry["genotypeId"],)).fetchone():
            _copy_candidate(archive, batch, entry, files)
        birth, _ = archive.record_native_birth(
            manifest, parent=pending["birth"], operation="build-carried-motion",
            changed=[{"path": "construction", "before": [pending["design"]], "after": [entry["genotypeId"]]}],
            native=native)
        with archive.db:
            archive.db.execute("UPDATE births SET status='motion-carried-forward' WHERE id=?", (pending["birth"],))
    with archive.db:
        archive.db.execute("INSERT INTO motion_realizations VALUES (?,?,?,?)",
                           (pending["birth"], birth, "generated", json.dumps(result)))
    record(archive, birth, "building", "Current paired-foot, ground-reserved realization; exact admission and fresh dynamics still required.")
    if archive.db.execute("SELECT status FROM births WHERE id=?", (birth,)).fetchone()[0] == "duplicate":
        return None
    return current_pending(archive, archive.db.execute(
        "SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth WHERE c.birth=?",
        (birth,)).fetchone())


def ensure_pending(archive, pending, generator, library, payload_profile=None):
    from .pipeline import record

    if archive.protocol().get("nativeDiscovery", {}).get("motionCarryForward") != POLICY:
        return pending
    birth = archive.db.execute("SELECT status FROM births WHERE id=?", (pending["birth"],)).fetchone()
    if birth["status"] != "generated":
        return None
    task = construction_task(archive, pending, generator)
    if task is None:
        return current_pending(archive, pending)
    result = prepare_construction(task, archive.directory, generator, library, payload_profile,
                                  lambda stage, detail: record(archive, pending["birth"], stage, detail))
    return finish_construction(archive, task, result)


def require_construction(archive, birth, checked):
    if (archive.protocol().get("nativeDiscovery", {}).get("motionCarryForward") != POLICY
            or archive.protocol().get("requalificationCandidates")):
        return
    pending = archive.db.execute(
        "SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth WHERE c.birth=?", (birth,)).fetchone()
    if pending is None or _receipt(archive, pending) is None:
        raise AdapterError("Ordinary trial requires current motion-first construction; inherited hardware is not eligible.")
    if checked.manifest["motion"]["genotypeId"] != pending["design"]:
        raise AdapterError("Trial model differs from its current constructed motion realization.")
