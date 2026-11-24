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
    "version": "motion-first-carry-forward-v1",
    "samples": 96, "groundClearanceLdu": 8,
    "packagingVersion": "packaging-realization-v19",
    "theoryVersion": "walker-construction-theory-v22",
    "maximumBuildAttempts": 4, "plateLayers": [1, 2],
    "preserves": "Reference walking motion and electrical control; old physical stock is realization advice.",
    "placement": "Try estimated CoM centering at fixed leg mounts, then inherited placement preferences; all new routes remain mandatory.",
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


def rebuild(parent, output, generator, library, payload_profile=None, *, preserve_layout=False):
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
    hints, layout, diagnostics = None, None, []
    if not preserve_layout:
        try:
            hints, layout = _centering(parent, library, payload_profile)
        except AdapterError as error:
            diagnostics.append(f"Source mass-centering estimate unavailable: {error}. Construction still requires fresh exact validation.")
    if layout is not None:
        write_json(output / "source-mass-layout.json", layout)
    hint_file = output / "target-placements.json"
    if hints is not None:
        write_json(hint_file, hints)
    attempts = []
    preferences = [hint_file, None] if hints is not None else [None]
    for placements in preferences:
        for layers in POLICY["plateLayers"]:
            folder = output / f"attempt-{len(attempts)+1:03d}"
            try:
                summary = generator.build_native(parent, folder, layers=layers, placements=placements)
            except subprocess.TimeoutExpired as error:
                raise AdapterError("Bounded motion construction timed out; its unfinished obligation is preserved.") from error
            except AdapterError as error:
                attempts.append({"directory": folder.name, "layers": layers, "status": "construction-rejected",
                                 "error": str(error)})
                continue
            attempts.append({"directory": folder.name, "layers": layers,
                             "status": "generated" if summary["generated"] else "construction-rejected",
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
    from .native_episode import _queue

    pending = current_pending(archive, pending)
    batch = archive.directory / pending["batch"]
    summary, files = _queue(batch)
    if files != json.loads(pending["files"]):
        raise AdapterError("Motion realization batch changed after its immutable birth.")
    path = batch / "motion-carry-forward.json"
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


def ensure_pending(archive, pending, generator, library, payload_profile=None):
    from .native_episode import _copy_candidate, _queue
    from .pipeline import record

    if archive.protocol().get("nativeDiscovery", {}).get("motionCarryForward") != POLICY:
        return pending
    pending = current_pending(archive, pending)
    if _receipt(archive, pending, generator) is not None:
        return pending
    source = archive.directory / pending["batch"] / pending["directory"]
    operation = archive.db.execute("SELECT operation FROM births WHERE id=?", (pending["birth"],)).fetchone()[0]
    record(archive, pending["birth"], "planning", "Carrying motion/control intent into current ground-aware construction.")
    output = archive.directory / "motion-builds" / f"{pending['birth']:06d}"
    result = rebuild(source, output, generator, library, payload_profile,
                     preserve_layout=operation in ("repair-realization", "battery-layout", "receiver-layout", "motor-layout", "motor-direction"))
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
    summary, files = _queue(batch)
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
