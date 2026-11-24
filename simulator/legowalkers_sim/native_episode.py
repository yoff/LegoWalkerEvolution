"""Finite native-walker queues using the existing archive and trial accounting."""

import hashlib
import json
from pathlib import Path
import sys

from .bridge import AdapterError, check, load_manifest, logical_foot_count, write_json
from .evaluation import evaluate
from .mass_layout import mass_layout_gate
from .evolution import MAX_FEET, MIN_WALKER_FEET, SEARCH_POLICY, render_representative, validate_search_candidate
from .screening import GATE_ORDER, screen_batch
from .pipeline import record, screening_observer


def refresh_seed_exports(source, cache, generator):
    """Re-export historical seeds without changing their design or original files."""
    import shutil
    import tempfile
    from .bridge import SIMULATOR_REVISION

    source = Path(source).resolve()
    descriptor = source / "seeds.json"
    names = json.loads(descriptor.read_text())["batches"] if descriptor.exists() else [None]
    batches = []
    historical = False
    for name in names:
        batch = source if name is None else (source / name).resolve()
        if name is not None and batch.parent != source:
            raise AdapterError("Seed batches must be immediate pool subdirectories.")
        summary = json.loads((batch / "candidates.json").read_text())
        for entry in summary["generated"]:
            candidate = (batch / entry["directory"]).resolve()
            if candidate.parent != batch:
                raise AdapterError("Native candidates must be immediate batch subdirectories.")
            manifest = json.loads((candidate / "manifest.json").read_text())
            historical |= manifest["simulator"]["revision"] != SIMULATOR_REVISION
            batches.append((candidate, manifest, entry, summary))
    if not historical:
        seed_batches(source)
        return source
    if generator is None:
        raise AdapterError("Historical simulator inputs require the current pinned generator for verified import.")
    fingerprint = hashlib.sha256(SIMULATOR_REVISION.encode())
    fingerprint.update(Path(__file__).read_bytes())
    for path in sorted(source.rglob("*")):
        if path.is_file():
            fingerprint.update(str(path.relative_to(source)).encode()+b"\0"+path.read_bytes()+b"\0")
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        fingerprint.update(Path(generator.command[1]).with_name(name).read_bytes())
    cache = Path(cache).resolve()
    target = cache / fingerprint.hexdigest()
    if target.exists():
        seed_batches(target)
        return target
    cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="reexport-", dir=cache) as temporary:
        temporary = Path(temporary)
        rebuilt = temporary / "inputs"
        shutil.copytree(source, rebuilt)
        for index, (candidate, manifest, entry, summary) in enumerate(batches):
            output = temporary / f"export-{index}"
            generator.invoke(
                "realize-walker", "--program", candidate / "program.json",
                "--genome", candidate / "realization.json",
                "--steps", str(manifest["motion"]["cycleSteps"]), "--output", output)
            generated = output / "candidate-001"
            current = load_manifest(generated / "manifest.json")
            changed = [field for field in ("expected", "parts", "payloads", "genome")
                       if current.get(field) != manifest.get(field)]
            previous_motion = json.loads(json.dumps(manifest["motion"]))
            generated_motion = json.loads(json.dumps(current["motion"]))
            # Revision migration preserves saved contact semantics, not newer compiler defaults.
            for field in ("genotypeId", "logicalFeet"):
                previous_motion.pop(field, None)
                generated_motion.pop(field, None)
            if previous_motion != generated_motion:
                changed.append("motion")
            for name in ("program.json", "realization.json"):
                if json.loads((generated / name).read_text()) != json.loads((candidate / name).read_text()):
                    changed.append(name)
            if ((generated / current["modelFile"]).read_bytes()
                    != (candidate / manifest["modelFile"]).read_bytes()):
                changed.append("LDraw geometry")
            if (manifest["motion"]["genotypeId"] != entry["genotypeId"]
                    or current["motion"]["assemblyId"] != entry["assemblyId"]
                    or current["motion"]["mechanismId"] != summary["mechanismId"]):
                changed.append("batch identities")
            if changed:
                raise AdapterError(f"Simulator revision migration changed a seed design at {candidate}: "
                                   f"{', '.join(changed)}; refusing carry-forward.")
            destination = rebuilt / candidate.relative_to(source)
            updated = {**manifest, "simulator": current["simulator"]}
            write_json(destination / "manifest.json", updated)
            load_manifest(destination / "manifest.json")
            provenance = destination.parent / "export-migration.json"
            mapping = json.loads(provenance.read_text()) if provenance.exists() else {
                "schemaVersion": 1, "simulatorRevision": SIMULATOR_REVISION, "designs": {}}
            mapping["designs"][entry["genotypeId"]] = {
                "sourceManifestSha256": hashlib.sha256((candidate / "manifest.json").read_bytes()).hexdigest(),
                "sourceSimulatorRevision": manifest["simulator"]["revision"],
                "regeneratedManifestSha256": hashlib.sha256((generated / "manifest.json").read_bytes()).hexdigest(),
                "ldrawByteIdentical": True,
                "preserved": ["designIdentities", "physicalMetadata", "contactRegions", "motionMetadata"],
            }
            write_json(provenance, mapping)
        seed_batches(rebuilt)
        rebuilt.rename(target)
    return target


def _queue(source):
    source = source.resolve()
    summary = json.loads((source / "candidates.json").read_text())
    if (not isinstance(summary, dict) or summary.get("schemaVersion") != 1
            or summary.get("gateVersion") != "motion-gates-v1" or "mechanismId" not in summary):
        raise AdapterError("Expected a versioned F# realization batch.")
    entries = summary.get("generated", [])
    if not isinstance(entries, list) or not 1 <= len(entries) <= 32:
        raise AdapterError("A native episode needs one to 32 explicitly realized walker candidates.")
    files = {"candidates.json": hashlib.sha256((source / "candidates.json").read_bytes()).hexdigest()}
    for name in ("feedback.json", "conflicts.json", "grammar-search.json", "export-migration.json",
                 "leg-family-migration.json", "request.json", "construction-search.json", "motion-carry-forward.json"):
        if (source / name).is_file():
            files[name] = hashlib.sha256((source / name).read_bytes()).hexdigest()
    if "builderPolicyId" in summary:
        builder = json.loads((source / "builder.json").read_text())
        if not isinstance(builder, dict) or builder.get("policyId") != summary["builderPolicyId"]:
            raise AdapterError("Builder policy identity differs from its immutable generation batch.")
        files["builder.json"] = hashlib.sha256((source / "builder.json").read_bytes()).hexdigest()
    names, identities = set(), set()
    for entry in entries:
        if (not isinstance(entry, dict) or not {"directory", "assemblyId", "genotypeId"}.issubset(entry)
                or not isinstance(entry["directory"], str) or not entry["directory"]):
            raise AdapterError("Every native entry needs a candidate directory and both identities.")
        candidate = (source / entry["directory"]).resolve()
        if candidate.parent != source or candidate.name in names:
            raise AdapterError("Native candidates must be unique immediate batch subdirectories.")
        names.add(candidate.name)
        manifest = load_manifest(candidate / "manifest.json")
        if manifest["grammarVersion"] != "technic-motion-v3":
            raise AdapterError("Native episodes require complete motion-v3 walkers, not leg prototypes.")
        motion = manifest["motion"]
        if (motion["mechanismId"] != summary["mechanismId"] or motion["assemblyId"] != entry["assemblyId"]
                or motion["genotypeId"] != entry["genotypeId"]):
            raise AdapterError("Native candidate identities differ from the generation batch.")
        if motion["genotypeId"] in identities:
            raise AdapterError("Native episode queues must not repeat a genotype.")
        identities.add(motion["genotypeId"])
        names_to_freeze = ["manifest.json", manifest["modelFile"], "program.json", "realization.json", "gates.json", "cycle.svg"]
        for name in ("construction.json", "grammar.json", "initial-ground-clearance.json", "motion-phases.json",
                     "motion-space.json", "construction-proof.json", "ground-plan.json"):
            if (candidate / name).is_file():
                names_to_freeze.append(name)
        for name in names_to_freeze:
            path = (candidate / name).resolve()
            if path.parent != candidate:
                raise AdapterError("Native candidate files must stay inside their candidate directory.")
            files[f"{candidate.name}/{name}"] = hashlib.sha256(path.read_bytes()).hexdigest()
    return summary, files


def seed_batches(source):
    descriptor = source / "seeds.json"
    if not descriptor.exists():
        summary, files = _queue(source)
        return [(source, summary, files)]
    from .population import POLICY as population_policy

    pool = json.loads(descriptor.read_text())
    rechecks = pool.get("requalificationBatches", []) if isinstance(pool, dict) else []
    if (not isinstance(rechecks, list) or len(rechecks) > 32
            or any(not isinstance(name, str) for name in rechecks)
            or len(set(rechecks)) != len(rechecks)):
        raise AdapterError("Expected at most 32 unique explicitly scheduled requalification batches.")
    capacity = population_policy["capacity"] + len(rechecks)
    if (not isinstance(pool, dict) or pool.get("schemaVersion") != 1
            or not isinstance(pool.get("batches"), list) or not 1 <= len(pool["batches"]) <= capacity
            or any(not isinstance(name, str) or not name for name in pool["batches"])
            or len(set(pool["batches"])) != len(pool["batches"])):
        raise AdapterError(f"Expected one to {capacity} unique seed-batch directories.")
    if not set(rechecks) <= set(pool["batches"]):
        raise AdapterError("Requalification batches must belong to the immutable seed pool.")
    batches = []
    for name in pool["batches"]:
        batch = (source / name).resolve()
        if batch.parent != source.resolve():
            raise AdapterError("Seed batches must be immediate pool subdirectories.")
        summary, files = _queue(batch)
        batches.append((batch, summary, files))
    return batches


def native_protocol(base, source, *, discover=False, reuse_archives=(), simulation_workers=1, screening_workers=1):
    from .parallel import worker_count

    worker_count(simulation_workers)
    from .prescreen import validate_workers
    validate_workers(screening_workers)
    if screening_workers > 1 and not discover:
        raise AdapterError("Parallel screening currently requires native discovery.")
    batches = seed_batches(source)
    if len(batches) > 1 and not discover:
        raise AdapterError("Multiple seed neighborhoods require --discover.")
    if (source / "seeds.json").exists():
        files = {"seeds.json": hashlib.sha256((source / "seeds.json").read_bytes()).hexdigest()}
        if (source / "leg-family-migration.json").is_file():
            files["leg-family-migration.json"] = hashlib.sha256((source / "leg-family-migration.json").read_bytes()).hexdigest()
        for batch, _, fingerprints in batches:
            files.update({f"{batch.name}/{name}": digest for name, digest in fingerprints.items()})
    else:
        files = batches[0][2]
    identifiers = [entry["genotypeId"] for _, summary, _ in batches for entry in summary["generated"]]
    if len(set(identifiers)) != len(identifiers):
        raise AdapterError("The seed pool must not repeat a founder genotype.")
    protocol = {
        **base, "grammarVersion": "technic-motion-v3",
        "pipelineTracking": "candidate-pipeline-v2-planning-and-builder-attempts",
        "categories": [{"footCount": count} for count in range(MIN_WALKER_FEET, MAX_FEET+1)],
        "searchPolicy": SEARCH_POLICY, "candidateBatch": files,
        "nativeCandidates": identifiers,
        "execution": {"version": "native-processes-v1", "simulationWorkers": simulation_workers,
                      "screeningWorkers": screening_workers, "screeningBuffer": 4,
                      "selection": "Asynchronous; parents come from completed compatible observations only."},
        "schedule": "Finite immutable queue, screened before trial reservation; no automatic topology evolution.",
    }
    if base.get("evaluation", {}).get("short_gate", 0) > 0:
        protocol["execution"]["simulationGateAccounting"] = {
            "version": "prefix-before-trial-v1", "maxAttemptsPerFullSimulation": 3,
            "constructionBudgetScope": "per-prefix-attempt",
            "limit": "Per unfilled full-simulation slot per explicitly started episode; pause when exhausted.",
        }
    if discover:
        from .discovery import POLICY
        protocol.update(nativeDiscovery=POLICY,
                        schedule="Balance new trials across founder neighborhoods; reused observations spend no authorization.")
        if (source / "seeds.json").exists():
            rechecks = json.loads((source / "seeds.json").read_text()).get("requalificationBatches", [])
            if rechecks:
                protocol["requalificationCandidates"] = [
                    entry["genotypeId"] for batch, summary, _ in batches if batch.name in rechecks
                    for entry in summary["generated"]]
    if reuse_archives:
        if not discover:
            raise AdapterError("Reusing observations currently requires native discovery.")
        protocol["reuseArchives"] = [str(Path(path).resolve()) for path in reuse_archives]
    return protocol


def _freeze_batch(source, target, files):
    for name, expected in files.items():
        path = (source / name).resolve()
        destination = (target / name).resolve()
        if not path.is_relative_to(source.resolve()) or not destination.is_relative_to(target.resolve()):
            raise AdapterError("Native batch files must remain inside their input and archive directories.")
        content = path.read_bytes()
        if hashlib.sha256(content).hexdigest() != expected:
            raise AdapterError("Native batch changed after protocol creation; use a fresh archive.")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if destination.read_bytes() != content:
                raise AdapterError("Archived native inputs changed; refusing to mix candidates.")
        else:
            destination.write_bytes(content)

def _copy_candidate(archive, frozen, entry, files):
    candidate = frozen / entry["directory"]
    manifest = load_manifest(candidate / "manifest.json")
    design = archive.directory / "designs" / entry["genotypeId"]
    destination = design / "candidate"
    # Construction provenance belongs to the birth, not its potentially shared design.
    candidate_files = {name.removeprefix(entry["directory"]+"/"): digest for name, digest in files.items()
                       if name.startswith(entry["directory"]+"/")
                       and not name.endswith(("/construction.json", "/grammar.json"))}
    _freeze_batch(candidate, destination, candidate_files)
    write_json(design / "genome.json", manifest["genome"])
    return manifest, destination


def stage_native(archive, source):
    source = source.resolve()
    files = native_protocol({}, source, discover=bool(archive.protocol().get("nativeDiscovery")))["candidateBatch"]
    if archive.protocol().get("candidateBatch") != files:
        raise AdapterError("Native batch differs from the archive's immutable episode protocol.")
    frozen = archive.directory / "native-input"
    _freeze_batch(source, frozen, files)
    for batch, summary, fingerprints in seed_batches(frozen):
        for entry in summary["generated"]:
            manifest, _ = _copy_candidate(archive, batch, entry, fingerprints)
            if not archive.db.execute("SELECT 1 FROM designs WHERE id=?", (entry["genotypeId"],)).fetchone():
                native = ({"assembly": entry["assemblyId"], "batch": str(batch.relative_to(archive.directory)),
                           "directory": entry["directory"], "files": fingerprints}
                          if archive.protocol().get("nativeDiscovery") else None)
                archive.record_native_birth(manifest, native=native)


class PreflightRejection(AdapterError):
    def __init__(self, status, message, layout=None):
        super().__init__(message)
        self.status, self.layout = status, layout


def prepare_native_candidate(manifest_path, library, options):
    manifest = load_manifest(manifest_path)
    gates = json.loads((manifest_path.parent / "gates.json").read_text())
    if [gate["gate"] for gate in gates] != GATE_ORDER:
        raise AdapterError("Native candidates require all ordered preflight gates.")
    checked = check(manifest_path, library)
    validate_search_candidate(checked, logical_foot_count(manifest))
    try:
        layout = mass_layout_gate(checked, options.payload_profile)
    except AdapterError as error:
        raise PreflightRejection("mass-layout-rejected", str(error)) from error
    if options.cycle_aware:
        from .cycles import planned_options
        try:
            planned_options(manifest, options)
        except AdapterError as error:
            raise PreflightRejection("evaluation-envelope-rejected", str(error), layout) from error
    return checked, layout


def evaluate_native_candidate(archive, birth, manifest_path, library, options, blender=None, *, pool=None):
    from .gallery import publish
    record(archive, birth, "preflight", "Mass layout, contact policy and evaluation envelope.")
    destination = archive.directory / "designs" / archive.db.execute(
        "SELECT design FROM births WHERE id=?", (birth,)).fetchone()[0]
    try:
        checked, layout = prepare_native_candidate(manifest_path, library, options)
    except PreflightRejection as error:
        if error.layout is not None:
            write_json(destination / "mass-layout.json", error.layout)
        with archive.db:
            archive.db.execute("UPDATE births SET status=?,error=? WHERE id=?", (error.status, str(error), birth))
        print(f"ERROR: Candidate rejected before trial reservation: {error}", file=sys.stderr, flush=True)
        publish(archive)
        return 1
    manifest = checked.manifest
    write_json(destination / "mass-layout.json", layout)
    write_json(destination / "compatibility.json", checked.report)
    record(archive, birth, "ready", "Preflight passed; awaiting trial reservation.")
    if pool is not None:
        pool.submit(birth, checked, options)
        return 0
    if archive.protocol().get("execution", {}).get("simulationGateAccounting"):
        from .parallel import TrialPool
        with TrialPool(archive, 1, evaluate, blender) as pool:
            pool.submit(birth, checked, options)
        return pool.failures
    from .motion_carry import require_construction
    require_construction(archive, birth, checked)
    trial = archive.reserve(birth)
    output = archive.directory / "evaluations" / f"{trial:04d}"
    publish(archive)
    print(f"Native trial {trial}: {logical_foot_count(manifest)} feet, {manifest['motion']['genotypeId'][:10]}", flush=True)
    failures = 0
    try:
        result = evaluate(checked, output, options)
        archive.finish(trial, result)
    except (AdapterError, OSError) as error:
        archive.finish(trial, error=str(error), status="failed")
        write_json(output / "error.json", {"status": "failed", "error": str(error)})
        failures = 1
        print(f"ERROR: Native trial {trial} failed: {error}", file=sys.stderr, flush=True)
    publish(archive)
    if blender is not None:
        representative = archive.best(f"{logical_foot_count(manifest)}-feet")
        if representative is not None:
            render_representative(archive, representative, blender)
            publish(archive)
    return failures


def search_native(archive, source: Path, library: Path, options, blender=None, *, workers=1):
    from .gallery import publish
    from .parallel import TrialPool, worker_count

    worker_count(workers)

    source = source.resolve()
    archive.library = library
    protocol = archive.protocol()
    summary, files = _queue(source)
    if protocol.get("grammarVersion") != "technic-motion-v3" or protocol.get("candidateBatch") != files:
        raise AdapterError("Native batch differs from the archive's immutable episode protocol.")
    frozen = archive.directory / "native-input"
    _freeze_batch(source, frozen, files)
    archive.recover()
    archive.set_status("running")
    publish(archive)
    failures = 0
    try:
        screened = screen_batch(frozen, archive.directory / "preflight", library, max_checks=32,
                                payload_profile=options.payload_profile, observer=screening_observer(archive))
        reports = {Path(report["candidate"]).name: report for report in screened["results"]}
        budget = archive.db.execute("SELECT budget FROM run WHERE id=1").fetchone()[0]
        with TrialPool(archive, workers, evaluate, blender, renderer=render_representative) as pool:
            try:
                for entry in summary["generated"]:
                    pool.reap()
                    if sum(archive.counts().values()) >= budget or pool.exhausted:
                        break
                    while pool.full:
                        pool.reap(block=True)
                        if sum(archive.counts().values()) >= budget or pool.exhausted:
                            break
                    if sum(archive.counts().values()) >= budget:
                        break
                    if pool.exhausted:
                        break
                    identifier = entry["genotypeId"]
                    previous = archive.db.execute("""
                        SELECT b.id,b.status,e.id AS trial FROM births b LEFT JOIN evaluations e ON e.birth=b.id
                        WHERE b.design=? ORDER BY b.id DESC LIMIT 1
                    """, (identifier,)).fetchone()
                    if previous is not None and (previous["trial"] is not None or previous["status"] != "generated"):
                        continue
                    report = reports.get(entry["directory"])
                    if report is None:
                        raise AdapterError(f"Native screening omitted candidate {entry['directory']}.")
                    candidate = frozen / entry["directory"]
                    manifest = load_manifest(candidate / "manifest.json")
                    ready = (report["status"] == "ready-for-dynamics"
                             or (report["status"] == "duplicate" and report["previousStatus"] == "ready-for-dynamics"))
                    if not ready:
                        from .screening_reports import rejection_detail
                        message = f"Native candidate {identifier[:10]} failed load-only screening: {rejection_detail(report)}"
                        archive.record_native_birth(manifest, status="screening-rejected", error=message)
                        failures += 1
                        print(f"ERROR: {message}", file=sys.stderr, flush=True)
                        continue
                    _, destination = _copy_candidate(archive, frozen, entry, files)
                    birth = previous["id"] if previous is not None else archive.record_native_birth(manifest)[0]
                    failures += evaluate_native_candidate(archive, birth, destination / "manifest.json",
                                                          library, options, pool=pool)
            except (AdapterError, OSError, json.JSONDecodeError):
                pool.drain()
                raise
        failures += pool.failures
        outcome = "gate-limit" if pool.exhausted else "complete"
        archive.set_status("complete")
    except (AdapterError, OSError, KeyboardInterrupt):
        archive.set_status("paused")
        publish(archive)
        raise
    publish(archive)
    return {"status": outcome, "evaluations": archive.counts(), "failures": failures,
            "cheapRejections": pool.rejections,
            "page": str(archive.directory / "preview.html")}
