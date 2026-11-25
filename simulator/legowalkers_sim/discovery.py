"""Bounded native motion mutation, physical alternatives, screening and selection."""

import json
import math
from pathlib import Path
import sys

from .bridge import AdapterError, write_json
from .evolution import canonical, now
from .native_episode import _copy_candidate, _queue, evaluate_native_candidate, stage_native
from .screening import cache_directory, screen_batch
from .observations import reuse_observations
from .population import POLICY as POPULATION_POLICY, choose, family, family_map, members
from .pipeline import screening_observer
from .screening_reports import motion_rejections, rejection_detail, rejections
from .user_pool import POLICY as USER_POOL_POLICY
from .leg_family_migration import POLICY as LEG_FAMILY_MIGRATION
from .motion_carry import POLICY as MOTION_CARRY_POLICY, construction_task, current_pending, ensure_pending, source_candidate

POLICY = {
    "version": "native-discovery-v40", "mutationVersion": "native-motion-mutations-v23",
    "motionCarryForward": MOTION_CARRY_POLICY,
    "physicalDeduplication": "assembly-and-controller-v1; polarity and torque limit distinguish trials",
    "polarityFeedback": {"version": "safe-backward-gait-repair-v1", "maximumAttemptsPerBirth": 1,
                         "qualification": "Fresh trial required; never inherit fitness.",
                         "mutation": "Control-only motor-direction, after consistent negative complete cycles and all other gates pass."},
    "massLayoutGate": {"version": "mass-layout-v2", "mass": "derived-from-parts-and-payload-profile",
                       "supportEnvelope": "diagnostic-only-no-static-balance-rejection"},
    "stanceSupportFeedback": {
        "version": "measured-stance-support-v1", "authority": "advisory-only-not-a-qualification-gate",
        "objective": "Reduce unsupported fraction, longest interval, and outside distance; never demand permanent static support.",
        "repairs": "Choose existing motion-preserving battery/receiver edits using frozen recorded contacts and estimated payload CoM shifts.",
        "limits": "Existing repair depth/realization limits apply. Custom mounts may have no supported edit. Fresh authorized trials remain mandatory.",
    },
    "limbProportions": {
        "version": "motion-first-stock-limb-lengths-v2",
        "mutation": "Change passive hinge spans by one Technic hole; coordinate opposite parallelogram spans.",
        "realization": "Refit supported stock beams and smooth pins instead of restricting motion to inherited holes.",
        "preserves": "Shared leg shape, mirrored mounting, relative phases, chassis anchors, motor and transmissions.",
        "limits": "Single-piece straight passive links without rigid mounts; existing catalogue, closure and exact reconstruction gates remain mandatory.",
    },
    "initialFootClearance": {
        "version": "initial-foot-clearance-v1", "authority": "advisory-builder-ranking-and-targeted-repair",
        "activation": "Pre-measurement forbidden floor contact before any permitted foot or wheel contact.",
        "repair": "Refit passive stock beams at unchanged sockets, coordinating homologous shared-family placements.",
        "limits": "Conservative initial envelopes only; not a balance or whole-gait gate. Existing repair depth, proposal, reconstruction and fresh trial limits apply.",
    },
    "maxProposalsPerTrial": 16, "maxRealizationsPerProposal": 2, "maxReconstructionChecksPerTrial": 8,
    "limitHandling": "finish-in-flight-work-before-pausing-v1",
    "selection": "Balanced original founder families; tournament/random selection from a bounded diverse population.",
    "population": POPULATION_POLICY,
    "userPool": USER_POOL_POLICY,
    "mutations": ["move-hinge-site", "input-phase", "branch-phase", "add-dyad", "remove-dyad",
                  "duplicate-branch", "remove-branch", "lengthen-foot", "bend-foot", "gear-reduction",
                  "battery-layout", "receiver-layout", "motor-layout", "add-hinged-foot", "motor-direction",
                  "replace-double-parallelogram-foot", "split-leg-family", "gait-half-cycle",
                  "shared-leg-move-hinge-site", "shared-leg-add-dyad", "shared-leg-remove-dyad",
                  "shared-leg-lengthen-foot", "shared-leg-bend-foot", "shared-leg-add-hinged-foot",
                  "shared-leg-replace-double-parallelogram-foot", "limb-length", "shared-leg-limb-length"],
    "legFamilies": {"version": "shared-planar-leg-families-v1",
                    "legacyConversion": LEG_FAMILY_MIGRATION,
                    "default": "New founders share a leg template; immutable upgrades migrate exact legacy copies.",
                    "mutation": "Atomic homologous leg edits preserve instance mounting, mirrored handedness and relative phases.",
                    "asymmetry": "Explicit split-leg-family; instance layout changes remain separate operations.",
                    "qualification": "Sharing is a genotype constraint, never evidence of walking fitness."},
    "levelFootReplacement": {"version": "independent-double-parallelogram-v4", "maximumChoicesPerFoot": 4,
                             "construction": "Generic motion productions; socket-preserving upright growth and shortest fully reached stock axles, including 10L.",
                             "lOrientation": "Prefer long arm upright, real molded bend/tip at corner/toe, and short branch away from grounded arm throughout sampled motion. Other admissible orientations remain alternatives, not hard rejections.",
                             "selection": "Preferred outward layouts retain the existing dyad branch when available. Reserve one of four choices for alternatives. Preserve construction rank and preference in the manual edit catalogue instead of sorting them away by hash. All exact gates remain mandatory.",
                             "anchors": "Prefer existing grounded anchor positions, dyad assembly branch and full 80 LDU long-arm span before other supported sockets/branches. Correcting toe direction should not gratuitously mirror the entire leg motion.",
                             "replacement": "Already-flat nonpreferred native legs remain replaceable; replacement identifiers cannot collide with retained anchors.",
                             "preserves": "Motor/controller settings, transmissions, foot identities and carried hardware.",
                             "qualification": "Fresh exact reconstruction and calibrated walking trial required; reference remains a holdout."},
    "mutationSelection": {"policy": "uniform-applicable-families-v1",
                          "selection": "Uniform applicable family, then uniform edit; seeded per-parent replay by attempt.",
                          "applicability": "Enumerator availability, not preflight or walking success."},
    "topologyMutation": True,
    "construction": {"version": "technic-construction-v11",
                     "motionFirstBuilder": {
                         "version": "sampled-motion-ground-reservation-v2",
                         "theoryVersion": "walker-construction-theory-v22",
                         "packagingVersion": "packaging-realization-v19",
                         "motorMountMotifs": ["front-forward", "front-rearward", "rear-saddle"],
                         "fixedHardware": "Support parts must also clear fixed motor/payload stock envelopes; coaxial connector engagement remains permitted.",
                         "routing": "Reject unsafe bridge extensions before recursing, so obstructed prefixes cannot consume the complete-route candidate cap.",
                         "activation": "build-walker and construct-walker without feedback reserve the full geared gait before hardware/support placement.",
                         "maximumStates": 512, "maximumFrontier": 64, "maximumTargetAssignments": 32,
                         "maximumRouteCandidatesPerTopology": 512,
                         "scope": "Conservative stock envelopes with declared connector/socket engagement; exact phase gate still mandatory.",
                         "limits": "No changes to export, reconstruction, prefix, repair-depth or full-trial allowances.",
                     },
                     "fullPhaseClearance": {
                         "version": "prescribed-phase-clearance-v1", "maximumSamples": 2329,
                         "scope": "Complete reconstructed assembly, including connector and mount collision geometry.",
                         "admission": "New phase-bearing exports must clear all prescribed samples before dynamics; legacy exports warn until re-exported.",
                         "repairs": "Measured hardware conflicts consider supported payload/layout alternatives as well as connection grammar.",
                         "limits": "Existing proposal, export, repair-depth and reconstruction caps. Sampled clearance is not continuous or walking certification.",
                     },
                     "motionClearance": {"version": "kinematic-clearance-v1", "authority": "advisory-ranking-only",
                                         "maxLayoutCandidates": 12, "maxMotorTurns": 32, "maxRefinedIntervals": 8,
                                         "selection": "Retain inherited realization first; use existing alternative slots for improved motion-preserving hardware layouts. Explicit feedback also ranks supported passive-link depth repairs."},
                     "motifs": ["smooth-passive-pivot", "passive-link-layer-clearance", "connection-link-grammar-v2",
                                "initial-foot-clearance-beam-fit"],
                     "grammar": {"version": "connection-link-grammar-v2",
                                 "activation": "Initial or moving self-interference repair with exact reported contact pairs above the existing penetration tolerance.",
                                 "axles": "Supported stock spans/placements preserve every originally reached part and complete socket walls.",
                                 "maximumStates": 16384, "maximumLayouts": 256,
                                 "maximumStacksPerLayout": 32, "maximumMovedLinks": 2},
                     "builder": {"policyVersion": "symbolic-builder-policy-v1",
                                 "catalogueVersion": "technic-construction-catalogue-v2",
                                 "selection": "Frozen compiled default; offline alternatives require manual promotion."},
                     "maxRepairDepth": 2, "maxRealizationsPerRepair": 1,
                     "selection": "Initial and moving-overlap repairs use connection grammar; partition repairs retain pin/layer motifs; pre-foot floor contact uses stock-beam refitting. All share existing proposal, depth and reconstruction limits; repaired children require fresh trials."},
}


class DiscoveryLimit(AdapterError):
    """A persistent proposal or reconstruction allowance is exhausted."""


def _pending(archive, founder=None, *, exclude=()):
    groups = family_map(archive)
    rows = archive.db.execute("""
        SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth
        LEFT JOIN evaluations e ON e.birth=b.id
        WHERE b.status='generated' AND e.id IS NULL
        AND NOT EXISTS(SELECT 1 FROM reused_evaluations r WHERE r.birth=b.id)
        ORDER BY CASE WHEN b.operation='user-contribution' THEN 0 ELSE 1 END,b.id
    """).fetchall()
    return next((row for row in rows if row["birth"] not in exclude
                 and (founder is None or family(archive, row["birth"], groups) == founder)), None)


def _repair_pending(archive, founder=None):
    attempted = {row["parent"] for row in archive.db.execute("SELECT parent,summary FROM native_proposals")
                 if row["summary"] and "repairOf" in json.loads(row["summary"])}
    groups = family_map(archive)
    reports = None
    for row in archive.db.execute("SELECT id,design,parent FROM births WHERE status IN ('screening-rejected','simulation-rejected','evaluated') ORDER BY id"):
        if row["id"] in attempted or (founder is not None and family(archive, row["id"], groups) != founder):
            continue
        depth, ancestor = 0, row["id"]
        while ancestor is not None:
            prior = archive.db.execute("SELECT parent,operation FROM births WHERE id=?", (ancestor,)).fetchone()
            if prior["operation"] != "repair-realization":
                break
            depth, ancestor = depth+1, prior["parent"]
        if depth >= POLICY["construction"]["maxRepairDepth"]:
            continue
        if reports is None:
            reports = rejections(archive.directory)
            reports.update(motion_rejections(archive))
        info = reports.get(row["design"])
        if info and info["repairHints"]:
            return {"birth": row["id"], "design": row["design"], "depth": depth, **info}
    return None


def backwards_only(result):
    metrics, gait = result.get("metrics", {}), result.get("gait", {})
    reasons = set(metrics.get("rankingRejectionReasons", []))
    intervals = gait.get("cycleIntervals", [])
    distance = metrics.get("forwardMeters")
    return (
        "nonpositive-forward-motion" in reasons
        and reasons <= {"nonpositive-forward-motion", "nonrepeatable-forward-motion"}
        and isinstance(distance, (int, float)) and math.isfinite(distance) and distance < 0
        and gait.get("completeCycles", 0) >= 2 and len(intervals) >= 2
        and all(isinstance(c.get("forwardMeters"), (int, float))
                and math.isfinite(c["forwardMeters"]) and c["forwardMeters"] < 0 for c in intervals)
    )


def _polarity_pending(archive, founder=None):
    attempted = {json.loads(row["summary"])["polarityRepairOf"]
                 for row in archive.db.execute("SELECT summary FROM native_proposals WHERE summary IS NOT NULL")
                 if "polarityRepairOf" in json.loads(row["summary"])}
    groups = family_map(archive)
    for row in archive.outcomes():
        if row["birth"] in attempted or (founder is not None and family(archive, row["birth"], groups) != founder):
            continue
        birth = archive.db.execute("SELECT parent,operation FROM births WHERE id=?", (row["birth"],)).fetchone()
        if birth["parent"] in attempted and birth["operation"] == "motor-direction":
            continue
        if backwards_only(json.loads(row["result"])):
            return {"birth": row["birth"], "design": row["design"], "trial": row["id"]}
    return None


def _neighborhood(archive, *, wait_for_results=False, planned=()):
    groups = family_map(archive)
    roots = sorted(set(groups.values()))
    contribution = _pending(archive, exclude=planned)
    if contribution is not None:
        operation = archive.db.execute("SELECT operation FROM births WHERE id=?", (contribution["birth"],)).fetchone()[0]
        if operation == "user-contribution":
            return family(archive, contribution["birth"], groups)
    counts = {root: 0 for root in roots}
    for row in archive.db.execute("SELECT birth FROM evaluations"):
        root = family(archive, row["birth"], groups)
        if root in counts:
            counts[root] += 1
    for birth in planned:
        counts[family(archive, birth, groups)] += 1
    observed = {family(archive, row["birth"], groups) for row in archive.outcomes()}
    runnable = {root for root in roots if _pending(archive, root, exclude=planned) is not None
                or _repair_pending(archive, root) is not None or _polarity_pending(archive, root) is not None}
    baselines = [root for root in roots if root not in observed and root in runnable]
    available = baselines or [root for root in roots if root in runnable or root in observed]
    if not available:
        if wait_for_results:
            return None
        raise AdapterError("No founder neighborhood has a pending candidate or compatible completed parent.")
    return min(available, key=lambda root: (counts[root], root))


def _screen_pending(archive, pending, library, used, payload_profile=None):
    batch = archive.directory / pending["batch"]
    summary, files = _queue(batch)
    if files != json.loads(pending["files"]):
        raise AdapterError("Archived offspring batch changed after its immutable birth.")
    spent = archive.db.execute("SELECT coalesce(sum(allowance),0) FROM native_checks WHERE after_trial=?", (used,)).fetchone()[0]
    allowance = min(POLICY["maxReconstructionChecksPerTrial"]-spent, len(summary["generated"]))
    if allowance <= 0:
        raise DiscoveryLimit("Native discovery paused: load-only reconstruction limit reached for the next trial.")
    with archive.db:
        row = archive.db.execute("INSERT INTO native_checks(after_trial,allowance,batch) VALUES (?,?,?)",
                                 (used, allowance, pending["batch"])).lastrowid
    screened = screen_batch(batch, cache_directory(archive.directory / "preflight", payload_profile=payload_profile),
                            library, max_checks=allowance, payload_profile=payload_profile,
                            observer=screening_observer(archive))
    with archive.db:
        archive.db.execute("UPDATE native_checks SET allowance=? WHERE id=?",
                           (screened["counts"]["reconstructionAttempts"], row))
        for report in screened["results"]:
            status = report.get("previousStatus") if report["status"] == "duplicate" else report["status"]
            if status == "deferred" or status == "ready-for-dynamics":
                continue
            name = Path(report["candidate"]).name
            message = f"Native load-only screening rejected {name}: {rejection_detail(report)}"
            archive.db.execute("""
                UPDATE births SET status='screening-rejected',error=? WHERE status='generated'
                AND (id=? OR id IN (SELECT birth FROM native_candidates WHERE batch=? AND directory=?))
            """, (message, pending["birth"] if name == pending["directory"] else None, pending["batch"], name))
            print(f"ERROR: {message}", file=sys.stderr, flush=True)
    target = next((report for report in screened["results"] if Path(report["candidate"]).name == pending["directory"]), None)
    if target is None:
        raise AdapterError("Native screening omitted the pending candidate's report.")
    ready = target["status"] == "ready-for-dynamics" or (
        target["status"] == "duplicate" and target["previousStatus"] == "ready-for-dynamics")
    if ready:
        entry = next(entry for entry in summary["generated"] if entry["directory"] == pending["directory"])
        return batch / entry["directory"] / "manifest.json"
    return None


def _propose(archive, generator, used, founder=None):
    count = archive.db.execute("SELECT count(*) FROM native_proposals WHERE after_trial=?", (used,)).fetchone()[0]
    if count >= POLICY["maxProposalsPerTrial"]:
        raise DiscoveryLimit("Native discovery paused: motion proposal limit reached for the next trial; diagnostics are archived.")
    repair = _repair_pending(archive, founder)
    polarity = _polarity_pending(archive, founder) if repair is None else None
    parents = [row for row in members(archive) if founder is None or row["family"] == founder]
    run_seed = archive.db.execute("SELECT seed FROM run").fetchone()[0]
    proposal_index = archive.db.execute("SELECT count(*) FROM native_proposals").fetchone()[0]
    from .user_pool import choose as choose_user
    user_parent = choose_user(archive, run_seed, proposal_index) if not (repair or polarity) else None
    parent, selection = ((repair, {"mode": "diagnostic-construction-repair", "selectedBirth": repair["birth"]})
                         if repair else (polarity, {"mode": "diagnostic-motor-polarity", "selectedBirth": polarity["birth"]})
                         if polarity else user_parent if user_parent is not None else choose(parents, run_seed, proposal_index))
    attempt = archive.db.execute("SELECT count(*) FROM native_proposals WHERE parent=?", (parent["birth"],)).fetchone()[0] % 64
    seed = (archive.db.execute("SELECT seed FROM run").fetchone()[0]+parent["birth"]*104729) % 2147483647
    initial = {"parentSelection": selection}
    if repair:
        initial.update(repairOf=repair["birth"], repairReport=repair["report"], repairHints=repair["repairHints"])
    if polarity:
        seed, attempt = 0, 0
        initial.update(polarityRepairOf=polarity["birth"], polarityResultTrial=polarity["trial"])
    with archive.db:
        proposal = archive.db.execute("""
            INSERT INTO native_proposals(parent,seed,attempt,after_trial,status,summary,created)
            VALUES (?,?,?,?,'running',?,?)
        """, (parent["birth"], seed, attempt, used, canonical(initial), now())).lastrowid
    batch = archive.directory / "proposals" / f"{proposal:06d}"
    try:
        source = archive.directory / "designs" / parent["design"] / "candidate"
        if archive.protocol().get("deferredConstruction"):
            native = archive.db.execute(
                "SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth WHERE c.birth=?",
                (parent["birth"],)).fetchone()
            if native is None:
                raise AdapterError("Selected motion parent has no immutable source binding.")
            source = source_candidate(archive, native, generator)
        if repair:
            feedback = archive.directory / "construction-feedback" / f"{proposal:06d}.json"
            write_json(feedback, repair["repairHints"])
            summary = generator.repair_native(source, batch, feedback, POLICY["construction"]["maxRealizationsPerRepair"])
        elif polarity:
            edits = generator.manual_edits(source)["edits"]
            directions = [edit for edit in edits if edit["operation"] == "motor-direction"]
            if len(directions) != 1:
                raise AdapterError("Backward-walking feedback requires exactly one native motor-direction edit.")
            summary = generator.apply_edit(source, batch, directions[0]["id"], limit=1)
            if summary["operation"] != "motor-direction" or len(summary["generated"]) != 1:
                raise AdapterError("Motor-direction feedback did not produce exactly one control-only candidate.")
            original = json.loads((source / "manifest.json").read_text())
            child = json.loads((batch / summary["generated"][0]["directory"] / "manifest.json").read_text())
            if (child["genome"] != {**original["genome"], "speed": -original["genome"]["speed"]}
                    or child["ldrawSha256"] != original["ldrawSha256"]):
                raise AdapterError("Motor-direction feedback changed physical geometry or non-control genome fields.")
        else:
            summary = generator.propose_native(source, batch, seed, attempt, POLICY["maxRealizationsPerProposal"])
        if (summary["parentGenotypeId"] != parent["design"]
                or (not repair and summary["mutationVersion"] != POLICY["mutationVersion"])):
            raise AdapterError("Native proposal provenance does not match its selected parent or mutation protocol.")
        summary.update(initial)
        if summary["generated"]:
            _, files = _queue(batch)
            for entry in summary["generated"]:
                exists = archive.db.execute("SELECT 1 FROM native_candidates WHERE batch=? AND directory=?",
                                             (str(batch.relative_to(archive.directory)), entry["directory"])).fetchone()
                if exists:
                    continue
                manifest, _ = _copy_candidate(archive, batch, entry, files)
                archive.record_native_birth(
                    manifest, parent=parent["birth"], operation=summary["operation"], changed=summary["changed"], seed=seed,
                    native={"assembly": entry["assemblyId"], "batch": str(batch.relative_to(archive.directory)),
                            "directory": entry["directory"], "files": files})
        with archive.db:
            archive.db.execute("UPDATE native_proposals SET status=?,summary=? WHERE id=?",
                               ("generated" if summary["generated"] else "rejected", canonical(summary), proposal))
    except (AdapterError, OSError, json.JSONDecodeError) as error:
        with archive.db:
            archive.db.execute("UPDATE native_proposals SET status='failed',error=? WHERE id=?", (str(error), proposal))
        raise


def _dispatch(archive, generator, library, options, pool, budget):
    from .gallery import publish
    failures = 0
    while sum(archive.counts().values()) < budget or pool.jobs:
        pool.reap()
        used = sum(archive.counts().values())
        if used >= budget:
            pool.drain()
            break
        if pool.exhausted:
            pool.drain()
            break
        if pool.full:
            pool.reap(block=True)
            continue
        founder = _neighborhood(archive, wait_for_results=bool(pool.jobs))
        if founder is None:
            pool.reap(block=True)
            continue
        pending = _pending(archive, founder)
        try:
            if pending is None:
                _propose(archive, generator, pool.generation_step, founder)
            else:
                pending = ensure_pending(archive, pending, generator, library, options.payload_profile)
                if pending is not None and (path := _screen_pending(
                        archive, pending, library, pool.generation_step, options.payload_profile)) is not None:
                    failures += evaluate_native_candidate(archive, pending["birth"], path, library, options, pool=pool)
        except DiscoveryLimit:
            if not pool.jobs:
                raise
            pool.reap(block=True)
            continue
        publish(archive)
    return failures


def _dispatch_parallel(archive, generator, library, options, pool, budget, workers, *, admissions=(), startup=None):
    from .gallery import publish
    from .prescreen import ScreeningPool

    buffer = archive.protocol()["execution"]["screeningBuffer"]
    if type(buffer) is not int or buffer < 1:
        raise AdapterError("Screening buffer capacity must be a positive integer.")
    ready = []
    constructed = []
    admissions = list(admissions)
    prefer_admission = True
    with ScreeningPool(archive, workers, library, options) as screens:
        while sum(archive.counts().values()) < budget or pool.jobs or screens.admitting:
            pool.reap()
            completed = screens.reap()
            constructed.extend(screens.constructed)
            screens.constructed.clear()
            ready.extend(completed)
            if completed:
                publish(archive)
            used = sum(archive.counts().values())
            if used >= budget:
                if ready or constructed or screens.births:
                    raise AdapterError("Preflight work exceeded the remaining trial allowance.")
                pool.drain()
                if screens.admitting:
                    screens.wait(pool)
                    continue
                break
            if pool.exhausted:
                pool.drain()
                if screens.admitting:
                    screens.wait(pool)
                    continue
                break
            if ready and not pool.full:
                birth, checked = ready.pop(0)
                pool.submit(birth, checked, options)
                continue
            if admissions and not screens.full and prefer_admission:
                screens.submit_admission(admissions.pop(0), startup, generator)
                prefer_admission = False
                publish(archive)
                continue
            if constructed and not screens.full:
                try:
                    screens.submit(constructed[0], pool.generation_step, POLICY["maxReconstructionChecksPerTrial"])
                except DiscoveryLimit:
                    if not screens.jobs and not pool.jobs and not ready:
                        raise
                    screens.wait(pool)
                    continue
                constructed.pop(0)
                prefer_admission = True
                publish(archive)
                continue
            planned = screens.births | {birth for birth, _ in ready} | {pending["birth"] for pending in constructed}
            capacity = min(buffer, budget - used - pool.pending_gates)
            if len(planned) < capacity and not screens.full:
                founder = _neighborhood(archive, wait_for_results=bool(planned or pool.jobs or screens.jobs or admissions),
                                        planned=planned)
                if founder is not None:
                    pending = _pending(archive, founder, exclude=planned)
                    try:
                        if pending is None:
                            if not screens.jobs and not pool.jobs and not admissions:
                                _propose(archive, generator, pool.generation_step, founder)
                            else:
                                founder = None
                        else:
                            task = construction_task(archive, pending, generator)
                            if task is not None:
                                screens.submit_construction(task, generator)
                            else:
                                screens.submit(current_pending(archive, pending), pool.generation_step,
                                               POLICY["maxReconstructionChecksPerTrial"])
                        if founder is not None:
                            prefer_admission = True
                    except DiscoveryLimit:
                        if not screens.jobs and not pool.jobs and not ready:
                            raise
                        screens.wait(pool)
                        continue
                    if founder is not None:
                        publish(archive)
                        continue
            if admissions and not screens.full:
                screens.submit_admission(admissions.pop(0), startup, generator)
                prefer_admission = False
                publish(archive)
                continue
            if not screens.jobs and not pool.jobs:
                raise AdapterError("Discovery has no runnable candidate or active pipeline worker.")
            screens.wait(pool)
        return screens.failures


def requalification_pending(archive):
    pending = []
    for design in archive.protocol().get("requalificationCandidates", []):
        row = archive.db.execute("""
            SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth
            WHERE b.design=? AND b.status='generated'
            AND NOT EXISTS(SELECT 1 FROM evaluations e WHERE e.birth=b.id)
            AND NOT EXISTS(SELECT 1 FROM reused_evaluations r WHERE r.birth=b.id)
        """, (design,)).fetchone()
        if row is not None:
            pending.append(row)
    return pending


def _dispatch_requalification(archive, pending, library, options, pool, budget):
    from .gallery import publish
    failures = 0
    for candidate in pending:
        pool.reap()
        while pool.full and pool.jobs:
            pool.reap(block=True)
        if sum(archive.counts().values()) >= budget or pool.exhausted:
            break
        path = _screen_pending(archive, candidate, library, pool.generation_step, options.payload_profile)
        if path is not None:
            failures += evaluate_native_candidate(archive, candidate["birth"], path, library, options, pool=pool)
        publish(archive)
    pool.drain()
    return failures


def search_discovery(archive, generator, source, library, options, blender=None, *, workers=1):
    from .gallery import publish
    from .parallel import TrialPool, worker_count
    from .native_episode import evaluate, render_representative

    worker_count(workers)
    if archive.protocol().get("nativeDiscovery") != POLICY:
        raise AdapterError("Native discovery policy differs; start a fresh protocol archive.")
    archive.library = library
    from .pipeline import begin_startup, startup_step
    startup = begin_startup(archive)
    with startup_step(archive, startup, "candidates"):
        stage_native(archive, source)
    with startup_step(archive, startup, "observations"):
        reuse_observations(archive, library)
    from .user_pool import queue as queue_user_pool
    admissions = []
    with startup_step(archive, startup, "contributions") as progress:
        if requalification_pending(archive):
            progress["detail"] = ("Contribution admissions deferred during the targeted requalification batch; "
                                  "active contributions are preserved for the next ordinary episode.")
        else:
            admissions = queue_user_pool(archive, startup_batch=startup)
            progress["detail"] = f"Queued {len(admissions)} contribution admissions for the shared screening pool."
    with startup_step(archive, startup, "recovery"):
        archive.recover()
        with archive.db:
            archive.db.execute("""
                UPDATE native_proposals SET status='interrupted',error='Interrupted proposal remains archived; not rerun.'
                WHERE status='running'
            """)
    archive.set_status("running")
    failures = 0
    try:
        budget = archive.db.execute("SELECT budget FROM run").fetchone()[0]
        screening_workers = archive.protocol().get("execution", {}).get("screeningWorkers", 1)
        rechecks = requalification_pending(archive)
        with TrialPool(archive, workers, evaluate, blender, renderer=render_representative) as pool:
            try:
                if rechecks:
                    failures += _dispatch_requalification(archive, rechecks, library, options, pool, budget)
                elif screening_workers > 1 or admissions:
                    failures += _dispatch_parallel(archive, generator, library, options, pool, budget, screening_workers,
                                                   admissions=admissions, startup=startup)
                else:
                    failures += _dispatch(archive, generator, library, options, pool, budget)
            except (AdapterError, OSError, json.JSONDecodeError):
                pool.drain()
                raise
        failures += pool.failures
        outcome = "gate-limit" if pool.exhausted else "complete"
        archive.set_status("complete")
    except (AdapterError, OSError, json.JSONDecodeError, KeyboardInterrupt):
        archive.set_status("paused")
        publish(archive)
        raise
    publish(archive)
    return {"status": outcome, "evaluations": archive.counts(), "failures": failures,
            "cheapRejections": pool.rejections,
            "page": str(archive.directory / "preview.html")}
