"""Read-only, evidence-linked triage of construction failures; never promotes motifs."""

import argparse
from collections import Counter, defaultdict
import contextlib
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
from urllib.parse import urljoin

from .bridge import AdapterError, write_json
from .screening_reports import rejection_detail


def role(name):
    name = re.sub(r"branch-\d+--", "", name)
    name = re.sub(r"leg-\d+(?:-\d+)?-", "leg-", name)
    return re.sub(r"-\d+(?=-|$)", "-#", name)


def _read(path):
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}.")
    return value


def _contexts(case, raw):
    detail = case["detail"]
    manifest = case["manifest"]
    construction = case["construction"]
    parts = {part["id"]: part for part in manifest["parts"]}

    def part_context(name):
        part = parts.get(name)
        return (role(name), part["part"] if part else "unknown-part")

    failed_pivots = [pivot for pivot in raw.get("jointPreservation", {}).get("pivots", [])
                     if pivot["status"] == "reject"]
    if failed_pivots:
        return [("shared-pivot", {
            "pivotRole": role(pivot["connector"].removeprefix("pivot-")),
            "bodies": sorted(role(body) for body in pivot["expectedBodies"]),
            "connector": part_context(pivot["connector"])[1],
            "extraParts": sorted(part_context(part) for part in pivot["unexpectedParts"]),
            "motif": construction.get("motif", "unrecorded"),
        }) for pivot in failed_pivots]
    if detail.startswith("Shared pivot reconstruction mismatch: "):
        name = detail.removeprefix("Shared pivot reconstruction mismatch: ").split(".", 1)[0]
        pivot = next((pivot for pivot in manifest["motion"].get("sharedPivots", []) if pivot["id"] == name), None)
        return [("shared-pivot", {
            "pivotRole": role(name), "bodies": sorted(role(body) for body in pivot["bodies"]) if pivot else [],
            "connector": part_context("pivot-"+name)[1], "motif": construction.get("motif", "unrecorded"),
        })]
    overlaps = raw.get("loadOnly", {}).get("initialSelfInterference", [])
    if overlaps:
        return [("initial-interference", {"contact": sorted(part_context(part) for part in overlap["parts"])})
                for overlap in overlaps]
    if detail.startswith("Rigid-body partition mismatch"):
        hints = raw.get("repairHints", {})
        if not hints:
            from .screening_reports import repair_hints
            hints = repair_hints({"error": detail}) or {}
        return [("rigid-partition", {
            "bodies": sorted(role(body) for body in hints.get("bodies", [])),
            "extraParts": sorted(part_context(part) for part in hints.get("parts", [])),
        })]
    return [("other-rejection", {"diagnostic": detail.split(":", 1)[0]})]


def _advice(kind, context, recovered, pending, worsened):
    if kind == "shared-pivot" and context.get("motif") == "smooth-pivot-shorter-pin":
        return ("existing-rule-applicability",
                "Check upstream contact capacity as well as full axial-stack coverage. Its connector detector caps "
                "contacts at max(2, round(mesh shaft length / 20 LDU)); a shorter pin can lose a third body even "
                "when catalog port centers still align. Compare with the unshortened sibling before changing the rule.")
    if recovered:
        return ("existing-motif-evidence",
                "Mine the successful repair sequence and its preconditions; repeated births of one motion are not independent evidence.")
    if pending:
        return ("awaiting-evidence", "Let queued/screening repairs finish before treating this context as an action gap.")
    if kind == "initial-interference":
        contact = str(context.get("contact", []))
        if "payload" in contact or worsened:
            return ("composite-or-layout-investigation",
                    "Investigate coordinated link-layer and hardware clearance. Check whether existing layout actions suffice before adding a motif.")
        return ("construction-gap-investigation",
                "Look for a same-motion successful construction delta; first distinguish missing actions from poor ordering or insufficient budget.")
    return ("diagnostic-investigation",
            "Establish a reusable local cause and a passing construction witness before proposing a new rule.")


def simulation_contacts(versions):
    from .evolution import evidence_invalidation
    groups, warnings = {}, []
    totals = Counter()
    for version in versions:
        root = Path(version["archive"]).resolve()
        prefix = f'/versions/{version["id"]}/'
        with contextlib.closing(sqlite3.connect(f'{(root / "archive.sqlite3").as_uri()}?mode=ro', uri=True)) as db:
            db.row_factory = sqlite3.Row
            db.execute("BEGIN")
            protocol = json.loads(db.execute("SELECT definition FROM protocols LIMIT 1").fetchone()[0])
            rows = [dict(row) for row in db.execute("""
                SELECT e.id,b.id AS birth,b.design,e.status,e.result,'evaluations' AS directory
                FROM evaluations e JOIN births b ON b.id=e.birth ORDER BY e.id
            """)]
            if db.execute("SELECT 1 FROM sqlite_master WHERE name='simulation_attempts'").fetchone():
                rows += [dict(row) for row in db.execute("""
                    SELECT a.id,b.id AS birth,b.design,a.status,NULL AS result,'simulation-attempts' AS directory
                    FROM simulation_attempts a JOIN births b ON b.id=a.birth WHERE a.trial IS NULL ORDER BY a.id
                """)]
        if evidence_invalidation(protocol):
            totals["invalidatedRecords"] += len(rows)
            continue
        for row in rows:
            directory = f'{row["directory"]}/{row["id"]:04d}'
            gate = root / directory / "simulation-gate.json"
            rejected = False
            try:
                gate_report = _read(gate) if gate.is_file() else None
                rejected = gate_report is not None and gate_report.get("status") == "rejected"
                if rejected:
                    data = gate_report
                    diagnostics = data.get("diagnostics")
                    if not isinstance(diagnostics, dict) or "groundContact" not in diagnostics:
                        raise AdapterError("Rejected prefix is missing ground-contact diagnostics.")
                    ground = diagnostics["groundContact"]
                    report_path = directory + "/simulation-gate.json"
                    totals["prefixRejections"] += 1
                    if version["id"] == versions[-1]["id"]:
                        totals["currentVersionPrefixRejections"] += 1
                elif row["status"] == "complete" and row["result"]:
                    data = json.loads(row["result"])
                    if not isinstance(data, dict):
                        raise AdapterError("Evaluation result must be a JSON object.")
                    ground = data.get("groundContact", {})
                    report_path = directory + "/result.json"
                else:
                    continue
                if not isinstance(ground, dict):
                    raise AdapterError("Ground-contact diagnostics must be a JSON object.")
                contacts = ground.get("forbiddenContacts", [])
                if not isinstance(contacts, list):
                    raise AdapterError("Forbidden contacts must be an array.")
                if not contacts:
                    if rejected:
                        totals["prefixRejectionsWithoutGroundContact"] += 1
                    continue
                if any(not isinstance(contact, dict) or not isinstance(contact.get("part"), str)
                       or type(contact.get("firstTime")) not in (int, float)
                       or not math.isfinite(contact["firstTime"]) or contact["firstTime"] < 0
                       for contact in contacts):
                    raise AdapterError("Forbidden contacts need a part and finite, nonnegative first time.")
                regions = ground.get("footRegions")
                if not isinstance(regions, list) or any(
                        not isinstance(region, dict) or not isinstance(region.get("partInstance"), str)
                        for region in regions):
                    raise AdapterError("Ground-contact diagnostics need explicit designated foot regions.")
                manifest = _read(root / "designs" / row["design"] / "candidate/manifest.json")
            except (OSError, ValueError) as error:
                warnings.append(f'{version["id"]} {directory}: unavailable simulation evidence: {error}')
                continue
            totals["groundContactEvaluations"] += 1
            first_time = min(contact["firstTime"] for contact in contacts)
            first = [contact for contact in contacts if contact["firstTime"] == first_time]
            designated = {region["partInstance"] for region in regions}
            foot_only = all(contact["part"] in designated for contact in first)
            totals["firstContactOnFootOnly" if foot_only else "firstContactIncludesOtherPart"] += 1
            settle = protocol.get("evaluation", {}).get("settle")
            phase = "unknown" if settle is None else "settling" if first_time <= settle else "driven"
            totals["firstContactDuring" + phase.capitalize()] += 1
            parts = {part["id"]: part for part in manifest["parts"]}
            logical = {foot["id"]: foot for foot in manifest.get("motion", {}).get("logicalFeet", [])}
            motion = manifest.get("motion", {})
            seen = set()
            for contact in first:
                instance = contact["part"]
                declared = next((foot for foot in manifest["feet"] if foot.get("partInstance") == instance), None)
                shape = logical.get(declared.get("logicalFoot"), {}).get("contactShape", "point-regions") if declared else None
                context = {
                    "classification": "foot-surface-coverage" if instance in designated else "unintended-support",
                    "part": parts[instance]["part"] if instance in parts else "unknown-part",
                    "partRole": role(instance), "contactShape": shape, "phase": phase,
                }
                key = json.dumps(context, sort_keys=True)
                if key in seen:
                    continue
                seen.add(key)
                group = groups.setdefault(key, {"context": context, "evidence": []})
                group["evidence"].append({
                    "version": version["id"], "birth": row["birth"], "source": row["directory"],
                    "evaluationOrAttempt": row["id"], "prefixRejected": rejected,
                    "href": prefix + f'history/{row["birth"]:04d}.html', "report": prefix + report_path,
                    "mechanismId": motion.get("mechanismId"), "genotypeId": motion.get("genotypeId", row["design"]),
                    "assemblyId": motion.get("assemblyId"), "firstTime": first_time, "settleSeconds": settle,
                    "allFirstContactsOnDesignatedFeet": foot_only, "contact": contact,
                    "reasons": data.get("reasons", []),
                })
    result = []
    for key, group in groups.items():
        evidence = group["evidence"]
        context = group["context"]
        result.append({
            "id": "support-" + hashlib.sha256(key.encode()).hexdigest()[:16], **group,
            "evaluations": len(evidence),
            "uniqueCases": len({(case["mechanismId"], case["genotypeId"]) for case in evidence}),
            "uniqueMotions": len({case["mechanismId"] for case in evidence if case["mechanismId"] is not None}),
            "uniqueAssemblies": len({case["assemblyId"] for case in evidence if case["assemblyId"] is not None}),
            "prefixRejections": sum(case["prefixRejected"] for case in evidence),
            "recommendation": (
                "Check the intended bounded sole/toe surface against its declared anchors. A foot part touching "
                "outside point regions is not automatically unintended leg support. Validate an opt-in bounded-surface "
                "repair with a static contact witness; do not enlarge radii or requalify historical measurements."
                if context["classification"] == "foot-surface-coverage" else
                "Keep this contact forbidden. Investigate physical clearance, support orientation or shoe construction; "
                "never extend a foot allowance to the touching rocker, connector or chassis part."),
        })
    result.sort(key=lambda group: (-group["prefixRejections"], -group["uniqueMotions"], -group["evaluations"], group["id"]))
    if totals["invalidatedRecords"]:
        warnings.append(f'{totals["invalidatedRecords"]} invalidated mass-property simulation records '
                        'were excluded from dynamic repair evidence.')
    return {"counts": dict(sorted(totals.items())), "groups": result, "warnings": warnings,
            "interpretation": "First forbidden contacts only; simultaneous contexts can overlap. Full-run evidence is not "
                              "a passed short gate. Reused observations are not counted again. Distinct IDs do not prove "
                              "independent topology families, and different protocols are not a controlled comparison."}


def report(versions):
    if not versions:
        raise AdapterError("At least one archive is required for motif triage.")
    cases, warnings, sources = [], [], []
    operations = Counter()
    proposal_attempts = Counter()
    selection_policies, eligible_families, selected_families = Counter(), Counter(), Counter()
    mutation_proposals = 0
    for version in versions:
        root = Path(version["archive"]).resolve()
        prefix = f'/versions/{version["id"]}/'
        with contextlib.closing(sqlite3.connect(f'{(root / "archive.sqlite3").as_uri()}?mode=ro', uri=True)) as db:
            db.row_factory = sqlite3.Row
            db.execute("BEGIN")
            tracked = db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='pipeline_events'").fetchone() is not None
            stage = "p.stage" if tracked else "NULL AS stage"
            pipeline_join = "LEFT JOIN pipeline_events p ON p.id=(SELECT max(id) FROM pipeline_events WHERE birth=b.id)" if tracked else ""
            births = [dict(row) for row in db.execute(f"""
                SELECT b.*,n.batch,n.directory,e.status AS trial_status,{stage}
                FROM births b LEFT JOIN native_candidates n ON n.birth=b.id
                LEFT JOIN evaluations e ON e.birth=b.id
                {pipeline_join}
                WHERE NOT EXISTS(SELECT 1 FROM reused_evaluations r WHERE r.birth=b.id)
                ORDER BY b.id""")]
            proposals = [dict(row) for row in db.execute("SELECT parent,attempt,status,summary FROM native_proposals ORDER BY id")]
            sources.append({"version": version["id"], "archive": str(root),
                            "pipelineTracking": tracked,
                            "birthsInSnapshot": db.execute("SELECT count(*) FROM births").fetchone()[0],
                            "proposalsInSnapshot": len(proposals)})
        for proposal in proposals:
            summary = json.loads(proposal["summary"]) if proposal["summary"] else {}
            if summary.get("operation") != "repair-realization":
                proposal_attempts[proposal["attempt"]] += 1
                mutation_proposals += 1
                selection = summary.get("mutationSelection")
                if selection is not None:
                    selection_policies[selection["policy"]] += 1
                    selected_families[selection["selectedFamily"]] += 1
                    for family in selection["availableFamilies"]:
                        eligible_families[family["operation"]] += 1
        local = {}
        for birth in births:
            candidate = root / "designs" / birth["design"] / "candidate"
            path = candidate / "manifest.json"
            if not path.is_file():
                if birth["status"] == "screening-rejected":
                    warnings.append(f'{version["id"]} birth {birth["id"]}: missing native manifest; excluded from context analysis.')
                continue
            manifest = _read(path)
            if "motion" not in manifest:
                continue
            construction = {}
            if birth["batch"]:
                path = root / birth["batch"] / birth["directory"] / "construction.json"
                if path.is_file():
                    construction = _read(path)
            passed = birth["trial_status"] is not None or birth["stage"] == "ready"
            pending = (version["id"] == versions[-1]["id"] and not passed
                       and birth["status"] == "generated" and birth["stage"] != "rejected")
            item = {**birth, "version": version["id"], "href": prefix+f'history/{birth["id"]:04d}.html',
                    "manifest": manifest, "construction": construction,
                    "mechanism": manifest["motion"]["mechanismId"],
                    "genotype": manifest["motion"]["genotypeId"], "assembly": manifest["motion"]["assemblyId"],
                    "passed": passed, "pending": pending, "detail": birth["error"] or "",
                    "report": None, "contexts": []}
            local[birth["id"]] = item
            if birth["operation"] != "import-motion-walker":
                operations[birth["operation"]] += 1
        # Screening report files are immutable once written; incomplete live files are explicit warnings.
        rejected = {item["genotype"]: item for item in local.values() if item["status"] == "screening-rejected"}
        reports = {}
        for folder in ("preflight", "preflight-isolated"):
            for path in sorted((root / folder).glob("**/screen-*.json")):
                try:
                    summary = _read(path)
                except (OSError, ValueError) as error:
                    warnings.append(f"Cannot read {path}: {error}")
                    continue
                for raw in summary.get("results", []):
                    if raw.get("genotypeId") not in rejected or raw.get("previousStatus", raw.get("status")) != "rejected":
                        continue
                    identifier = raw.get("screeningId")
                    canonical = path.parent / "reports" / f"{identifier}.json" if isinstance(identifier, str) and re.fullmatch("[0-9a-f]{64}", identifier) else path
                    if canonical != path and not canonical.is_file():
                        warnings.append(f"Missing detailed report {canonical}; using archived summary.")
                        canonical = path
                    try:
                        detailed = _read(canonical) if canonical != path else raw
                    except (OSError, ValueError) as error:
                        warnings.append(f"Cannot read {canonical}: {error}")
                        continue
                    reports[raw["genotypeId"]] = (detailed, prefix+canonical.relative_to(root).as_posix())
        for item in local.values():
            if item["status"] == "screening-rejected":
                raw, link = reports.get(item["genotype"], ({}, None))
                if raw:
                    item["detail"] = rejection_detail(raw)
                    item["report"] = link
                else:
                    warnings.append(f'{version["id"]} birth {item["id"]}: no readable screening report; using birth diagnostic only.')
                item["contexts"] = _contexts(item, raw)
            ancestors, cursor = [], item
            while cursor["operation"] == "repair-realization" and cursor["parent"] in local:
                if cursor["parent"] in ancestors:
                    raise AdapterError("Repair ancestry contains a cycle.")
                ancestors.append(cursor["parent"])
                cursor = local[cursor["parent"]]
            item["repairRoot"] = cursor["id"]
            item["repairAncestors"] = ancestors
        cases.extend(local.values())
    by_motion = defaultdict(list)
    for case in cases:
        by_motion[case["mechanism"]].append(case)
    groups = {}
    for case in cases:
        seen_contexts = set()
        for kind, context in case["contexts"]:
            key = json.dumps([kind, context], sort_keys=True, separators=(",", ":"))
            if key in seen_contexts:
                continue
            seen_contexts.add(key)
            group = groups.setdefault(key, {"kind": kind, "context": context, "cases": []})
            group["cases"].append(case)
    results = []
    for key, group in groups.items():
        occurrences = group.pop("cases")
        unique = {}
        for case in occurrences:
            unique.setdefault((case["mechanism"], case["genotype"]), case)
        evidence, sequences = [], set()
        recovered_motions, pending_motions, passing_siblings, pending_realizations, worsened_motions = set(), set(), set(), set(), set()
        for case in occurrences:
            descendants = [candidate for candidate in cases
                           if candidate["version"] == case["version"] and case["id"] in candidate["repairAncestors"]]
            recovered = [candidate for candidate in descendants if candidate["passed"]]
            pending = any(candidate["pending"] for candidate in descendants)
            if recovered:
                recovered_motions.add(case["mechanism"])
                for candidate in recovered:
                    sequence = [ancestor["construction"].get("motif", "unrecorded")
                                for ancestor in sorted(descendants, key=lambda row: len(row["repairAncestors"]))
                                if ancestor["id"] == candidate["id"] or ancestor["id"] in candidate["repairAncestors"]]
                    sequences.add(tuple(sequence))
            if pending:
                pending_motions.add(case["mechanism"])
            if any(sibling["passed"] for sibling in by_motion[case["mechanism"]]):
                passing_siblings.add(case["mechanism"])
            if any(sibling["pending"] for sibling in by_motion[case["mechanism"]]):
                pending_realizations.add(case["mechanism"])
            baseline_contacts = {json.dumps(context, sort_keys=True) for kind, context in case["contexts"] if kind == "initial-interference"}
            worsened = bool(baseline_contacts) and any({json.dumps(context, sort_keys=True) for kind, context in candidate["contexts"]
                            if kind == "initial-interference"} - baseline_contacts for candidate in descendants)
            if worsened:
                worsened_motions.add(case["mechanism"])
            evidence.append({
                "version": case["version"], "birth": case["id"], "href": case["href"],
                "mechanismId": case["mechanism"], "genotypeId": case["genotype"], "assemblyId": case["assembly"],
                "detail": case["detail"], "report": case["report"],
                "construction": case["construction"],
                "repairState": "recovered" if recovered else "pending" if pending else "no-passing-repair-observed",
                "searchExhausted": None, "repairHasAdditionalReportedContexts": worsened,
                "repairAncestry": [{"birth": ancestor, "href": f'/versions/{case["version"]}/history/{ancestor:04d}.html'}
                                   for ancestor in reversed(case["repairAncestors"])],
                "descendants": [{"birth": child["id"], "href": child["href"], "passed": child["passed"],
                                 "pending": child["pending"], "motif": child["construction"].get("motif", "unrecorded")}
                                for child in descendants],
            })
        motions = {case["mechanism"] for case in unique.values()}
        blocked = motions-passing_siblings-pending_realizations
        current_motions = {case["mechanism"] for case in occurrences if case["version"] == versions[-1]["id"]}
        category, recommendation = _advice(group["kind"], group["context"], bool(recovered_motions),
                                            bool(pending_realizations), bool(worsened_motions))
        results.append({
            "id": hashlib.sha256(key.encode()).hexdigest()[:16], **group,
            "classification": category, "recommendation": recommendation,
            "failureBirths": len(occurrences), "uniqueCases": len(unique),
            "uniqueAssemblies": len({case["assembly"] for case in unique.values()}),
            "uniqueMotions": len(motions), "recoveredMotions": len(recovered_motions),
            "currentVersionMotions": len(current_motions),
            "currentVersionMotionsWithoutObservedPassOrPendingRepair": len(current_motions & blocked),
            "versions": sorted({case["version"] for case in occurrences}),
            "motionsWithPassingRealization": len(passing_siblings), "motionsWithPendingRepairs": len(pending_motions),
            "motionsWithPendingRealizations": len(pending_realizations),
            "motionsWithoutObservedPassOrPendingRepair": len(blocked),
            "motionsWithAdditionalRepairContexts": len(worsened_motions),
            "successfulRepairSequences": [list(sequence) for sequence in sorted(sequences)], "evidence": evidence,
        })
    results.sort(key=lambda row: (-row["currentVersionMotionsWithoutObservedPassOrPendingRepair"],
                                 -row["currentVersionMotions"],
                                 -row["motionsWithoutObservedPassOrPendingRepair"], -row["uniqueMotions"],
                                 -row["motionsWithAdditionalRepairContexts"], row["id"]))
    return {
        "schemaVersion": 1, "mode": "read-only-motif-opportunity-triage", "sources": sources,
        "currentVersion": versions[-1]["id"],
        "ranking": "Current-version motions without a pass or pending repair, then current-version occurrence, then all-version unresolved/distinct motions and additional repair contexts; deterministic context-ID ties.",
        "limitations": [
            "Groups may overlap: one collision report can contribute several contact contexts.",
            "Distinct motion IDs, assemblies and births do not prove independent topology families.",
            "Repeated motion/genotype pairs are deduplicated across versions for unique-case counts.",
            "Cross-version protocols can differ; this is an evidence inventory, not a controlled policy comparison.",
            "Reads are transactions per archive, not a globally atomic campaign snapshot; missing or incomplete reports produce warnings.",
            "No passing repair observed is not proof of exhausted search; queued, deferred and paused work must not be counted as mechanical failure.",
            "Readiness means a simulation was reserved or final preflight recorded ready, not qualified walking.",
            "Recommendations are hypotheses for manual investigation, not mined or promoted construction rules.",
        ],
        "failureBirths": sum(case["status"] == "screening-rejected" for case in cases),
        "groups": results, "warnings": warnings,
        "simulationContacts": simulation_contacts(versions),
        "mutationExposure": {
            "recordedBirthOperations": dict(sorted(operations.items())),
            "proposalAttemptHistogram": dict(sorted(proposal_attempts.items())),
            "selectionPolicies": dict(sorted(selection_policies.items())),
            "eligibleFamilyProposals": dict(sorted(eligible_families.items())),
            "selectedFamilyProposals": dict(sorted(selected_families.items())),
            "proposalsWithoutSelectionProvenance": mutation_proposals-sum(selection_policies.values()),
            "motorLayoutBirths": operations["motor-layout"], "batteryLayoutBirths": operations["battery-layout"],
            "receiverLayoutBirths": operations["receiver-layout"],
            "interpretation": "Zero births do not mean unsupported or never selected: later gates can reject an edit. Historical v5 used late per-parent slots and fallback without availability logging. V6 samples applicable families uniformly, then edits uniformly, with seeded replay; eligibility and selection counts cover only proposals with recorded selection provenance, not historical unknowns.",
        },
        "dynamicTrials": 0, "simulationSeconds": 0, "promotion": "manual-review-required",
    }


def page(data, base_url=""):
    from .gallery import document, esc
    def href(path):
        return esc(urljoin(base_url, path))
    rows = []
    simulation = data["simulationContacts"]
    rows.append('<section class="panel"><h2>Simulation contact opportunities</h2><p>'
                + esc(simulation["interpretation"]) + '</p><pre>'
                + esc(json.dumps(simulation["counts"], indent=2)) + '</pre></section>')
    for group in simulation["groups"]:
        rows.append(f'<section class="panel" id="{group["id"]}"><h3>{esc(group["context"]["classification"])}</h3>'
                    '<pre>' + esc(json.dumps(group["context"], indent=2)) + '</pre>'
                    f'<p>{group["evaluations"]} evaluations/attempts · {group["uniqueCases"]} distinct motion/genotype cases · '
                    f'{group["uniqueAssemblies"]} assemblies · {group["prefixRejections"]} prefix rejections</p>'
                    f'<p>{esc(group["recommendation"])}</p>')
        for evidence in group["evidence"]:
            rows.append(f'<details><summary><a href="{href(evidence["href"])}">{esc(evidence["version"])} '
                        f'birth {evidence["birth"]}</a> · {1000*evidence["firstTime"]:.3f} ms</summary>'
                        f'<p><a href="{href(evidence["report"])}">Simulation evidence</a></p>'
                        '<pre>' + esc(json.dumps(evidence["contact"], indent=2)) + '</pre></details>')
        rows.append("</section>")
    if simulation["warnings"]:
        rows.append('<section class="panel"><h3>Simulation evidence warnings</h3><ul>'
                    + ''.join(f'<li>{esc(warning)}</li>' for warning in simulation["warnings"]) + '</ul></section>')
    for index, group in enumerate(data["groups"], 1):
        rows.append(f'<section class="panel" id="{group["id"]}"><h2>{index}. {esc(group["classification"])}</h2>'
                    f'<pre>{esc(json.dumps(group["context"], indent=2))}</pre>'
                    f'<p>{group["uniqueMotions"]} motions · {group["uniqueAssemblies"]} assemblies · '
                    f'{group["failureBirths"]} failure births · {group["recoveredMotions"]} motions recovered by repair · '
                    f'{group["motionsWithPendingRealizations"]} with pending realizations · '
                    f'{group["currentVersionMotions"]} motions seen in current version</p><p>{esc(group["recommendation"])}</p>')
        if group["successfulRepairSequences"]:
            rows.append('<p>Observed preflight-passing repair sequences (not walking proof):</p><ul>'
                        +''.join(f'<li>{esc(" -> ".join(sequence))}</li>' for sequence in group["successfulRepairSequences"])+'</ul>')
        for evidence in group["evidence"]:
            links = f'<a href="{href(evidence["href"])}">{esc(evidence["version"])} birth {evidence["birth"]}</a>'
            if evidence["report"]:
                links += f' · <a href="{href(evidence["report"])}">Screening report</a>'
            descendants = " · ".join(f'<a href="{href(child["href"])}">Repair birth {child["birth"]}: '
                                     f'{"passed" if child["passed"] else "pending" if child["pending"] else "no pass recorded"}</a>'
                                     for child in evidence["descendants"])
            ancestry = " · ".join(f'<a href="{href(ancestor["href"])}">Birth {ancestor["birth"]}</a>'
                                  for ancestor in evidence["repairAncestry"])
            rows.append(f'<details><summary>{links} · {esc(evidence["repairState"])}</summary>'
                        f'<p>{esc(evidence["detail"])}</p>'
                        +(f'<p>Repair ancestry: {ancestry}</p>' if ancestry else '')+f'<p>{descendants}</p></details>')
        rows.append("</section>")
    return document("Motif opportunities", f'<a href="{href("/")}">Dashboard</a> · <a href="{href("/motif-gallery.html")}">Motif gallery</a>'
                    '<h1>Motif opportunities</h1><p>Read-only diagnostic mining, not automatic motif promotion. '
                    'Counts are snapshots, not realization-success rates.</p>'
                    f'<p><a href="{"./motif-opportunities.json" if base_url else "/api/motif-opportunities"}">Download evidence (JSON)</a></p>'
                    '<h2>Interpretation and ranking</h2><p>'+esc(data["ranking"])+'</p><ul>'
                    +''.join(f'<li>{esc(item)}</li>' for item in data["limitations"])+'</ul>'
                    '<h2>Mutation exposure</h2><p>'+esc(data["mutationExposure"]["interpretation"])+'</p><pre>'
                    +esc(json.dumps(data["mutationExposure"]["recordedBirthOperations"], indent=2))+'</pre>'
                    '<h3>Recorded selection exposure</h3><pre>'
                    +esc(json.dumps({key: data["mutationExposure"][key] for key in (
                        "selectionPolicies", "eligibleFamilyProposals", "selectedFamilyProposals",
                        "proposalsWithoutSelectionProvenance")}, indent=2))+'</pre>'
                    +('<h2>Input warnings</h2><ul>'+''.join(f'<li>{esc(item)}</li>' for item in data["warnings"])+'</ul>' if data["warnings"] else '')
                    +''.join(rows))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--campaign", type=Path)
    source.add_argument("--archive", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--base-url", default="http://127.0.0.1:8788")
    args = parser.parse_args(argv)
    if args.output.exists() and any(args.output.iterdir()):
        raise AdapterError("Motif-opportunity output must be empty.")
    versions = _read(args.campaign / "campaign.json")["versions"] if args.campaign else [
        {"id": args.archive.name, "archive": str(args.archive.resolve())}]
    if any(args.output.resolve().is_relative_to(Path(version["archive"]).resolve()) for version in versions):
        raise AdapterError("Write the report outside scientific archives.")
    data = report(versions)
    write_json(args.output / "motif-opportunities.json", data)
    (args.output / "motif-opportunities.html").write_text(page(data, args.base_url))
    print(json.dumps({"groups": len(data["groups"]), "failureBirths": data["failureBirths"], "warnings": data["warnings"]}))


if __name__ == "__main__":
    main()
