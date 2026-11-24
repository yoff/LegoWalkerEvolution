import json
import threading
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen

import pytest

from legowalkers_sim import motif_opportunities as mining, pipeline
from legowalkers_sim.bridge import AdapterError, write_json
from legowalkers_sim.dashboard import DashboardHandler
from legowalkers_sim.evolution import Archive
from test_campaign import setup as campaign_setup
from test_dashboard import controller
from test_evolution import genome, protocol


@pytest.fixture
def archive(tmp_path):
    archive = Archive(tmp_path / "version")
    archive.initialize(protocol(), 8, 42, False)
    pipeline.ensure_schema(archive.db)
    yield archive
    archive.db.close()


def candidate(archive, token, *, parent=None, status="screening-rejected", operation="move-hinge-site",
              mechanism="motion", overlaps=None):
    error = "Shared pivot reconstruction mismatch: leg-0-shared-upper-pivot."
    if overlaps is not None:
        error = "Initial self-interference: test contact."
    birth, design = archive.record_birth({**genome(), "phaseQuarterTurns": token}, parent, operation, [],
                                         status, error if status == "screening-rejected" else None)
    manifest = {
        "motion": {"genotypeId": design, "mechanismId": mechanism, "assemblyId": design,
                   "sharedPivots": [{"id": "leg-0-shared-upper-pivot", "bodies": ["chassis", "leg-0-brace", "leg-0-upper"]}]},
        "parts": [{"id": name, "part": code} for name, code in [
            ("pivot-leg-0-shared-upper-pivot", "3673.dat"), ("payload-receiver", "58123.dat"),
            ("leg-0-body-crossbar", "32523.dat"), ("leg-0-body-short-leg", "6632.dat")]],
    }
    write_json(archive.directory / "designs" / design / "candidate/manifest.json", manifest)
    batch = f"proposals/{birth:06d}"
    with archive.db:
        archive.db.execute("INSERT INTO native_candidates VALUES (?,?,?,'{}')", (birth, batch, "candidate-001"))
    write_json(archive.directory / batch / "candidate-001/construction.json", {
        "motif": "passive-link-layer-clearance" if operation == "repair-realization" else "smooth-pivot-shorter-pin"})
    if overlaps is not None:
        write_json(archive.directory / "preflight" / f"screen-{birth:04d}.json", {"results": [{
            "genotypeId": design, "status": "rejected",
            "loadOnly": {"initialSelfInterference": [{"parts": pair, "depthMeters": .001} for pair in overlaps]},
        }]})
    return birth


def versions(archive):
    return [{"id": "v-test", "archive": str(archive.directory)}]


def test_shared_pivot_context_is_local_and_report_does_not_write_scientific_data(archive):
    candidate(archive, 0)
    before = list(archive.db.iterdump())
    value = mining.report(versions(archive))
    group = value["groups"][0]
    assert group["classification"] == "existing-rule-applicability"
    assert group["context"]["bodies"] == ["chassis", "leg-brace", "leg-upper"]
    assert "contact capacity" in group["recommendation"]
    assert value["mutationExposure"]["motorLayoutBirths"] == 0
    assert value["dynamicTrials"] == value["simulationSeconds"] == 0
    assert list(archive.db.iterdump()) == before and archive.counts() == {}
    assert mining.report(versions(archive)) == value
    assert mining.role("pivot-branch-2--leg-1-shared-upper-pivot") == "pivot-leg-shared-upper-pivot"


def test_shared_pivot_context_preserves_identity_with_engagement_details():
    detail = "Shared pivot reconstruction mismatch: pivot-name."
    case = {"detail": detail, "construction": {},
            "manifest": {"motion": {"sharedPivots": [{"id": "pivot-name", "bodies": ["brace", "crossbar"]}]},
                         "parts": [{"id": "pivot-pivot-name", "part": "3673.dat"}]}}
    expected = mining._contexts(case, {})
    case["detail"] += " Connector pivot-pivot-name selected [('gear', 'shaft', 'revolute')], expected bodies ['brace', 'crossbar']."
    assert mining._contexts(case, {}) == expected
    assert expected[0][1]["bodies"] == ["brace", "crossbar"]
    assert expected[0][1]["connector"] == "3673.dat"


def test_preservation_evidence_exposes_each_failed_pivot_and_competing_part():
    case = {"detail": "Joint preservation failed: 1 obligations.", "construction": {},
            "manifest": {"parts": [{"id": "pivot-shared", "part": "3673.dat"},
                                    {"id": "new-gear", "part": "3648b.dat"}]}}
    raw = {"jointPreservation": {"pivots": [
        {"status": "pass"},
        {"status": "reject", "connector": "pivot-shared", "expectedBodies": ["brace", "crossbar"],
         "unexpectedParts": ["new-gear"]}]}}
    contexts = mining._contexts(case, raw)
    assert len(contexts) == 1
    assert contexts[0][0] == "shared-pivot"
    assert contexts[0][1]["connector"] == "3673.dat"
    assert contexts[0][1]["extraParts"] == [("new-gear", "3648b.dat")]


@pytest.mark.parametrize("cheap", [False, True])
def test_simulation_mining_separates_support_coverage_and_other_parts_without_spending_trials(archive, cheap):
    birth = candidate(archive, 0, status="generated")
    design = archive.db.execute("SELECT design FROM births WHERE id=?", (birth,)).fetchone()[0]
    path = archive.directory / "designs" / design / "candidate/manifest.json"
    manifest = json.loads(path.read_text())
    foot = "leg-0-body-crossbar"
    manifest["feet"] = [{"id": "foot", "partInstance": foot}]
    write_json(path, manifest)
    ground = {"footRegions": [{"partInstance": foot}], "forbiddenContacts": [
        {"part": foot, "firstTime": .014}, {"part": "payload-receiver", "firstTime": .014},
        {"part": "leg-0-body-short-leg", "firstTime": 1.2}]}
    report = {"status": "rejected", "reasons": ["forbidden-ground-contact"],
              "diagnostics": {"groundContact": ground}}
    if cheap:
        from legowalkers_sim.parallel import ensure_attempt_schema
        ensure_attempt_schema(archive.db)
        with archive.db:
            archive.db.execute("INSERT INTO simulation_gate_rounds VALUES (1,8,3,'now','complete')")
            archive.db.execute("""INSERT INTO simulation_attempts(id,birth,round,slot,status,started)
                VALUES (1,?,1,1,'rejected','now')""", (birth,))
        directory = "simulation-attempts/0001"
    else:
        trial = archive.reserve(birth)
        archive.finish(trial, error="Legacy spent rejection.", status="failed")
        directory = f"evaluations/{trial:04d}"
    write_json(archive.directory / directory / "simulation-gate.json", report)
    before = list(archive.db.iterdump())
    value = mining.report(versions(archive))
    contacts = value["simulationContacts"]
    assert contacts["counts"]["prefixRejections"] == contacts["counts"]["groundContactEvaluations"] == 1
    assert contacts["counts"]["firstContactIncludesOtherPart"] == 1
    assert {group["context"]["classification"] for group in contacts["groups"]} == {
        "foot-surface-coverage", "unintended-support"}
    assert len(contacts["groups"]) == 2
    assert all(group["evaluations"] == group["uniqueCases"] == 1 for group in contacts["groups"])
    assert all(directory in group["evidence"][0]["report"] for group in contacts["groups"])
    assert "Simulation contact opportunities" in mining.page(value)
    assert list(archive.db.iterdump()) == before
    assert value["dynamicTrials"] == value["simulationSeconds"] == 0
    assert archive.counts() == ({} if cheap else {"failed": 1})


def test_incomplete_simulation_evidence_is_a_visible_warning(archive):
    birth = candidate(archive, 0, status="generated")
    trial = archive.reserve(birth)
    archive.finish(trial, error="Spent rejection.", status="failed")
    write_json(archive.directory / f"evaluations/{trial:04d}/simulation-gate.json", {"status": "rejected"})
    contacts = mining.simulation_contacts(versions(archive))
    assert contacts["groups"] == []
    assert "missing ground-contact diagnostics" in contacts["warnings"][0]


def test_pending_sibling_is_not_a_blocked_or_exhausted_motion(archive):
    candidate(archive, 0)
    candidate(archive, 1, status="generated")
    group = mining.report(versions(archive))["groups"][0]
    assert group["motionsWithPendingRealizations"] == 1
    assert group["motionsWithoutObservedPassOrPendingRepair"] == 0
    assert group["evidence"][0]["searchExhausted"] is None


def test_selection_exposure_distinguishes_eligibility_from_births_and_unknown_history(archive):
    birth = candidate(archive, 0, status="generated")
    selection = {"policy": "uniform-applicable-families-v1", "selectedFamily": "motor-layout",
                 "availableFamilies": [{"operation": "motor-layout", "edits": 2},
                                       {"operation": "input-phase", "edits": 2}]}
    with archive.db:
        for attempt, summary in enumerate([
            {"operation": "motor-layout", "mutationSelection": selection},
            {"operation": "input-phase"}, {"operation": "repair-realization"},
        ]):
            archive.db.execute("""INSERT INTO native_proposals(parent,seed,attempt,after_trial,status,summary,created)
                                  VALUES (?,42,?,0,'rejected',?,'test')""", (birth, attempt, json.dumps(summary)))
    exposure = mining.report(versions(archive))["mutationExposure"]
    assert exposure["eligibleFamilyProposals"] == {"input-phase": 1, "motor-layout": 1}
    assert exposure["selectedFamilyProposals"] == {"motor-layout": 1}
    assert exposure["selectionPolicies"] == {"uniform-applicable-families-v1": 1}
    assert exposure["proposalsWithoutSelectionProvenance"] == 1
    assert exposure["motorLayoutBirths"] == 0


def test_repair_recovery_keeps_ancestry_and_deduplicates_motions(archive):
    failed = candidate(archive, 0)
    child = candidate(archive, 1, parent=failed, operation="repair-realization", status="generated")
    pipeline.record(archive, child, "ready")
    sibling = candidate(archive, 2, parent=failed, operation="repair-realization", status="generated")
    archive.finish(archive.reserve(sibling), error="Trial failed after preflight.", status="failed")
    group = mining.report(versions(archive))["groups"][0]
    assert group["recoveredMotions"] == group["motionsWithPassingRealization"] == 1
    assert group["successfulRepairSequences"] == [["passive-link-layer-clearance"]]
    assert group["evidence"][0]["descendants"][0]["birth"] == child
    assert group["evidence"][0]["repairState"] == "recovered"
    assert len(group["evidence"][0]["descendants"]) == 2
    assert archive.counts() == {"failed": 1}


def test_cross_version_duplicate_does_not_hide_later_repair_evidence(archive, tmp_path):
    other = Archive(tmp_path / "other")
    other.initialize(protocol(), 8, 42, False)
    pipeline.ensure_schema(other.db)
    try:
        candidate(archive, 0)
        failed = candidate(other, 0)
        child = candidate(other, 1, parent=failed, operation="repair-realization", status="generated")
        pipeline.record(other, child, "ready")
        group = mining.report(versions(archive)+[{"id": "v-other", "archive": str(other.directory)}])["groups"][0]
        assert group["failureBirths"] == 2 and group["uniqueCases"] == group["uniqueMotions"] == 1
        assert group["recoveredMotions"] == 1
        assert len(group["evidence"]) == 2
    finally:
        other.db.close()


def test_new_repair_contacts_are_reported_without_double_counting_mesh_contacts(archive):
    pair = ["payload-receiver", "leg-0-body-crossbar"]
    failed = candidate(archive, 0, overlaps=[pair, pair])
    candidate(archive, 1, parent=failed, operation="repair-realization",
              overlaps=[pair, ["leg-0-body-crossbar", "leg-0-body-short-leg"]])
    value = mining.report(versions(archive))
    group = next(group for group in value["groups"] if "payload-receiver" in str(group["context"]))
    assert group["failureBirths"] == 2 and group["motionsWithAdditionalRepairContexts"] == 1
    assert group["classification"] == "composite-or-layout-investigation"
    assert group["evidence"][0]["report"].endswith("screen-0001.json")
    assert group["evidence"][0]["repairHasAdditionalReportedContexts"]


def test_reconstruction_failure_does_not_imply_new_contacts_were_introduced(archive):
    failed = candidate(archive, 0)
    candidate(archive, 1, parent=failed, operation="repair-realization",
              overlaps=[["payload-receiver", "leg-0-body-crossbar"]])
    group = next(group for group in mining.report(versions(archive))["groups"] if group["kind"] == "shared-pivot")
    assert group["motionsWithAdditionalRepairContexts"] == 0
    assert not group["evidence"][0]["repairHasAdditionalReportedContexts"]


@pytest.mark.parametrize("contents", ["{", "[]"])
def test_legacy_stage_schema_and_unreadable_live_reports_are_explicit(archive, contents):
    candidate(archive, 0)
    with archive.db:
        archive.db.execute("DROP TABLE pipeline_events")
    path = archive.directory / "preflight/screen-9999.json"
    path.parent.mkdir()
    path.write_text(contents)
    value = mining.report(versions(archive))
    assert not value["sources"][0]["pipelineTracking"]
    assert value["warnings"] and "Cannot read" in value["warnings"][0]
    assert "Input warnings" in mining.page(value)


def test_current_version_opportunities_rank_before_unretested_history(archive, tmp_path):
    for token in range(3):
        candidate(archive, token, mechanism=f"old-motion-{token}")
    other = Archive(tmp_path / "next-version")
    other.initialize(protocol(), 8, 42, False)
    try:
        candidate(other, 0, mechanism="current-motion", overlaps=[["payload-receiver", "leg-0-body-crossbar"]])
        groups = mining.report(versions(archive)+[{"id": "v-next", "archive": str(other.directory)}])["groups"]
        assert groups[0]["currentVersionMotionsWithoutObservedPassOrPendingRepair"] == 1
        assert groups[1]["uniqueMotions"] == 3 and groups[1]["currentVersionMotions"] == 0
    finally:
        other.db.close()


def test_diagnostic_html_is_escaped_and_missing_reports_are_visible(archive):
    birth = candidate(archive, 0)
    with archive.db:
        archive.db.execute("UPDATE births SET error=? WHERE id=?", ("<script>bad()</script>", birth))
    data = mining.report(versions(archive))
    page = mining.page(data)
    assert "<script>bad()" not in page and "&lt;script&gt;bad()" in page
    assert "no readable screening report" in data["warnings"][0]
    with pytest.raises(AdapterError, match="At least one archive"):
        mining.report([])


def test_snapshot_links_and_output_scope_are_safe(archive, tmp_path):
    candidate(archive, 0)
    output = tmp_path / "report"
    mining.main(["--archive", str(archive.directory), "--output", str(output)])
    page = (output / "motif-opportunities.html").read_text()
    assert 'href="./motif-opportunities.json"' in page
    assert 'http://127.0.0.1:8788/versions/' in page
    with pytest.raises(AdapterError, match="empty"):
        mining.main(["--archive", str(archive.directory), "--output", str(output)])
    with pytest.raises(AdapterError, match="outside scientific"):
        mining.main(["--archive", str(archive.directory), "--output", str(archive.directory / "report")])


def test_http_report_and_json_do_not_authorize_or_start_workers(controller):
    with ThreadingHTTPServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = controller
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(base+"/motif-opportunities.html") as response:
                assert response.status == 200 and b"not automatic motif promotion" in response.read()
            with urlopen(base+"/api/motif-opportunities") as response:
                data = json.load(response)
                assert data["groups"] == [] and data["mode"] == "read-only-motif-opportunity-triage"
            with urlopen(Request(base+"/motif-opportunities.html", method="HEAD")) as response:
                assert response.status == 200 and response.read() == b""
            assert controller.status()["used"] == controller.status()["budget"] == 0
            assert controller.launched == []
        finally:
            server.shutdown()
            thread.join(timeout=5)


def test_campaign_http_report_snapshots_registered_versions_without_mutation(campaign_setup):
    campaign = campaign_setup.create()
    before = json.dumps(campaign.state, sort_keys=True)
    with ThreadingHTTPServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = campaign
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/motif-opportunities") as response:
                data = json.load(response)
            assert [source["version"] for source in data["sources"]] == [version["id"] for version in campaign.state["versions"]]
            assert json.dumps(campaign.state, sort_keys=True) == before
            assert campaign_setup.launched == []
        finally:
            server.shutdown()
            thread.join(timeout=5)
