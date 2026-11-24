import json
from pathlib import Path
import sys
import threading
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace

import pytest

from legowalkers_sim import pipeline, previews
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.dashboard import EpisodeController
from legowalkers_sim.evolution import Archive
from test_evolution import genome, protocol, result


@pytest.fixture
def archive(tmp_path):
    value = Archive(tmp_path)
    value.initialize(protocol(), 4, 42, False)
    previews.PreviewController(tmp_path, tmp_path / "library", None)
    from legowalkers_sim.rendering import RenderController
    RenderController(tmp_path, tmp_path / "library", None)
    yield value
    value.db.close()


def candidate(archive, token=0, parent=None, operation="test"):
    birth, design = archive.record_birth({**genome(), "phaseQuarterTurns": token},
                                        parent, operation, [], "generated")
    path = archive.directory / "designs" / design / "candidate"
    path.mkdir(parents=True, exist_ok=True)
    (path / "manifest.json").write_text("{}")
    (path / "cycle.svg").write_text('<svg xmlns="http://www.w3.org/2000/svg"/>')
    return birth, design


def snapshot(archive, running=True, tracked=True):
    return pipeline.snapshot(archive.db, archive.directory, running=running, tracked=tracked)


def stages(value):
    return {stage["id"]: stage for stage in value["stages"]}


def test_compact_board_includes_every_queued_walker_without_expanding_detailed_galleries(archive):
    births = [candidate(archive, token)[0] for token in range(12)]
    archive.db.execute("""CREATE TABLE IF NOT EXISTS user_pool_entries(
        contribution TEXT PRIMARY KEY,birth INTEGER,status TEXT NOT NULL,error TEXT,provenance TEXT NOT NULL)""")
    archive.db.execute("INSERT INTO user_pool_entries VALUES ('named',?,'eligible',NULL,?)",
                       (births[-1], json.dumps({"label": "Outward L"})))
    before = list(archive.db.iterdump())
    queued = stages(snapshot(archive))["queued"]
    assert queued["count"] == len(queued["walkers"]) == 12
    assert len(queued["items"]) == 8
    assert [item["birth"] for item in queued["walkers"]] == births
    assert queued["walkers"][-1]["name"] == "Outward L"
    assert queued["walkers"][0]["name"] == f"Birth {births[0]}"
    assert all(item["visuals"]["diagram"] for item in queued["walkers"])
    assert all("screening" not in item and "detail" not in item for item in queued["walkers"])
    assert list(archive.db.iterdump()) == before
    assert archive.counts() == {}


def test_compact_finished_board_reports_total_beyond_bounded_recent_details(archive):
    for token in range(12):
        birth, _ = candidate(archive, token)
        archive.db.execute("UPDATE births SET status='failed',error='saved failure' WHERE id=?", (birth,))
    value = snapshot(archive)
    assert len(value["recent"]) == 8
    assert value["finished"]["count"] == len(value["finished"]["walkers"]) == 12
    assert all(item["stage"] == "failed" for item in value["finished"]["walkers"])
    assert archive.counts() == {}


@pytest.mark.parametrize("qualified", [True, False, None])
def test_compact_walker_preserves_recorded_qualification_without_inferring_it(qualified):
    item = {
        "key": "birth-33", "birth": 33, "history": "history/0033.html",
        "stage": "complete", "visuals": {},
        "qualification": None if qualified is None else {"qualified": qualified, "rejectionReasons": []},
    }
    compact = pipeline.compact_walker(item, {})
    assert compact["qualification"] == item["qualification"]


def test_startup_tasks_have_explicit_lifecycle_and_elapsed_times_without_trial_changes(archive):
    before = archive.counts()
    batch = pipeline.begin_startup(archive)
    items = snapshot(archive)["startup"]["items"]
    assert len(items) == 4 and all(item["status"] == "queued" for item in items)
    assert all(item["elapsedSeconds"] is None for item in items)
    with pipeline.startup_step(archive, batch, "candidates"):
        row = snapshot(archive)["startup"]["items"][0]
        assert row["status"] == "running" and row["elapsedSeconds"] >= 0
    finished = snapshot(archive)["startup"]["items"][0]
    assert finished["status"] == "complete" and finished["completed"]
    assert snapshot(archive)["startup"]["items"][0]["elapsedSeconds"] == finished["elapsedSeconds"]
    assert archive.counts() == before
    assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 4
    with pytest.raises(AdapterError, match="cannot transition"):
        pipeline.update_startup(archive, batch, "candidates", "running")
    with pytest.raises(AdapterError, match="Invalid startup outcome"):
        pipeline.update_startup(archive, batch, "observations", "qualified")


def test_startup_failure_and_interruption_are_visible_without_rewriting_records(archive):
    batch = pipeline.begin_startup(archive)
    with pytest.raises(AdapterError, match="broken import"):
        with pipeline.startup_step(archive, batch, "candidates"):
            raise AdapterError("broken import")
    active = snapshot(archive)["startup"]
    assert active["items"][0]["status"] == "failed"
    assert active["items"][0]["detail"] == "broken import"
    before = list(archive.db.iterdump())
    stopped = snapshot(archive, running=False)["startup"]
    assert stopped["items"][0]["status"] == "failed"
    assert all(item["status"] == "interrupted" and item["elapsedSeconds"] is None
               for item in stopped["items"][1:])
    assert list(archive.db.iterdump()) == before


def test_latest_startup_attempt_does_not_mix_with_previous_episode_or_outer_exception(archive):
    first = pipeline.begin_startup(archive)
    try:
        raise ValueError("already handled")
    except ValueError:
        with pipeline.startup_step(archive, first, "candidates"):
            pass
    assert snapshot(archive)["startup"]["items"][0]["status"] == "complete"
    second = pipeline.begin_startup(archive)
    assert second == first+1
    assert all(item["status"] == "queued" for item in snapshot(archive)["startup"]["items"])
    assert archive.db.execute("SELECT count(*) FROM pipeline_startup").fetchone()[0] == 8


def test_named_startup_admission_links_to_birth_only_after_admission(archive):
    birth, _ = candidate(archive)
    batch = pipeline.begin_startup(archive)
    pipeline.queue_startup(archive, batch, [("human", "admission", "4longer")])
    with pipeline.startup_step(archive, batch, "human") as outcome:
        started = (datetime.now(timezone.utc)-timedelta(seconds=10)).isoformat()
        with archive.db:
            archive.db.execute("UPDATE pipeline_startup SET started=? WHERE task='human'", (started,))
        item = snapshot(archive)["startup"]["items"][-1]
        assert item["label"] == "4longer" and item["history"] is None
        assert 9 <= item["elapsedSeconds"] < 12
        outcome.update(birth=birth, detail="Admission passed, not walking qualification.")
    item = snapshot(archive)["startup"]["items"][-1]
    assert item["history"] == f"history/{birth:04d}.html"
    assert item["elapsedSeconds"] >= 9
    assert item["status"] == "complete"
    assert item["visuals"]["diagram"]
    assert archive.counts() == {}


def test_admission_and_candidate_cards_read_live_screening_checkpoints(archive):
    from legowalkers_sim.screening_reports import ScreeningProgress
    birth, design = candidate(archive)
    batch = pipeline.begin_startup(archive)
    pipeline.queue_startup(archive, batch, [("human", "admission", "Candidate")])
    progress = ScreeningProgress(archive.directory / "user-admissions/human/protocol", design)
    with pipeline.startup_step(archive, batch, "human"):
        progress.start("inputs")
        progress.complete()
        progress.start("reconstruction")
        item = snapshot(archive)["startup"]["items"][-1]
        checks = {check["id"]: check["status"] for check in item["screening"]["checks"]}
        assert checks["inputs"] == "pass" and checks["reconstruction"] == "running"
    screen = ScreeningProgress(archive.directory / "preflight-isolated" / f"{birth:06d}" / "protocol", design)
    screen.start("inputs")
    pipeline.record(archive, birth, "screening")
    card = stages(snapshot(archive))["screening"]["items"][0]
    assert card["screening"]["checks"][0]["status"] == "running"
    stopped = stages(snapshot(archive, running=False))["paused"]["items"][0]
    assert stopped["screening"]["checks"][0]["status"] == "interrupted"
    assert archive.counts() == {}


def test_dashboard_checklist_renders_actual_statuses_and_diagnostics_without_html():
    import subprocess
    script = (Path(pipeline.__file__).with_name("dashboard.js")).read_text()
    function = script[script.index("    function screeningChecks("):script.index("    function startupCard(")]
    harness = """
const assert = require("node:assert/strict");
const document = {createElement: tag => ({
    tag, open:false, dataset:{}, children: [], textContent: "", append(...items) {this.children.push(...items);}
})};
""" + function + """
const card = document.createElement("li");
screeningChecks(card, {detail:"No dynamics trial spent.", checks:[
    {label:"Inputs", status:"pass", detail:"Verified"},
    {label:"Reconstruction", status:"running", detail:"<script>not executable</script>"},
    {label:"Contacts", status:"pending", detail:""}
]});
assert.equal(card.children[0].textContent, "Checks passed: 1. No dynamics trial spent.");
const lines = card.children[1].children.map(item => item.textContent);
assert.deepEqual(lines, ["Inputs: pass — Verified",
    "Reconstruction: running — <script>not executable</script>", "Contacts: pending"]);
const recent = document.createElement("li");
screeningChecks(recent, {detail:"Saved outcomes.", checks:[
    {label:"Inputs", status:"pass", detail:"Verified"}
]}, true);
assert.equal(recent.children.length, 1);
const diagnostics = recent.children[0];
assert.equal(diagnostics.tag, "details");
assert.equal(diagnostics.open, false);
assert.equal(diagnostics.dataset.liveKey, "screening-diagnostics");
assert.equal(diagnostics.children[0].tag, "summary");
assert.equal(diagnostics.children[0].textContent, "Screening diagnostics");
assert.equal(diagnostics.children[2].children[0].textContent, "Inputs: pass — Verified");
"""
    subprocess.run(["node", "-e", harness], check=True, capture_output=True, text=True)


def test_recent_pipeline_section_is_open_but_requests_collapsed_member_diagnostics():
    import subprocess
    script = Path(pipeline.__file__).with_name("dashboard.js").read_text()
    function = script[script.index("        function group("):script.index("        for (const stage of state.pipeline.stages) group(")]
    harness = """
const assert = require("node:assert/strict");
const document = {createElement: tag => ({
    tag, open:false, dataset:{}, children:[], append(...items) {this.children.push(...items);}
})};
const incoming = document.createElement("div");
const prefix = "";
const pipelineCard = (item, prefix, recent) => ({...item, collapsedDiagnostics:recent});
""" + function + """
group("recent", "Recently finished / rejected", [{}], 1, true);
group("screening", "Screening", [{}], 1);
assert.equal(incoming.children[0].tag, "details");
assert.equal(incoming.children[0].open, true);
assert.equal(incoming.children[0].children[0].tag, "summary");
assert.equal(incoming.children[0].children[1].children[0].collapsedDiagnostics, true);
assert.equal(incoming.children[1].tag, "section");
assert.equal(incoming.children[1].children[0].tag, "h4");
assert.equal(incoming.children[1].children[1].children.length, 1);
assert.equal(incoming.children[1].children[1].children[0].collapsedDiagnostics, false);
"""
    subprocess.run(["node", "-e", harness], check=True, capture_output=True, text=True)

def test_saved_motion_diagrams_gain_animation_when_served_without_rewriting_old_pages(archive):
    from legowalkers_sim.gallery import birth_previews, publish

    birth, design = candidate(archive)
    root = archive.directory / "designs" / design / "candidate"
    page = archive.directory / "history" / f"{birth:04d}.html"
    publish(archive)
    legacy = page.read_text()
    assert 'class="motion-player"' not in legacy
    (root / "program.json").write_text("{}")
    served = birth_previews(legacy, archive.directory, f"history/{birth:04d}.html")
    assert served.count('class="motion-player"') == 1
    assert f'data-motion-source="../designs/{design}/candidate/motion.json"' in served
    assert "cycle.svg" in served and page.read_text() == legacy
    assert birth_previews(served, archive.directory, f"history/{birth:04d}.html") == served
    publish(archive)
    assert 'class="motion-player"' in page.read_text()
    assert archive.counts() == {}


def test_legacy_worker_has_no_invented_startup_records(archive):
    assert snapshot(archive)["startup"] == {"tracked": False, "items": [], "counts": {}}


def test_qualified_gallery_uses_completed_results_not_screening_and_preserves_budget(archive):
    first, _ = candidate(archive)
    prototype, _ = candidate(archive, 2)
    duplicate, _ = candidate(archive, parent=first)
    pending, _ = candidate(archive, 3)
    archive.finish(archive.reserve(first), result(qualified=True, speed=.01))
    archive.finish(archive.reserve(prototype), result())
    archive.finish(archive.reserve(duplicate), result(qualified=True, speed=.02))
    pipeline.record(archive, pending, "ready", "Static checks passed.")
    before = list(archive.db.iterdump())
    value = snapshot(archive)["qualified"]
    assert value["count"] == 1
    item = value["items"][0]
    assert item["birth"] == duplicate
    assert item["speedMmPerSecond"] == 20
    assert item["qualification"] == {"qualified": True, "rejectionReasons": []}
    assert item["history"] == f"history/{duplicate:04d}.html"
    assert item["video"] is None
    assert list(archive.db.iterdump()) == before
    assert archive.counts() == {"complete": 3}


def test_recent_qualified_keeps_twelve_newest_unique_designs_with_total_count():
    items = [{"design": str(i), "birth": i, "updated": f"2026-11-{i+1:02d}"} for i in range(15)]
    items.append({"design": "0", "birth": 20, "updated": "2026-12-01"})
    value = pipeline.recent_qualified(items)
    assert value["count"] == 15
    assert [item["birth"] for item in value["items"]] == [20, *range(14, 3, -1)]
    assert pipeline.recent_qualified([]) == {"count": 0, "items": []}


def test_dashboard_section_navigation_has_stable_targets_and_opens_collapsed_ancestors():
    import subprocess
    script = Path(pipeline.__file__).with_name("dashboard.js").read_text()
    functions = script[script.index("    function revealSection("):script.index("    function archiveLinks(")]
    harness = """
const assert = require("node:assert/strict");
function element(id="", textContent="") {
    return {id, textContent, children:[], dataset:{}, append(...items) {this.children.push(...items);},
        cloneNode() {return element(this.id);}, scrollIntoView() {this.scrolled=true;}};
}
const navigation = element("dashboard-navigation");
const collapsed = {tagName:"DETAILS", open:false};
const section = element(); section.dataset.liveKey="recent"; section.parentElement=collapsed;
section.querySelector=()=>({textContent:"Recently finished / rejected"});
const headings = [element("", "Evolution progress"), element("", "Protocol and honest limits")];
const controls = element("live-controls"), qualified = element("qualified-walkers");
const rendering = element("animation-rendering");
const candidatePipeline = {querySelectorAll:()=>[section]};
const document = {
    getElementById: id=>[navigation,controls,qualified,rendering,section,...headings].find(e=>e.id===id),
    querySelectorAll:()=>headings, createElement:()=>element(), addEventListener:()=>{}
};
const addEventListener=()=>{};
let pendingSectionHash="#pipeline-recent";
function patch(current,incoming) {current.children=incoming.children;}
""" + functions + """
updateNavigation();
assert.equal(collapsed.open,true);
assert.equal(section.scrolled,true);
assert.equal(pendingSectionHash,"");
assert.deepEqual(navigation.children.map(a=>a.href),["#live-controls","#qualified-walkers",
    "#animation-rendering","#pipeline-recent","#section-evolution-progress","#section-protocol-and-honest-limits"]);
updateNavigation();
assert.equal(navigation.children.length,6);
assert.equal(headings[0].id,"section-evolution-progress");
assert.equal(revealSection("#unavailable"),false);
"""
    subprocess.run(["node", "-e", harness], check=True, capture_output=True, text=True)


def test_qualified_gallery_renders_source_links_metrics_and_explicit_empty_state():
    import subprocess
    script = Path(pipeline.__file__).with_name("dashboard.js").read_text()
    function = script[script.index("    function renderQualified("):script.index("    function renderPipeline(")]
    harness = """
const assert = require("node:assert/strict");
function element(tag) {
    return {tag, children:[], append(...items) {this.children.push(...items);},
        cloneNode() {return element(this.tag);}};
}
const current = element("div");
const document = {getElementById:()=>current, createElement:element};
const pipelineCard = (item,prefix,collapsed)=>({...element("li"),prefix,collapsed});
const patch = (current,incoming)=>{current.children=incoming.children;};
""" + function + """
renderQualified({count:0,items:[]});
assert.match(current.children[0].textContent,/No recorded qualified walkers yet/);
renderQualified({count:1,items:[{prefix:"/versions/v0041/",version:"v0041",
    speedMmPerSecond:20,forwardMeters:.4,updated:"2026-11-01",video:"designs/robot/render/walker.mp4"}]});
assert.match(current.children[0].textContent,/Showing 1 of 1/);
assert.match(current.children[0].textContent,/historical qualification is not approval under current rules/);
const card=current.children[1].children[0];
assert.equal(card.prefix,"/versions/v0041/");
assert.equal(card.collapsed,true);
assert.match(card.children[0].textContent,/v0041.*20.00 mm\\/s.*0.400 m/);
assert.equal(card.children[1].children[0].href,"/versions/v0041/designs/robot/render/walker.mp4");
"""
    subprocess.run(["node", "-e", harness], check=True, capture_output=True, text=True)


def test_pipeline_distinguishes_pending_active_paused_and_terminal_without_spending_trials(archive):
    birth, design = candidate(archive)
    assert stages(snapshot(archive))["queued"]["count"] == 1
    observe = pipeline.screening_observer(archive)
    observe(design)
    assert stages(snapshot(archive))["screening"]["items"][0]["birth"] == birth
    assert stages(snapshot(archive, running=False))["paused"]["count"] == 1
    observe(design, {"status": "ready-for-dynamics"})
    assert stages(snapshot(archive))["screened"]["count"] == 1
    pipeline.record(archive, birth, "preflight", "checking")
    assert stages(snapshot(archive))["preflight"]["items"][0]["detail"] == "checking"
    assert archive.counts() == {}
    trial = archive.reserve(birth)
    assert stages(snapshot(archive))["simulating"]["count"] == 1
    archive.finish(trial, result())
    value = snapshot(archive)
    assert stages(value)["simulating"]["count"] == 0
    assert value["recent"][0]["stage"] == "complete"
    assert archive.counts() == {"complete": 1}


def test_pipeline_shows_both_visuals_and_explicit_preview_failure(archive):
    birth, design = candidate(archive)
    with archive.db:
        archive.db.execute("INSERT INTO assembly_previews VALUES (?,'failed','renderer unavailable',NULL)", (design,))
    card = stages(snapshot(archive))["queued"]["items"][0]
    assert card["visuals"] == {"assembly": None, "diagram": f"designs/{design}/candidate/cycle.svg"}
    assert card["previewError"] == "renderer unavailable"
    target = archive.directory / "designs" / design / "assembly-render"
    target.mkdir()
    (target / "walker-v2.png").write_bytes(b"still")
    card = stages(snapshot(archive))["queued"]["items"][0]
    assert card["visuals"]["assembly"] == f"designs/{design}/assembly-render/walker-v2.png"
    assert card["visuals"]["diagram"]


def test_pipeline_links_all_direct_repair_births_without_claiming_success(archive):
    parent, _ = candidate(archive)
    pipeline.record(archive, parent, "rejected", "Initial self-interference.")
    first, _ = candidate(archive, 1, parent, "repair-realization")
    second, _ = candidate(archive, 2, parent, "repair-realization")
    ordinary, _ = candidate(archive, 3, parent)
    unrelated, _ = candidate(archive, 4, operation="repair-realization")
    grandchild, _ = candidate(archive, 5, first, "repair-realization")
    with archive.db:
        archive.db.execute("UPDATE births SET status='screening-rejected' WHERE id=?", (first,))
    before = list(archive.db.iterdump())
    value = pipeline.snapshot(archive.db, archive.directory, running=True, tracked=True, limit=2)
    item = next(item for item in value["recent"] if item["birth"] == parent)
    assert item["repairs"] == [
        {"birth": first, "status": "screening-rejected", "history": f"history/{first:04d}.html"},
        {"birth": second, "status": "generated", "history": f"history/{second:04d}.html"},
    ]
    assert ordinary not in [repair["birth"] for repair in item["repairs"]]
    assert list(archive.db.iterdump()) == before
    assert archive.counts() == {}


def test_dashboard_repair_links_use_archive_prefix_and_stay_outside_diagnostics():
    import subprocess
    script = Path(pipeline.__file__).with_name("dashboard.js").read_text()
    function = script[script.index("    function pipelineCard("):script.index("    function screeningChecks(")]
    harness = """
const assert = require("node:assert/strict");
const document = {createElement: tag => ({
    tag, dataset:{}, children:[], append(...items) {this.children.push(...items);}
})};
function screeningChecks(card) {card.append({tag:"diagnostics"});}
""" + function + """
const card = pipelineCard({key:"birth-10", birth:10, history:"history/0010.html",
    category:"4-feet", stage:"rejected", visuals:{}, repairs:[
        {birth:24, status:"generated", history:"history/0024.html"},
        {birth:25, status:"screening-rejected", history:"history/0025.html"}
    ]}, "/versions/v0047/", true);
const links = card.children[2];
assert.equal(links.dataset.liveKey, "repair-births");
assert.equal(links.children[0], "Repair attempts: ");
assert.equal(links.children[1].href, "/versions/v0047/history/0024.html");
assert.equal(links.children[1].textContent, "Birth 24 (generated)");
assert.equal(links.children[3].href, "/versions/v0047/history/0025.html");
assert.equal(card.children[3].tag, "diagnostics");
const noRepair = pipelineCard({key:"birth-11", birth:11, history:"history/0011.html",
    category:"4-feet", stage:"rejected", visuals:{}}, "");
assert.equal(noRepair.children[2].tag, "diagnostics");
"""
    subprocess.run(["node", "-e", harness], check=True, capture_output=True, text=True)


def test_trial_state_replaces_stale_preflight_detail_without_rewriting_events(archive):
    birth, design = candidate(archive)
    waiting = "Preflight passed; awaiting a simulation slot."
    pipeline.record(archive, birth, "ready", waiting)
    assert stages(snapshot(archive))["ready"]["items"][0]["detail"] == waiting
    assert stages(snapshot(archive, running=False))["paused"]["items"][0]["detail"] == "Pipeline paused."
    trial = archive.reserve(birth)
    assert stages(snapshot(archive))["simulating"]["items"][0]["detail"] == "Simulation running."
    archive.finish(trial, result())
    assert snapshot(archive)["recent"][0]["detail"] == "Simulation completed."
    with archive.db:
        archive.db.execute("INSERT INTO render_jobs(evaluation,design,status) VALUES (?,?,'rendering')", (trial, design))
    assert stages(snapshot(archive))["rendering"]["items"][0]["detail"] == "Rendering simulation animation."
    assert archive.db.execute("SELECT detail FROM pipeline_events WHERE birth=?", (birth,)).fetchone()[0] == waiting


@pytest.mark.parametrize("rendering", [False, True])
@pytest.mark.parametrize("qualified", [False, True])
def test_completed_cards_show_recorded_qualification_even_while_rendering(archive, rendering, qualified):
    birth, design = candidate(archive)
    trial = archive.reserve(birth)
    measured = result(qualified=qualified)
    if not qualified:
        measured["status"] = "interference"
        measured["metrics"]["rankingRejectionReasons"] = [
            "self-interference", "incomplete-motor-cycle", "forbidden-ground-contact",
            "incomplete-gait-cycles", "nonrepeatable-forward-motion",
        ]
    archive.finish(trial, measured)
    if rendering:
        with archive.db:
            archive.db.execute("INSERT INTO render_jobs(evaluation,design,status) VALUES (?,?,'rendering')", (trial, design))
    before = list(archive.db.iterdump())
    value = snapshot(archive)
    item = stages(value)["rendering"]["items"][0] if rendering else value["recent"][0]
    assert item["qualification"] == {
        "qualified": qualified, "rejectionReasons": measured["metrics"]["rankingRejectionReasons"]}
    assert item["stage"] == ("rendering" if rendering else "complete")
    assert list(archive.db.iterdump()) == before


def test_unfinished_and_failed_trials_do_not_claim_a_qualification_verdict(archive):
    birth, _ = candidate(archive)
    assert stages(snapshot(archive))["queued"]["items"][0]["qualification"] is None
    trial = archive.reserve(birth)
    assert stages(snapshot(archive))["simulating"]["items"][0]["qualification"] is None
    archive.finish(trial, error="Simulation worker failed.", status="failed")
    item = snapshot(archive)["recent"][0]
    assert item["qualification"] is None and item["detail"] == "Simulation worker failed."


@pytest.mark.parametrize("outcome", ["rejected", "failed"])
def test_prefix_has_separate_stage_and_only_scientific_rejection_has_a_verdict(archive, outcome):
    from legowalkers_sim.bridge import write_json
    from legowalkers_sim.parallel import ensure_attempt_schema

    birth, _ = candidate(archive)
    ensure_attempt_schema(archive.db)
    with archive.db:
        archive.db.execute("INSERT INTO simulation_gate_rounds VALUES (1,4,3,'now','running')")
        archive.db.execute("""INSERT INTO simulation_attempts(id,birth,round,slot,status,pid,log,started)
            VALUES (1,?,1,1,'running',123,'simulation-attempts/0001.log','now')""", (birth,))
    item = stages(snapshot(archive))["gating"]["items"][0]
    assert item["trial"] is None and item["gateAttempt"] == 1 and item["pid"] == 123
    assert item["qualification"] is None and archive.counts() == {}
    if outcome == "rejected":
        write_json(archive.directory / "simulation-attempts/0001/simulation-gate.json",
                   {"status": "rejected", "reasons": ["forbidden-ground-contact"]})
    with archive.db:
        archive.db.execute("UPDATE simulation_attempts SET status=?,error='prefix evidence' WHERE id=1", (outcome,))
    before = list(archive.db.iterdump())
    item = snapshot(archive)["recent"][0]
    assert item["stage"] == outcome and item["trial"] is None and item["detail"] == "prefix evidence"
    assert item["qualification"] == ({"qualified": False, "rejectionReasons": ["forbidden-ground-contact"]}
                                      if outcome == "rejected" else None)
    assert list(archive.db.iterdump()) == before and archive.counts() == {}


def test_completed_trial_without_result_is_an_explicit_error(archive):
    from legowalkers_sim.bridge import AdapterError

    birth, _ = candidate(archive)
    archive.finish(archive.reserve(birth))
    with pytest.raises(AdapterError, match="Completed trial 1 has no recorded result"):
        snapshot(archive)


def test_completed_card_uses_full_qualification_not_just_ranking_eligibility(archive):
    birth, _ = candidate(archive)
    measured = result(qualified=True)
    measured["gait"] = {"completeCycles": 0, "requiredCycles": 2, "repeatableForwardMotion": False}
    archive.finish(archive.reserve(birth), measured)
    assert snapshot(archive)["recent"][0]["qualification"] == {"qualified": False, "rejectionReasons": []}


def test_completed_card_matches_gallery_legacy_policy_without_rewriting_result(archive):
    birth, _ = archive.record_birth({**genome(), "towWagon": True}, None, "legacy", [], "generated")
    archive.finish(archive.reserve(birth), result(qualified=True))
    before = archive.db.execute("SELECT result FROM evaluations").fetchone()[0]
    assert snapshot(archive)["recent"][0]["qualification"] == {
        "qualified": False, "rejectionReasons": ["rigid-wagon-hitch"]}
    assert archive.db.execute("SELECT result FROM evaluations").fetchone()[0] == before


def test_generation_uses_explicitly_labeled_parent_and_bounds_pending_and_recent(archive):
    parent, _ = candidate(archive)
    with archive.db:
        archive.db.execute("""INSERT INTO native_proposals(parent,seed,attempt,after_trial,status,summary,created)
            VALUES (?,42,0,0,'running','{}','now')""", (parent,))
    generation = stages(snapshot(archive))["generating"]["items"][0]
    assert generation["parentPreview"] and generation["birth"] == parent
    assert generation["history"] == "forest.html#proposal-1"
    for token in range(1, 12):
        candidate(archive, token)
    group = stages(snapshot(archive))["queued"]
    assert group["count"] == 12 and len(group["items"]) == 8
    with archive.db:
        archive.db.execute("UPDATE births SET status='screening-rejected',error='test rejection'")
    assert len(snapshot(archive)["recent"]) == 8
    assert not snapshot(archive, tracked=False)["tracked"]

def proposal(archive, parent, status="running", summary=None):
    with archive.db:
        return archive.db.execute("""INSERT INTO native_proposals(parent,seed,attempt,after_trial,status,summary,created)
            VALUES (?,42,0,0,?,?,'now')""", (parent, status, json.dumps(summary or {}))).lastrowid


def test_live_planning_and_builder_events_are_bounded_and_child_retains_evidence(archive):
    from legowalkers_sim.bridge import write_json
    parent, _ = candidate(archive)
    child, design = candidate(archive, 1, parent=parent)
    identifier = proposal(archive, parent)
    path = archive.directory / "proposals" / f"{identifier:06d}"
    events = [{"stage": "planning", "status": "running", "attempt": 0, "detail": "Selecting stock."}]
    write_json(path / "construction-progress.json", {"schemaVersion": 1, "events": events})
    item = stages(snapshot(archive))["planning"]["items"][0]
    assert item["parentPreview"] and item["construction"]["tracked"]
    events.extend({"stage": "building", "status": "rejected", "attempt": i, "detail": f"Rejected stack {i}."}
                  for i in range(1, 30))
    write_json(path / "construction-progress.json", {"schemaVersion": 1, "events": events})
    item = stages(snapshot(archive))["building"]["items"][0]
    assert len(item["construction"]["events"]) == 24 and item["detail"] == "Rejected stack 29."
    summary = {"operation": "input-phase", "generated": [{"genotypeId": design}],
               "counts": {"realizationAttempts": 2, "exported": 1, "rejected": 1, "duplicates": 0}}
    write_json(path / "candidates.json", summary)
    with archive.db:
        archive.db.execute("UPDATE native_proposals SET status='generated',summary=? WHERE id=?",
                           (json.dumps(summary), identifier))
    child_item = next(item for item in stages(snapshot(archive))["queued"]["items"] if item["birth"] == child)
    assert child_item["construction"]["counts"]["realizationAttempts"] == 2
    assert child_item["construction"]["report"] == f"proposals/{identifier:06d}/candidates.json"
    assert archive.counts() == {}


def test_rejected_legacy_proposals_show_saved_attempts_without_invented_live_stages(archive):
    parent, _ = candidate(archive)
    summary = {"operation": "gear-reduction", "generated": [],
               "rejected": [{"attempt": 1, "decision": {"code": "invalid-mesh"}}]}
    identifier = proposal(archive, parent, "rejected", summary)
    before = list(archive.db.iterdump())
    item = snapshot(archive)["recent"][0]
    assert item["proposal"] == identifier and item["stage"] == "rejected"
    assert not item["construction"]["tracked"]
    assert item["construction"]["report"] is None
    assert item["construction"]["events"][0]["stage"] == "saved realization"
    assert "invalid-mesh" in item["construction"]["events"][0]["detail"]
    assert list(archive.db.iterdump()) == before and archive.counts() == {}


@pytest.mark.parametrize("document", [
    [], "{",
    {"schemaVersion": 1, "events": [{"stage": "made-up", "status": "complete", "attempt": 0, "detail": ""}]},
])
def test_invalid_progress_is_reported_not_disguised_as_generation(archive, document):
    from legowalkers_sim.bridge import write_json
    parent, _ = candidate(archive)
    identifier = proposal(archive, parent)
    write_json(archive.directory / f"proposals/{identifier:06d}/construction-progress.json",
               document)
    with pytest.raises(AdapterError, match="Invalid construction (event|progress)"):
        snapshot(archive)


@pytest.mark.integration
def test_actual_cli_records_planning_and_each_bounded_realization_without_trials(tmp_path):
    from legowalkers_sim.evolution import Generator, ROOT
    generator = Generator(ROOT / ".tools/dotnet/dotnet",
                          ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    if not all(Path(path).is_file() for path in generator.command):
        pytest.skip("Build the F# CLI for construction progress integration.")
    founder = tmp_path / "founder"
    generator.invoke("realize-walker", "--feet", 4, "--steps", 72, "--output", founder)
    source = founder / "candidate-001"
    edit = next(edit for edit in generator.manual_edits(source)["edits"] if edit["operation"] == "motor-direction")
    for command, folder in (("propose-walker", "proposal"), ("construct-walker", "construction")):
        output = tmp_path / folder
        if command == "propose-walker":
            summary = generator.apply_edit(source, output, edit["id"], limit=1)
        else:
            generator.invoke(command, "--parent", source, "--output", output, "--limit", 1)
            summary = json.loads((output / "candidates.json").read_text())
        document = json.loads((output / "construction-progress.json").read_text())
        assert document["schemaVersion"] == 1
        events = document["events"]
        assert events[0]["stage"] == "planning" and events[0]["status"] == "running"
        assert events[-1]["stage"] == "building" and events[-1]["status"] in ("complete", "rejected")
        finished = [event for event in events if event["attempt"] > 0 and event["status"] != "running"]
        assert len(finished) == summary["counts"]["realizationAttempts"]
        assert summary["counts"]["dynamicTrials"] == 0
        assert not (output / "construction-progress.json.tmp").exists()


@pytest.mark.parametrize("folder,cached", [("preflight", False), ("preflight", True), ("preflight-isolated", False)])
def test_screening_diagnostics_enrich_old_cards_birth_pages_and_forest_without_rewriting_history(archive, folder, cached):
    from legowalkers_sim.bridge import write_json
    from legowalkers_sim.gallery import publish
    from legowalkers_sim.proposal_forest import nodes
    from legowalkers_sim.screening_reports import birth_page

    birth, design = candidate(archive)
    original = "Native load-only screening rejected candidate-002; see archived reports."
    with archive.db:
        archive.db.execute("UPDATE births SET status='screening-rejected',error=? WHERE id=?", (original, birth))
    root = archive.directory / folder / "profile"
    identifier = "a" * 64
    report = {"genotypeId": design, "screeningId": identifier, "status": "rejected",
              "loadOnly": {"initialSelfInterference": [{"parts": ["rail", "pivot"], "depthMeters": .000365741}]}}
    write_json(root / "reports" / f"{identifier}.json", report)
    alias = {"genotypeId": design, "screeningId": identifier, "status": "duplicate",
             "previousStatus": "rejected"} if cached else report
    summary = root / "screen-0001.json"
    write_json(summary, {"results": [alias]})
    before = summary.read_bytes()
    item = snapshot(archive)["recent"][0]
    assert item["detail"] == "Initial self-interference: rail / pivot: 0.366 mm overlap (collision model)."
    assert item["report"] == f"{folder}/profile/reports/{identifier}.json"
    assert "0.366 mm" in nodes(archive.db)[0].detail and original not in nodes(archive.db)[0].detail
    text = f'<html><body><section class="panel bad">{original}</section></body></html>'
    enriched = birth_page(text, archive.directory, f"history/{birth:04d}.html")
    assert "0.366 mm" in enriched and original not in enriched
    assert f'href="../{item["report"]}"' in enriched
    assert birth_page(enriched, archive.directory, f"history/{birth:04d}.html") == enriched
    assert birth_page(text, archive.directory, "preview.html") == text
    publish(archive)
    page = (archive.directory / "history" / f"{birth:04d}.html").read_text()
    assert "0.366 mm" in page and "Detailed screening report (JSON)" in page
    assert summary.read_bytes() == before
    assert archive.db.execute("SELECT error FROM births WHERE id=?", (birth,)).fetchone()[0] == original
    assert archive.counts() == {}


def test_reconstruction_rejection_and_diagnostic_html_preserve_escaped_errors(archive):
    from legowalkers_sim.bridge import write_json
    from legowalkers_sim.screening_reports import birth_page, rejection_detail

    birth, design = candidate(archive)
    with archive.db:
        archive.db.execute("UPDATE births SET status='screening-rejected',error='old generic message'")
    error = 'Rigid-body partition mismatch: expected <shaft>, detected "coupler".'
    root = archive.directory / "preflight"
    write_json(root / "screen-0001.json", {"results": [
        {"genotypeId": design, "status": "rejected", "error": error}]})
    assert snapshot(archive)["recent"][0]["detail"] == error
    page = birth_page("<body></body>", archive.directory, f"history/{birth:04d}.html")
    assert "&lt;shaft&gt;" in page and "<shaft>" not in page
    assert 'href="../preflight/screen-0001.json"' in page
    assert "invalid-mass-layout: bad center" == rejection_detail({
        "gates": [{"outcome": "reject", "code": "invalid-mass-layout", "diagnostics": ["bad center"]}]})


def test_unreadable_screening_report_keeps_original_error_and_logs_warning(archive, caplog):
    birth, _ = candidate(archive)
    with archive.db:
        archive.db.execute("UPDATE births SET status='screening-rejected',error='original rejection'")
    path = archive.directory / "preflight/screen-0001.json"
    path.parent.mkdir()
    path.write_text("{")
    assert snapshot(archive)["recent"][0]["detail"] == "original rejection"
    assert "Cannot read screening diagnostics" in caplog.text


@pytest.mark.parametrize("render", [None, "assembly-render", "render"])
def test_birth_pages_pair_motion_diagram_with_available_render_and_animation(archive, render):
    from legowalkers_sim.gallery import birth_previews, document, media, publish

    birth, design = candidate(archive)
    root = archive.directory / "designs" / design
    page = archive.directory / "history" / f"{birth:04d}.html"
    if render:
        (root / render).mkdir()
        (root / render / "walker-v2.png").write_bytes(b"png")
        if render == "render":
            (root / render / "walker-v2-anchored.mp4").write_bytes(b"video")
    publish(archive)
    saved = page.read_text()
    assert saved.count("data-motion-diagram") == 1
    assert f'src="../designs/{design}/candidate/cycle.svg"' in saved
    assert "Solved kinematics; not the full LEGO assembly or measured physics." in saved
    assert saved.index("<h2>Saved preview</h2>") < saved.index("<h2>Ancestry</h2>")
    assert ('<video controls' in saved) == (render == "render")
    legacy_preview = ('<section class="panel"><h2>Saved preview</h2>'
                      + media(root, page, True) + "</section>") if render else ""
    legacy = document("Old birth", legacy_preview + '<section class="panel"><h2>Ancestry</h2></section>')
    page.write_text(legacy)
    served = birth_previews(legacy, archive.directory, f"history/{birth:04d}.html")
    assert served.count("data-motion-diagram") == 1
    assert ('<video controls' in served) == (render == "render")
    assert served.index("<h2>Saved preview</h2>") < served.index("<h2>Ancestry</h2>")
    assert birth_previews(served, archive.directory, f"history/{birth:04d}.html") == served
    assert page.read_text() == legacy
    assert archive.counts() == {}


def test_birth_pages_do_not_invent_motion_diagrams_for_legacy_models(archive):
    from legowalkers_sim.gallery import birth_previews, document, publish

    birth, design = candidate(archive)
    (archive.directory / "designs" / design / "candidate/cycle.svg").unlink()
    publish(archive)
    page = archive.directory / "history" / f"{birth:04d}.html"
    saved = page.read_text()
    assert "data-motion-diagram" not in saved
    assert birth_previews(saved, archive.directory, f"history/{birth:04d}.html") == saved
    assert birth_previews(document("Other page", ""), archive.directory, "preview.html") == document("Other page", "")


def test_preview_queue_is_automatic_bounded_and_survives_dashboard_restart(archive, monkeypatch):
    _, design = candidate(archive)
    launched = []
    monkeypatch.setattr(previews.subprocess, "Popen", lambda command, **options:
                        launched.append((command, options)) or SimpleNamespace(pid=123456, poll=lambda: None))
    controller = previews.PreviewController(archive.directory, archive.directory, Path(sys.executable))
    before = [tuple(row) for row in archive.db.execute("SELECT * FROM run")]
    state = controller.status()
    assert state["running"] and state["counts"] == {"queued": 1}
    assert "--budget" not in launched[0][0] and launched[0][1]["start_new_session"]
    assert controller.status()["running"] and len(launched) == 1
    restarted = previews.PreviewController(archive.directory, archive.directory, Path(sys.executable))
    monkeypatch.setattr(previews.os, "kill", lambda *args: None)
    assert restarted.status()["running"] and len(launched) == 1
    assert [tuple(row) for row in archive.db.execute("SELECT * FROM run")] == before
    assert archive.counts() == {}


def test_preview_worker_keeps_failures_visible_and_retries_only_explicitly(archive, monkeypatch, tmp_path):
    _, first = candidate(archive)
    _, second = candidate(archive, 1)
    monkeypatch.setattr(previews, "ROOT", tmp_path)
    calls = []
    def render(directory, design, library, blender):
        calls.append(design)
        if design == first:
            raise OSError("test renderer failure")
        target = directory / "designs" / design / "assembly-render"
        target.mkdir(exist_ok=True)
        (target / "walker-v2.png").write_bytes(b"still")
        return 1.25
    monkeypatch.setattr(previews, "render", render)
    previews.run_worker(archive.directory, tmp_path, Path(sys.executable))
    rows = {row["design"]: dict(row) for row in archive.db.execute("SELECT * FROM assembly_previews")}
    assert rows[first]["status"] == "failed" and "test renderer failure" in rows[first]["error"]
    assert rows[second]["status"] == "complete" and rows[second]["elapsed"] == 1.25
    assert f"designs/{second}/assembly-render/walker-v2.png" in (archive.directory / "index.html").read_text()
    assert "<h2>Saved preview</h2>" in (archive.directory / "history/0002.html").read_text()
    previews.run_worker(archive.directory, tmp_path, Path(sys.executable))
    assert calls.count(first) == calls.count(second) == 1
    controller = previews.PreviewController(archive.directory, tmp_path, None)
    controller.retry()
    assert archive.db.execute("SELECT status FROM assembly_previews WHERE design=?", (first,)).fetchone()[0] == "queued"
    assert not controller.status()["available"] and not archive.counts()


def test_still_render_uses_thumbnail_without_evaluation_and_publishes_only_small_assets(archive, monkeypatch):
    _, design = candidate(archive)
    commands = []
    def run(command, **options):
        commands.append(command)
        assets = Path(command[command.index("--assets") + 1])
        assets.mkdir(parents=True, exist_ok=True)
        (assets / "metadata.json").write_text("{}")
        if len(commands) == 2:
            (assets / "walker-v2.png").write_bytes(b"still")
    monkeypatch.setattr(previews.subprocess, "run", run)
    elapsed = previews.render(archive.directory, design, archive.directory, Path(sys.executable))
    assert elapsed >= 0 and len(commands) == 2
    assert "prepare-still" in commands[0] and "--evaluation" not in commands[0]
    assert commands[1][commands[1].index("--threads") + 1] == "2"
    assert all(command[-1] == "thumbnail" for command in commands)
    target = archive.directory / "designs" / design / "assembly-render"
    info = json.loads((target / "preview-info.json").read_text())
    assert (info["width"], info["height"], info["simulationSteps"]) == (480, 300, 0)
    assert {path.name for path in target.iterdir()} == {"walker-v2.png", "metadata.json", "preview-info.json"}
    assert not list(target.parent.glob("assembly-preview-*")) and not archive.counts()


def test_dashboard_status_includes_pipeline_and_keeps_missing_blender_explicit(tmp_path):
    controller = EpisodeController(tmp_path, protocol(), ["test-worker"])
    value = controller.status()
    assert value["pipeline"]["recent"] == []
    assert all(stage["count"] == 0 for stage in value["pipeline"]["stages"])
    assert not value["assemblyPreviews"]["available"] and not value["running"]


def test_preview_watcher_discovers_later_realizations_without_browser_polling(archive, monkeypatch, tmp_path):
    monkeypatch.setattr(previews, "ROOT", tmp_path)
    rendered = threading.Event()
    errors = []
    def render(directory, design, library, blender):
        target = directory / "designs" / design / "assembly-render"
        target.mkdir()
        (target / "walker-v2.png").write_bytes(b"still")
        rendered.set()
        return .1
    monkeypatch.setattr(previews, "render", render)
    archive.set_status("running")
    def worker():
        try:
            previews.run_worker(archive.directory, tmp_path, Path(sys.executable))
        except Exception as error:
            errors.append(error)
    thread = threading.Thread(target=worker)
    thread.start()
    try:
        _, design = candidate(archive)
        assert rendered.wait(5)
        assert thread.is_alive()
    finally:
        archive.set_status("complete")
        thread.join(timeout=10)
    assert not thread.is_alive() and not errors
    assert archive.db.execute("SELECT status FROM assembly_previews WHERE design=?", (design,)).fetchone()[0] == "complete"
    assert not archive.counts()
