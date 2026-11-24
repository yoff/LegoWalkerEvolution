import hashlib
import json
import os
import sqlite3
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError, write_json
from legowalkers_sim.evolution import Archive, CATEGORIES, LEGACY_CATEGORIES, SEARCH_POLICY, ROOT, category, category_genome, design_identity, fitness, freeze_runtime, is_qualified, search, validate_search_candidate
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.gallery import category_anchors, collapsible_genes, preview_first, publish
from legowalkers_sim.payload import apply_payloads, load_profile


def genome(**changes):
    return dict(pairs=1, remote=False, seed=42, groundHoles=4, couplerHoles=4, rockerHoles=4,
                phaseQuarterTurns=1, motorDirection=1, batterySlot=1, batteryLevel=0,
                receiverSlot=0, receiverSide=1, towWagon=False, batteryOnWagon=False,
                receiverOnWagon=False, wagonAxleHole=4, **changes)


def protocol():
    return {"payloadProfile": load_profile(), "categories": [category_genome(*item) for item in LEGACY_CATEGORIES],
            "qualification": {"allowedGroundContacts": "designated-foot-regions-and-designated-wagon-wheels"}}


@pytest.fixture
def archive(tmp_path):
    archive = Archive(tmp_path)
    archive.initialize(protocol(), 12, 42, False)
    yield archive
    archive.db.close()


def result(*, qualified=False, speed=.001, turns=.2):
    return {
        "status": "evaluated",
        "metrics": {"eligibleForRanking": qualified, "speedMetersPerSecond": speed,
                    "absoluteMechanicalWorkJoules": 1, "closureWithinTolerance": True,
                    "fallen": False, "motorTurns": [1.2 if qualified else turns], "forwardMeters": speed*19,
                    "simulatedMassKg": .5, "maxTiltDegrees": 1, "maxClosureErrorMeters": .0001,
                    "rankingRejectionReasons": [] if qualified else ["incomplete-motor-cycle"]},
        "collision": {"fullMotorCycle": qualified, "maxSelfPenetrationMeters": .00001},
        "groundContact": {"forbiddenContactDetected": False, "forbiddenContacts": [],
                          "footContactSamples": 1, "wheelContactSamples": 0},
    }


def test_design_identity_excludes_provenance_seed_but_not_payload_or_polarity():
    g = genome()
    assert design_identity(g) == design_identity({**g, "seed": 900})
    assert design_identity(g) != design_identity({**g, "motorDirection": -1})
    assert design_identity(g) != design_identity({**g, "remote": True})
    assert category(g) == "four-bar-2-wired-solo"
    assert category({**g, "remote": True}) == "four-bar-2-remote-solo"
    assert category({**g, "towWagon": True}) == "four-bar-2-wired-towing"


def test_protocol_isolation_and_resume(archive):
    with pytest.raises(AdapterError, match="already exists"):
        archive.initialize(protocol(), 12, 42, False)
    archive.initialize(protocol(), 12, 42, True)
    with pytest.raises(AdapterError, match="differs"):
        archive.initialize({**protocol(), "friction": .5}, 12, 42, True)
    with pytest.raises(AdapterError, match="differs"):
        archive.initialize(protocol(), 6, 42, True)


def test_birth_ancestry_is_not_design_identity_and_is_acyclic(archive):
    first, design = archive.record_birth(genome(), None, "founder", [], "generated")
    second, same = archive.record_birth(genome(), first, "clone", [], "duplicate")
    assert second != first and same == design
    with pytest.raises(sqlite3.IntegrityError, match="backwards"):
        archive.record_birth(genome(), 999, "bad", [], "duplicate")
    assert archive.db.execute("SELECT count(*) FROM designs").fetchone()[0] == 1
    with pytest.raises(sqlite3.IntegrityError, match="immutable"):
        archive.db.execute("UPDATE births SET parent=? WHERE id=?", (second, first))
    with pytest.raises(sqlite3.IntegrityError, match="immutable"):
        archive.db.execute("UPDATE designs SET genome='{}'")


def test_finalized_outcomes_cannot_be_overwritten(archive):
    birth, _ = archive.record_birth(genome(), None, "founder", [], "generated")
    trial = archive.reserve(birth)
    archive.finish(trial, result())
    with pytest.raises(AdapterError, match="immutable"):
        archive.finish(trial, result(qualified=True))
    assert not json.loads(archive.outcomes()[0]["result"])["metrics"]["eligibleForRanking"]


def test_runtime_snapshot_keeps_hash_verified_sources_and_binaries(archive, tmp_path):
    binary = tmp_path / "build"
    binary.mkdir()
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        (binary / name).write_bytes(name.encode())
    source = "simulator/legowalkers_sim/evolution.py"
    protocol = {
        "sourceSha256": {source: hashlib.sha256((ROOT / source).read_bytes()).hexdigest()},
        "generatorBinarySha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in binary.glob("*.dll")},
    }
    generator = SimpleNamespace(command=["unused-dotnet", str(binary / "LegoWalkers.Cli.dll")])
    freeze_runtime(archive, protocol, generator)
    assert Path(generator.command[1]).parent == archive.directory / "protocol-source/generator"
    frozen = archive.directory / "protocol-source" / source
    assert frozen.read_bytes() == (ROOT / source).read_bytes()
    frozen.write_text("changed")
    with pytest.raises(AdapterError, match="Frozen protocol source checksum"):
        freeze_runtime(archive, protocol, generator)


def test_budget_counts_failed_and_interrupted_trials_without_repeating_them(archive):
    for i in range(12):
        birth, _ = archive.record_birth(genome(), None, "founder", [], "generated")
        trial = archive.reserve(birth)
        if i % 2:
            archive.finish(trial, error="numerical failure", status="failed")
    archive.recover()
    assert archive.counts() == {"interrupted": 6, "failed": 6}
    archive.recover()
    assert archive.counts() == {"interrupted": 6, "failed": 6}
    birth, _ = archive.record_birth(genome(), None, "founder", [], "generated")
    with pytest.raises(AdapterError, match="budget exhausted"):
        archive.reserve(birth)
    assert sum(archive.counts().values()) == 12


def test_cheap_rejections_duplicates_and_qualification_do_not_create_false_champions(archive):
    root, _ = archive.record_birth(genome(), None, "founder", [], "generation-rejected", "<script>bad</script>")
    assert archive.counts() == {}
    birth, _ = archive.record_birth(genome(), root, "retry", [], "generated")
    archive.finish(archive.reserve(birth), result(speed=10))
    assert fitness(result(qualified=True, speed=.00001)) > fitness(result(speed=10))
    publish(archive)
    html = (archive.directory / "preview.html").read_text()
    assert html.count("No qualified walker yet") == 12
    assert "Best tested qualified walker" not in html
    assert "Best unqualified prototype" in html
    assert "No qualified speed history yet" in html
    assert "&lt;script&gt;bad&lt;/script&gt;" in (archive.directory / "history/0001.html").read_text()
    assert "<script>bad</script>" not in html
    assert "birth 1" in (archive.directory / "history/0002.html").read_text()
    inconsistent = {**result(qualified=True), "status": "interference"}
    assert not is_qualified(inconsistent)
    assert fitness(inconsistent) < fitness(result(qualified=True))
    inconsistent = {**result(qualified=True), "groundContact": {
        "forbiddenContactDetected": True, "forbiddenContacts": []}}
    assert not is_qualified(inconsistent)


@pytest.mark.parametrize("status", ["failed", "interrupted"])
def test_gallery_reports_failed_trials_without_implying_pending_results(archive, status):
    birth, _ = archive.record_birth(genome(), None, "founder", [], "generated")
    trial = archive.reserve(birth)
    archive.finish(trial, error="Mesh export failed: <mesh_0>", status=status)
    publish(archive)
    html = (archive.directory / "preview.html").read_text()
    assert f"Trial {trial}: {status}" in html
    assert "Mesh export failed: &lt;mesh_0&gt;" in html
    assert "Failure details and lineage" in html
    assert "Awaiting a completed evaluation" not in html
    assert "Best tested qualified walker" not in html
    assert archive.counts() == {status: 1}


def test_gallery_separates_categories_and_embeds_available_media(archive):
    for pairs, remote in [(1, False), (1, True)]:
        g = {**genome(), "pairs": pairs, "remote": remote}
        birth, design = archive.record_birth(g, None, "founder", [], "generated")
        archive.finish(archive.reserve(birth), result(qualified=not remote))
        render = archive.directory / "designs" / design / "render"
        render.mkdir(parents=True)
        (render / "walker-v2.png").write_bytes(b"png")
        (render / "walker-v2-anchored.mp4").write_bytes(b"mp4")
    publish(archive)
    embedded = (archive.directory / "preview.html").read_text()
    linked = (archive.directory / "index.html").read_text()
    assert embedded.count("Best tested qualified walker") == 1
    assert embedded.count("No qualified walker yet") == 11
    assert "data:video/mp4;base64," in embedded and "data:image/png;base64," in embedded
    assert "data:video" not in linked
    assert "remote / external" not in embedded
    assert "Best tested qualified forward walking speed" in embedded
    for number in (1, 2):
        history = (archive.directory / f"history/{number:04d}.html").read_text()
        assert history.index("<h1>Birth") < history.index("<h2>Saved preview</h2>")
        assert history.index("<h2>Saved preview</h2>") < history.index("<h2>Ancestry</h2>")
        assert history.index("<h2>Saved preview</h2>") < history.index("<h2>Canonical genome")
        assert history.count('<details class="birth-genes">') == 2
        assert '<summary><h3>Actual gene differences from parent</h3></summary><pre>' in history
        assert '<summary><h2>Canonical genome (provenance seed excluded)</h2></summary><pre>' in history
        assert '<details class="birth-genes" open' not in history
        assert "data:video/mp4;base64," in history


def test_saved_birth_gene_sections_collapse_without_changing_data_or_hiding_results():
    genes = '<pre>{\n  "part": "&lt;beam&gt;"\n}</pre>'
    old = ('<html><head></head><body><h3>Recorded motion edits</h3><pre>[]</pre>'
           '<h3>Actual gene differences from parent</h3>\n' + genes
           + '<section><h2>Canonical genome (provenance seed excluded)</h2>' + genes
           + '</section><section><h2>Trial: complete</h2><p>forbidden-ground-contact</p></section></body></html>')
    updated = collapsible_genes(old)
    assert updated.count('<details class="birth-genes">') == 2
    assert '<details class="birth-genes" open' not in updated
    assert updated.count(genes) == 2
    assert '<h3>Recorded motion edits</h3><pre>[]</pre>' in updated
    assert '</details></section><section><h2>Trial: complete</h2><p>forbidden-ground-contact</p>' in updated
    assert ".birth-genes>summary{cursor:pointer}" in updated
    assert collapsible_genes(updated) == updated
    assert collapsible_genes("<h1>Other page</h1>") == "<h1>Other page</h1>"


def test_legacy_birth_preview_is_moved_without_losing_or_rewriting_gene_details():
    header = "<h1>Birth 13</h1><p>Design 123</p>"
    genes = '<section class="panel"><h2>Ancestry</h2><pre>' + "gene data" * 1000 + "</pre></section>"
    preview = '<section class="panel"><h2>Saved preview</h2><video controls></video></section>'
    trial = "<section>Trial details</section>"
    old = header + genes + preview + trial
    updated = preview_first(old)
    assert updated == header + preview + genes + trial
    assert preview_first(updated) == updated
    assert preview_first(header + genes + trial) == header + genes + trial
    with pytest.raises(ValueError, match="incomplete"):
        preview_first(header + genes + preview.removesuffix("</section>"))


def test_old_category_tiles_gain_stable_focus_targets_without_duplicate_ids():
    old = '<html><head></head><body><article class="card" data-live-key="4-feet">Prototype</article></body></html>'
    updated = category_anchors(old)
    assert 'id="category-4-feet" tabindex="-1"' in updated
    assert ".card:target" in updated
    assert category_anchors(updated) == updated


def test_rigid_wagon_history_is_preserved_but_cannot_be_a_current_champion(archive):
    birth, _ = archive.record_birth({**genome(), "towWagon": True}, None, "attach-wagon", [], "generated")
    archive.finish(archive.reserve(birth), result(qualified=True))
    publish(archive)
    html = (archive.directory / "index.html").read_text()
    assert "rigid-wagon-hitch" in html and "Superseded towing rule" in html
    assert "Best tested qualified walker" not in html
    assert "No qualified speed history yet" in html
    assert "rigid-wagon-hitch" in (archive.directory / "history/0001.html").read_text()
    assert json.loads(archive.outcomes()[0]["result"])["metrics"]["eligibleForRanking"]


def test_invalid_payload_profiles_are_explicit_errors(tmp_path):
    profile = load_profile()
    assert profile["components"]["8881"]["massKg"] == .197
    for value in [-1, 0, float("nan"), "heavy", True]:
        profile["components"]["8881"]["massKg"] = value
        path = tmp_path / "bad.json"
        path.write_text(json.dumps(profile))
        with pytest.raises(AdapterError, match="Invalid payload"):
            load_profile(path)
    for invalid in [[], {}, {"schemaVersion": 1, "components": []},
                    {"schemaVersion": 1, "components": {"8881": None, "8884": None}}]:
        path.write_text(json.dumps(invalid))
        with pytest.raises(AdapterError, match="Payload profile|payload component"):
            load_profile(path)


def test_incomplete_result_json_is_preserved_and_interruption_stays_spent(archive):
    birth, _ = archive.record_birth(genome(), None, "founder", [], "generated")
    trial = archive.reserve(birth)
    output = archive.directory / "evaluations" / f"{trial:04d}"
    output.mkdir(parents=True)
    (output / "result.json").write_text('{"metrics":')
    archive.recover()
    assert archive.counts() == {"interrupted": 1}
    assert (output / "result-partial.json").read_text() == '{"metrics":'
    assert "incomplete result" in json.loads((output / "error.json").read_text())["error"]
    archive.recover()
    assert archive.counts() == {"interrupted": 1}


def test_payload_mass_com_and_parallel_axis_inertia_replace_not_add_fake_plastic():
    from lego_technic_sim.ldraw.model import Triangle

    # A closed tetrahedron is enough to exercise mesh-mass replacement.
    vertices = np.array([[0., 0, 0], [20., 0, 0], [0., 20, 0], [0., 0, 20]])
    triangles = [Triangle(vertices[a], vertices[b], vertices[c], 7)
                 for a, b, c in [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]]
    geometry = SimpleNamespace(triangles=triangles, transform=np.eye(4), position=np.zeros(3))
    part = SimpleNamespace(instance="battery", unit=0, geometry=geometry)

    @__import__("dataclasses").dataclass
    class Unit:
        mass: float
        center_of_mass: np.ndarray

    @__import__("dataclasses").dataclass
    class Scene:
        units: list

    scene = Scene([Unit(100, np.ones(3))])
    profile = load_profile()
    manifest = {"payloads": [{"partInstance": "battery", "product": "8881"}]}
    updated, tensors = apply_payloads(scene, [part], manifest, profile)
    assert updated.units[0].mass == pytest.approx(.197)
    assert updated.units[0].center_of_mass == pytest.approx([0, 0, 0])
    assert np.linalg.eigvalsh(tensors[0]).min() > 0
    assert scene.units[0].mass == 100
    profile["components"]["8881"]["centerLdu"] = [100, 0, 0]
    with pytest.raises(AdapterError, match="center outside physical geometry"):
        apply_payloads(scene, [part], manifest, profile)
    profile["components"]["8881"]["centerLdu"] = [10, 0, 0]
    shifted, _ = apply_payloads(scene, [part], manifest, profile)
    assert shifted.units[0].center_of_mass == pytest.approx([.004, 0, 0])


@pytest.mark.integration
def test_real_pilot_archive_budget_payload_lineage_and_captured_poses():
    directory = os.environ.get("LEGOWALKERS_PILOT")
    if directory is None:
        pytest.skip("Set LEGOWALKERS_PILOT to a completed real pilot archive.")
    directory = Path(directory)
    db = sqlite3.connect(f"file:{directory / 'archive.sqlite3'}?mode=ro", uri=True)
    db.row_factory = sqlite3.Row
    try:
        assert not db.execute("PRAGMA foreign_key_check").fetchall()
        run = db.execute("SELECT * FROM run").fetchone()
        assert run["status"] == "complete"
        rows = db.execute("""
            SELECT e.*,b.design,b.parent,b.operation,d.genome,d.category FROM evaluations e
            JOIN births b ON b.id=e.birth JOIN designs d ON d.id=b.design ORDER BY e.id
        """).fetchall()
        assert len(rows) == run["budget"] == 12
        kinds = {row["category"] for row in rows}
        assert len(kinds) == 12
        assert all(sum(row["category"] == kind for row in rows) == 1 for kind in kinds)
        masses = {}
        for row in rows:
            assert row["protocol"] == run["protocol"]
            assert row["status"] in ("complete", "failed", "interrupted")
            path = directory / "evaluations" / f"{row['id']:04d}"
            if row["status"] != "complete":
                assert row["result"] is None and row["error"]
                error = json.loads((path / "error.json").read_text())
                assert error["status"] == row["status"] and error["error"] == row["error"]
                continue
            r = json.loads(row["result"])
            poses = json.loads((path / "poses.json").read_text())
            trajectory = json.loads((path / "trajectory.json").read_text())
            assert poses["modelSha256"] == hashlib.sha256((path / "model.xml").read_bytes()).hexdigest()
            assert poses["ldrawSha256"] == r["compatibility"]["ldrawSha256"]
            assert len(poses["frames"]) == 1001
            assert poses["frames"][0]["time"] == 0
            assert poses["frames"][-1]["time"] == pytest.approx(20)
            assert np.array(poses["frames"][-1]["bodies"]["unit_0"]["position"])/25 == pytest.approx(trajectory["frames"][-1]["chassis"])
            g = json.loads(row["genome"])
            assert r["control"]["effectiveSpeedRadPerSecond"] == 3*g["motorDirection"]
            assert not r["options"]["anchor"] and r["options"]["self_collision"]
            assert bool(r["metrics"]["rankingRejectionReasons"]) != r["metrics"]["eligibleForRanking"]
            if g["remote"]:
                assert r["payloadProfile"]["components"]["8881"]["massKg"] == .197
                assert r["payloadProfile"]["components"]["8884"]["massKg"] == .024
            masses[(g["pairs"], g["remote"], g["towWagon"])] = r["metrics"]["simulatedMassKg"]
            assert len(r["groundContact"]["designatedWheels"]) == (2 if g["towWagon"] else 0)
            assert ("forbidden-ground-contact" in r["metrics"]["rankingRejectionReasons"]) == r["groundContact"]["forbiddenContactDetected"]
            if row["operation"] == "single-gene-mutation":
                parent = db.execute("""
                    SELECT d.genome FROM births b JOIN designs d ON d.id=b.design WHERE b.id=?
                """, (row["parent"],)).fetchone()
                before = json.loads(parent["genome"])
                assert sum(value != before[key] for key, value in g.items() if key != "seed") == 1
            assert row["protocol"] == run["protocol"]
        for pairs in (1, 2, 3):
            for towing in (False, True):
                loaded, unloaded = (pairs, True, towing), (pairs, False, towing)
                if loaded in masses and unloaded in masses:
                    assert masses[loaded] - masses[unloaded] > .221
        counts = {row["status"]: row["n"] for row in db.execute("SELECT status,count(*) n FROM evaluations GROUP BY status")}
        assert sum(counts.values()) == 12
        assert json.loads((directory / "progress.json").read_text())["counts"] == counts
    finally:
        db.close()


def test_search_is_balanced_parent_selected_and_resume_spends_nothing_twice(archive, monkeypatch):
    import legowalkers_sim.evolution as evolution

    trials = []

    class Generator:
        def propose(self, pairs, remote, towing, seed, attempt, parent):
            g = json.loads(parent.read_text()) if parent else genome()
            before = g["remote"]
            before_towing = g["towWagon"]
            g = {**g, "pairs": pairs, "remote": remote, "towWagon": towing, "seed": seed}
            operation = "founder"
            if parent:
                operation = "attach-wagon" if towing != before_towing else ("add-electronics" if remote != before else "single-gene-mutation")
                if remote == before and towing == before_towing:
                    g["motorDirection"] *= -1
            return dict(genome=g, operation=operation, changed=[], accepted=True, errors=[])

        def invoke(self, *args):
            if args[0] == "generate":
                source, output = Path(args[2]), Path(args[4])
                write_json(output / "manifest.json", {"ldrawSha256": "fixture"})
                (output / "model.ldr").write_text("fixture")

    def evaluated(checked, output, options):
        trials.append(options)
        output.mkdir(parents=True)
        return result()

    monkeypatch.setattr(evolution, "check", lambda *_: SimpleNamespace(report={}))
    monkeypatch.setattr(evolution, "evaluate", evaluated)
    with archive.db:
        archive.db.execute("UPDATE run SET budget=24")
    options = EvaluationOptions(duration=20, timestep=.002, capture_poses=True)
    search(archive, Generator(), Path("unused-library"), options)
    assert len(trials) == 24 and archive.counts() == {"complete": 24}
    counts = archive.db.execute("""
        SELECT d.category,count(*) n FROM evaluations e JOIN births b ON b.id=e.birth
        JOIN designs d ON d.id=b.design GROUP BY d.category
    """).fetchall()
    assert len(counts) == 12 and all(row["n"] == 2 for row in counts)
    parents = archive.db.execute("SELECT operation,parent FROM births ORDER BY id").fetchall()
    assert sum(row["operation"] == "single-gene-mutation" for row in parents) == 12
    assert sum(row["operation"] == "add-electronics" and row["parent"] is not None for row in parents) == 3
    search(archive, Generator(), Path("unused-library"), options)
    assert len(trials) == 24
    assert json.loads((archive.directory / "progress.json").read_text())["status"] == "complete"


@pytest.mark.parametrize("cycle_aware", [False, True])
@pytest.mark.parametrize("mutations", [
    ["move-hinge-site"],
    ["move-hinge-site", "duplicate-branch", "gear-reduction", "motor-layout", "battery-layout", "receiver-layout"],
])
def test_gallery_describes_recorded_mutations_and_trial_timing(tmp_path, cycle_aware, mutations):
    active = {**protocol(), "searchPolicy": SEARCH_POLICY,
              "categories": [category_genome(*item) for item in CATEGORIES],
              "nativeDiscovery": {"mutations": mutations},
              "evaluation": {"duration": 30, "settle": .5, "timestep": .004, "cycle_aware": cycle_aware}}
    archive = Archive(tmp_path)
    try:
        archive.initialize(active, 4, 42, False)
        publish(archive)
        page = (tmp_path / "index.html").read_text()
        assert "Enabled mutations: " + ", ".join(name.replace("-", " ") for name in mutations) in page
        assert "Foot count is preserved" not in page
        assert "0.5 s settling; 4 ms timesteps" in page
        if cycle_aware:
            assert "at least 30 simulated seconds" in page
            assert "2 nominal gait cycles across all motor-coupled feet" in page
            assert "capped at 60 seconds" in page
        else:
            assert "Each trial: 30 simulated seconds." in page
            assert "nominal gait cycles" not in page
    finally:
        archive.db.close()


@pytest.mark.parametrize("evaluation", [{}, {"duration": 30}])
def test_gallery_does_not_invent_missing_trial_timing(tmp_path, evaluation):
    archive = Archive(tmp_path)
    try:
        archive.initialize({**protocol(), "evaluation": evaluation}, 4, 42, False)
        publish(archive)
        page = (tmp_path / "index.html").read_text()
        assert "does not fully specify trial timing" in page
        assert "Each trial:" not in page
    finally:
        archive.db.close()


def test_active_search_is_remote_solo_and_lineage_crosses_foot_count_bins(tmp_path, monkeypatch):
    import legowalkers_sim.evolution as evolution
    assert CATEGORIES == [(1, True, False), (2, True, False), (3, True, False)]
    active = {**protocol(), "searchPolicy": SEARCH_POLICY, "grammarVersion": "technic-fourbar-v5",
              "categories": [category_genome(*item) for item in CATEGORIES]}
    archive = Archive(tmp_path)
    archive.initialize(active, 6, 42, False)
    seen = []

    class Generator:
        def propose(self, pairs, remote, towing, seed, attempt, parent):
            before = json.loads(parent.read_text()) if parent else {**genome(), "remote": True}
            child = {**before, "pairs": pairs, "remote": remote, "towWagon": towing, "seed": seed}
            if parent:
                child["motorDirection"] *= -1
            return {"genome": child, "accepted": True, "errors": [],
                    "operation": "change-foot-count" if before["pairs"] != pairs else "mutate",
                    "changed": [key for key in child if key != "seed" and child[key] != before[key]]}

        def invoke(self, *args):
            g = json.loads(Path(args[2]).read_text())
            write_json(Path(args[4])/"manifest.json", {"grammarVersion": "technic-fourbar-v5", "genome": g,
                                                      "feet": [{} for _ in range(g["pairs"]*2)],
                                                      "groundWheels": []})

    def check_fixture(path, _):
        return SimpleNamespace(manifest=json.loads(path.read_text()), report={})

    def evaluate_fixture(checked, output, _):
        seen.append(checked.manifest)
        output.mkdir(parents=True)
        return result(qualified=True, speed=checked.manifest["genome"]["pairs"]*.001)

    monkeypatch.setattr(evolution, "check", check_fixture)
    monkeypatch.setattr(evolution, "evaluate", evaluate_fixture)
    try:
        search(archive, Generator(), Path("unused"), EvaluationOptions())
        assert len(seen) == 6
        assert all(m["genome"]["remote"] and not m["genome"]["towWagon"] for m in seen)
        rows = archive.db.execute("""
            SELECT child.category child_category, ancestor.category parent_category
            FROM births b JOIN designs child ON child.id=b.design
            JOIN births p ON p.id=b.parent JOIN designs ancestor ON ancestor.id=p.design
        """).fetchall()
        assert any(row["child_category"] != row["parent_category"] for row in rows)
        assert {row["category"] for row in archive.outcomes()} == {"2-feet", "4-feet", "6-feet"}
        search(archive, Generator(), Path("unused"), EvaluationOptions())
        assert len(seen) == 6
        page = (tmp_path/"preview.html").read_text()
        assert "3 foot-count categories" in page and "ancestry boundary" in page
        assert "Pulls a wagon" not in page and "Wired / external power" not in page
        assert "2-feet" in page and "change-foot-count" in page
        with pytest.raises(AdapterError, match="remote, non-towing"):
            archive.record_birth(genome(), None, "founder", [], "generated")
    finally:
        archive.db.close()


@pytest.mark.parametrize("feet,remote,towing,wheels", [
    (7, True, False, []), (0, True, False, []), (1, True, False, []), (2, False, False, []),
    (2, True, True, []), (2, True, False, ["wheel"]), (4, True, False, []),
])
def test_search_checks_realized_feet_and_remote_policy_before_reserving(feet, remote, towing, wheels):
    checked = SimpleNamespace(manifest={
        "grammarVersion": "technic-fourbar-v5", "feet": [{} for _ in range(feet)],
        "genome": {"remote": remote, "towWagon": towing}, "groundWheels": wheels,
    }, report={})
    with pytest.raises(AdapterError, match="inferred-foot-count"):
        validate_search_candidate(checked, 2)


def test_search_minimum_is_not_a_required_even_foot_count():
    assert SEARCH_POLICY["minimumFeet"] == 2 and SEARCH_POLICY["maximumFeet"] == 6
    for count in range(2, 7):
        checked = SimpleNamespace(manifest={
            "grammarVersion": "technic-fourbar-v5", "feet": [{} for _ in range(count)],
            "genome": {"remote": True, "towWagon": False}, "groundWheels": [],
        }, report={})
        validate_search_candidate(checked, count)
        assert checked.report["logicalFootCount"] == count
    one = SimpleNamespace(manifest={
        "grammarVersion": "technic-fourbar-v5", "feet": [{}],
        "genome": {"remote": True, "towWagon": False}, "groundWheels": [],
    }, report={})
    with pytest.raises(AdapterError, match="inferred-foot-count"):
        validate_search_candidate(one, 1)


@pytest.mark.parametrize("count,qualified", [(None, False), (0, False), (1, False), (2, True),
                                           (3, True), (4, True), (5, True), (6, True), (7, False)])
def test_new_results_cannot_rank_single_foot_prototypes_as_walking_robots(count, qualified):
    measured = result(qualified=True)
    measured["metrics"]["logicalFootCount"] = count
    assert is_qualified(measured) is qualified
