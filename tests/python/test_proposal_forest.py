import json
import xml.etree.ElementTree as ET

import pytest

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.evolution import Archive
from legowalkers_sim.gallery import publish
from legowalkers_sim.proposal_forest import Node, diagnostics, diagram, insert, layout, nodes, panel, snapshot
from test_evolution import genome, protocol, result


@pytest.fixture
def archive(tmp_path):
    value = Archive(tmp_path)
    value.initialize(protocol(), 12, 42, False)
    yield value
    value.db.close()


def birth(archive, status="generated", parent=None, operation="mutation"):
    return archive.record_birth(genome(), parent, operation, [], status)[0]


def proposal(archive, status, parent, operation="move-hinge-site"):
    with archive.db:
        return archive.db.execute("""
            INSERT INTO native_proposals(parent,seed,attempt,after_trial,status,summary,error,created)
            VALUES (?,42,0,0,?,?,NULL,'now')
        """, (parent, status, json.dumps({"operation": operation}))).lastrowid


def candidate(archive, child, attempt):
    with archive.db:
        archive.db.execute("INSERT INTO native_candidates VALUES (?,?,?,?)",
                           (child, f"proposals/{attempt:06d}", f"candidate-{child}", "{}"))


def test_outcomes_are_distinct_and_running_comes_from_the_trial_not_birth(archive):
    expected = {}
    for state, qualified in (("qualified", True), ("unqualified", False), ("running", None),
                             ("failed", None), ("interrupted", None)):
        child = birth(archive)
        trial = archive.reserve(child)
        if state in ("qualified", "unqualified"):
            archive.finish(trial, result(qualified=qualified))
        elif state != "running":
            archive.finish(trial, error="load failed", status=state)
        expected[f"b{child}"] = state
    for status, state in (("generated", "pending"), ("duplicate", "duplicate"),
                          ("screening-rejected", "rejected"), ("mass-layout-rejected", "rejected"),
                          ("evaluation-envelope-rejected", "rejected")):
        expected[f"b{birth(archive, status)}"] = state
    graph = nodes(archive.db)
    assert {node.key: node.state for node in graph} == expected
    page = panel(archive.db)
    assert page.count('class="forest-running-ring"') == 1
    assert 'data-state="qualified"' in page and 'data-state="unqualified"' in page
    assert 'prefers-reduced-motion' in page
    assert archive.counts() == {"complete": 2, "failed": 1, "interrupted": 1, "running": 1}


def test_proposals_include_cheap_rejections_and_connect_all_physical_alternatives(archive):
    root = birth(archive)
    rejected = proposal(archive, "rejected", root)
    generated = proposal(archive, "generated", root)
    child, sibling = birth(archive, parent=root), birth(archive, parent=root)
    candidate(archive, child, generated)
    candidate(archive, sibling, generated)
    duplicate = birth(archive, "duplicate", parent=root)
    repeated = proposal(archive, "generated", root)
    candidate(archive, duplicate, repeated)
    states = {}
    for status in ("failed", "interrupted", "running"):
        states[f"p{proposal(archive, status, root)}"] = status
    graph = {node.key: node for node in nodes(archive.db, "/versions/v0001/")}
    assert graph[f"p{rejected}"].state == "rejected"
    assert graph[f"p{generated}"].state == "generated"
    assert graph[f"p{repeated}"].state == "duplicate"
    assert graph[f"b{child}"].parents == graph[f"b{sibling}"].parents == (f"p{generated}",)
    assert graph[f"p{generated}"].parents == (f"b{root}",)
    assert all(graph[key].state == state for key, state in states.items())
    assert graph[f"p{rejected}"].href == f"/versions/v0001/forest.html#proposal-{rejected}"
    assert graph[f"b{child}"].href.endswith(f"history/{child:04d}.html")
    assert not archive.counts()


def test_reused_observations_are_labeled_without_counting_as_simulations(archive):
    child = birth(archive)
    with archive.db:
        archive.db.execute("INSERT INTO reused_evaluations VALUES (?,?,?,?,?,?,?)",
                           (child, "/source", 1, "source-protocol", "signature",
                            json.dumps(result(qualified=True)), "now"))
    node = nodes(archive.db)[0]
    assert node.reused and node.state == "qualified"
    assert "no new trial" in node.detail
    assert not archive.counts()


def test_forest_uses_the_full_qualification_gate(archive):
    child = birth(archive)
    measured = result(qualified=True)
    measured["groundContact"]["forbiddenContactDetected"] = True
    archive.finish(archive.reserve(child), measured)
    assert nodes(archive.db)[0].state == "unqualified"


def test_live_snapshot_tracks_trial_changes_without_republishing_frozen_pages(archive):
    child = birth(archive)
    publish(archive)
    original = (archive.directory / "index.html").read_bytes()
    assert 'data-state="running"' not in snapshot(archive.directory)
    trial = archive.reserve(child)
    assert 'data-state="running"' in snapshot(archive.directory)
    archive.finish(trial, result(qualified=False))
    assert 'data-state="unqualified"' in snapshot(archive.directory)
    assert 'data-state="running"' not in snapshot(archive.directory)
    assert (archive.directory / "index.html").read_bytes() == original


def test_badges_match_displayed_representatives_without_calling_prototypes_champions(archive):
    fast_unqualified = birth(archive)
    qualified = birth(archive)
    slower_qualified = birth(archive)
    prototype, _ = archive.record_birth({**genome(), "pairs": 2}, None, "founder", [], "generated")
    for child, measured in (
        (fast_unqualified, result(qualified=False, speed=100)),
        (qualified, result(qualified=True, speed=.01)),
        (slower_qualified, result(qualified=True, speed=.005)),
        (prototype, result(qualified=False)),
    ):
        archive.finish(archive.reserve(child), measured)
    graph = {node.key: node for node in nodes(archive.db)}
    assert graph[f"b{qualified}"].representative == "champion"
    assert graph[f"b{prototype}"].representative == "prototype"
    assert graph[f"b{fast_unqualified}"].representative is None
    assert graph[f"b{slower_qualified}"].representative is None
    markup = panel(archive.db, "/versions/v0001/", tile_prefix="/")
    assert markup.count('data-champion-link="champion"') == 1
    assert markup.count('data-champion-link="prototype"') == 1
    assert 'href="/index.html#category-four-bar-2-wired-solo"' in markup
    assert 'href="/index.html#category-four-bar-4-wired-solo"' in markup
    historic = panel(archive.db, "/versions/v0001/")
    assert 'href="/versions/v0001/index.html#category-four-bar-2-wired-solo"' in historic


def test_reused_representative_gets_a_badge_without_new_simulation(archive):
    child = birth(archive)
    with archive.db:
        archive.db.execute("INSERT INTO reused_evaluations VALUES (?,?,?,?,?,?,?)",
                           (child, "/source", 1, "source-protocol", "signature",
                            json.dumps(result(qualified=True)), "now"))
    node = nodes(archive.db)[0]
    assert node.reused and node.representative == "champion"
    assert not archive.counts()


def test_merge_connectors_converge_and_arrowheads_point_to_the_child():
    graph = [
        Node("b1", (), "B1", "founder", "pending", "history/0001.html", "root"),
        Node("b2", (), "B2", "founder", "pending", "history/0002.html", "root"),
        Node("b3", ("b1", "b2"), "B3", "future crossover", "pending", "history/0003.html", "shared child"),
    ]
    positions = layout(graph)
    assert positions["b3"][0] > positions["b1"][0]
    assert positions["b3"][0] > positions["b2"][0]
    markup = diagram(graph)
    tree = ET.fromstring(markup)
    namespace = {"s": "http://www.w3.org/2000/svg"}
    connectors = tree.findall(".//s:path[@data-child='b3']", namespace)
    assert len(connectors) == 2
    assert len({" ".join(path.attrib["d"].split()[-2:]) for path in connectors}) == 1
    assert len(tree.findall(".//s:path[@marker-end]", namespace)) == 1
    assert len(tree.findall(".//s:a", namespace)) == 3


@pytest.mark.parametrize("pairs", [1, 2, 3])
def test_birth_leg_count_uses_the_recorded_physical_category(archive, pairs):
    child, _ = archive.record_birth({**genome(), "pairs": pairs}, None, "founder", [], "generated")
    node = nodes(archive.db)[0]
    assert node.key == f"b{child}" and node.legs == pairs * 2
    assert f"{pairs * 2} legs" in node.detail
    assert f">{pairs * 2} legs</text>" in diagram([node])


@pytest.mark.parametrize("feet", [2, 3, 4, 5, 6])
def test_native_births_show_logical_leg_count_including_odd_counts(tmp_path, feet):
    from legowalkers_sim.native_episode import native_protocol
    from test_native_episode import native_fixture

    source, manifest = native_fixture(tmp_path, feet=feet)
    archive = Archive(tmp_path / "native")
    try:
        archive.initialize(native_protocol(protocol(), source, discover=True), 0, 42, False)
        archive.record_native_birth(manifest)
        node = nodes(archive.db)[0]
        assert node.legs == feet
        assert f">{feet} legs</text>" in diagram([node])
        assert archive.counts() == {}
    finally:
        archive.db.close()


def test_proposed_count_is_not_copied_from_parent_and_missing_counts_stay_unknown(archive):
    root = birth(archive)
    attempt = proposal(archive, "generated", root)
    missing = proposal(archive, "running", root)
    with archive.db:
        archive.db.execute("UPDATE native_proposals SET summary=? WHERE id=?",
                           (json.dumps({"operation": "duplicate-branch", "footCount": 4}), attempt))
    graph = {node.key: node for node in nodes(archive.db)}
    assert graph[f"b{root}"].legs == 2
    assert graph[f"p{attempt}"].legs == 4
    assert graph[f"p{missing}"].legs is None
    assert "Proposed leg count: 4" in graph[f"p{attempt}"].detail
    assert ">? legs</text>" in diagram(list(graph.values()))


@pytest.mark.parametrize("count", [-1, True, "4", 3.5])
def test_invalid_proposal_leg_counts_are_explicit_errors(archive, count):
    root = birth(archive)
    attempt = proposal(archive, "rejected", root)
    with archive.db:
        archive.db.execute("UPDATE native_proposals SET summary=? WHERE id=?",
                           (json.dumps({"footCount": count}), attempt))
    with pytest.raises(AdapterError, match="leg count must be"):
        nodes(archive.db)


def test_arrow_labels_have_a_straight_segment_and_are_not_repeated_under_nodes(archive):
    root = birth(archive, operation="founder")
    attempt = proposal(archive, "generated", root, operation="duplicate-branch")
    child = birth(archive, parent=root, operation="duplicate-branch")
    candidate(archive, child, attempt)
    sloping_attempt = proposal(archive, "rejected", root)
    graph = nodes(archive.db)
    tree = ET.fromstring(diagram(graph))
    namespace = {"s": "http://www.w3.org/2000/svg"}
    labels = tree.findall(".//s:text[@class='forest-operation']", namespace)
    assert {label.attrib["data-child"]: label.find("s:title", namespace).text for label in labels} == {
        f"b{child}": "realize", f"p{attempt}": "duplicate-branch", f"p{sloping_attempt}": "move-hinge-site",
    }
    assert not tree.findall(".//s:a/s:text[@class='forest-operation']", namespace)
    positions = layout(graph)
    for label in labels:
        x, y = positions[label.attrib["data-child"]]
        assert x - 180 < float(label.attrib["x"]) < x - 26
        assert float(label.attrib["y"]) == y - 7
    for edge in tree.findall(".//s:path[@marker-end]", namespace):
        start, end = edge.attrib["d"].split(" H")
        left = float(start.removeprefix("M").split()[0])
        assert float(end) - left >= 150
    assert positions[f"p{sloping_attempt}"][1] > positions[f"b{root}"][1]
    assert all(
        positions[parent][0] < positions[node.key][0] - 180
        for node in graph for parent in node.parents
    )


@pytest.mark.parametrize("parents", [("missing",), ("b1",)])
def test_layout_rejects_missing_parents_and_cycles(parents):
    with pytest.raises(AdapterError, match="cycle or missing parent"):
        layout([Node("b1", parents, "B1", "", "pending", "", "")])


def test_static_publication_links_and_diagnostics_are_safe_and_read_only(archive):
    root = birth(archive, operation='<script>alert("x")</script>')
    attempt = proposal(archive, "rejected", root, '<img src=x onerror="alert(1)">')
    before = archive.db.total_changes
    markup = snapshot(archive.directory, details=True)
    assert '<script>alert' not in markup and '<img src=x' not in markup
    assert '&lt;script&gt;' in markup and '&lt;img' in markup
    assert f'id="proposal-{attempt}"' in markup
    assert f'href="history/{root:04d}.html"' in markup
    assert archive.db.total_changes == before
    publish(archive)
    for name in ("index.html", "preview.html", "forest.html"):
        text = (archive.directory / name).read_text()
        assert text.count('id="proposal-forest"') == 1
        assert 'href="forest.html#proposal-' in text
        assert 'function toggleZoom(view, x, y)' in text
        assert 'var(--forest-scale,1)' in text
    assert 'id="proposal-1"' in (archive.directory / "forest.html").read_text()
    assert "parent" in diagnostics(archive.db)


def test_two_level_zoom_keeps_intrinsic_dimensions_and_links():
    node = Node("b1", (), "B1", "founder", "pending", "history/0001.html", "root")
    tree = ET.fromstring(diagram([node]))
    svg = tree.find("{http://www.w3.org/2000/svg}svg")
    assert tree.attrib["tabindex"] == "0"
    assert "toggle zoom" in tree.attrib["aria-label"]
    for dimension in ("width", "height"):
        assert f"{dimension}:calc({svg.attrib[dimension]}px * var(--forest-scale,1))" in svg.attrib["style"]
    assert svg.find("{http://www.w3.org/2000/svg}a").attrib["href"] == node.href


def test_empty_forest_and_replacement_do_not_duplicate_the_panel(archive):
    forest = panel(archive.db)
    assert "No proposals or births" in forest
    content = "before" + forest + "after"
    replacement = insert(content, forest)
    assert replacement == content
    assert insert("before", forest).count('id="proposal-forest"') == 1
    with pytest.raises(AdapterError, match="incomplete"):
        insert("<!-- proposal-forest:start -->", forest)


def test_compact_payload_is_safe_and_labels_contributions_without_rewriting_archive(archive):
    root = birth(archive, operation="founder")
    contributed = birth(archive, parent=root, operation="user-contribution")
    proposal(archive, "rejected", contributed, "</script><img src=x>")
    before = list(archive.db.iterdump())
    markup = panel(archive.db)
    start = '<script type="application/json" class="forest-data">'
    payload = markup.split(start, 1)[1].split("</script>", 1)[0]
    assert "<img" not in payload and "</script>" not in payload
    data = {node["key"]: node for node in json.loads(payload)}
    assert data[f"b{contributed}"]["contribution"] is True
    assert data[f"b{root}"]["contribution"] is False
    assert 'class="forest-full" data-live-key="forest-full"><summary>' in markup
    assert list(archive.db.iterdump()) == before


def test_compact_population_matches_ordinary_retention_without_archive_migrations(tmp_path):
    from legowalkers_sim.native_episode import native_protocol
    from legowalkers_sim.population import members
    from test_native_episode import native_fixture

    source, manifest = native_fixture(tmp_path)
    value = Archive(tmp_path / "population")
    try:
        value.initialize(native_protocol(protocol(), source, discover=True), 1, 42, False)
        child, _ = value.record_native_birth(manifest)
        value.finish(value.reserve(child), result(qualified=True))
        before = list(value.db.iterdump())
        expected = {row["birth"] for row in members(value)}
        actual = {int(node.key[1:]) for node in nodes(value.db) if node.population}
        assert actual == expected == {child}
        assert list(value.db.iterdump()) == before
    finally:
        value.db.close()


def test_contribution_identity_comes_from_saved_provenance_not_genotype_or_parent(archive):
    root = birth(archive, operation="founder")
    identifiers = ["a" * 32, "a" * 32, "b" * 32]
    children = []
    for identifier in identifiers:
        child, _ = archive.record_birth(genome(), root, "user-contribution",
            [{"path": "user-contribution", "before": [], "after": [identifier]}], "generated")
        children.append(child)
    before = list(archive.db.iterdump())
    graph = {node.key: node for node in nodes(archive.db)}
    assert [graph[f"b{child}"].contribution_id for child in children] == identifiers
    assert graph[f"b{root}"].contribution_id is None
    assert list(archive.db.iterdump()) == before


@pytest.mark.parametrize("media", ["assembly-render", "render", "diagram", None])
def test_named_contribution_previews_use_saved_assets_without_rendering_or_rewriting(archive, media):
    identifier, workbench_node = "a" * 32, "b" * 32
    name = 'Outward L </script><img src="x">'
    child, design = archive.record_birth(genome(), None, "user-contribution",
        [{"path": "user-contribution", "before": [], "after": [identifier]}], "generated")
    archive.db.execute("""CREATE TABLE IF NOT EXISTS user_pool_entries(
        contribution TEXT PRIMARY KEY,birth INTEGER,status TEXT NOT NULL,error TEXT,provenance TEXT NOT NULL)""")
    archive.db.execute("INSERT INTO user_pool_entries VALUES (?,?,?,NULL,?)",
                       (identifier, child, "eligible", json.dumps({"label": name, "node": workbench_node})))
    if media:
        relative = f"designs/{design}/candidate/cycle.svg" if media == "diagram" else f"designs/{design}/{media}/walker-v2.png"
        target = archive.directory / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(b"saved-asset")
    before = list(archive.db.iterdump())
    node = nodes(archive.db, "/versions/v0049/")[0]
    assert node.contribution_name == name
    assert node.contribution_href == f"/workbench.html?node={workbench_node}"
    assert node.preview == ("/versions/v0049/" + relative if media else None)
    if media == "diagram":
        assert "not live physics" in node.preview_caption
    elif media:
        assert "not qualification" in node.preview_caption
    payload = panel(archive.db).split('<script type="application/json" class="forest-data">')[1].split("</script>")[0]
    assert "</script>" not in payload and "<img" not in payload
    assert json.loads(payload)[0]["contribution_name"] == name
    assert list(archive.db.iterdump()) == before


@pytest.mark.parametrize("metadata", [{"label": {}}, {"node": "../wrong"}])
def test_invalid_saved_contribution_metadata_is_reported(archive, metadata):
    identifier = "a" * 32
    child, _ = archive.record_birth(genome(), None, "user-contribution",
        [{"path": "user-contribution", "before": [], "after": [identifier]}], "generated")
    archive.db.execute("""CREATE TABLE IF NOT EXISTS user_pool_entries(
        contribution TEXT PRIMARY KEY,birth INTEGER,status TEXT NOT NULL,error TEXT,provenance TEXT NOT NULL)""")
    archive.db.execute("INSERT INTO user_pool_entries VALUES (?,?,?,NULL,?)",
                       (identifier, child, "eligible", json.dumps(metadata)))
    with pytest.raises(AdapterError, match="invalid saved contribution metadata"):
        nodes(archive.db)


@pytest.mark.parametrize("identifiers", [[], ["bad"], ["a" * 32, "b" * 32], [None]])
def test_invalid_contribution_provenance_is_an_explicit_error(archive, identifiers):
    archive.record_birth(genome(), None, "user-contribution",
        [{"path": "user-contribution", "before": [], "after": identifiers}], "generated")
    with pytest.raises(AdapterError, match="invalid contribution provenance"):
        nodes(archive.db)
