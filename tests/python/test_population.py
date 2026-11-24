import copy
import json

import pytest

from legowalkers_sim import discovery, native_episode, population, upgrades
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import Archive, fitness
from test_evolution import protocol, result
from test_native_discovery import Proposals, setup_discovery
from test_native_episode import native_fixture


@pytest.fixture
def parent(tmp_path):
    _, manifest = native_fixture(tmp_path)

    def row(birth, *, family=1, qualified=True, speed=1, category="3-feet", layout=0):
        genome = copy.deepcopy(manifest["genome"])
        genome["speed"] = birth
        motor = next(part for body in genome["bodies"] for part in body["parts"] if part["id"] == "motor")
        motor["position"]["x"] += layout
        return {"birth": birth, "design": f"design-{birth}", "family": family, "category": category,
                "genome": json.dumps(genome), "result": json.dumps(result(qualified=qualified, speed=speed))}

    return row


def test_capacity_deduplication_and_diversity_do_not_just_keep_the_top_scores(parent):
    clones = [parent(i, speed=i) for i in range(1, 31)]
    novel = parent(31, speed=.01, layout=200)
    second_family = parent(32, speed=.001, family=2)
    other_category = parent(33, speed=.0001, category="4-feet")
    rows = clones + [novel, second_family, other_category, clones[-1]]
    retained = population.retain(rows)
    assert len(retained) == 12
    assert len({row["design"] for row in retained}) == 12
    assert {30, 31, 32, 33} <= {row["birth"] for row in retained}
    assert retained == population.retain(rows)


def test_promising_prototype_survives_a_population_of_qualified_walkers(parent):
    rows = [parent(i, speed=i) for i in range(1, 20)]
    prototype = parent(20, qualified=False)
    assert prototype in population.retain(rows + [prototype])
    assert rows[-1] in population.retain(rows + [prototype])


def test_parent_selection_explores_non_elites_and_is_restart_deterministic(parent):
    rows = [parent(i, speed=i) for i in range(1, 5)]
    selections = [population.choose(rows, 42, index) for index in range(200)]
    assert {row["birth"] for row, _ in selections} == {1, 2, 3, 4}
    assert {decision["mode"] for _, decision in selections} == {"random", "tournament"}
    for index, (row, decision) in enumerate(selections):
        assert population.choose(rows, 42, index) == (row, decision)
        assert decision["selectedBirth"] == row["birth"]
        assert decision["populationBirths"] == [1, 2, 3, 4]
        assert row["birth"] in decision["contestants"]
        if decision["mode"] == "tournament":
            contestants = [p for p in rows if p["birth"] in decision["contestants"]]
            assert len(contestants) == 2
            assert fitness(json.loads(row["result"])) == max(fitness(json.loads(p["result"])) for p in contestants)
        else:
            assert len(decision["contestants"]) == 1


def test_selection_errors_are_explicit(parent):
    with pytest.raises(AdapterError, match="no measured population parent"):
        population.choose([], 42, 0)
    with pytest.raises(AdapterError, match="cannot preserve"):
        population.retain([parent(i, family=i) for i in range(13)])


@pytest.mark.parametrize("count", [0, 13])
def test_seed_batch_bound_matches_population_capacity(tmp_path, count):
    (tmp_path / "seeds.json").write_text(json.dumps(
        {"schemaVersion": 1, "batches": [f"seed-{i}" for i in range(count)]}))
    with pytest.raises(AdapterError, match="one to 12 unique seed-batch"):
        native_episode.seed_batches(tmp_path)


def test_population_and_decisions_follow_completed_results_without_extra_trials(tmp_path, monkeypatch):
    source, _, archive, calls = setup_discovery(tmp_path, monkeypatch, budget=6)
    try:
        discovery.search_discovery(archive, Proposals(), source, tmp_path, EvaluationOptions())
        assert len(calls) == 6 and len(population.members(archive)) == 6
        for proposal in archive.db.execute("SELECT * FROM native_proposals"):
            decision = json.loads(proposal["summary"])["parentSelection"]
            assert proposal["parent"] == decision["selectedBirth"]
            assert proposal["parent"] in decision["contestants"]
            completed = {row[0] for row in archive.db.execute(
                "SELECT birth FROM evaluations WHERE status='complete' AND completed<=?", (proposal["created"],))}
            assert set(decision["populationBirths"]) == completed
        page = (archive.directory / "index.html").read_text()
        assert "Parent population" in page and "6 / 12 measured parents" in page
        assert "20% of selections" in page
        assert archive.counts() == {"complete": 6}
    finally:
        archive.db.close()


def test_population_upgrades_keep_multiple_parents_and_their_original_family(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=3)
    try:
        discovery.search_discovery(archive, Proposals(), source, tmp_path, EvaluationOptions())
        before = archive.db.execute("SELECT * FROM evaluations").fetchall()
        target, seeds = upgrades.prepare(archive.directory)
        metadata = json.loads((target / "upgrade.json").read_text())
        assert len(metadata["parents"]) == len(native_episode.seed_batches(seeds)) == 3
        assert {tuple(parent["founder"]) for parent in metadata["parents"]} == {(str(archive.directory), 1)}
        assert {parent["sourceBirth"] for parent in metadata["parents"]} == {1, 2, 3}
        assert archive.db.execute("SELECT * FROM evaluations").fetchall() == before
        upgraded = Archive(target)
        try:
            upgraded.initialize(native_episode.native_protocol(protocol(), seeds, discover=True), 3, 42, False)
            native_episode.stage_native(upgraded, seeds)
            groups = population.family_map(upgraded)
            assert groups == {1: 1, 2: 1, 3: 1}
            assert discovery._neighborhood(upgraded) == 1
            for birth in (1, 2, 3):
                upgraded.finish(upgraded.reserve(birth), result(qualified=True))
            assert len(population.members(upgraded)) == 3
            assert {row["family"] for row in population.members(upgraded)} == {1}
            again, _ = upgrades.prepare(target)
            parents = json.loads((again / "upgrade.json").read_text())["parents"]
            assert len(parents) == 3 and all(parent["founder"] == [str(archive.directory), 1] for parent in parents)
        finally:
            upgraded.db.close()
    finally:
        archive.db.close()


def test_first_population_upgrade_seeds_the_current_elite_without_discarding_other_data(tmp_path):
    source, _ = native_fixture(tmp_path)
    definition = copy.deepcopy(native_episode.native_protocol(protocol(), source, discover=True))
    del definition["nativeDiscovery"]["population"]
    definition["nativeDiscovery"]["version"] = "native-discovery-v5"
    archive = Archive(tmp_path / "legacy")
    try:
        archive.initialize(definition, 3, 42, False)
        native_episode.stage_native(archive, source)
        parent = archive.directory / "designs" / definition["nativeCandidates"][0] / "candidate"
        batch = archive.directory / "offspring"
        summary = Proposals(siblings=2).propose_native(parent, batch, 42, 0, 2)
        _, files = native_episode._queue(batch)
        for entry in summary["generated"]:
            manifest, _ = native_episode._copy_candidate(archive, batch, entry, files)
            archive.record_native_birth(manifest, parent=1, native={
                "assembly": entry["assemblyId"], "batch": "offspring",
                "directory": entry["directory"], "files": files})
        for birth, speed in ((1, 1), (2, 3), (3, 2)):
            archive.finish(archive.reserve(birth), result(qualified=True, speed=speed))
        before = archive.db.execute("SELECT * FROM evaluations").fetchall()
        original_protocol = (archive.directory / "protocol.json").read_bytes()
        target, seeds = upgrades.prepare(archive.directory)
        parents = json.loads((target / "upgrade.json").read_text())["parents"]
        assert len(parents) == len(native_episode.seed_batches(seeds)) == 1
        assert parents[0]["sourceBirth"] == 2
        assert archive.db.execute("SELECT * FROM evaluations").fetchall() == before
        assert (archive.directory / "protocol.json").read_bytes() == original_protocol
        assert archive.db.execute("SELECT count(*) FROM births").fetchone()[0] == 3
    finally:
        archive.db.close()


def test_running_and_failed_trials_are_not_population_parents(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch, budget=3)
    try:
        native_episode.stage_native(archive, source)
        archive.finish(archive.reserve(1), result(qualified=True))
        discovery._propose(archive, Proposals(siblings=2), 1, 1)
        archive.reserve(2)
        archive.finish(archive.reserve(3), error="fixture failure", status="failed")
        assert [row["birth"] for row in population.members(archive)] == [1]
    finally:
        archive.db.close()
