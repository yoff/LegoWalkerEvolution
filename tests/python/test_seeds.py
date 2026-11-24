import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from legowalkers_sim import seeds
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.evolution import ROOT
from test_native_episode import native_fixture


def portable(tmp_path):
    source, _ = native_fixture(tmp_path)
    population = tmp_path / "population"
    seed = population / "example"
    seed.mkdir(parents=True)
    hashes = {}
    for name in ("program.json", "realization.json"):
        data = (source / "candidate-0" / name).read_bytes()
        (seed / name).write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    (population / "population.json").write_text(json.dumps({
        "schemaVersion": 1, "seeds": [{"id": "example", "directory": "example", "sha256": hashes}]}))
    return population


def test_recommended_population_is_portable_and_has_no_imported_fitness():
    root = ROOT / "seeds/recommended"
    population = json.loads((root / "population.json").read_text())
    assert len(population["seeds"]) == 8
    for entry in population["seeds"]:
        assert set(entry) == {"id", "label", "directory", "logicalFeet", "notes", "sha256"}
        for name, digest in entry["sha256"].items():
            path = root / entry["directory"] / name
            assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
            assert "/Users/" not in path.read_text()
        genome = json.loads((root / entry["directory"] / "realization.json").read_text())
        assert entry["logicalFeet"] == len(genome["feet"])
    trudge = json.loads((root / "trudge-balanced/realization.json").read_text())
    assert trudge["speed"] < 0


@pytest.mark.parametrize("problem", ["tamper", "selection", "existing"])
def test_materialization_rejects_changed_inputs_unknown_ids_or_existing_output(tmp_path, problem):
    population = portable(tmp_path)
    output = tmp_path / "output"
    selected = None
    if problem == "tamper":
        (population / "example/program.json").write_text("{}")
    elif problem == "selection":
        selected = ["unknown"]
    else:
        output.mkdir()
    with pytest.raises(AdapterError):
        seeds.materialize(population, output, SimpleNamespace(command=[]), selected)


def test_failed_export_never_publishes_completed_seed_descriptor(tmp_path, monkeypatch):
    population = portable(tmp_path)
    monkeypatch.setattr(seeds.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(
        returncode=1, stderr="no physical realization", stdout=""))
    with pytest.raises(AdapterError, match="no physical realization"):
        seeds.materialize(population, tmp_path / "output", SimpleNamespace(command=[]))
    assert not (tmp_path / "output/seeds.json").exists()
