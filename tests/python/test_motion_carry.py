import json
import sqlite3
from pathlib import Path
from types import SimpleNamespace

import pytest

from legowalkers_sim import motion_carry, native_episode
from legowalkers_sim.bridge import AdapterError, load_manifest
from legowalkers_sim.evolution import Generator
from test_native_discovery import Proposals, setup_discovery
from test_native_episode import native_fixture


def pending(archive):
    return archive.db.execute(
        "SELECT c.*,b.design FROM native_candidates c JOIN births b ON b.id=c.birth WHERE c.birth=1").fetchone()


def test_same_genotype_construction_binds_new_proof_without_mutating_original_inputs(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    try:
        native_episode.stage_native(archive, source)
        original = dict(pending(archive))
        constructed = motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path)
        assert dict(pending(archive)) == original
        assert constructed["birth"] == 1 and constructed["design"] == original["design"]
        assert constructed["batch"] != original["batch"]
        assert archive.counts() == {}
        checked = SimpleNamespace(manifest=load_manifest(
            archive.directory / constructed["batch"] / constructed["directory"] / "manifest.json"))
        motion_carry.require_construction(archive, 1, checked)
        assert motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path) == constructed
        with pytest.raises(sqlite3.IntegrityError, match="immutable"):
            archive.db.execute("UPDATE motion_candidates SET batch='stale' WHERE birth=1")
        proof = archive.directory / constructed["batch"] / "motion-carry-forward.json"
        proof.write_text("{}")
        with pytest.raises(AdapterError, match="changed after"):
            motion_carry.require_construction(archive, 1, checked)
    finally:
        archive.db.close()


def test_unbuilt_motion_cannot_reserve_a_trial(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    try:
        native_episode.stage_native(archive, source)
        checked = SimpleNamespace(manifest=load_manifest(source / "candidate-0/manifest.json"))
        with pytest.raises(AdapterError, match="inherited hardware"):
            motion_carry.require_construction(archive, 1, checked)
        assert archive.counts() == {}
    finally:
        archive.db.close()


def test_construction_rejection_keeps_motion_source_without_spending_budget(tmp_path, monkeypatch):
    source, _, archive, _ = setup_discovery(tmp_path, monkeypatch)
    try:
        native_episode.stage_native(archive, source)
        original = dict(pending(archive))
        monkeypatch.setattr(Proposals, "build_native", lambda *args, **kwargs: {
            "generated": [], "rejected": [{"code": "no-route"}], "gates": []})
        assert motion_carry.ensure_pending(archive, pending(archive), Proposals(), tmp_path) is None
        assert dict(pending(archive)) == original and archive.counts() == {}
        assert archive.db.execute("SELECT status FROM births WHERE id=1").fetchone()[0] == "construction-rejected"
        result = json.loads((archive.directory / "motion-builds/000001/result.json").read_text())
        assert len(result["attempts"]) == 2 and result["dynamicTrials"] == 0
    finally:
        archive.db.close()


@pytest.mark.parametrize("change", ["reference", "clock", "speed", "proof"])
def test_generator_rejects_changed_intent_control_or_missing_construction_proof(tmp_path, monkeypatch, change):
    source, _ = native_fixture(tmp_path)
    parent = source / "candidate-0"
    output = tmp_path / "build"
    program = {"id": "retained-motion"}
    (parent / "program.json").write_text(json.dumps({"program": program}))

    def run(*args, **kwargs):
        summary = Proposals().build_native(parent, output)
        summary.update(parentGenotypeId=load_manifest(parent / "manifest.json")["motion"]["genotypeId"],
                       counts={"exported": 1, "dynamicTrials": 0},
                       motionFirst={"packagingVersion": motion_carry.POLICY["packagingVersion"],
                                    "theoryVersion": motion_carry.POLICY["theoryVersion"]})
        summary["rejected"] = []
        (output / "candidates.json").write_text(json.dumps(summary))
        request = {"reference": {} if change == "reference" else program,
                   "drive": {"motorClockScale": 5 if change == "clock" else 1},
                   "groundClearance": {"minimumNonFootClearanceLdu": 8}}
        (output / "request.json").write_text(json.dumps(request))
        candidate = output / "candidate-001"
        for name in ("ground-plan.json", "construction-proof.json"):
            if change != "proof":
                (candidate / name).write_text("{}")
        if change == "speed":
            genome = json.loads((candidate / "realization.json").read_text())
            genome["speed"] = -genome["speed"]
            (candidate / "realization.json").write_text(json.dumps(genome))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr("legowalkers_sim.evolution.subprocess.run", run)
    with pytest.raises(AdapterError, match="reference intent|electrical control|construction proof"):
        Generator(Path("dotnet"), Path("generator.dll")).build_native(parent, output)
