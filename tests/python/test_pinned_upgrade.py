import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from legowalkers_sim import screening, upgrades
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.evolution import Archive
from legowalkers_sim.native_episode import native_protocol, seed_batches, stage_native
from legowalkers_sim.pinned import verify_snapshot
from test_evolution import protocol, result
from test_native_episode import native_fixture


def test_snapshot_requires_exact_source_and_compiler_bytes(tmp_path):
    source = tmp_path / "protocol-source/simulator/legowalkers_sim/evaluation.py"
    binary = tmp_path / "protocol-source/generator/compiler.dll"
    source.parent.mkdir(parents=True)
    binary.parent.mkdir(parents=True)
    source.write_text("physics")
    binary.write_bytes(b"compiler")
    (tmp_path / "protocol.json").write_text(json.dumps({
        "sourceSha256": {"simulator/legowalkers_sim/evaluation.py": hashlib.sha256(source.read_bytes()).hexdigest()},
        "generatorBinarySha256": {"compiler.dll": hashlib.sha256(binary.read_bytes()).hexdigest()}}))
    assert verify_snapshot(tmp_path) == tmp_path / "protocol-source"
    definition = json.loads((tmp_path / "protocol.json").read_text())
    definition["payloadProfile"] = {"fixture": "original"}
    (tmp_path / "protocol.json").write_text(json.dumps(definition))
    profile = tmp_path / "protocol-source/selected-payloads.json"
    profile.write_text(json.dumps(definition["payloadProfile"]))
    assert verify_snapshot(tmp_path)
    profile.write_text('{"fixture":"changed"}')
    with pytest.raises(ValueError, match="payload profile"):
        verify_snapshot(tmp_path)
    profile.write_text(json.dumps(definition["payloadProfile"]))
    source.write_text("different")
    with pytest.raises(ValueError, match="source changed"):
        verify_snapshot(tmp_path)


def test_screening_cache_versions_preserve_previous_reports(tmp_path, monkeypatch):
    monkeypatch.setattr(screening, "_protocol", lambda loaded: {"implementation": "v1", "loaded": loaded})
    old = screening.cache_directory(tmp_path)
    old.mkdir()
    report = old / "screen-0001.json"
    report.write_text("original evidence")
    monkeypatch.setattr(screening, "_protocol", lambda loaded: {"implementation": "v2", "loaded": loaded})
    assert screening.cache_directory(tmp_path) != old
    assert report.read_text() == "original evidence"


@pytest.mark.parametrize("filename", ["joint_tree.py", "spatial_gears.py", "stance_support.py"])
def test_joint_tree_changes_invalidate_scientific_and_screening_fingerprints(tmp_path, monkeypatch, filename):
    from legowalkers_sim import evolution
    from legowalkers_sim.evaluation import EvaluationOptions
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        (tmp_path / name).write_bytes(b"fixture")
    from legowalkers_sim.motion_carry import POLICY
    generator = SimpleNamespace(command=["dotnet", str(tmp_path / "LegoWalkers.Cli.dll")],
                                invoke=lambda command: json.dumps(POLICY) if command == "construction-policy" else "3001.dat")
    monkeypatch.setattr(evolution, "library_fingerprint", lambda *args: "fixture")
    original = evolution.protocol_definition(generator, tmp_path, EvaluationOptions(), {})
    original_screen = screening._protocol(False)
    key = f"simulator/legowalkers_sim/{filename}"
    assert key in original["sourceSha256"]
    read_bytes = Path.read_bytes

    def changed(path):
        contents = read_bytes(path)
        return contents+b"\n# fixture change" if path.name == filename else contents

    monkeypatch.setattr(Path, "read_bytes", changed)
    updated = evolution.protocol_definition(generator, tmp_path, EvaluationOptions(), {})
    assert updated["sourceSha256"][key] != original["sourceSha256"][key]
    assert screening._protocol(False)["sourceSha256"] != original_screen["sourceSha256"]


@pytest.mark.parametrize("compiled", ['{"theoryVersion":"old","packagingVersion":"old"}', 'not-json', None])
def test_runtime_preparation_rejects_stale_or_invalid_compiler_policy_before_reading_inputs(tmp_path, compiled):
    from legowalkers_sim.evolution import protocol_definition
    from legowalkers_sim.evaluation import EvaluationOptions

    def invoke(command):
        assert command == "construction-policy"
        if compiled is None:
            raise AdapterError("Unknown command: construction-policy")
        return compiled

    with pytest.raises(AdapterError, match="rebuild the CLI"):
        protocol_definition(SimpleNamespace(invoke=invoke), tmp_path, EvaluationOptions(), {})

def test_upgrade_preserves_source_authorization_and_records_cross_archive_parents(tmp_path):
    source, data = native_fixture(tmp_path)
    directory = tmp_path / "campaign"
    archive = Archive(directory)
    archive.initialize(native_protocol(protocol(), source, discover=True), 4, 42, False)
    stage_native(archive, source)
    birth = archive.db.execute("SELECT id FROM births").fetchone()[0]
    archive.finish(archive.reserve(birth), result())
    before = (archive.db.execute("SELECT * FROM run").fetchall(), archive.db.execute("SELECT * FROM evaluations").fetchall())
    target, seeds = upgrades.prepare(directory)
    provenance = json.loads((target / "upgrade.json").read_text())
    assert provenance["parents"][0]["sourceBirth"] == birth
    assert provenance["sourceArchive"] == str(directory)
    assert provenance["policy"] == "gait-cycles-v1"
    assert provenance["mutationVersion"] == "native-motion-mutations-v23"
    assert "Zero trials authorized" in provenance["note"]
    assert not (target / "archive.sqlite3").exists()
    assert len(seed_batches(seeds)) == 1
    assert (archive.db.execute("SELECT * FROM run").fetchall(), archive.db.execute("SELECT * FROM evaluations").fetchall()) == before
    second, _ = upgrades.prepare(directory)
    assert second != target
    archive.db.close()
