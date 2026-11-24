import hashlib
import ast
import json
from pathlib import Path
import shutil
import sqlite3
from types import SimpleNamespace

import pytest

from legowalkers_sim import discovery, native_episode
from legowalkers_sim import observations
from legowalkers_sim.bridge import AdapterError, library_fingerprint, load_manifest, write_json
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import Archive
from legowalkers_sim.gallery import publish
from legowalkers_sim.observations import PHYSICS_SOURCES, evaluation_signature, reuse_observations
from test_evolution import protocol, result
from test_native_discovery import Proposals, mock_constructor
from test_native_episode import native_fixture


def physics_protocol():
    return {**protocol(), "simulatorRevision": "pinned", "partsDependencySha256": "same-library",
            "versions": {"mujoco": "3.2.7"}, "evaluation": {"duration": 20}, "controller": "same-controller",
            "sourceSha256": {f"simulator/legowalkers_sim/{name}": name for name in PHYSICS_SOURCES}}

def measured_compatibility(library, manifest):
    for directory in ("parts", "p"):
        (library / directory).mkdir(exist_ok=True)
    primitive = library / "p/observation-primitive.dat"
    if not primitive.exists():
        primitive.write_text("0 observation primitive\n")
    for part in manifest["parts"]:
        path = library / "parts" / part["part"]
        if not path.exists():
            path.write_text("0 observation part\n1 16 0 0 0 1 0 0 0 1 0 0 0 1 observation-primitive.dat\n")
    return {"ldrawSha256": manifest["ldrawSha256"],
            "partsDependencySha256": library_fingerprint(library, [part["part"] for part in manifest["parts"]])}


def measured_founder(tmp_path, *, base=None):
    source, data = native_fixture(tmp_path)
    archive = Archive(tmp_path / "old-archive")
    archive.initialize(native_episode.native_protocol(physics_protocol() if base is None else base, source), 1, 42, False)
    native_episode.stage_native(archive, source)
    birth = archive.db.execute("SELECT id FROM births").fetchone()[0]
    trial = archive.reserve(birth)
    measured = result(qualified=True)
    measured["compatibility"] = measured_compatibility(tmp_path, data)
    archive.finish(trial, measured)
    write_json(archive.directory / "evaluations/0001/result.json", measured)
    directory = archive.directory
    archive.db.close()
    return source, directory


def test_search_changes_preserve_evaluation_identity_but_physics_changes_do_not():
    old = physics_protocol()
    changed = {**old, "nativeDiscovery": {"new": "operators"},
               "sourceSha256": {**old["sourceSha256"], "src/LegoWalkers.Core/WalkerEvolution.fs": "different"}}
    assert evaluation_signature(old) == evaluation_signature(changed)
    assert evaluation_signature(old) == evaluation_signature({**old, "partsDependencySha256": "catalogue-with-L"})
    assert evaluation_signature(old) == evaluation_signature({**old, "evaluation": {**old["evaluation"], "short_gate": 2}})
    assert evaluation_signature(old) != evaluation_signature({**old, "evaluation": {"duration": 21}})
    with pytest.raises(AdapterError, match="complete evaluation protocol"):
        evaluation_signature(protocol())


def test_floor_contact_settings_and_timestep_have_distinct_evaluation_identities():
    from dataclasses import asdict, replace
    options = EvaluationOptions(timestep=.0005, floor_contact_time_constant=.002)
    baseline = {**physics_protocol(), "evaluation": asdict(options)}
    for changed in (replace(options, floor_contact_time_constant=.004),
                    replace(options, timestep=.00025),
                    replace(options, floor_contact_time_constant=None)):
        assert evaluation_signature(baseline) != evaluation_signature({
            **baseline, "evaluation": asdict(changed)})


@pytest.mark.parametrize("change", ["none", "physics", "cache", "guard", "cleanup", "default",
                                  "source-snapshot", "target-snapshot"])
@pytest.mark.parametrize("target_mode", ["workspace", "frozen"])
def test_optional_parser_cache_proof_is_exact_and_preserves_snapshot_verification(tmp_path, monkeypatch, change, target_mode):
    current = (Path(observations.__file__).with_name("bridge.py")).read_text()
    legacy_check = ast.parse('''
def check(path: Path, library: Path) -> CheckedModel:
    path, library = path.resolve(), library.expanduser().resolve()
    manifest = load_manifest(path)
    fingerprint = library_fingerprint(library, [part["part"] for part in manifest["parts"]])
    distribution = importlib.metadata.distribution("lego-technic-sim")
    direct = distribution.read_text("direct_url.json")
    source = json.loads(direct) if direct else {}
    installed_revision = source.get("vcs_info", {}).get("commit_id")
    if installed_revision and installed_revision != SIMULATOR_REVISION:
        raise AdapterError(f"Installed simulator revision {installed_revision} differs from {SIMULATOR_REVISION}.")
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        build = LDrawParser(parts_dir=library).parse_build(path.parent / manifest["modelFile"])
        scene = build_units_and_joints(build)
    if caught:
        raise AdapterError("Simulator warnings: " + "; ".join(str(item.message) for item in caught))
    bodies, joints = compare_scene(manifest, build, scene)
    report = {
        "schemaVersion": 1, "status": "compatible", "modelKind": manifest["modelKind"],
        "ldrawSha256": manifest["ldrawSha256"], "partsDependencySha256": fingerprint,
        "simulatorRevision": installed_revision,
        "expectedSimulatorRevision": SIMULATOR_REVISION,
        "parts": len(build.parts), "bodies": len(scene.units), "joints": len(scene.joints),
        "gearMeshes": len(scene.gears), "motors": len(scene.motors),
        "bodyIndices": bodies, "jointIndices": joints,
        "warnings": manifest["warnings"] + [
            "Upstream compatibility units exclude connectors; evaluation reattaches their geometry/mass.",
            *([] if installed_revision else ["Installed simulator revision cannot be verified from package metadata."]),
        ],
    }
    return CheckedModel(manifest, scene, bodies, joints, report, build)
''').body[0]
    legacy_tree = ast.parse(current)
    legacy_tree.body = [
        legacy_check if isinstance(node, ast.FunctionDef) and node.name == "check" else node
        for node in legacy_tree.body
        if not (isinstance(node, ast.ClassDef) and node.name == "PartParserCache")
    ]
    legacy = ast.unparse(legacy_tree)
    replacements = {
        "physics": ("bodies, joints = compare_scene(manifest, build, scene)",
                    "bodies, joints = different_scene(manifest, build, scene)"),
        "cache": ("self._dependencies.update(dependencies)", "self._dependencies.clear()"),
        "guard": ("if parser_cache is None else", "if True else"),
        "cleanup": ("if not parsed and parser_cache is not None:", "if False:"),
        "default": ("parser_cache: PartParserCache | None = None",
                    "parser_cache: PartParserCache | None = PartParserCache()"),
    }
    if change in replacements:
        old, new = replacements[change]
        assert old in current
        current = current.replace(old, new)
    original, target = physics_protocol(), physics_protocol()
    name = "simulator/legowalkers_sim/bridge.py"
    source, target_archive = tmp_path / "old", tmp_path / "current"
    target_root = target_archive / "protocol-source" if target_mode == "frozen" else target_archive
    old_path, new_path = source / "protocol-source" / name, target_root / name
    old_path.parent.mkdir(parents=True)
    new_path.parent.mkdir(parents=True)
    old_path.write_text(legacy + ("\n# altered frozen source\n" if change == "source-snapshot" else ""))
    new_path.write_text(current + ("\n# altered target source\n" if change == "target-snapshot" else ""))
    original["sourceSha256"][name] = hashlib.sha256(legacy.encode()).hexdigest()
    target["sourceSha256"][name] = hashlib.sha256(current.encode()).hexdigest()
    monkeypatch.setattr(observations, "ROOT", tmp_path / "unrelated-working-tree" if target_mode == "frozen" else target_root)
    options = {"target_source": target_archive if target_mode == "frozen" else None}
    if change.endswith("-snapshot"):
        with pytest.raises(AdapterError, match="Physics-source snapshot changed: .*bridge.py"):
            observations.compatible_evaluation(original, target, source, **options)
    else:
        assert observations.compatible_evaluation(original, target, source, **options) is (change == "none")


@pytest.mark.parametrize("change", ["none", "physics", "guard", "schema", "opted-in", "snapshot"])
def test_support_upgrade_proof_applies_only_to_identical_legacy_manifests(tmp_path, monkeypatch, change):
    current = Path(observations.__file__).parent
    original = physics_protocol()
    target = physics_protocol()
    source = tmp_path / "old"
    target_root = tmp_path / "new"
    for filename in ("ground.py", "schema.py"):
        name = "simulator/legowalkers_sim/" + filename
        text = (current / filename).read_text()
        old = ast.unparse(observations._WithoutOptionalSupport().visit(ast.parse(text)))
        if change == "physics" and filename == "ground.py":
            text = text.replace("if distance > 0:", "if distance > .001:")
        if change == "guard" and filename == "ground.py":
            text = text.replace('if any("contactShape" in foot for foot in manifest.get("motion", {}).get("logicalFeet", [])):', "if True:")
        if change == "schema" and filename == "schema.py":
            text = text.replace('"maximum": .004', '"maximum": .005')
        path = source / "protocol-source" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(old)
        path = target_root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text + ("\n# changed after freezing\n" if change == "snapshot" else ""))
        original["sourceSha256"][name] = hashlib.sha256(old.encode()).hexdigest()
        target["sourceSha256"][name] = hashlib.sha256(text.encode()).hexdigest()
    name = "simulator/legowalkers_sim/support_surfaces.py"
    text = (current / "support_surfaces.py").read_bytes()
    (target_root / name).write_bytes(text)
    target["sourceSha256"][name] = hashlib.sha256(text).hexdigest()
    monkeypatch.setattr(observations, "ROOT", target_root)
    manifest = {"motion": {"logicalFeet": [{"contactShape": "rectangle"} if change == "opted-in" else {}]}}
    if change == "snapshot":
        with pytest.raises(AdapterError, match="snapshot changed"):
            observations.compatible_evaluation(original, target, source, manifest=manifest)
    else:
        assert observations.compatible_evaluation(original, target, source, manifest=manifest) is (change == "none")
    assert not observations.compatible_evaluation(original, target, source)


@pytest.mark.parametrize("change", ["none", "physics", "observer", "snapshot", "partial"])
def test_gate_only_upgrade_reuses_full_measurements_but_requires_exact_physics_proof(tmp_path, monkeypatch, change):
    legacy = """def evaluate(output, options):
    settle_steps = 1000
    steps = 20000
    for step in range(steps):
        physics_step()
    return score()
"""
    gated = """from .simulation_gate import SimulationPrefixGate, validate_short_gate
from copy import deepcopy
def evaluate(output, options):
    settle_steps = 1000
    steps = 20000
    gate = SimulationPrefixGate(output, options.short_gate, options.timestep, settle_steps, steps)
    if gate.active:
        gate.observe(0, float(data.time), max_tilt, max_closure, deepcopy(penetrations), deepcopy(ground_contacts.report()), frames)
    for step in range(steps):
        physics_step()
        if gate.active:
            gate.observe(step + 1, float(data.time), max_tilt, max_closure, deepcopy(penetrations), deepcopy(ground_contacts.report()), frames)
    if options.short_gate:
        result["simulationGate"] = gate.report
    return score()
"""
    if change == "physics":
        gated = gated.replace("physics_step()", "different_physics()")
    elif change == "observer":
        gated = gated.replace("gate.observe(step + 1,", "gate.observe(step + 2,")
    name = "simulator/legowalkers_sim/evaluation.py"
    old_protocol = physics_protocol()
    old_protocol["sourceSha256"][name] = hashlib.sha256(legacy.encode()).hexdigest()
    source, old = measured_founder(tmp_path, base=old_protocol)
    snapshot = old / "protocol-source" / name
    snapshot.parent.mkdir(parents=True)
    snapshot.write_text(legacy + ("\n# changed after freezing\n" if change == "snapshot" else ""))
    current = tmp_path / "new-source"
    path = current / name
    path.parent.mkdir(parents=True)
    path.write_text(gated)
    monkeypatch.setattr(observations, "ROOT", current)
    target = {**old_protocol,
              "evaluation": {**old_protocol["evaluation"], "short_gate": 2},
              "sourceSha256": {**old_protocol["sourceSha256"], name: hashlib.sha256(gated.encode()).hexdigest()}}
    if change == "partial":
        measured = json.loads((old / "evaluations/0001/result.json").read_text())
        measured["simulationGate"] = {"status": "rejected"}
        write_json(old / "evaluations/0001/result.json", measured)
        with sqlite3.connect(old / "archive.sqlite3") as db:
            db.execute("UPDATE evaluations SET result=?", (json.dumps(measured),))
    before = (old / "evaluations/0001/result.json").read_bytes()
    archive = Archive(tmp_path / "gated-archive")
    archive.initialize(native_episode.native_protocol(target, source, discover=True, reuse_archives=[old]), 0, 42, False)
    try:
        native_episode.stage_native(archive, source)
        if change == "none":
            reuse_observations(archive, tmp_path)
            assert archive.best()["reused"] == 1
            assert discovery._pending(archive) is None
            assert json.loads((archive.directory / "observations/0001/result.json").read_text()) == json.loads(before)
        else:
            message = ("snapshot changed" if change == "snapshot" else
                       "not a complete reusable measurement" if change == "partial" else "different physics")
            with pytest.raises(AdapterError, match=message):
                reuse_observations(archive, tmp_path)
            assert not archive.outcomes()
        assert archive.counts() == {}
        assert (old / "evaluations/0001/result.json").read_bytes() == before
    finally:
        archive.db.close()


def test_existing_measurement_is_visible_and_selectable_without_spending_a_new_trial(tmp_path):
    source, old = measured_founder(tmp_path)
    archive = Archive(tmp_path / "new-archive")
    archive.initialize(native_episode.native_protocol(physics_protocol(), source, discover=True, reuse_archives=[old]), 4, 42, False)
    try:
        native_episode.stage_native(archive, source)
        reuse_observations(archive, tmp_path)
        assert archive.counts() == {} and archive.best()["reused"] == 1
        assert discovery._pending(archive) is None
        assert archive.db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == 1
        reuse_observations(archive, tmp_path)
        assert archive.db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == 1
        publish(archive)
        page = (archive.directory / "index.html").read_text()
        assert "reused observations / zero new trials" in page
        assert "observations/0001/provenance.json" in page
        assert "Reused observation (no new trial)" in (archive.directory / "history/0001.html").read_text()
        assert (archive.directory / "observations/0001/source-protocol.json").exists()
    finally:
        archive.db.close()

def test_unused_catalogue_extension_reuses_the_original_score_with_used_part_provenance(tmp_path, monkeypatch):
    source, old = measured_founder(tmp_path)
    original_result = (old / "evaluations/0001/result.json").read_bytes()
    (tmp_path / "parts/32526.dat").write_text("0 newly available, unused molded L\n")
    base = {**physics_protocol(), "partsDependencySha256": "catalogue-with-L"}
    archive = Archive(tmp_path / "new-archive")
    archive.initialize(native_episode.native_protocol(base, source, discover=True, reuse_archives=[old]), 0, 42, False)
    try:
        native_episode.stage_native(archive, source)
        monkeypatch.chdir(tmp_path)
        reuse_observations(archive, Path("."))
        assert archive.counts() == {} and archive.best()["reused"] == 1
        assert discovery._pending(archive) is None
        assert json.loads((archive.directory / "observations/0001/result.json").read_text()) == json.loads(original_result)
        provenance = json.loads((archive.directory / "observations/0001/provenance.json").read_text())
        measured = json.loads(original_result)
        assert provenance["partsDependencySha256"] == measured["compatibility"]["partsDependencySha256"]
        assert provenance["newDynamicTrials"] == 0
        assert (old / "evaluations/0001/result.json").read_bytes() == original_result
    finally:
        archive.db.close()


@pytest.mark.parametrize("change", ["part", "subpart", "missing-proof", "missing-file"])
def test_used_part_dependencies_must_match_even_when_catalogue_fingerprint_is_unchanged(tmp_path, change):
    source, old = measured_founder(tmp_path)
    if change in ("part", "missing-file"):
        manifest = load_manifest(source / "candidate-0/manifest.json")
        path = tmp_path / "parts" / manifest["parts"][0]["part"]
        if change == "part":
            path.write_text(path.read_text() + "0 changed used part\n")
        else:
            path.unlink()
    elif change == "subpart":
        (tmp_path / "p/observation-primitive.dat").write_text("0 changed recursive dependency\n")
    else:
        path = old / "evaluations/0001/result.json"
        measured = json.loads(path.read_text())
        del measured["compatibility"]["partsDependencySha256"]
        write_json(path, measured)
        with sqlite3.connect(old / "archive.sqlite3") as db:
            db.execute("UPDATE evaluations SET result=?", (json.dumps(measured),))
    archive = Archive(tmp_path / "new-archive")
    archive.initialize(native_episode.native_protocol(physics_protocol(), source, discover=True, reuse_archives=[old]), 0, 42, False)
    try:
        native_episode.stage_native(archive, source)
        with pytest.raises(AdapterError, match="recursive part dependencies|Missing LDraw dependency"):
            reuse_observations(archive, tmp_path)
        assert archive.counts() == {} and not archive.outcomes()
        assert not (archive.directory / "observations/0001/provenance.json").exists()
    finally:
        archive.db.close()


@pytest.mark.parametrize("change", ["physics", "manifest"])
def test_incompatible_observations_are_rejected_before_new_trials(tmp_path, change):
    source, old = measured_founder(tmp_path)
    base = physics_protocol()
    if change == "physics":
        base["evaluation"]["duration"] = 21
    archive = Archive(tmp_path / "new-archive")
    archive.initialize(native_episode.native_protocol(base, source, discover=True, reuse_archives=[old]), 4, 42, False)
    try:
        native_episode.stage_native(archive, source)
        if change == "manifest":
            candidate = next((archive.directory / "designs").iterdir()) / "candidate/manifest.json"
            data = json.loads(candidate.read_text())
            data["genome"]["speed"] = data["expected"]["drives"][0]["speed"] = -3
            candidate.write_text(json.dumps(data))
        with pytest.raises(AdapterError, match="different physics|identical complete"):
            reuse_observations(archive, tmp_path)
        assert archive.counts() == {} and not archive.outcomes()
    finally:
        archive.db.close()


def test_four_new_trials_are_balanced_across_both_neighborhoods_with_a_reused_founder(tmp_path, monkeypatch):
    source, old = measured_founder(tmp_path)
    second = tmp_path / "second"
    second.mkdir()
    other, data = native_fixture(second, feet=2)
    entry = json.loads((other / "candidates.json").read_text())
    data["motion"]["genotypeId"] = entry["generated"][0]["genotypeId"] = "1"*64
    data["motion"]["assemblyId"] = entry["generated"][0]["assemblyId"] = "2"*64
    (other / "candidate-0/manifest.json").write_text(json.dumps(data))
    (other / "candidates.json").write_text(json.dumps(entry))
    pool = tmp_path / "pool"
    pool.mkdir()
    shutil.copytree(source, pool / "four-bar")
    shutil.copytree(other, pool / "walker1")
    write_json(pool / "seeds.json", {"schemaVersion": 1, "batches": ["four-bar", "walker1"]})
    archive = Archive(tmp_path / "search")
    base = physics_protocol()
    mock_constructor(tmp_path, monkeypatch, base)
    archive.initialize(native_episode.native_protocol(base, pool, discover=True, reuse_archives=[old]), 4, 42, False)
    calls = []
    def screen(batch, output, library, **kwargs):
        summary, _ = native_episode._queue(batch)
        return {"counts": {"reconstructionAttempts": 1}, "results": [
            {"candidate": str(batch / e["directory"]), "status": "ready-for-dynamics"} for e in summary["generated"]]}
    def evaluate(checked, output, options):
        calls.append(checked.manifest["motion"]["genotypeId"])
        return result(qualified=True, speed=.001+len(calls)*.001)
    monkeypatch.setattr(discovery, "screen_batch", screen)
    monkeypatch.setattr(native_episode, "check", lambda path, library: SimpleNamespace(manifest=load_manifest(path), report={}))
    monkeypatch.setattr(native_episode, "mass_layout_gate", lambda *args: {"gate": {"outcome": "pass"}})
    monkeypatch.setattr(native_episode, "evaluate", evaluate)
    try:
        discovery.search_discovery(archive, Proposals(siblings=2), pool, tmp_path, EvaluationOptions())
        assert len(calls) == 4 and archive.counts() == {"complete": 4}
        roots = [archive.founder(row[0]) for row in archive.db.execute("SELECT birth FROM evaluations")]
        assert roots.count(1) == roots.count(2) == 2
        assert archive.db.execute("SELECT count(*) FROM reused_evaluations").fetchone()[0] == 1
        assert calls[0] == "1"*64
    finally:
        archive.db.close()


def test_native_sole_patches_are_grouped_as_two_logical_feet_not_eight(tmp_path):
    source, data = native_fixture(tmp_path, feet=2)
    next(p for p in data["parts"] if p["id"] == "beam-b")["part"] = "3701.dat"
    for body in data["genome"]["bodies"]:
        for part in body["parts"]:
            if part["id"] == "beam-b":
                part["part"] = "3701.dat"
    regions = []
    for foot in data["feet"]:
        names = []
        for index, (x,z) in enumerate(((-40,-10), (-40,10), (40,-10), (40,10))):
            name = f"{foot['id']}-{index}"
            names.append(name)
            regions.append({**foot, "id": name, "position": [foot["position"][0]+x, 14, z]})
        next(f for f in data["motion"]["logicalFeet"] if f["id"] == foot["id"])["regions"] = names
    data["feet"] = regions
    path = source / "candidate-0/manifest.json"
    path.write_text(json.dumps(data))
    from legowalkers_sim.bridge import logical_foot_count
    assert logical_foot_count(load_manifest(path)) == 2
