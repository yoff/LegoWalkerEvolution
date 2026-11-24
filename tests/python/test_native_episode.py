import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim import native_episode, screening
from legowalkers_sim.bridge import AdapterError, load_manifest, logical_foot_count
from legowalkers_sim.evaluation import EvaluationOptions, motor_speed
from legowalkers_sim.evolution import Archive, ROOT, SEARCH_POLICY
from legowalkers_sim.payload import load_profile
from test_evolution import result
from test_screening import batch_fixture


def native_fixture(tmp_path, feet=3):
    source = batch_fixture(tmp_path, count=1)
    candidate = source / "candidate-0"
    path = candidate / "manifest.json"
    data = json.loads(path.read_text())
    data.update(grammarVersion="technic-motion-v3", modelKind="walker")
    for name, part in (("payload-battery", "58119.dat"), ("payload-receiver", "58123.dat")):
        data["parts"].append({"id": name, "part": part, "color": 7, "position": [0, -100, 0],
                              "rotation": np.eye(3).tolist(), "body": "chassis"})
    data["payloads"] = [{"partInstance": name, "product": product, "carrier": "walker"}
                        for name, product in (("payload-battery", "8881"), ("payload-receiver", "8884"))]

    def placement(p):
        return {"id": p["id"], "part": p["part"], "color": p["color"],
                "position": dict(zip(("x", "y", "z"), p["position"], strict=True)),
                "rotation": {column: {row: p["rotation"][i][j] for i, row in enumerate(("x", "y", "z"))}
                             for j, column in enumerate(("x", "y", "z"))}}

    data["feet"] = [{"id": f"f{i}", "body": "link", "position": [i*20, 20, 0],
                     "partInstance": "beam-b", "contactRadiusMeters": .004, "logicalFoot": f"f{i}"}
                    for i in range(feet)]
    data["motion"]["footCount"] = feet
    data["motion"]["logicalFeet"] = [{"id": f"f{i}", "body": "link", "regions": [f"f{i}"]} for i in range(feet)]
    data["genome"] = {
        "schemaVersion": 1, "recipeVersion": "walker-parts-v1", "mechanismId": data["motion"]["mechanismId"],
        "remote": True, "towWagon": False,
        "bodies": [{"body": body, "parts": [placement(p) for p in data["parts"] if p["body"] == body]}
                   for body in ("chassis", "link")],
        "connections": [{"group": "hinge", "hinges": ["hinge"], "placement": placement(data["parts"][2])}],
        "mounts": [], "feet": [{"foot": f["id"], "part": f["partInstance"], "depth": 0} for f in data["feet"]],
        "motorPart": "motor", "speed": 3, "maxTorque": .4,
    }
    path.write_text(json.dumps(data))
    (candidate / "realization.json").write_text(json.dumps(data["genome"]))
    (candidate / "program.json").write_text('{"unitTest":true}')
    (candidate / "cycle.svg").write_text('<svg xmlns="http://www.w3.org/2000/svg"/>')
    return source, data


def initialized(tmp_path, source, budget=1, *, discover=False):
    protocol = native_episode.native_protocol({
        "qualification": {"allowedGroundContacts": "designated-foot-regions-only"},
        "payloadProfile": load_profile(),
    }, source, discover=discover)
    archive = Archive(tmp_path / "archive")
    archive.initialize(protocol, budget, 42, False)
    return archive


@pytest.mark.parametrize("invalid", [None, "depth", "underside", "missing-choice"])
@pytest.mark.parametrize("layers", [0, 1, 2, 3])
def test_native_parallel_soles_validate_schema_and_actual_undersides(tmp_path, invalid, layers):
    source, data = native_fixture(tmp_path)
    logical = data["motion"]["logicalFeet"][0]
    choice = data["genome"]["feet"][0]
    choice.update(part="sole", additionalContacts=[{"part": "inner-sole", "depth": 40}])
    patches = []
    surfaces = []
    body = next(body for body in data["genome"]["bodies"] if body["body"] == "link")
    for name, depth in (("sole", 0), ("inner-sole", 40)):
        part = {"id": name, "part": "3701.dat", "color": 7, "position": [0, 0, depth],
                "rotation": np.eye(3).tolist(), "body": "link"}
        data["parts"].append(part)
        body["parts"].append({
            "id": name, "part": "3701.dat", "color": 7, "position": {"x": 0, "y": 0, "z": depth},
            "rotation": {"x": {"x": 1, "y": 0, "z": 0}, "y": {"x": 0, "y": 1, "z": 0},
                         "z": {"x": 0, "y": 0, "z": 1}},
        })
        contact_part = name
        for layer in range(1, layers+1):
            contact_part = f"{name}-plate-{layer}"
            plate = {"id": contact_part, "part": "3710.dat", "color": 2,
                     "position": [0, 24+8*(layer-1), depth],
                     "rotation": np.eye(3).tolist(), "body": "link"}
            data["parts"].append(plate)
            body["parts"].append({
                "id": contact_part, "part": "3710.dat", "color": 2,
                "position": {"x": 0, "y": 24+8*(layer-1), "z": depth},
                "rotation": {"x": {"x": 1, "y": 0, "z": 0}, "y": {"x": 0, "y": 1, "z": 0},
                             "z": {"x": 0, "y": 0, "z": 1}},
            })
        if name == "sole":
            choice["part"] = contact_part
        else:
            choice["additionalContacts"][0]["part"] = contact_part
        ids = []
        for index, (x, z) in enumerate((x, z) for x in (-40, 40) for z in (-10, 10)):
            ids.append(f"{name}-{index}")
            patches.append({"id": ids[-1], "body": "link", "logicalFoot": logical["id"],
                            "partInstance": contact_part, "position": [x, 24+8*layers, depth+z], "contactRadiusMeters": .004})
        surfaces.append({"contactShape": "rectangle", "regions": ids})
    logical.update(contactShape="rectangle", surfaces=surfaces, regions=[p["id"] for p in patches])
    data["feet"] = patches + data["feet"][1:]
    if invalid == "depth":
        choice["additionalContacts"][0]["depth"] = 20
    elif invalid == "underside":
        for patch in patches[4:]:
            patch["position"][1] = -24
    elif invalid == "missing-choice":
        choice.pop("additionalContacts")
    path = source / "candidate-0" / "manifest.json"
    path.write_text(json.dumps(data))
    if invalid or layers>2:
        with pytest.raises(AdapterError):
            load_manifest(path)
    else:
        assert logical_foot_count(load_manifest(path)) == 3


@pytest.mark.parametrize("changed_design", [False, True])
@pytest.mark.parametrize("new_contact_defaults", [False, True])
def test_revision_migration_reexports_without_rewriting_original_inputs(tmp_path, changed_design, new_contact_defaults):
    import shutil
    from legowalkers_sim.bridge import SIMULATOR_REVISION

    source, data = native_fixture(tmp_path)
    candidate = source / "candidate-0"
    data["motion"]["cycleSteps"] = 72
    data["simulator"]["revision"] = "0"*40
    (candidate / "manifest.json").write_text(json.dumps(data))
    original = {p.name: p.read_bytes() for p in candidate.iterdir() if p.is_file()}
    binary = tmp_path / "compiler"
    binary.mkdir()
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        (binary / name).write_bytes(b"fixture")
    calls = []

    def invoke(*args):
        calls.append(args)
        destination = Path(args[args.index("--output")+1]) / "candidate-001"
        shutil.copytree(candidate, destination)
        current = json.loads((destination / "manifest.json").read_text())
        current["simulator"]["revision"] = SIMULATOR_REVISION
        if new_contact_defaults:
            current["motion"]["genotypeId"] = "b"*64
            current["feet"][0]["position"][0] += 1
        if changed_design:
            current["motion"]["assemblyId"] = "f"*64
        (destination / "manifest.json").write_text(json.dumps(current))

    generator = SimpleNamespace(command=["dotnet", str(binary/"LegoWalkers.Cli.dll")], invoke=invoke)
    if changed_design:
        with pytest.raises(AdapterError, match="changed a seed design"):
            native_episode.refresh_seed_exports(source, tmp_path/"cache", generator)
    else:
        updated = native_episode.refresh_seed_exports(source, tmp_path/"cache", generator)
        assert updated != source
        assert load_manifest(updated/"candidate-0/manifest.json")["simulator"]["revision"] == SIMULATOR_REVISION
        migrated = load_manifest(updated/"candidate-0/manifest.json")
        assert migrated["feet"] == data["feet"]
        assert migrated["motion"] == data["motion"]
        provenance = json.loads((updated/"export-migration.json").read_text())
        assert provenance["designs"][data["motion"]["genotypeId"]]["ldrawByteIdentical"]
        assert "export-migration.json" in native_episode.native_protocol({}, updated)["candidateBatch"]
        assert native_episode.seed_batches(updated)[0][1]["generated"][0]["genotypeId"] == data["motion"]["genotypeId"]
        assert native_episode.refresh_seed_exports(source, tmp_path/"cache", generator) == updated
        assert len(calls) == 1
    assert {p.name: p.read_bytes() for p in candidate.iterdir() if p.is_file()} == original


def test_current_revision_seed_migration_is_a_read_only_noop(tmp_path):
    source, _ = native_fixture(tmp_path)
    assert native_episode.refresh_seed_exports(source, tmp_path/"cache", None) == source.resolve()
    assert not (tmp_path/"cache").exists()


def test_freezing_declared_contribution_files_rejects_escaping_paths(tmp_path):
    source, target = tmp_path / "source", tmp_path / "archive"
    source.mkdir()
    with pytest.raises(AdapterError, match="must remain inside"):
        native_episode._freeze_batch(source, target, {"../outside.json": "0" * 64})
    assert not target.exists()


@pytest.mark.parametrize("shape", ["capsule", "rectangle"])
@pytest.mark.parametrize("change", [None, "inflated", "translated", "wrong-part"])
def test_walker_support_anchors_must_match_the_selected_physical_part(tmp_path, shape, change):
    source, data = native_fixture(tmp_path, feet=2)
    part = next(part for part in data["parts"] if part["id"] == "beam-b")
    if shape == "rectangle":
        part["part"] = "3701.dat"
        for body in data["genome"]["bodies"]:
            for choice in body["parts"]:
                if choice["id"] == part["id"]:
                    choice["part"] = part["part"]
    points = ([[0, -10, 0], [0, 10, 0]] if shape == "capsule" else
              [[x, 24, z] for x in (-40, 40) for z in (-10, 10)])
    if change == "inflated":
        points = (np.array(points) * 2).tolist()
    elif change == "translated":
        points = (np.array(points) + [0, 1, 0]).tolist()
    original = data["feet"]
    data["feet"] = []
    for logical, foot in zip(data["motion"]["logicalFeet"], original, strict=True):
        patches = [{**foot, "id": f'{foot["id"]}-{i}',
                    "position": (np.array(point) + part["position"]).tolist(),
                    "contactRadiusMeters": .0016 if shape == "rectangle" else .004,
                    "partInstance": "beam-a" if change == "wrong-part" else part["id"]}
                   for i, point in enumerate(points)]
        data["feet"].extend(patches)
        logical.update(contactShape=shape, regions=[patch["id"] for patch in patches])
    path = source / "candidate-0/manifest.json"
    path.write_text(json.dumps(data))
    if change is None:
        assert load_manifest(path) == data
    else:
        with pytest.raises(AdapterError):
            load_manifest(path)


def mock_native(monkeypatch, archive, data, *, ready=True):
    calls = []

    def screen(source, output, library, **kwargs):
        calls.append("screen")
        assert not archive.counts()
        return {"results": [{"candidate": str(source / "candidate-0"),
                             "status": "ready-for-dynamics" if ready else "rejected"}]}

    def check(path, library):
        calls.append("check")
        return SimpleNamespace(manifest=load_manifest(path), report={"ldrawSha256": data["ldrawSha256"]})

    def evaluate(checked, output, options):
        calls.append("evaluate")
        assert archive.counts() == {"running": 1}
        measured = result(qualified=True)
        measured["metrics"]["logicalFootCount"] = logical_foot_count(checked.manifest)
        return measured

    monkeypatch.setattr(native_episode, "screen_batch", screen)
    monkeypatch.setattr(native_episode, "check", check)
    monkeypatch.setattr(native_episode, "mass_layout_gate", lambda *args: {"gate": {"outcome": "pass"}})
    monkeypatch.setattr(native_episode, "evaluate", evaluate)
    return calls


@pytest.mark.parametrize("name", ["candidate-0/construction.json", "candidate-0/grammar.json",
                                 "candidate-0/initial-ground-clearance.json",
                                 "candidate-0/motion-phases.json",
                                 "candidate-0/motion-space.json",
                                 "candidate-0/construction-proof.json",
                                 "conflicts.json", "grammar-search.json"])
def test_construction_provenance_is_frozen_with_native_candidate_inputs(tmp_path, name):
    source, _ = native_fixture(tmp_path)
    path = source / name
    path.write_text('{"constructionVersion":"technic-construction-v1","motif":"smooth-pivot-reversed-pin"}')
    archive = initialized(tmp_path, source)
    try:
        native_episode.stage_native(archive, source)
        assert (archive.directory / "native-input" / name).read_bytes() == path.read_bytes()
        design = archive.db.execute("SELECT design FROM births").fetchone()[0]
        if name in ("candidate-0/construction.json", "candidate-0/grammar.json"):
            assert not (archive.directory / "designs" / design / "candidate" / Path(name).name).exists()
        elif name.startswith("candidate-0/"):
            assert (archive.directory / "designs" / design / "candidate" / Path(name).name).read_bytes() == path.read_bytes()
        path.write_text('{"motif":"tampered"}')
        with pytest.raises(AdapterError, match="immutable episode protocol"):
            native_episode.stage_native(archive, source)
        assert archive.counts() == {}
    finally:
        archive.db.close()


def test_builder_policy_snapshot_is_frozen_and_identity_checked(tmp_path):
    source, _ = native_fixture(tmp_path)
    summary_path = source / "candidates.json"
    summary = json.loads(summary_path.read_text())
    summary["builderPolicyId"] = "a"*64
    summary_path.write_text(json.dumps(summary))
    builder = source / "builder.json"
    builder.write_text(json.dumps({"policyId": "a"*64, "policy": {"schemaVersion": 1}}))
    archive = initialized(tmp_path, source, discover=True)
    try:
        native_episode.stage_native(archive, source)
        assert (archive.directory / "native-input/builder.json").read_bytes() == builder.read_bytes()
        from legowalkers_sim.gallery import publish
        publish(archive)
        page = (archive.directory / "history/0001.html").read_text()
        assert "Builder policy and versioned motif catalogue" in page
        assert 'href="../native-input/builder.json"' in page
        builder.write_text(json.dumps({"policyId": "b"*64}))
        with pytest.raises(AdapterError, match="Builder policy identity"):
            native_episode.stage_native(archive, source)
        assert archive.counts() == {}
    finally:
        archive.db.close()


def test_invalid_mass_is_rejected_before_any_trial_reservation(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    archive = initialized(tmp_path, source)
    try:
        calls = mock_native(monkeypatch, archive, data)
        def invalid(*args):
            raise AdapterError("Invalid mass center fixture.")
        monkeypatch.setattr(native_episode, "mass_layout_gate", invalid)
        assert native_episode.search_native(archive, source, tmp_path, EvaluationOptions())["failures"] == 1
        assert archive.counts() == {}
        assert "evaluate" not in calls
        assert archive.db.execute("SELECT status FROM births").fetchone()[0] == "mass-layout-rejected"
    finally:
        archive.db.close()


def test_native_episode_uses_existing_accounting_and_publishes_odd_foot_bins(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    archive = initialized(tmp_path, source)
    try:
        calls = mock_native(monkeypatch, archive, data)
        measured = native_episode.search_native(archive, source, tmp_path, EvaluationOptions())
        assert calls == ["screen", "check", "evaluate"]
        assert measured["evaluations"] == {"complete": 1} and measured["failures"] == 0
        assert archive.outcomes()[0]["category"] == "3-feet"
        assert json.loads(archive.outcomes()[0]["genome"]) == data["genome"]
        page = (archive.directory / "index.html").read_text()
        assert "5 foot-count categories" in page and 'data-live-key="3-feet"' in page
        assert "topology evolution is not yet connected" in page
        assert "current paired four-bar backend" not in page
        assert (archive.directory / "native-input/candidate-0/program.json").exists()
        monkeypatch.setattr(native_episode, "screen_batch", lambda *args, **kwargs: {"results": []})
        native_episode.search_native(archive, source, tmp_path, EvaluationOptions())
        assert calls.count("evaluate") == 1
    finally:
        archive.db.close()


def test_native_screening_rejection_spends_no_slot_and_is_visible(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    archive = initialized(tmp_path, source)
    try:
        calls = mock_native(monkeypatch, archive, data, ready=False)
        measured = native_episode.search_native(archive, source, tmp_path, EvaluationOptions())
        assert calls == ["screen"] and archive.counts() == {} and measured["failures"] == 1
        assert "failed load-only screening" in (archive.directory / "history/0001.html").read_text()
    finally:
        archive.db.close()

def test_native_queue_keeps_existing_optional_preview_rendering(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    archive = initialized(tmp_path, source)
    try:
        mock_native(monkeypatch, archive, data)
        rendered = []
        monkeypatch.setattr(native_episode, "render_representative",
                            lambda archive, row, blender: rendered.append((row["category"], blender)))
        native_episode.search_native(archive, source, tmp_path, EvaluationOptions(), Path("/test/blender"))
        assert rendered == [("3-feet", Path("/test/blender"))]
    finally:
        archive.db.close()


def test_native_resume_recovers_birth_created_before_trial_reservation(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    archive = initialized(tmp_path, source)
    try:
        archive.record_native_birth(data)
        mock_native(monkeypatch, archive, data)
        native_episode.search_native(archive, source, tmp_path, EvaluationOptions())
        assert archive.counts() == {"complete": 1}
        assert archive.db.execute("SELECT count(*) FROM births").fetchone()[0] == 1
    finally:
        archive.db.close()


def test_native_interrupted_trial_stays_spent_and_is_not_evaluated_twice(tmp_path, monkeypatch):
    source, data = native_fixture(tmp_path)
    archive = initialized(tmp_path, source)
    try:
        mock_native(monkeypatch, archive, data)
        def interrupted(*args):
            raise KeyboardInterrupt()
        monkeypatch.setattr(native_episode, "evaluate", interrupted)
        with pytest.raises(KeyboardInterrupt):
            native_episode.search_native(archive, source, tmp_path, EvaluationOptions())
        assert archive.counts() == {"interrupted": 1}
        monkeypatch.setattr(native_episode, "screen_batch", lambda *args, **kwargs: {"results": []})
        native_episode.search_native(archive, source, tmp_path, EvaluationOptions())
        assert archive.counts() == {"interrupted": 1}
    finally:
        archive.db.close()


def test_native_protocol_snapshots_inputs_and_detects_changed_programs(tmp_path):
    source, _ = native_fixture(tmp_path)
    archive = initialized(tmp_path, source)
    try:
        assert archive.protocol()["searchPolicy"] == SEARCH_POLICY
        (source / "candidate-0/program.json").write_text('{"changed":true}')
        with pytest.raises(AdapterError, match="immutable episode protocol"):
            native_episode.search_native(archive, source, tmp_path, EvaluationOptions())
        assert archive.counts() == {}
    finally:
        archive.db.close()


@pytest.mark.parametrize("mutation", ["hardware", "payload", "one-foot", "drive", "unknown-choice"])
def test_native_manifest_rejects_inconsistent_realizations(tmp_path, mutation):
    source, data = native_fixture(tmp_path)
    path = source / "candidate-0/manifest.json"
    if mutation == "hardware":
        data["genome"]["bodies"][0]["parts"][0]["position"]["x"] += 20
    elif mutation == "payload":
        data["payloads"][0]["carrier"] = "wagon"
    elif mutation == "one-foot":
        data["motion"]["footCount"] = 1
        data["motion"]["logicalFeet"] = data["motion"]["logicalFeet"][:1]
    elif mutation == "drive":
        data["genome"]["speed"] = 0
    else:
        data["genome"]["pairs"] = 2
    path.write_text(json.dumps(data))
    with pytest.raises(AdapterError):
        load_manifest(path)


def test_native_motor_polarity_is_explicit_and_protocol_controls_magnitude(tmp_path):
    _, data = native_fixture(tmp_path)
    assert motor_speed(data, EvaluationOptions(speed=6)) == 6
    data["genome"]["speed"] = -3
    assert motor_speed(data, EvaluationOptions(speed=6)) == -6


@pytest.mark.integration
@pytest.mark.parametrize("feet", [2, 3, 4, 5, 6])
def test_complete_native_walkers_reconstruct_and_load_without_stepping(tmp_path, monkeypatch, feet):
    library = os.environ.get("LDRAW_LIBRARY")
    if not library:
        pytest.skip("Set LDRAW_LIBRARY for real native walker reconstruction.")
    source = tmp_path / "generated"
    subprocess.run([str(ROOT / ".tools/dotnet/dotnet"),
                    str(ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll"),
                    "realize-walker", "--feet", str(feet), "--steps", "72", "--output", str(source)],
                   check=True, capture_output=True, text=True, timeout=120)
    import mujoco
    monkeypatch.setattr(mujoco, "mj_step", lambda *args: pytest.fail("Load-only validation must not step physics."))
    checked = screening.screen_batch(source, tmp_path / "screened", Path(library), max_checks=1)
    assert checked["counts"]["ready"] == 1, json.dumps(checked, indent=2)
    assert checked["counts"]["dynamicTrials"] == 0
    row = checked["results"][0]
    assert row["logicalFootCount"] == feet and row["compatibility"]["motors"] == 1
    assert row["loadOnly"]["simulationSeconds"] == 0
    assert row["loadOnly"]["maxClosureErrorMeters"] < 1e-5
    assert row["loadOnly"]["initialSelfInterference"] == []
    data = load_manifest(source / "candidate-001/manifest.json")
    assert "pairs" not in data["genome"] and len(data["payloads"]) == 2
