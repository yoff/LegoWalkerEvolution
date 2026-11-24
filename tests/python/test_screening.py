import copy
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim import screening
from legowalkers_sim.bridge import AdapterError, check, compare_scene, load_manifest, logical_foot_count
from legowalkers_sim.collision import CollisionPart, prepare_parts
from test_bridge import manifest_data, scene_data
from test_collision import box
from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType
from lego_technic_sim.physics.model import Joint, JointType, PhysicsScene, Unit
from lego_technic_sim.ldraw.model import LDrawBuild


def batch_fixture(tmp_path, count=4):
    source = tmp_path/"generated"
    source.mkdir()
    gates = [{"gate": name, "outcome": "pass", "code": "test-fixture", "diagnostics": []}
             for name in screening.GATE_ORDER]
    entries = []
    mechanism = "a"*64
    for index in range(count):
        root = source/f"candidate-{index}"
        root.mkdir()
        data = manifest_data()
        model = f"1 7 {index if index < 3 else 0} 0 0 1 0 0 0 1 0 0 0 1 32523.dat\n"
        (root/"model.ldr").write_text(model)
        data.update(grammarVersion="technic-motion-v1", modelKind="leg",
                    ldrawSha256=hashlib.sha256(model.encode()).hexdigest(), payloads=[], groundWheels=[])
        data["parts"].append({"id": "motor", "part": "58121.dat", "body": "chassis", "color": 7,
                              "position": [0, 0, 0], "rotation": np.eye(3).tolist()})
        data["expected"]["drives"] = [{"motorPart": "motor", "joint": "hinge", "speed": 3, "maxTorque": .4}]
        data["expected"]["joints"][0]["connector"] = "pivot-hinge"
        data["parts"][2]["id"] = "pivot-hinge"
        data["genome"] = {
            "schemaVersion": 1, "recipeVersion": "planar-parts-v1", "mechanismId": mechanism, "side": 1,
            "bodies": [{"body": name, "part": "32523.dat", "layer": i, "offset": 0}
                       for i, name in enumerate(("chassis", "link"))],
            "connections": [{"group": "hinge", "part": "3673.dat", "center": 10}],
        }
        physical = hashlib.sha256(model.strip().encode()).hexdigest()
        data["motion"] = {
            "mechanismId": mechanism, "assemblyId": physical, "genotypeId": str(index)*64,
            "gateVersion": "motion-gates-v1", "cycleSteps": 72,
            "sharedPivots": [{"id": "hinge", "abstractHinges": ["hinge"], "bodies": ["chassis", "link"]}],
        }
        (root/"manifest.json").write_text(json.dumps(data))
        (root/"gates.json").write_text(json.dumps(gates))
        entries.append({"directory": root.name, "assemblyId": physical, "genotypeId": str(index)*64})
    (source/"candidates.json").write_text(json.dumps({
        "schemaVersion": 1, "gateVersion": "motion-gates-v1", "mechanismId": mechanism,
        "gates": gates[:3], "generated": entries, "rejected": [],
        "counts": {"proposals": 1, "realizationAttempts": count, "exported": count,
                   "duplicates": 0, "rejected": 0, "dynamicTrials": 0},
    }))
    return source


def mock_reconstruction(monkeypatch):
    calls = []
    monkeypatch.setattr(screening, "_protocol", lambda load_model: {"version": "test", "loadModel": load_model})
    monkeypatch.setattr(screening, "library_fingerprint", lambda *args: "parts-v1")

    def reconstruct(*args):
        calls.append("reconstruct")
        return SimpleNamespace(report={"parts": 4})

    def attach(*args, profile=None):
        calls.append("attach")

    def load(*args):
        calls.append("load")
        return {"simulationSeconds": 0, "dynamicTrials": 0,
                "maxClosureErrorMeters": 0, "initialSelfInterference": []}

    monkeypatch.setattr(screening, "check", reconstruct)
    monkeypatch.setattr(screening, "prepare_parts", attach)
    monkeypatch.setattr(screening, "_load_only", load)
    return calls


@pytest.mark.parametrize("outcome", ["collision", "incomplete", "clear-at-sampled-phases"])
def test_full_phase_admission_and_cached_repair_targets_preserve_no_trial_accounting(tmp_path, monkeypatch, outcome):
    source = batch_fixture(tmp_path, count=1)
    calls = mock_reconstruction(monkeypatch)
    candidate = source / "candidate-0"
    (candidate / "motion-phases.json").write_text('{"fixture":0}')

    def load(checked, output, *args):
        (output / "collision-model.json").write_text("{}")
        return {"simulationSeconds": 0, "dynamicTrials": 0, "maxClosureErrorMeters": 0,
                "initialSelfInterference": [], "modelFile": str(output / "model.xml")}

    def phases(checked, model, collision, document, **kwargs):
        assert kwargs["tolerance"] == screening.PENETRATION_TOLERANCE
        return {"version": "prescribed-phase-clearance-v1", "status": outcome, "sampledPoses": 73,
                "contacts": [{"parts": ["beam-b", "pivot-hinge"], "depthMeters": .001,
                              "inputAngleRadians": 1.57}] if outcome == "collision" else [],
                "reason": "incomplete-gait-sampling" if outcome == "incomplete" else None}

    monkeypatch.setattr(screening, "_load_only", load)
    monkeypatch.setattr("legowalkers_sim.phase_clearance.evaluate_phases", phases)
    output = tmp_path / "screen"
    first = screening.screen_batch(source, output, tmp_path, max_checks=1)
    report = first["results"][0]
    assert first["counts"]["dynamicTrials"] == 0
    assert report["status"] == ("ready-for-dynamics" if outcome == "clear-at-sampled-phases" else "rejected")
    from legowalkers_sim.screening_reports import screening_progress
    progress = screening_progress(output)
    phase_check = next(check for check in progress["checks"] if check["id"] == "phases")
    assert phase_check["status"] == ("pass" if outcome == "clear-at-sampled-phases" else "rejected")
    second = screening.screen_batch(source, output, tmp_path, max_checks=1)
    assert second["counts"]["reconstructionAttempts"] == 0
    if outcome == "collision":
        hints = report["repairHints"]
        assert hints["kind"] == "motion-self-interference" and hints["pairs"] == [["beam-b", "pivot-hinge"]]
        assert second["results"][0]["repairHints"] == hints
        assert second["results"][0]["phaseClearance"] == report["phaseClearance"]
    before = len(calls)
    (candidate / "motion-phases.json").write_text('{"fixture":1}')
    third = screening.screen_batch(source, output, tmp_path, max_checks=1)
    assert third["counts"]["reconstructionAttempts"] == 1
    assert len(calls) > before


def test_invalid_phase_feedback_is_logged_instead_of_fabricating_targets(caplog):
    from legowalkers_sim.screening_reports import repair_hints
    report = {"phaseClearance": {"version": "prescribed-phase-clearance-v1", "status": "collision",
                                "contacts": [{"parts": ["same", "same"]}]}}
    assert repair_hints(report) is None
    assert "lack valid pairs" in caplog.text


def test_checkpoints_are_visible_while_reconstruction_runs_and_keep_trial_count_zero(tmp_path, monkeypatch, capsys):
    from legowalkers_sim.screening_reports import screening_progress
    source = batch_fixture(tmp_path, 1)
    calls = mock_reconstruction(monkeypatch)
    output = tmp_path / "screened"

    def reconstruct(*args):
        state = screening_progress(output, running=True)
        checks = {check["id"]: check for check in state["checks"]}
        assert checks["inputs"]["status"] == checks["dependencies"]["status"] == "pass"
        assert checks["reconstruction"]["status"] == "running"
        assert checks["contacts"]["status"] == "pending"
        assert checks["reconstruction"]["started"] and checks["reconstruction"]["completed"] is None
        calls.append("reconstruct")
        return SimpleNamespace(report={})

    monkeypatch.setattr(screening, "check", reconstruct)
    result = screening.screen_batch(source, output, tmp_path)
    state = screening_progress(output)
    checks = {check["id"]: check for check in state["checks"]}
    assert checks["reconstruction"]["status"] == checks["closure"]["status"] == checks["contacts"]["status"] == "pass"
    assert checks["mass"]["status"] == "not-run"  # This fixture is not a native payload model.
    assert checks["export"]["status"] == "not-run"  # The test load-only function was mocked.
    assert result["counts"]["dynamicTrials"] == 0
    assert "Parts, rigid bodies and moving joints - running" in capsys.readouterr().err


def test_failed_check_keeps_earlier_success_and_does_not_invent_later_success(tmp_path, monkeypatch):
    from legowalkers_sim.screening_reports import screening_progress
    source = batch_fixture(tmp_path, 1)
    mock_reconstruction(monkeypatch)

    def fail(*args, profile=None):
        raise AdapterError("Broken connector fixture")

    monkeypatch.setattr(screening, "prepare_parts", fail)
    output = tmp_path / "screened"
    result = screening.screen_batch(source, output, tmp_path)
    checks = {check["id"]: check for check in screening_progress(output)["checks"]}
    assert checks["reconstruction"]["status"] == "pass"
    assert checks["collision"]["status"] == "failed"
    assert checks["collision"]["detail"] == "Broken connector fixture"
    assert checks["load"]["status"] == checks["contacts"]["status"] == "not-run"
    assert result["results"][0]["status"] == "rejected"


def test_interrupted_check_is_presentation_only_and_cached_checks_are_not_rerun(tmp_path, monkeypatch):
    from legowalkers_sim.screening_reports import ScreeningProgress, screening_progress
    pending = ScreeningProgress(tmp_path / "partial", "a" * 64)
    pending.start("inputs")
    before = pending.path.read_bytes()
    assert screening_progress(tmp_path / "partial", running=False)["checks"][0]["status"] == "interrupted"
    assert pending.path.read_bytes() == before
    source = batch_fixture(tmp_path, 1)
    calls = mock_reconstruction(monkeypatch)
    output = tmp_path / "screened"
    screening.screen_batch(source, output, tmp_path)
    screening.screen_batch(source, output, tmp_path)
    assert calls.count("reconstruct") == 1
    state = screening_progress(output)
    assert "Saved screening outcome: ready-for-dynamics; not rerun." in state["detail"]
    checks = {check["id"]: check for check in state["checks"]}
    assert checks["reconstruction"]["status"] == "not-run"
    with pytest.raises(AdapterError, match="valid genotype"):
        ScreeningProgress(tmp_path, "../unsafe")


def test_progress_reader_uses_legacy_outcomes_without_guessing_and_rejects_bad_json(tmp_path):
    from legowalkers_sim.screening_reports import screening_progress
    assert screening_progress(tmp_path)["checks"] == []
    path = tmp_path / "screen-0001.json"
    path.write_text(json.dumps({"results": [{"genotypeId": "a" * 64, "gates": [
        {"gate": "geometry", "outcome": "warning", "diagnostics": ["Not a collision proof."]}]}]}))
    state = screening_progress(tmp_path, genotype="a" * 64)
    assert not state["tracked"] and state["checks"][0]["status"] == "warning"
    assert state["checks"][0]["detail"] == "Not a collision proof."
    (tmp_path / "progress").mkdir()
    (tmp_path / "progress" / ("a" * 64 + ".json")).write_text("{")
    with pytest.raises(AdapterError, match="Invalid screening progress"):
        screening_progress(tmp_path)


def test_payload_profile_content_changes_screening_cache_namespace(tmp_path, monkeypatch):
    from legowalkers_sim.payload import load_profile
    monkeypatch.setattr(screening, "_protocol", lambda loaded: {"loaded": loaded})
    profile = load_profile()
    path = tmp_path / "payloads.json"
    path.write_text(json.dumps(profile))
    original = screening.cache_directory(tmp_path, payload_profile=path)
    profile["components"]["8881"]["massKg"] += .01
    path.write_text(json.dumps(profile))
    assert screening.cache_directory(tmp_path, payload_profile=path) != original


@pytest.mark.parametrize("valid", [True, False])
def test_native_mass_gate_precedes_model_loading_and_uses_selected_profile(tmp_path, monkeypatch, valid):
    from legowalkers_sim.payload import load_profile
    from test_native_episode import native_fixture
    source, _ = native_fixture(tmp_path)
    calls = mock_reconstruction(monkeypatch)
    profile = load_profile()
    profile["components"]["8881"]["massKg"] = .3
    path = tmp_path / "custom-payload.json"
    path.write_text(json.dumps(profile))
    def prepare(checked, *, profile):
        assert profile["components"]["8881"]["massKg"] == .3
        return None, [], {}
    monkeypatch.setattr(screening, "prepare_parts", prepare)
    tensors = {0: object()}
    monkeypatch.setattr(screening, "apply_payloads", lambda scene, parts, manifest, selected: (scene, tensors))
    def analyze(checked, scene, parts, selected, *, inertia_tensors):
        calls.append("mass")
        assert inertia_tensors is tensors
        assert selected["components"]["8881"]["massKg"] == .3
        if not valid:
            raise AdapterError("Invalid component center fixture.")
        return {"gate": {"gate": "mass-layout", "outcome": "warning"}}
    monkeypatch.setattr(screening, "analyze_layout", analyze)
    result = screening.screen_batch(source, tmp_path / "screened", tmp_path, payload_profile=path)
    assert calls == (["reconstruct", "mass", "load"] if valid else ["reconstruct", "mass"])
    assert result["counts"]["dynamicTrials"] == 0
    if valid:
        assert result["results"][0]["massLayout"]["gate"]["outcome"] == "warning"
        assert result["counts"]["ready"] == 1
    else:
        assert result["results"][0]["gates"][-1]["gate"] == "mass-layout"
        assert result["counts"]["ready"] == 0


def test_selected_realization_screens_without_processing_its_siblings(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path)
    original = (source / "candidates.json").read_bytes()
    calls = mock_reconstruction(monkeypatch)
    screened = screening.screen_batch(source, tmp_path / "screened", tmp_path, max_checks=1,
                                       candidate_ids={"1" * 64})
    assert calls == ["reconstruct", "attach", "load"]
    assert len(screened["results"]) == 1
    assert Path(screened["results"][0]["candidate"]).name == "candidate-1"
    assert screened["counts"]["reconstructionAttempts"] == 1
    assert screened["counts"]["dynamicTrials"] == 0
    assert (source / "candidates.json").read_bytes() == original


@pytest.mark.parametrize("selected", [set(), {"not-in-batch"}])
def test_screening_selector_rejects_empty_or_unknown_candidates(tmp_path, selected):
    with pytest.raises(AdapterError, match="belong to the immutable batch"):
        screening.screen_batch(batch_fixture(tmp_path), tmp_path / "screened", tmp_path,
                               candidate_ids=selected)


def test_bounded_checks_deduplicate_physical_assemblies_and_never_spend_trials(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path)
    calls = mock_reconstruction(monkeypatch)
    output = tmp_path/"screened"
    result = screening.screen_batch(source, output, tmp_path, max_checks=1)
    assert calls == ["reconstruct", "attach", "load"]
    assert result["counts"]["ready"] == 1
    assert result["counts"]["screeningDuplicates"] == 1
    assert result["counts"]["deferred"] == 2
    assert result["counts"]["dynamicTrials"] == 0
    gates = result["results"][0]["gates"]
    assert [gate["gate"] for gate in gates] == screening.GATE_ORDER + [
        "simulator-reconstruction", "dynamic-evaluation"]
    assert gates[-1]["outcome"] == "not-run"
    resumed = screening.screen_batch(source, output, tmp_path, max_checks=2)
    assert resumed["counts"]["reconstructionAttempts"] == 2
    assert resumed["counts"]["screeningDuplicates"] == 2
    assert resumed["counts"]["ready"] == 2
    with sqlite3.connect(output/"candidates.sqlite3") as db:
        assert db.execute("SELECT count(*) FROM assemblies").fetchone()[0] == 3
        assert db.execute("SELECT count(*) FROM batches").fetchone()[0] == 2
    assert len(list(output.glob("screen-*.json"))) == 2


def test_one_bad_realization_does_not_reject_its_mechanism_or_other_assemblies(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path, 2)
    calls = mock_reconstruction(monkeypatch)

    def attach(*args, profile=None):
        calls.append("attach")
        if calls.count("attach") == 1:
            raise AdapterError("undeclared socket/body")

    monkeypatch.setattr(screening, "prepare_parts", attach)
    result = screening.screen_batch(source, tmp_path/"screened", tmp_path)
    assert [row["status"] for row in result["results"]] == ["rejected", "ready-for-dynamics"]
    assert result["counts"]["screeningRejections"] == 1
    assert result["results"][0]["gates"][-1]["diagnostics"] == ["undeclared socket/body"]
    assert result["counts"]["dynamicTrials"] == 0


def test_static_interference_rejects_but_unloaded_checks_do_not_claim_ready(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path, 1)
    mock_reconstruction(monkeypatch)
    monkeypatch.setattr(screening, "_load_only", lambda *args: {
        "simulationSeconds": 0, "maxClosureErrorMeters": 0,
        "initialSelfInterference": [{"parts": ["a", "b"], "depthMeters": .001}],
    })
    rejected = screening.screen_batch(source, tmp_path/"rejected", tmp_path)
    assert rejected["results"][0]["gates"][-1]["code"] == "initial-self-interference"
    assert rejected["counts"]["ready"] == 0
    cached = screening.screen_batch(source, tmp_path / "rejected", tmp_path)
    assert cached["results"][0]["previousStatus"] == "rejected"
    assert cached["results"][0]["rejectionDetail"] == (
        "Initial self-interference: a / b: 1.000 mm overlap (collision model).")
    assert cached["counts"]["reconstructionAttempts"] == 0
    unloaded = screening.screen_batch(source, tmp_path/"unloaded", tmp_path, load_model=False)
    assert unloaded["results"][0]["status"] == "reconstructed-not-loaded"
    assert unloaded["counts"]["ready"] == 0


def test_protocol_changes_and_identity_mismatches_are_explicit(tmp_path, monkeypatch):
    source = batch_fixture(tmp_path, 1)
    mock_reconstruction(monkeypatch)
    screening.screen_batch(source, tmp_path/"screened", tmp_path)
    with pytest.raises(AdapterError, match="protocol/source changed"):
        screening.screen_batch(source, tmp_path/"screened", tmp_path, load_model=False)
    batch_path = source/"candidates.json"
    data = json.loads(batch_path.read_text())
    data["generated"][0]["assemblyId"] = "f"*64
    batch_path.write_text(json.dumps(data))
    result = screening.screen_batch(source, tmp_path/"other", tmp_path)
    assert result["results"][0]["status"] == "rejected"
    assert result["counts"]["reconstructionAttempts"] == 0
    assert "identities disagree" in result["results"][0]["error"]
    with pytest.raises(AdapterError, match="between 1 and 32"):
        screening.screen_batch(source, tmp_path/"other", tmp_path, max_checks=0)


def test_logical_feet_are_inferred_independently_of_contact_regions(tmp_path):
    source = batch_fixture(tmp_path, 1)
    path = source/"candidate-0/manifest.json"
    manifest = json.loads(path.read_text())
    assert load_manifest(path)["grammarVersion"] == "technic-motion-v1"
    with pytest.raises(AdapterError, match="no logical foot grouping"):
        logical_foot_count(manifest)
    manifest["grammarVersion"] = "technic-motion-v2"
    regions = [f"sole-{index}" for index in range(4)]
    manifest["motion"].update(footCount=1, logicalFeet=[{"id": "sole", "body": "link", "regions": regions}])
    manifest["feet"] = [{"id": name, "body": "link", "position": [0, 20, 0],
                         "partInstance": "beam-b", "contactRadiusMeters": .004, "logicalFoot": "sole"}
                        for name in regions]
    path.write_text(json.dumps(manifest))
    loaded = load_manifest(path)
    assert logical_foot_count(loaded) == 1 and len(loaded["feet"]) == 4
    manifest["motion"]["footCount"] = 4
    path.write_text(json.dumps(manifest))
    with pytest.raises(AdapterError, match="inferred foot count"):
        load_manifest(path)
    manifest["motion"]["footCount"] = 1
    manifest["feet"][0]["logicalFoot"] = "other"
    path.write_text(json.dumps(manifest))
    with pytest.raises(AdapterError, match="own actual body and contact regions"):
        load_manifest(path)


def test_coaxial_hinge_trees_are_equivalent_but_disconnected_or_offset_ones_are_not():
    manifest, build, scene = scene_data()
    manifest["grammarVersion"] = "technic-motion-v1"
    manifest["parts"][2]["id"] = "pivot-shared"
    manifest["expected"]["joints"][0].update(connector="pivot-shared")
    third = copy.deepcopy(manifest["parts"][0])
    third.update(id="third", body="third", position=[0, 40, 0])
    manifest["parts"].append(third)
    extra = box("32523.dat", [0, 40, 0], [5, 5, 5])
    build.parts.append(extra)
    scene.units.append(Unit([extra]))
    manifest["expected"]["bodies"].append("third")
    second = copy.deepcopy(manifest["expected"]["joints"][0])
    second.update(id="second", bodyB="third")
    manifest["expected"]["joints"].append(second)
    manifest["motion"] = {"sharedPivots": [{"id": "shared", "bodies": ["chassis", "link", "third"]}]}
    scene.joints.append(Joint(0, 2, JointType.REVOLUTE, np.array([0, .004, 0]), np.array([0, 1, 0])))
    mapping, joints = compare_scene(manifest, build, scene)
    assert set(mapping) == {"chassis", "link", "third"}
    assert len(joints) == 2
    scene.joints[1].position = np.array([.001, .004, 0])
    with pytest.raises(AdapterError, match="Shared pivot reconstruction mismatch"):
        compare_scene(manifest, build, scene)


def test_shared_pivot_feedback_names_the_competing_selected_part(monkeypatch):
    from legowalkers_sim import bridge

    manifest, build, scene = scene_data()
    manifest["grammarVersion"] = "technic-motion-v1"
    manifest["parts"][2]["id"] = "pivot-shared"
    manifest["expected"]["joints"][0]["connector"] = "pivot-shared"
    competing = copy.deepcopy(manifest["parts"][0])
    competing.update(id="competing-gear", body="shaft", position=[0, 40, 0])
    manifest["parts"].append(competing)
    extra = box("32523.dat", [0, 40, 0], [5, 5, 5])
    build.parts.append(extra)
    scene.units.append(Unit([extra]))
    scene.joints = [Joint(1, 2, JointType.REVOLUTE, np.array([0, .004, 0]), np.array([0, -1, 0]))]
    manifest["motion"] = {"sharedPivots": [{"id": "shared", "bodies": ["chassis", "link"]}]}
    monkeypatch.setattr(bridge, "_find_port_connections", lambda *args: [(0, "revolute"), (3, "revolute")])
    with pytest.raises(AdapterError, match="competing-gear") as caught:
        compare_scene(manifest, build, scene)
    assert caught.value.repair_hints == {
        "kind": "joint-port-engagement", "parts": ["competing-gear", "pivot-shared"], "bodies": ["chassis", "link"]}


def test_shared_pin_mass_attaches_once_and_only_real_socket_pairs_are_exempt():
    beams = [box("32523.dat", [0, 0, depth], [8, 8, 10]) for depth in (0, 20, 40)]
    pin = box("32556.dat", [0, 0, 20], [5, 5, 30], shaft=True)
    for beam in beams:
        beam.ports = [ConnectionPort(PortType.ROUND_HOLE, beam.position, np.array([0, 0, 1]))]
    geometry = [*beams, pin]
    parts = [{"id": f"p{i}", "part": p.part_id, "body": f"b{i}" if i < 3 else None,
              "rotation": p.transform[:3, :3].tolist()} for i, p in enumerate(geometry)]
    joints = [{"id": f"j{i}", "connector": "p3", "bodyA": f"b{i}", "bodyB": f"b{i+1}",
               "axis": [0, 0, 1], "position": [0, 0, 20]} for i in range(2)]
    from legowalkers_sim.bridge import CheckedModel
    checked = CheckedModel({"parts": parts, "expected": {"joints": joints, "gears": []}},
                           PhysicsScene(units=[Unit([b], .01, b.position*.0004) for b in beams]),
                           {f"b{i}": i for i in range(3)}, {}, {}, LDrawBuild("test", geometry))
    scene, attached, exemptions = prepare_parts(checked)
    assert len(attached) == 4
    assert [p.instance for p in attached if p.unit == 1] == ["p1", "p3"]
    assert len(exemptions) == 3
    from legowalkers_sim.payload import part_mass_center
    beam_mass, _ = part_mass_center(CollisionPart("beam", beams[0], 0))
    pin_mass, _ = part_mass_center(CollisionPart("pin", pin, 1))
    assert scene.units[0].mass == pytest.approx(beam_mass)
    assert scene.units[2].mass == pytest.approx(beam_mass)
    assert scene.units[1].mass == pytest.approx(beam_mass+pin_mass)
    joints[1]["position"] = [1, 0, 20]
    with pytest.raises(AdapterError, match="noncoaxial"):
        prepare_parts(checked)


def test_shaft_endpoint_touch_is_not_an_engaged_or_exempt_third_socket():
    from legowalkers_sim.bridge import CheckedModel
    beams = [box("32523.dat", [0, 0, depth], [8, 8, 10]) for depth in (0, 20, 40)]
    pin = box("3673.dat", [0, 0, 10], [5, 5, 20], shaft=True)
    for beam, face in zip(beams, (10, 10, 30), strict=True):
        beam.ports = [ConnectionPort(PortType.ROUND_HOLE, np.array([0, 0, face]), np.array([0, 0, 1]))]
    geometry = [*beams, pin]
    parts = [{"id": f"p{i}", "part": p.part_id, "body": f"b{i}" if i < 3 else None,
              "rotation": p.transform[:3, :3].tolist()} for i, p in enumerate(geometry)]
    joints = [{"connector": "p3", "bodyA": "b0", "bodyB": "b1", "axis": [0, 0, 1], "position": [0, 0, 10]}]
    checked = CheckedModel({"parts": parts, "expected": {"joints": joints, "gears": []}},
                           PhysicsScene(units=[Unit([b], .01, b.position*.0004) for b in beams]),
                           {f"b{i}": i for i in range(3)}, {}, {}, LDrawBuild("test", geometry))
    _, _, exemptions = prepare_parts(checked)
    assert set(exemptions) == {("p0", "p3"), ("p1", "p3")}


@pytest.mark.integration
@pytest.mark.parametrize("example,body_count,hinges", [("four-bar", 4, 4), ("walker1", 8, 10)])
def test_real_parts_reconstruct_and_load_without_any_physics_step(tmp_path, monkeypatch, example, body_count, hinges):
    library = os.environ.get("LDRAW_LIBRARY")
    if not library:
        pytest.skip("Set LDRAW_LIBRARY to run real-part realization checks.")
    project = Path(__file__).resolve().parents[2]
    cli = project/"src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll"
    dotnet = project/".tools/dotnet/dotnet"
    source = tmp_path/"generated"
    subprocess.run([str(dotnet), str(cli), "realize", "--example", example, "--output", str(source),
                    "--limit", "2", "--steps", "72"], check=True, capture_output=True, text=True, timeout=120)
    import mujoco
    monkeypatch.setattr(mujoco, "mj_step", lambda *args: pytest.fail("Screening must never step physics."))
    result = screening.screen_batch(source, tmp_path/"screened", Path(library))
    assert result["counts"]["dynamicTrials"] == 0
    assert result["counts"]["screeningRejections"] == 0, json.dumps(result, indent=2)
    assert result["counts"]["ready"] == 2
    for row in result["results"]:
        assert row["compatibility"]["bodies"] == body_count
        assert row["compatibility"]["joints"] == hinges
        assert row["compatibility"]["motors"] == 1
        assert row["logicalFootCount"] == 1
        assert row["loadOnly"]["simulationSeconds"] == 0
        # The pinned MJCF exporter rounds body transforms; 10 micrometres is
        # still substantially tighter than the 2 mm dynamic eligibility limit.
        assert row["loadOnly"]["maxClosureErrorMeters"] < 1e-5
