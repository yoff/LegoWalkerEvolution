import copy
import hashlib
import json
import os
from pathlib import Path

import numpy as np
import pytest
from lego_technic_sim.ldraw.model import LDrawBuild, LDrawPart, Triangle
from lego_technic_sim.physics.model import Joint, JointType, PhysicsScene, Unit

from legowalkers_sim.bridge import AdapterError, SIMULATOR_REVISION, check, compare_scene, library_fingerprint, load_manifest
from legowalkers_sim.cli import main
from legowalkers_sim.evaluation import EvaluationOptions, evaluate


def manifest_data():
    parts = [
        {"id": "beam-a", "part": "32523.dat", "color": 7, "position": [0, 0, 0],
         "rotation": np.eye(3).tolist(), "body": "chassis"},
        {"id": "beam-b", "part": "32523.dat", "color": 4, "position": [0, 20, 0],
         "rotation": np.eye(3).tolist(), "body": "link"},
        {"id": "pin", "part": "3673.dat", "color": 8, "position": [0, 10, 0],
         "rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]], "body": None},
    ]
    return {
        "schemaVersion": 1, "grammarVersion": "technic-fourbar-v1", "modelKind": "hinge",
        "modelFile": "model.ldr", "ldrawSha256": hashlib.sha256(b"0 test\n").hexdigest(),
        "simulator": {"repository": "https://github.com/yoff/LegoTechnicSimulation",
                      "revision": SIMULATOR_REVISION},
        "units": {"length": "ldu", "lduToMeters": 0.0004, "up": [0, -1, 0], "forward": [1, 0, 0]},
        "genome": {"pairs": 3, "groundHoles": 4, "couplerHoles": 4, "rockerHoles": 4,
                   "phaseQuarterTurns": 1, "seed": 0},
        "chassis": "chassis", "parts": parts,
        "expected": {
            "bodies": ["chassis", "link"],
            "joints": [{"id": "hinge", "bodyA": "chassis", "bodyB": "link", "kind": "revolute",
                        "position": [0, 10, 0], "axis": [0, 1, 0], "connector": "pin"}],
            "gears": [], "drives": [],
        },
        "feet": [], "footPaths": [], "warnings": [],
    }


def write_manifest(tmp_path, data):
    (tmp_path / "model.ldr").write_bytes(b"0 test\n")
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def scene_data():
    manifest = manifest_data()
    actual_parts = []
    for part in manifest["parts"]:
        transform = np.eye(4)
        transform[:3, :3] = part["rotation"]
        transform[:3, 3] = part["position"]
        triangle = Triangle(np.zeros(3), np.ones(3), np.array([0, 1, 0]))
        actual_parts.append(LDrawPart(part["part"], part["color"], transform, [triangle]))
    build = LDrawBuild("test", actual_parts)
    # Reverse unit order to prove that matching uses membership, not unit indices.
    scene = PhysicsScene(
        units=[Unit([actual_parts[1]]), Unit([actual_parts[0]])],
        joints=[Joint(1, 0, JointType.REVOLUTE, np.array([0, 0.004, 0]), np.array([0, -1, 0]))],
    )
    return manifest, build, scene


def test_chassis_canonicalization_preserves_physics_and_remaps_every_unit_reference():
    from lego_technic_sim.physics.model import GearConstraint, Motor
    from legowalkers_sim.bridge import _canonical_chassis_scene

    manifest, build, scene = scene_data()
    mapping, _ = compare_scene(manifest, build, scene)
    scene.motors = [Motor(0, speed=-3, max_torque=.4)]
    axis = np.array([0., 1., 0.])
    scene.gears = [GearConstraint(0, 1, 2., axis, axis)]
    canonical, bodies = _canonical_chassis_scene(scene, mapping, "chassis")
    assert bodies == {"chassis": 0, "link": 1}
    assert canonical.units[0] is scene.units[1]
    assert canonical.units[1] is scene.units[0]
    assert (canonical.joints[0].unit_a_index, canonical.joints[0].unit_b_index) == (0, 1)
    assert (canonical.gears[0].unit_a_index, canonical.gears[0].unit_b_index) == (1, 0)
    assert canonical.joints[0].position is scene.joints[0].position
    assert canonical.joints[0].axis is scene.joints[0].axis
    assert canonical.gears[0].ratio == scene.gears[0].ratio
    assert canonical.gears[0].axis_a is scene.gears[0].axis_a
    assert canonical.motors == scene.motors
    assert canonical.gravity is scene.gravity
    assert (scene.joints[0].unit_a_index, scene.joints[0].unit_b_index) == (1, 0)
    assert (scene.gears[0].unit_a_index, scene.gears[0].unit_b_index) == (0, 1)
    unchanged, same_indices = _canonical_chassis_scene(canonical, bodies, "chassis")
    assert unchanged is canonical and same_indices is bodies


def test_manifest_contract(tmp_path):
    data = manifest_data()
    assert load_manifest(write_manifest(tmp_path, data)) == data


def test_payload_extensions_are_versioned_and_require_actual_electronics(tmp_path):
    data = manifest_data()
    data["payloads"] = []
    with pytest.raises(AdapterError, match="require grammar v3"):
        load_manifest(write_manifest(tmp_path, data))
    data["grammarVersion"] = "technic-fourbar-v3"
    with pytest.raises(AdapterError, match="all payload/control genes"):
        load_manifest(write_manifest(tmp_path, data))
    data["genome"].update(remote=False, batterySlot=1, batteryLevel=0, receiverSlot=0,
                          receiverSide=1, motorDirection=-1)
    assert load_manifest(write_manifest(tmp_path, data)) == data
    data["genome"]["batterySlot"] = 0
    with pytest.raises(AdapterError, match="canonical inactive"):
        load_manifest(write_manifest(tmp_path, data))
    data["genome"].update(remote=True, batterySlot=1)
    with pytest.raises(AdapterError, match="exactly one"):
        load_manifest(write_manifest(tmp_path, data))
    data["payloads"] = [{"product": "8881", "partInstance": "beam-a"},
                        {"product": "8884", "partInstance": "beam-b"}]
    with pytest.raises(AdapterError, match="correct electronics"):
        load_manifest(write_manifest(tmp_path, data))


@pytest.mark.parametrize("mutation", [
    lambda data: data.update(schemaVersion=2),
    lambda data: data["expected"]["bodies"].append("link"),
    lambda data: data["parts"][1].update(id="beam-a"),
    lambda data: data["expected"]["joints"][0].update(bodyB="missing"),
    lambda data: data["expected"]["joints"][0].update(axis=[0, 2, 0]),
    lambda data: data["parts"][0].update(rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
    lambda data: data.update(modelFile="../other.ldr"),
])
def test_invalid_manifest_is_explicitly_rejected(tmp_path, mutation):
    data = manifest_data()
    mutation(data)
    with pytest.raises(AdapterError):
        load_manifest(write_manifest(tmp_path, data))


def test_checksum_and_nonfinite_json_rejected(tmp_path):
    path = write_manifest(tmp_path, manifest_data())
    (tmp_path / "model.ldr").write_text("0 changed\n", encoding="utf-8")
    with pytest.raises(AdapterError, match="checksum"):
        load_manifest(path)
    path.write_text('{"schemaVersion": NaN}', encoding="utf-8")
    with pytest.raises(AdapterError, match="Non-finite"):
        load_manifest(path)
    path.write_text('{"schemaVersion": 1e1000}', encoding="utf-8")
    with pytest.raises(AdapterError, match="Non-finite"):
        load_manifest(path)


def test_round_trip_matches_body_members_and_axis_lines():
    manifest, build, scene = scene_data()
    bodies, joints = compare_scene(manifest, build, scene)
    assert bodies == {"chassis": 1, "link": 0}
    assert joints == {"hinge": 0}


@pytest.mark.parametrize("mutation", [
    lambda scene: scene.units.append(Unit([])),
    lambda scene: scene.units[0].bricks.extend(scene.units[1].bricks),
    lambda scene: scene.joints[0].position.__setitem__(0, 0.01),
    lambda scene: scene.joints[0].axis.__setitem__(slice(None), [1, 0, 0]),
    lambda scene: scene.joints.append(copy.deepcopy(scene.joints[0])),
])
def test_mechanics_mismatches_do_not_become_success(mutation):
    manifest, build, scene = scene_data()
    mutation(scene)
    with pytest.raises(AdapterError):
        compare_scene(manifest, build, scene)


def test_missing_recursive_parts_are_not_silently_ignored(tmp_path):
    (tmp_path / "parts").mkdir()
    (tmp_path / "p").mkdir()
    (tmp_path / "parts" / "test.dat").write_text(
        "1 16 0 0 0 1 0 0 0 1 0 0 0 1 missing.dat\n", encoding="utf-8")
    with pytest.raises(AdapterError, match="Missing LDraw dependency"):
        library_fingerprint(tmp_path, ["test.dat"])


@pytest.mark.parametrize("options", [
    EvaluationOptions(duration=float("nan")),
    EvaluationOptions(duration=1, settle=1),
    EvaluationOptions(timestep=0),
    EvaluationOptions(friction=0),
    EvaluationOptions(torque=-1),
    EvaluationOptions(speed=0),
    EvaluationOptions(duration=1.0005, settle=0),
])
def test_invalid_evaluation_parameters_rejected(options):
    with pytest.raises(AdapterError):
        options.validate()

def test_reverse_motor_speed_is_supported():
    EvaluationOptions(speed=-3).validate()


def test_cli_reports_missing_library(capsys, monkeypatch):
    monkeypatch.delenv("LDRAW_LIBRARY", raising=False)
    assert main(["check", "does-not-exist.json"]) == 2
    assert "LDRAW_LIBRARY" in capsys.readouterr().err


def integration_paths():
    library = os.environ.get("LDRAW_LIBRARY")
    candidates = os.environ.get("LEGOWALKERS_CANDIDATES")
    if not library or not candidates:
        pytest.skip("Set LDRAW_LIBRARY and LEGOWALKERS_CANDIDATES for real simulator integration.")
    return Path(library), Path(candidates)


@pytest.fixture(scope="module")
def real_model():
    library, candidates = integration_paths()
    cache = {}

    def get(kind):
        if kind not in cache:
            cache[kind] = check(candidates / kind / "manifest.json", library)
        return cache[kind]

    return get


@pytest.mark.integration
@pytest.mark.parametrize("kind,counts", [
    ("rigid", (1, 0, 0, 0)), ("hinge", (2, 1, 0, 0)), ("motor", (2, 1, 0, 1)),
    ("leg", (6, 6, 2, 1)), ("walker", (24, 29, 10, 1)),
])
def test_real_ldraw_round_trip(kind, counts, real_model):
    checked = real_model(kind)
    assert tuple(checked.report[key] for key in ("bodies", "joints", "gearMeshes", "motors")) == counts


@pytest.mark.integration
def test_real_mujoco_output_limits_and_loop_closure(tmp_path, real_model):
    checked = real_model("walker")
    result = evaluate(checked, tmp_path / "evaluation",
                      EvaluationOptions(duration=20, settle=0.02, timestep=0.002, anchor=True))
    assert result["status"] == "evaluated"
    assert result["regime"] == "anchored-mechanism"
    assert result["metrics"]["peakActuatorForceNm"] <= 0.4 + 1e-9
    assert result["metrics"]["maxClosureErrorMeters"] <= 0.002
    assert result["metrics"]["forwardMeters"] == pytest.approx(0, abs=1e-9)
    assert abs(result["metrics"]["motorTurns"][0]) >= 1
    assert not result["metrics"]["eligibleForRanking"]
    assert result["collision"]["mode"] == "part-hulls-and-axle-rods"
    assert result["collision"]["partGeometries"] == len(checked.manifest["parts"])
    assert result["collision"]["connectorGeometries"] > 0
    assert result["collision"]["maxSelfPenetrationMeters"] <= result["collision"]["penetrationToleranceMeters"]
    assert result["collision"]["fullMotorCycle"]
    assert (tmp_path / "evaluation" / "model.xml").is_file()
    assert (tmp_path / "evaluation" / "trajectory.json").is_file()
    assert (tmp_path / "evaluation" / "result.json").is_file()
    assert (tmp_path / "evaluation" / "collision-model.json").is_file()


@pytest.mark.integration
def test_real_long_axles_cannot_pass_through_couplers(tmp_path, real_model):
    from lego_technic_sim.ldraw.parser import LDrawParser
    from legowalkers_sim.collision import prepare_parts

    library, _ = integration_paths()
    checked = copy.copy(real_model("walker"))
    checked.manifest = copy.deepcopy(checked.manifest)
    checked.build = copy.copy(checked.build)
    checked.build.parts = list(checked.build.parts)
    # Reproduce the original projecting 12L shaft at the first leg station.
    index = next(i for i, part in enumerate(checked.manifest["parts"]) if part["id"] == "axle-2")
    source = checked.manifest["parts"][index]
    source["part"] = "3708.dat"
    values = [*source["position"], *np.array(source["rotation"]).reshape(-1)]
    model_path = tmp_path / "long-axle.ldr"
    model_path.write_text("1 8 " + " ".join(map(str, values)) + " 3708.dat\n", encoding="utf-8")
    checked.build.parts[index] = LDrawParser(parts_dir=library).parse_build(model_path).parts[0]
    # Reusing checked structural units is deliberate: only the omitted connector changes.
    prepare_parts(checked)
    result = evaluate(checked, tmp_path / "long-axle", EvaluationOptions(duration=0.05, settle=0, anchor=True))
    assert result["status"] == "interference"
    assert not result["metrics"]["eligibleForRanking"]
    assert any("axle-2" in contact["parts"]
               and any(part.startswith("leg-0-") and part.endswith("coupler-part") for part in contact["parts"])
               and contact["depthMeters"] > result["collision"]["penetrationToleranceMeters"]
               for contact in result["collision"]["selfContacts"])
