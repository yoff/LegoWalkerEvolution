import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim import support_feasibility as support


def fixture(positions=None):
    positions = positions or [[-100, 100, -60], [100, 100, -60],
                              [-100, 100, 60], [100, 100, 60]]
    feet = [{"id": f"toe-{i}", "logicalFoot": f"leg-{i}", "body": f"leg-{i}",
             "partInstance": f"part-{i}", "position": point, "contactRadiusMeters": .004}
            for i, point in enumerate(positions)]
    parts = [{"id": "chassis", "body": "chassis"}]+[
        {"id": foot["partInstance"], "body": foot["body"]} for foot in feet]
    bodies = [{"id": part["body"], "initial": {"origin": {"x": 0, "y": 0, "z": 0}, "angle": 0}}
              for part in parts]
    poses = [{"body": body["id"], "origin": [0, 0, 0], "angle": 0} for body in bodies]
    manifest = {"feet": feet, "parts": parts}
    motion = {"program": {"bodies": bodies},
              "frames": [{"inputAngle": angle, "poses": copy.deepcopy(poses)} for angle in (0, 2*math.pi)]}
    vertices = {"chassis": [[-50, 0, -30], [50, 0, 30]]}
    vertices.update({foot["partInstance"]: [foot["position"]] for foot in feet})
    return manifest, motion, vertices


def test_equal_height_feet_are_potential_contacts_not_walking_qualification():
    result = support.analyze(*fixture())
    assert result["maximumUprightLogicalContacts"] == 4
    assert result["uprightClearFrames"] == result["uprightTwoFootFrames"] == 2
    assert result["alerts"] == []
    assert result["motorTurns"] == pytest.approx(1)
    assert result["simulatedSteps"] == result["campaignTrials"] == 0
    assert result["liveGateChanged"] is False
    assert "not-balance" in result["mode"]


def test_high_foot_shortfall_is_measured_in_meters_without_requiring_all_feet():
    manifest, motion, vertices = fixture()
    manifest["feet"][0]["position"][1] = 20
    result = support.analyze(manifest, motion, vertices)
    assert result["maximumUprightLogicalContacts"] == 3
    assert result["feet"][0]["uprightMinimumShortfallMeters"] == pytest.approx(.032)
    assert result["feet"][0]["uprightClearContactSamples"] == 0
    assert any("leg-0" in alert for alert in result["alerts"])
    assert "no-sampled-two-foot-ground-pose" not in result["alerts"]


def test_swing_foot_can_reach_in_a_later_phase():
    manifest, motion, vertices = fixture()
    motion["frames"][0]["poses"][1]["origin"][1] = -20
    motion["frames"][1]["poses"][1]["origin"][1] = 20
    result = support.analyze(manifest, motion, vertices)
    assert result["feet"][0]["uprightClearContactSamples"] >= 1
    assert result["feet"][0]["uprightMinimumShortfallMeters"] == 0
    assert not any("never-reaches-clear-ground:leg-0" in alert for alert in result["alerts"])


def test_physical_geometry_uses_first_solved_pose_not_program_initial_guess():
    manifest, motion, vertices = fixture()
    for body in motion["program"]["bodies"]:
        body["initial"]["angle"] = 1.2
        body["initial"]["origin"]["x"] = 300
    assert support.analyze(manifest, motion, vertices)["alerts"] == []


def test_moving_mesh_and_foot_follow_the_same_rigid_transform_and_preserve_depth():
    manifest, motion, vertices = fixture()
    manifest["parts"].append({"id": "heel", "body": "leg-0"})
    vertices["heel"] = [[-100, 130, -60]]
    before = support.analyze(manifest, motion, vertices)
    assert before["uprightClearFrames"] == 0
    assert before["uprightFrames"][0]["limitingPart"] == "heel"
    assert before["uprightFrames"][0]["forbiddenClearanceMeters"] == 0
    motion["frames"][1]["poses"][1]["angle"] = math.pi
    after = support.analyze(manifest, motion, vertices)
    assert after["uprightClearFrames"] == 1
    assert after["uprightFrames"][1]["potentialLogicalContacts"] == ["leg-1", "leg-2", "leg-3"]


def test_forbidden_body_can_rule_out_every_sampled_ground_pose():
    manifest, motion, vertices = fixture()
    vertices["chassis"] = [[0, 1000, 0]]
    result = support.analyze(manifest, motion, vertices)
    assert "no-sampled-clear-ground-pose" in result["alerts"]
    assert "no-sampled-two-foot-ground-pose" in result["alerts"]
    assert result["optimisticPoseWitness"] is None


def test_contact_radius_cannot_hide_a_chassis_below_the_real_foot_mesh():
    manifest, motion, vertices = fixture()
    vertices["chassis"] = [[0, 105, 0]]
    result = support.analyze(manifest, motion, vertices)
    assert result["uprightClearFrames"] == 0
    assert result["uprightFrames"][0]["forbiddenClearanceMeters"] == 0
    assert result["feet"][0]["uprightMinimumShortfallMeters"] == pytest.approx(.002)


def test_contact_tolerance_boundary_is_measurable():
    manifest, motion, vertices = fixture()
    manifest["feet"][0]["position"][1] -= 2
    assert support.analyze(manifest, motion, vertices, contact_tolerance_meters=.00079)[
        "feet"][0]["uprightClearContactSamples"] == 0
    assert support.analyze(manifest, motion, vertices, contact_tolerance_meters=.00081)[
        "feet"][0]["uprightClearContactSamples"] == 2


def test_leaning_is_explored_but_not_confused_with_upright_support():
    positions = [[-100, 100-200*math.tan(math.pi/12), 0], [100, 100, 0]]
    result = support.analyze(*fixture(positions))
    assert result["maximumUprightLogicalContacts"] == 1
    assert result["maximumTiltedLogicalContacts"] == 2
    assert "sampled-two-foot-support-requires-leaning" in result["alerts"]
    assert result["optimisticPoseWitness"]["pitchDegrees"] != 0
    normals = support.ground_normals()
    assert normals[0]["normal"] == [0, 1, 0]
    assert all(np.linalg.norm(item["normal"]) == pytest.approx(1) and item["normal"][1] >= .5-1e-12
               for item in normals)


def test_capsule_interior_is_allowed_and_two_regions_are_one_logical_foot():
    manifest, motion, vertices = fixture([[0, 100, 0], [150, 105, 0]])
    first = manifest["feet"].pop(0)
    manifest["feet"] = [{**first, "id": "a", "position": [0, 100, -50]},
                        {**first, "id": "b", "position": [0, 100, 50]}]+manifest["feet"]
    manifest["motion"] = {"logicalFeet": [
        {"id": "leg-0", "body": "leg-0", "contactShape": "capsule", "regions": ["a", "b"]}]}
    vertices["part-0"] = [[0, 100, 0], [0, 105, 0]]
    result = support.analyze(manifest, motion, vertices)
    assert result["maximumUprightLogicalContacts"] == 2
    assert result["uprightClearFrames"] == 2
    assert len(result["feet"]) == 2


def test_unowned_connectors_are_explicitly_omitted_not_assigned_invented_motion():
    manifest, motion, vertices = fixture()
    manifest["parts"].append({"id": "unowned-pin", "body": None})
    vertices["unowned-pin"] = [[0, 1000, 0]]
    result = support.analyze(manifest, motion, vertices)
    assert result["omittedUnownedConnectors"] == ["unowned-pin"]
    assert result["uprightClearFrames"] == 2


def test_rectangle_interior_uses_existing_bounded_surface_predicate():
    manifest, motion, vertices = fixture([[0, 100, 0], [150, 105, 0]])
    first = manifest["feet"].pop(0)
    anchors = [[-40, 100, -10], [40, 100, -10], [-40, 100, 10], [40, 100, 10]]
    manifest["feet"] = [{**first, "id": f"corner-{i}", "position": point}
                        for i, point in enumerate(anchors)]+manifest["feet"]
    manifest["motion"] = {"logicalFeet": [
        {"id": "leg-0", "body": "leg-0", "contactShape": "rectangle",
         "regions": [f"corner-{i}" for i in range(4)]}]}
    vertices["part-0"] = [[0, 105, 0]]
    result = support.analyze(manifest, motion, vertices)
    assert result["uprightClearFrames"] == 2
    assert result["maximumUprightLogicalContacts"] == 2


def test_stock_meshes_match_ldraw_placements_and_refresh_when_library_changes(tmp_path):
    library, source = tmp_path / "ldraw", tmp_path / "candidate"
    (library / "parts").mkdir(parents=True)
    (library / "p").mkdir()
    source.mkdir()
    stock = library / "parts" / "beam.dat"
    stock.write_text("3 16 0 0 0 0 10 0 10 0 0\n")
    (source / "model.ldr").write_text("1 7 20 30 40 0 -1 0 1 0 0 0 0 1 beam.dat\n")
    manifest = {"modelFile": "model.ldr", "parts": [
        {"id": "beam", "part": "beam.dat", "position": [20, 30, 40],
         "rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}]}
    cache, meshes = support.PartParserCache(), {}
    first = support._meshes(manifest, source, library, cache, meshes)["beam"]
    assert sorted(map(tuple, first)) == [(10, 30, 40), (20, 30, 40), (20, 40, 40)]
    assert len(meshes) == 1
    support._meshes(manifest, source, library, cache, meshes)
    assert len(meshes) == 1
    stock.write_text("3 16 0 0 0 0 20 0 20 0 0\n")
    second = support._meshes(manifest, source, library, cache, meshes)["beam"]
    assert len(meshes) == 2
    assert sorted(map(tuple, second)) == [(0, 30, 40), (20, 30, 40), (20, 50, 40)]
    manifest["parts"][0]["position"][0] += 1
    with pytest.raises(AdapterError, match="differs from manifest"):
        support._meshes(manifest, source, library, cache, meshes)
    manifest["parts"][0]["position"][0] -= 1
    stock.write_text("0 No geometry\n")
    with pytest.raises(AdapterError, match="finite stock mesh"):
        support._meshes(manifest, source, library, cache, meshes)


@pytest.mark.parametrize("kind", ["pose", "mesh", "radius", "missing-body", "tolerance", "frames"])
def test_invalid_inputs_fail_explicitly(kind):
    manifest, motion, vertices = fixture()
    kwargs = {}
    if kind == "pose":
        motion["frames"][0]["poses"][0]["angle"] = float("nan")
    elif kind == "mesh":
        vertices["chassis"][0][0] = float("inf")
    elif kind == "radius":
        manifest["feet"][0]["contactRadiusMeters"] = -1
    elif kind == "missing-body":
        motion["frames"][1]["poses"].pop()
    elif kind == "frames":
        motion["frames"].pop()
    else:
        kwargs["contact_tolerance_meters"] = -.1
    with pytest.raises(AdapterError, match="Support diagnostic"):
        support.analyze(manifest, motion, vertices, **kwargs)


def test_run_writes_evidence_without_touching_archive_and_reuses_identical_motion(tmp_path, monkeypatch):
    manifest, motion, vertices = fixture()
    archive = tmp_path / "campaign" / "versions" / "v0001"
    archive.mkdir(parents=True)
    (archive / "archive.sqlite3").write_bytes(b"untouched database")
    source = archive / "candidate"
    source.mkdir()
    for name, value in (("program.json", {"program": motion["program"]}), ("manifest.json", manifest)):
        (source / name).write_text(json.dumps(value))
    (source / "model.ldr").write_text("unchanged model")
    manifest["modelFile"] = "model.ldr"
    cases = [{"design": f"design-{i}", "source": source, "births": [
        {"id": i, "archive": str(archive)}], "measurements": [
            {"qualified": True, "status": "evaluated", "rejectionReasons": []}]} for i in (1, 2)]
    monkeypatch.setattr(support, "_cohort", lambda archives: copy.deepcopy(cases))
    monkeypatch.setattr(support, "load_manifest", lambda path: manifest)
    monkeypatch.setattr(support, "cycle_plan", lambda *args: {"motorTurnsPerCycle": 2})
    monkeypatch.setattr(support, "_meshes", lambda *args: vertices)
    calls = []

    def export(command, **kwargs):
        calls.append(command)
        target = Path(command[command.index("--output")+1])
        target.mkdir()
        generated = copy.deepcopy(motion)
        generated["frames"] = [copy.deepcopy(motion["frames"][0]) for _ in range(145)]
        for i, frame in enumerate(generated["frames"]):
            frame["inputAngle"] = i*math.pi/36
        (target / "motion.json").write_text(json.dumps(generated))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(support.subprocess, "run", export)
    output = tmp_path / "report"
    executable = tmp_path / "generator"
    executable.touch()
    before = {path: path.read_bytes() for path in archive.rglob("*") if path.is_file()}
    result = support.run([archive], output, tmp_path, executable, executable)
    assert len(calls) == 1
    assert calls[0][-2:] == ["--turns", "2"]
    assert result["summary"]["qualifiedControls"] == 2
    assert result["summary"]["qualifiedControlsWithWarnings"] == 0
    assert (output / "support-feasibility.html").is_file()
    assert json.loads((output / "support-feasibility.json").read_text())["summary"]["campaignTrials"] == 0
    assert not (output / "progress.json").exists()
    assert {path: path.read_bytes() for path in archive.rglob("*") if path.is_file()} == before
    with pytest.raises(AdapterError, match="fresh"):
        support.run([archive], output, tmp_path, executable, executable)
    with pytest.raises(AdapterError, match="outside"):
        support.run([archive], archive.parent / "diagnostic", tmp_path, executable, executable)
