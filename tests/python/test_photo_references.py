"""Isolated reference and exact single-leg regressions; no dynamics."""

import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def reference_output(tmp_path_factory):
    output = tmp_path_factory.mktemp("photo-references") / "generated"
    command = [str(ROOT / ".tools/dotnet/dotnet"), "fsi", str(ROOT / "scripts/photo_references.fsx"),
               "--output", str(output)]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    return output, command


def test_reference_graph_preserves_confirmed_structure_and_timing(reference_output):
    output, _ = reference_output
    summary = json.loads((output / "references.json").read_text())
    assert summary["dynamicTrials"] == 0
    assert [r["id"] for r in summary["references"]] == ["double-parallelogram"]
    for entry in summary["references"]:
        evidence = entry["verification"]
        assert evidence["sampledCrankTurns"] == 2 and evidence["frames"] == 145
        for field in ("maxConstraintResidualLdu", "maxCycleRepeatErrorLdu", "maxDiagonalPairErrorLdu",
                      "maxHalfTurnPairErrorLdu", "maxShoeAngleRadians", "maxUprightAngleRadians"):
            assert 0 <= evidence[field] < 1e-6
        assert len(evidence["feet"]) == 4
        assert all(foot["strideLdu"] > 90 and foot["liftLdu"] > 40 for foot in evidence["feet"])
        directory = output / entry["id"]
        program = json.loads((directory / "program.json").read_text())["program"]
        assert len(program["bodies"]) == 29 and len(program["hinges"]) == 40
        assert program["motor"] == {"body": "front-right-crank", "phase": 0}
        phases = {t["output"]: t["phase"] for t in program["transmissions"]}
        assert phases["rear-left-crank"] == 0
        assert phases["front-left-crank"] == phases["rear-right-crank"] == pytest.approx(3.141592653589793)
        assert all(t["ratio"] == 1 for t in program["transmissions"])
        for prefix in ("front-right", "front-left", "rear-right", "rear-left"):
            shared = [h for h in program["hinges"] if h["id"].startswith(prefix+"-shared-corner-")]
            assert len(shared) == 2
            assert {endpoint["body"] for h in shared for endpoint in (h["a"], h["b"])} == {
                prefix+"-lower-arm", prefix+"-driven-leg", prefix+"-upright-l"}


def test_layouts_are_explicitly_not_simulation_candidates(reference_output):
    output, _ = reference_output
    for directory in (output / "double-parallelogram",):
        reference = json.loads((directory / "reference.json").read_text())
        assert reference["nativeCandidate"] is False
        assert reference["dynamicTrials"] == reference["reconstructionChecks"] == 0
        assert len(reference["photoFiles"]) == 18
        assert len(reference["assumptions"]) >= 8
        assert not (directory / "manifest.json").exists()
        assert not (directory / "realization.json").exists()
        model = (directory / "layout-study.ldr").read_text()
        assert "NOT A BUILDABLE OR SIMULATION-READY MODEL" in model
        assert len([row for row in model.splitlines() if row.startswith("1 ")]) == len(reference["parts"])
        assert len({part["id"] for part in reference["parts"]}) == len(reference["parts"])
        assert (directory / "single-leg/program.json").is_file()
        assert (directory / "single-leg/preview.html").is_file()


def test_l_branch_points_away_from_chassis_without_mirroring_crank(reference_output):
    output, _ = reference_output
    program = json.loads((output / "double-parallelogram/single-leg/program.json").read_text())["program"]
    bodies = {body["id"]: body for body in program["bodies"]}
    sites = {site["id"]: site["position"] for site in bodies["upright-l"]["sites"]}
    assert sites["toe-link"] == {"x": 40, "y": 0, "z": 0}
    assert sites["top"] == {"x": 0, "y": -60, "z": 0}
    assert bodies["crank"]["initial"]["origin"] == {"x": -20, "y": 80, "z": 0}
    assert bodies["driven-leg"]["initial"]["origin"] == {"x": 0, "y": 0, "z": 0}
    assert bodies["parallel-leg"]["initial"]["origin"] == {"x": 40, "y": 0, "z": 0}
    assert bodies["shoe"]["initial"]["origin"] == pytest.approx({"x": 0, "y": 200, "z": 0}, abs=1e-9)
    hinges = {hinge["id"]: hinge for hinge in program["hinges"]}
    assert hinges["parallel-shoe"]["b"] == {"body": "shoe", "site": "right"}
    assert hinges["driven-shoe"]["b"] == {"body": "shoe", "site": "left"}


def test_chassis_rails_follow_brick_x_axis_not_liftarm_z_axis(reference_output):
    output, _ = reference_output
    reference = json.loads((output / "double-parallelogram/reference.json").read_text())
    rails = [part for part in reference["parts"] if part["id"].startswith("chassis-rail-")]
    assert len(rails) == 2
    for rail in rails:
        assert rail["part"] == "3703.dat"
        assert rail["rotation"] == {
            "x": {"x": 1, "y": 0, "z": 0},
            "y": {"x": 0, "y": 1, "z": 0},
            "z": {"x": 0, "y": 0, "z": 1},
        }
        assert rail["position"]["x"] == 140
        assert rail["position"]["y"] == 10
        assert abs(rail["position"]["z"]) == 60
    rows = (output / "double-parallelogram/layout-study.ldr").read_text().splitlines()
    rail_rows = [rows[index+1].split() for index, row in enumerate(rows) if row.startswith("0 // instance: chassis-rail-")]
    assert len(rail_rows) == 2
    for row in rail_rows:
        assert row[-1] == "3703.dat"
        assert [float(value) for value in row[5:14]] == [1, 0, 0, 0, 1, 0, 0, 0, 1]


def test_reference_generation_is_byte_deterministic(reference_output, tmp_path):
    output, command = reference_output
    replay = tmp_path / "replay"
    subprocess.run([*command[:-1], str(replay)], cwd=ROOT, check=True, capture_output=True, text=True)

    def hashes(directory):
        return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in directory.rglob("*") if p.is_file()}

    assert hashes(output) == hashes(replay)


def test_reference_generator_refuses_to_overwrite_evidence(reference_output):
    output, command = reference_output
    original = (output / "references.json").read_bytes()
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert "Output must be empty" in result.stderr
    assert (output / "references.json").read_bytes() == original


@pytest.fixture(scope="module")
def physical_leg(reference_output, tmp_path_factory):
    reference, _ = reference_output
    output = tmp_path_factory.mktemp("photo-leg") / "generated"
    command = [str(ROOT / ".tools/dotnet/dotnet"), "fsi", str(ROOT / "scripts/photo_leg_fixture.fsx"),
               "--program", str(reference / "double-parallelogram/single-leg/program.json"),
               "--output", str(output)]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    return output, command


def fixture_checker():
    spec = importlib.util.spec_from_file_location("check_photo_leg_fixture", ROOT / "scripts/check_photo_leg_fixture.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_physical_leg_reconstructs_and_loads_without_stepping(physical_leg, tmp_path, monkeypatch):
    import mujoco

    def forbid_step(*args, **kwargs):
        pytest.fail("The photo fixture must not perform dynamics.")

    monkeypatch.setattr(mujoco, "mj_step", forbid_step)
    monkeypatch.setattr(mujoco, "mj_step1", forbid_step)
    monkeypatch.setattr(mujoco, "mj_step2", forbid_step)
    source, _ = physical_leg
    report = fixture_checker().verify(source, tmp_path / "checks", ROOT / ".tools/ldraw")
    assert report["status"] == "pass"
    assert report["dynamicTrials"] == report["simulationSeconds"] == 0
    assert report["reconstruction"]["parts"] == 23
    assert report["reconstruction"]["bodies"] == 8
    assert report["reconstruction"]["joints"] == 10
    assert report["reconstruction"]["motors"] == 1
    assert report["reconstruction"]["bodyIndices"]["chassis"] == 0
    assert report["loadOnly"]["initialSelfInterference"] == []
    assert report["loadOnly"]["maxClosureErrorMeters"] < 1e-6
    contacts = report["connectorEngagement"]
    assert len(contacts) == 12
    assert all(item["fullSocketWalls"] and item["droppedContacts"] == 0 for item in contacts)
    shared = next(item for item in contacts if item["connector"] == "pivot-shared-corner-a")
    assert set(shared["retainedParts"]) == {"lower-arm-part", "driven-leg-part", "upright-l-part"}
    assert shared["shaftLengthLdu"] == 60
    fixture = json.loads((source / "fixture.json").read_text())
    assert fixture["nativeCandidate"] is False
    assert fixture["recipe"] == "photo-double-parallelogram-leg-fixture-v4"
    assert fixture["sampledFrames"] == 145 and fixture["maxKinematicResidualLdu"] < 1e-6
    manifest = json.loads((source / "manifest.json").read_text())
    parts = {part["id"]: part for part in manifest["parts"]}
    assert parts["upright-l-part"]["rotation"] == [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
    assert parts["parallel-leg-part"]["position"][0] == pytest.approx(40, abs=1e-9)
    assert parts["shoe-part"]["position"][0] == 20
    assert parts["pivot-bearing"]["part"] == "44294.dat"
    assert parts["pivot-bearing"]["position"] == [-20, 80, -59.5]
    axle = next(item for item in contacts if item["connector"] == "pivot-bearing")
    assert axle["shaftLengthLdu"] == 139
    assert max(axle["shaftStartLdu"][2], axle["shaftEndLdu"][2]) == 10


def test_physical_leg_does_not_accept_a_disconnected_shared_pin(physical_leg, tmp_path):
    from legowalkers_sim.bridge import AdapterError

    original, _ = physical_leg
    source = tmp_path / "broken"
    shutil.copytree(original, source)
    manifest = json.loads((source / "manifest.json").read_text())
    index = next(i for i, part in enumerate(manifest["parts"]) if part["id"] == "pivot-shared-corner-a")
    manifest["parts"][index]["position"][2] = 120
    lines = (source / "model.ldr").read_text().splitlines(keepends=True)
    geometry_lines = [i for i, line in enumerate(lines) if line.startswith("1 ")]
    fields = lines[geometry_lines[index]].split()
    fields[4] = "120"
    lines[geometry_lines[index]] = " ".join(fields) + "\n"
    model = "".join(lines).encode()
    (source / "model.ldr").write_bytes(model)
    manifest["ldrawSha256"] = hashlib.sha256(model).hexdigest()
    (source / "manifest.json").write_text(json.dumps(manifest))
    output = tmp_path / "rejected"
    with pytest.raises(AdapterError):
        fixture_checker().verify(source, output, ROOT / ".tools/ldraw")
    report = json.loads((output / "check.json").read_text())
    assert report["status"] == "rejected" and report["dynamicTrials"] == 0


def test_physical_fixture_is_deterministic_and_refuses_overwrite(physical_leg, tmp_path):
    source, command = physical_leg
    replay = tmp_path / "replayed"
    subprocess.run([*command[:-1], str(replay)], cwd=ROOT, check=True, capture_output=True, text=True)
    assert {path.name: path.read_bytes() for path in source.iterdir()} == {
        path.name: path.read_bytes() for path in replay.iterdir()}
    original = (source / "manifest.json").read_bytes()
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0 and "Fixture output must be empty" in result.stderr
    assert (source / "manifest.json").read_bytes() == original
