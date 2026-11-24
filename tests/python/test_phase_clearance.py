import math
from types import SimpleNamespace
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.phase_clearance import evaluate_phases


def fixture(tmp_path, *, turns=1, clear=False):
    model = tmp_path / "model.xml"
    model.write_text(f"""
    <mujoco><worldbody>
      <geom name="ground" type="plane" size="1 1 .1"/>
      <body name="unit_0" pos=".03 .04 .05"><freejoint/>
        <geom name="bracket" type="sphere" size=".003" pos="0 0 {'-.05' if clear else '-.02'}"/>
        <body name="unit_1" pos=".02 0 0">
          <joint name="joint_0" axis="0 -1 0" pos="-.02 0 0"/>
          <geom name="shaft" type="sphere" size=".003"/>
        </body>
      </body>
    </worldbody><contact><pair geom1="bracket" geom2="shaft"/></contact></mujoco>
    """)
    manifest = {"chassis": "chassis", "motion": {
        "mechanismId": "motion", "genotypeId": "genotype", "assemblyId": "assembly"},
        "parts": [{"id": "receiver-bracket"}, {"id": "protruding-axle"}]}
    checked = SimpleNamespace(manifest=manifest, body_indices={"chassis": 0, "link": 1})
    collision = {"unitCentersMeters": {"0": [0., 0., 0.], "1": [.02, 0., 0.]},
                 "geometryInstances": {"bracket": "receiver-bracket", "shaft": "protruding-axle"}}
    document = {
        "version": "prescribed-motion-phases-v1", **manifest["motion"],
        "parts": ["receiver-bracket", "protruding-axle"], "endpointRepeats": True, "samplingFailure": None,
        "frames": [{"inputAngleRadians": float(angle), "bodies": [
            {"id": "chassis", "originLdu": [0., 0., 0.], "angleRadians": 0.},
            {"id": "link", "originLdu": [0., 0., 0.], "angleRadians": float(angle / turns)}]}
            for angle in np.linspace(0, turns * 2 * math.pi, turns * 72 + 1)],
    }
    return checked, model, collision, document


def test_complete_assembly_detects_connector_bracket_clash_away_from_start_without_dynamics(tmp_path):
    checked, model, collision, document = fixture(tmp_path)
    report = evaluate_phases(checked, model, collision, document, tolerance=.0001)
    assert report["status"] == "collision"
    assert report["initialGeometryMatches"] is True
    assert report["sampledPoses"] == 73
    assert report["contacts"][0]["parts"] == ["protruding-axle", "receiver-bracket"]
    assert report["contacts"][0]["inputAngleRadians"] == pytest.approx(math.pi / 2)
    assert report["simulationSeconds"] == report["dynamicTrials"] == 0


def test_full_geared_gait_is_checked_beyond_first_motor_turn(tmp_path):
    checked, model, collision, document = fixture(tmp_path, turns=5)
    report = evaluate_phases(checked, model, collision, document, tolerance=.0001)
    assert report["motorTurnsCovered"] == 5
    assert report["sampledPoses"] == 361
    assert report["status"] == "collision"
    assert report["contacts"][0]["inputAngleRadians"] > 2 * math.pi


def test_nonzero_initial_body_angle_rotates_about_the_exported_origin_not_its_com(tmp_path):
    checked, model, collision, document = fixture(tmp_path)
    offset = math.pi / 4
    position = [.02 * math.cos(offset), 0., -.02 * math.sin(offset)]
    xml = ET.parse(model)
    body = xml.getroot().find(".//body[@name='unit_1']")
    body.set("pos", " ".join(map(str, position)))
    body.set("quat", f"{math.cos(offset / 2)} 0 {math.sin(offset / 2)} 0")
    xml.write(model)
    collision["unitCentersMeters"]["1"] = position
    for frame in document["frames"]:
        frame["bodies"][1]["angleRadians"] += offset
    report = evaluate_phases(checked, model, collision, document, tolerance=.0001)
    assert report["initialGeometryMatches"]
    assert report["status"] == "collision"
    assert report["contacts"][0]["inputAngleRadians"] == pytest.approx(math.pi / 4)


def test_clear_cycle_is_not_presented_as_continuous_or_walking_certification(tmp_path):
    checked, model, collision, document = fixture(tmp_path, clear=True)
    report = evaluate_phases(checked, model, collision, document, tolerance=.0001)
    assert report["status"] == "clear-at-sampled-phases"
    assert report["contacts"] == []
    assert report["partCount"] == 2
    assert any("between-sample" in limitation for limitation in report["limitations"])
    assert report["simulationSeconds"] == report["dynamicTrials"] == 0


@pytest.mark.parametrize("failure", ["sampling", "nonrepeating"])
def test_incomplete_coverage_never_becomes_a_clear_result(tmp_path, failure):
    checked, model, collision, document = fixture(tmp_path)
    if failure == "sampling":
        document["samplingFailure"] = "bounded sampling failed"
    else:
        document["frames"][-1]["bodies"][1]["angleRadians"] += .1
    report = evaluate_phases(checked, model, collision, document, tolerance=.0001)
    assert report["status"] == "incomplete"
    assert report["sampledPoses"] == 0


@pytest.mark.parametrize("failure", ["parts", "identity", "bodies", "nan", "sparse", "centers", "geometry"])
def test_invalid_or_partial_prescribed_assemblies_fail_explicitly(tmp_path, failure):
    checked, model, collision, document = fixture(tmp_path)
    if failure == "parts":
        document["parts"].pop()
    elif failure == "identity":
        document["genotypeId"] = "different"
    elif failure == "bodies":
        document["frames"][1]["bodies"].pop()
    elif failure == "nan":
        document["frames"][1]["bodies"][0]["angleRadians"] = float("nan")
    elif failure == "sparse":
        document["frames"] = document["frames"][::3]
    elif failure == "centers":
        collision["unitCentersMeters"].pop("1")
    else:
        collision["geometryInstances"].pop("shaft")
    with pytest.raises(AdapterError):
        evaluate_phases(checked, model, collision, document, tolerance=.0001)
