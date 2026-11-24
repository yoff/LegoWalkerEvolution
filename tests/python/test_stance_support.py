import json
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError, write_json
from legowalkers_sim.screening_reports import motion_repair_hints
from legowalkers_sim.stance_support import (
    VERSION, StanceSupport, compare_shift, contact_hull, outside_distance, repair_mass_layout,
)
from test_simulation_gate import rig, options, fault


SQUARE = [[0., 0.], [1., 0.], [1., 1.], [0., 1.]]


def test_contact_area_not_foot_centers_and_exact_outside_distance():
    hull = contact_hull([SQUARE[2], SQUARE[0], SQUARE[3], SQUARE[1], SQUARE[0]])
    assert outside_distance([.5, .5], hull) == 0
    assert outside_distance([1, .5], hull) == 0
    assert outside_distance([1.3, .5], hull) == pytest.approx(.3)
    assert outside_distance([1.3, 1.4], hull) == pytest.approx(.5)
    assert outside_distance([.5, .5], []) is None
    assert outside_distance([1, 1], [[0, 0]]) == pytest.approx(np.sqrt(2))
    line = contact_hull([[0, 0], [1, 0], [.5, 0]])
    assert outside_distance([.5, .2], line) == pytest.approx(.2)
    assert outside_distance([.5, 0], line) == 0


def test_measured_duration_fraction_longest_interval_and_flight_are_distinct():
    monitor = StanceSupport(1.)
    monitor.observe(1.1, [.5, .5, 3], SQUARE)
    monitor.observe(1.3, [1.3, .5, 3], SQUARE)
    monitor.observe(1.6, [1.4, .5, 3], SQUARE)
    monitor.observe(1.7, [.5, .5, 3], SQUARE)
    monitor.observe(2., [.5, .5, 3], [])
    report = monitor.report()
    assert report["measurementSeconds"] == pytest.approx(1)
    assert report["unsupportedFraction"] == pytest.approx(.8)
    assert report["longestUnsupportedSeconds"] == pytest.approx(.5)
    assert report["noSupportSeconds"] == pytest.approx(.3)
    assert report["maximumOutsideDistanceMeters"] == pytest.approx(.4)
    assert report["outsideDistanceTimeMetersSeconds"] == pytest.approx(.18)
    assert report["qualificationGate"] is False
    with pytest.raises(AdapterError, match="increasing"):
        monitor.observe(2., [.5, .5, 3], SQUARE)


def test_unmeasured_and_flight_only_do_not_invent_zero_outside_distance():
    monitor = StanceSupport(1.)
    assert monitor.report()["status"] == "not-measured"
    assert monitor.report()["unsupportedFraction"] is None
    monitor.observe(2., [0, 0, 1], [])
    report = monitor.report()
    assert report["unsupportedFraction"] == 1
    assert report["maximumOutsideDistanceMeters"] is None
    with pytest.raises(AdapterError, match="finite"):
        monitor.observe(3., [float("nan"), 0, 0], SQUARE)


@pytest.mark.parametrize("points", [None, 1, [1, 2], [[0, float("inf")]], [["bad", 0]]])
def test_invalid_contact_geometry_is_explicitly_rejected(points):
    with pytest.raises(AdapterError):
        contact_hull(points)


def test_degenerate_or_reordered_recorded_hulls_cannot_produce_nan_distances():
    assert outside_distance([.5, 1], [[0, 0], [1, 0], [1, 0], [0, 0]]) == pytest.approx(1)
    assert outside_distance([.5, .5], list(reversed(SQUARE))) == 0


def trace():
    return {
        "version": VERSION, "startTime": 0., "ldrawSha256": "fixture",
        "frames": [{"time": .1, "centerOfMassMeters": [1.1, .5, 1.],
                    "supportPolygonXYMeters": SQUARE, "chassisRotation": np.eye(3).tolist()}],
    }


def test_shift_comparison_uses_recorded_chassis_rotation_and_never_changes_support():
    data = trace()
    before = json.dumps(data)
    assert compare_shift(data, [0, 0, 0])["unsupportedFraction"] == 1
    assert compare_shift(data, [-500, 0, 0])["unsupportedFraction"] == 0
    assert json.dumps(data) == before
    data["frames"][0]["chassisRotation"] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    assert compare_shift(data, [-500, 0, 0])["unsupportedFraction"] == 1
    assert compare_shift(data, [0, 0, -500])["unsupportedFraction"] == 0


@pytest.mark.parametrize("field,value", [
    ("chassisRotation", [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
    ("chassisRotation", [[1, 0, 0], [0, 1, 0], [0, 0, 2]]),
    ("centerOfMassMeters", [float("nan"), 0, 0]),
    ("centerOfMassMeters", [0]),
    ("supportPolygonXYMeters", [[0, float("nan")]]),
    ("time", 0),
])
def test_corrupt_recorded_stance_samples_fail_explicitly(field, value):
    data = trace()
    data["frames"][0][field] = value
    with pytest.raises(AdapterError):
        compare_shift(data, [0, 0, 0])


def test_full_and_prefix_results_persist_diagnostics_without_altering_gates_or_state(rig, tmp_path):
    from legowalkers_sim import evaluation
    baseline = evaluation.evaluate(rig.checked, tmp_path / "baseline", options(short_gate=0))
    measured = evaluation.evaluate(rig.checked, tmp_path / "gated", options())
    assert baseline["stanceSupport"] == measured["stanceSupport"]
    assert baseline["metrics"] == measured["metrics"]
    assert measured["stanceSupport"]["unsupportedFraction"] == 1  # Fixture deliberately has no ground contacts.
    assert not any("support" in reason for reason in measured["metrics"]["rankingRejectionReasons"])
    assert (tmp_path / "baseline/stance-support.json").read_bytes() == (tmp_path / "gated/stance-support.json").read_bytes()
    assert rig.counts == {"prepare": 2, "load": 2, "data": 2, "step": 60}
    saved = json.loads((tmp_path / "gated/stance-support.json").read_text())
    center = saved["frames"][-1]["centerOfMassMeters"]
    np.testing.assert_array_equal(center, rig.data[-1].subtree_com[1])
    assert np.linalg.norm(np.asarray(center)-rig.data[-1].xipos[1]) > .0001
    rig.fault = fault("closure", lambda tick: tick == 6)
    with pytest.raises(AdapterError, match="joint-closure-error"):
        evaluation.evaluate(rig.checked, tmp_path / "prefix", options())
    prefix = json.loads((tmp_path / "prefix/simulation-gate.json").read_text())
    partial = json.loads((tmp_path / "prefix/stance-support.json").read_text())
    assert prefix["diagnostics"]["stanceSupport"] == partial["summary"]
    assert partial["summary"]["measurementSeconds"] == pytest.approx(.002)
    assert partial["frames"][-1]["time"] == pytest.approx(.006)
    assert not (tmp_path / "prefix/result.json").exists()


def test_anchored_previews_do_not_report_free_walking_instability(rig, tmp_path):
    from legowalkers_sim import evaluation
    result = evaluation.evaluate(rig.checked, tmp_path / "anchored", options(anchor=True, short_gate=0))
    report = result["stanceSupport"]
    assert report["status"] == "not-applicable" and report["applicability"] == "anchored-chassis"
    assert report["unsupportedFraction"] is None
    assert json.loads((tmp_path / "anchored/stance-support.json").read_text())["frames"] == []


@pytest.mark.parametrize("loaded", [False, True])
def test_evaluation_uses_only_permitted_contacts_with_positive_normal_force(rig, tmp_path, monkeypatch, loaded):
    import mujoco
    import xml.etree.ElementTree as ET
    from legowalkers_sim import evaluation
    prepare = evaluation.prepare_xml
    contact_force = mujoco.mj_contactForce
    reactions = []

    def grounded(checked, output, options):
        xml, joints, feet, collision = prepare(checked, output, options)
        root = ET.parse(xml)
        root.find("option").set("gravity", "0 0 -9.81")
        ground = root.find(".//geom[@name='ground']")
        ground.set("contype", "1")
        ground.set("conaffinity", "1")
        body = root.find(".//body[@name='unit_0']")
        body.set("pos", "0 0 .009")
        body.find("site[@name='foot_0']").set("pos", "0 0 -.01")
        foot = body.find("geom[@name='frame']")
        foot.set("contype", "1")
        foot.set("conaffinity", "1")
        ET.SubElement(body, "geom", name="battery", type="sphere", size=".01", pos=".03 0 0", mass=".01")
        ET.SubElement(body, "geom", name="wheel", type="sphere", size=".01", pos="-.03 0 0", mass=".01")
        root.write(xml)
        collision["geometryInstances"].update(battery="payload-battery", wheel="wheel")
        return xml, joints, feet, collision

    def reaction(model, data, index, force):
        contact_force(model, data, index, force)
        contact = data.contact[index]
        names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(i))
                 for i in (contact.geom1, contact.geom2)}
        reactions.append((names, float(force[0])))
        if not loaded:
            force[:] = 0

    rig.checked.manifest["groundWheels"] = ["wheel"]
    monkeypatch.setattr(evaluation, "prepare_xml", grounded)
    monkeypatch.setattr(mujoco, "mj_contactForce", reaction)
    result = evaluation.evaluate(rig.checked, tmp_path / "floor", options(short_gate=0))
    frames = json.loads((tmp_path / "floor/stance-support.json").read_text())["frames"]
    assert reactions and any(force > 0 for _, force in reactions)
    assert all(names <= {"ground", "frame", "wheel"} for names, _ in reactions)
    assert any("frame" in names for names, _ in reactions) and any("wheel" in names for names, _ in reactions)
    assert result["groundContact"]["forbiddenContactDetected"]  # Battery remains a real collision.
    if loaded:
        assert any(frame["supportPolygonXYMeters"] for frame in frames)
        assert all(point[0] < .01 for frame in frames for point in frame["supportPolygonXYMeters"])
    else:
        assert all(frame["supportPolygonXYMeters"] == [] for frame in frames)
        assert result["stanceSupport"]["noSupportSeconds"] == pytest.approx(.026)


def test_feedback_requires_ground_failure_and_new_measured_support_evidence():
    report = {"metrics": {"rankingRejectionReasons": ["forbidden-ground-contact"]},
              "groundContact": {"forbiddenContactDetected": True},
              "stanceSupport": {"version": VERSION, "status": "measured", "unsupportedFraction": .2}}
    hints = motion_repair_hints(report)
    assert hints["kind"] == "stance-support-mass-layout"
    assert motion_repair_hints({**report, "stanceSupport": {}}) is None
    assert motion_repair_hints({**report, "groundContact": {}}) is None
    report["stanceSupport"]["unsupportedFraction"] = 0
    assert motion_repair_hints(report) is None


def test_gallery_renders_optional_diagnostics_without_relabeling_qualification():
    from legowalkers_sim.gallery import metrics
    from test_evolution import result
    report = result()
    historical = metrics(report)
    assert "Stance support" not in historical
    before = json.dumps(report)
    report["stanceSupport"] = compare_shift(trace(), [0, 0, 0])
    panel = metrics(report)
    assert "Stance support (diagnostic only)" in panel
    assert "100.00%" in panel and "100.000 mm" in panel
    report["stanceSupport"]["maximumOutsideDistanceMeters"] = None
    assert "undefined (no loaded contact)" in metrics(report)
    report.pop("stanceSupport")
    assert json.dumps(report) == before


@pytest.mark.parametrize("with_edit", [True, False])
@pytest.mark.parametrize("hardware,mass", [("battery", .5), ("receiver", .25)])
def test_repairs_select_existing_hardware_edits_without_trials(tmp_path, monkeypatch, with_edit, hardware, mass):
    from legowalkers_sim.evolution import Generator
    archive = tmp_path / "archive"
    evidence = archive / "evaluations/0001"
    feedback = archive / "construction-feedback/0001.json"
    write_json(evidence / "stance-support.json", trace())
    write_json(evidence / "mass-layout.json", {
        "totalMassKg": 1., "components": [{"partInstance": f"payload-{hardware}", "massKg": mass}]})
    hints = {"kind": "stance-support-mass-layout", "supportTrace": "evaluations/0001/stance-support.json"}
    write_json(feedback, hints)
    parent = tmp_path / "parent"
    parent.mkdir()
    manifest = {"ldrawSha256": "fixture", "motion": {"genotypeId": "parent", "mechanismId": "mechanism"}}
    monkeypatch.setattr("legowalkers_sim.bridge.load_manifest", lambda _: manifest)
    called = []

    def apply(parent, output, edit, limit):
        called.append((edit, limit))
        return {"operation": f"{hardware}-layout", "generated": [{"directory": "candidate-001"}],
                "counts": {"dynamicTrials": 0}}

    edits = [
        {"id": "worse", "operation": f"{hardware}-layout", "motionUnchanged": True,
         "changed": [{"path": f"hardware/{hardware}/position", "before": [0, 0, 0], "after": [1000, 0, 0]}]},
        {"id": "better", "operation": f"{hardware}-layout", "motionUnchanged": True,
         "changed": [{"path": f"hardware/{hardware}/position", "before": [0, 0, 0], "after": [-1000, 0, 0]}]},
        {"id": "changes-gait", "operation": f"{hardware}-layout", "motionUnchanged": False, "changed": []},
    ] if with_edit else []
    generator = SimpleNamespace(manual_edits=lambda _: {"edits": edits}, apply_edit=apply)
    result = Generator.repair_native(generator, parent, tmp_path / "output", feedback, 1)
    assert result["operation"] == "repair-realization"
    assert result["counts"]["dynamicTrials"] == 0
    if with_edit:
        assert called == [("better", 1)]
        assert result["stanceGuidance"]["before"]["unsupportedFraction"] == 1
        assert result["stanceGuidance"]["frozenMotionAfter"]["unsupportedFraction"] == 0
        assert result["stanceGuidance"]["estimatedComShiftLdu"] == [-1000*mass, 0, 0]
    else:
        assert not called and result["generated"] == []
        assert result["gates"][0]["code"] == "no-supported-improving-mass-layout"
    with pytest.raises(AdapterError, match="inside"):
        repair_mass_layout(generator, parent, tmp_path / "output", feedback,
                           {**hints, "supportTrace": "../../outside.json"}, 1)
    manifest["ldrawSha256"] = "different-model"
    with pytest.raises(AdapterError, match="physical model"):
        repair_mass_layout(generator, parent, tmp_path / "output", feedback, hints, 1)
