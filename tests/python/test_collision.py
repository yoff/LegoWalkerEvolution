import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from lego_technic_sim.ldraw.model import LDrawBuild, LDrawPart, Triangle
from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType
from lego_technic_sim.physics.model import PhysicsScene, Unit

from legowalkers_sim.bridge import AdapterError, CheckedModel
from legowalkers_sim import cli
from legowalkers_sim.cli import parser
from legowalkers_sim.collision import (
    AXLE_ROD_RADIUS_METERS, CollisionPart, configure_floor_contacts, install_contacts, prepare_parts, to_mujoco,
)
from legowalkers_sim.evaluation import EvaluationOptions, ranking_rejections


def box(part_id, center, half_extents, *, shaft=False):
    vertices = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ]) * half_extents + center
    faces = [
        [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
    ]
    transform = np.eye(4)
    transform[:3, 3] = center
    if shaft:
        transform[:3, :3] = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]
    return LDrawPart(part_id, 7, transform, [Triangle(*vertices[face]) for face in faces])


def test_collision_coordinates_support_points_and_meshes():
    assert to_mujoco([1, 2, 3]).tolist() == [1, -3, -2]
    assert to_mujoco([[1, 2, 3], [4, 5, 6]]).tolist() == [[1, -3, -2], [4, -6, -5]]


def test_oversized_unit_visuals_split_without_changing_mass_or_contact_pairs(tmp_path, monkeypatch):
    import legowalkers_sim.collision as collision
    mujoco = pytest.importorskip("mujoco")
    monkeypatch.setattr(collision, "MAX_STL_FACES", 16)
    bricks = [box("beam.dat", [x, 0, 0], [5, 5, 5]) for x in [0, 20]]
    scene = PhysicsScene(units=[Unit(bricks, mass=.1, center_of_mass=np.zeros(3))])
    root = ET.fromstring(f"""
        <mujoco><compiler meshdir="{tmp_path}"/><asset>
          <mesh name="mesh_0" file="unit_0.stl"/>
        </asset><worldbody><geom name="ground" type="plane" size="1 1 .1"/>
        <body name="unit_0" pos="0 0 .1"><freejoint/>
          <inertial mass=".1" pos="0 0 0" diaginertia=".001 .001 .001"/>
          <geom name="original_visual" mesh="mesh_0" type="mesh"/>
        </body></worldbody></mujoco>
    """)
    parts = [CollisionPart(f"beam-{i}", brick, 0) for i, brick in enumerate(bricks)]
    report = install_contacts(root, scene, parts, {}, tmp_path, enabled=True, anchor=False, friction=1)
    assert root.find("asset/mesh[@name='mesh_0']") is None
    assert len(root.findall(".//geom[@name='visual_part_0']")) == 1
    assert len(root.findall(".//geom[@name='visual_part_1']")) == 1
    assert report["partGeometries"] == 2 and report["selfContactPairs"] == 0
    assert len(root.findall("contact/pair")) == 2
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.body_mass.sum() == pytest.approx(.1)


def test_part_contacts_are_the_default_and_ground_only_is_explicit():
    assert EvaluationOptions().self_collision
    assert parser().parse_args(["evaluate", "manifest.json", "--output", "result"]).self_collision
    assert not parser().parse_args(["evaluate", "manifest.json", "--output", "result", "--ground-only"]).self_collision


@pytest.mark.parametrize("value", [0, -.001, .0009, float("nan"), float("inf"), True, "0.002"])
def test_invalid_or_silently_clamped_floor_stiffness_rejects(value):
    with pytest.raises(AdapterError, match="floor_contact_time_constant"):
        EvaluationOptions(timestep=.0005, floor_contact_time_constant=value).validate()


@pytest.mark.parametrize("command", ["evaluate", "batch"])
def test_floor_contact_cli_wires_the_explicit_setting(command, tmp_path, monkeypatch):
    seen = []
    def stop(checked, output, options):
        seen.append(options.floor_contact_time_constant)
        raise AdapterError("stop after option wiring")
    monkeypatch.setattr(cli, "check", lambda *args: object())
    monkeypatch.setattr(cli, "evaluate", stop)
    args = parser().parse_args([command, str(tmp_path), "--output", str(tmp_path / "out"),
                               "--ldraw-library", str(tmp_path), "--floor-contact-time-constant", ".002",
                               "--timestep", ".0005"])
    if command == "batch":
        (tmp_path / "sweep.json").write_text(json.dumps({
            "schemaVersion": 1, "generated": [{"directory": "candidate"}]}))
        cli.run(args)
    else:
        with pytest.raises(AdapterError, match="stop after"):
            cli.run(args)
    assert seen == [.002]


@pytest.mark.parametrize("time_constant,timestep", [
    (None, .0005), (.001, .0005), (.002, .0005),
    (.0004, .0002), (.0004, .0001), (.0006, .0002), (.0006, .0001),
])
def test_floor_settings_affect_explicit_floor_pairs_only(tmp_path, time_constant, timestep):
    mujoco = pytest.importorskip("mujoco")
    bricks = [box("beam.dat", [0, 0, 0], [5, 5, 5]) for _ in range(2)]
    scene = PhysicsScene(units=[Unit([brick], mass=.1, center_of_mass=np.zeros(3)) for brick in bricks])
    root = ET.fromstring(f"""
      <mujoco><compiler meshdir="{tmp_path}"/><option timestep="{timestep}"/><asset/>
      <worldbody><geom name="ground" type="plane" size="1 1 .1"/>
      <body name="unit_0" pos="0 0 .1"/><body name="unit_1" pos="0 0 .2"/>
      </worldbody></mujoco>""")
    parts = [CollisionPart(f"beam-{i}", brick, i) for i, brick in enumerate(bricks)]
    install_contacts(root, scene, parts, {}, tmp_path, enabled=True, anchor=False, friction=1,
                     floor_contact_time_constant=time_constant)
    pairs = root.findall("contact/pair")
    assert len(pairs) == 3
    for pair in pairs:
        assert ("solref" in pair.attrib) == (time_constant is not None and pair.get("geom2") == "ground")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    for index in range(model.npair):
        is_floor = model.pair_geom2[index] == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        # MuJoCo may canonicalize geom ordering.
        is_floor |= model.pair_geom1[index] == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        assert model.pair_solref[index].tolist() == pytest.approx(
            [time_constant if is_floor and time_constant is not None else .02, 1])


def test_anchored_floor_configuration_rejects_instead_of_silently_doing_nothing():
    with pytest.raises(AdapterError, match="anchored"):
        EvaluationOptions(anchor=True, timestep=.0005, floor_contact_time_constant=.001).validate()


@pytest.mark.parametrize("time_constant,timestep", [
    (.001, .0005), (.002, .0005), (.0004, .0002), (.0006, .0002),
])
def test_loaded_foot_fixture_is_stiffer_and_converges_with_half_timestep(time_constant, timestep):
    mujoco = pytest.importorskip("mujoco")
    def simulate(dt, time_constant):
        root = ET.fromstring(f"""
          <mujoco><option timestep="{dt}" iterations="50" tolerance="1e-10"/>
          <worldbody><geom name="ground" type="plane" size="1 1 .1"/>
            <body pos="0 0 .0068"><freejoint/>
              <geom name="foot" type="box" size=".016 .004 .0048" mass=".45"/>
            </body>
          </worldbody><contact><pair geom1="foot" geom2="ground"/></contact></mujoco>""")
        configure_floor_contacts(root, time_constant)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        data = mujoco.MjData(model)
        worst = 0.0
        peak = 0.0
        for _ in range(round(.4/dt)):
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            depth = max((max(0., -float(c.dist)) for c in data.contact), default=0.)
            peak = max(peak, depth)
            if data.time > .2:
                worst = max(worst, depth)
        assert not np.any(data.warning.number)
        return worst, data.qpos[2], peak
    legacy, _, _ = simulate(.0005, None)
    coarse, coarse_height, coarse_peak = simulate(timestep, time_constant)
    fine, fine_height, fine_peak = simulate(timestep/2, time_constant)
    assert max(coarse, fine) < .0001
    assert coarse < legacy/10
    assert abs(coarse-fine) < .00001
    assert abs(coarse_height-fine_height) < .00001
    if time_constant < .001:
        assert max(coarse_peak, fine_peak) < .0001
        assert abs(coarse_peak-fine_peak) < .00002


@pytest.mark.parametrize("options,changes,reason", [
    (EvaluationOptions(), {}, None),
    (EvaluationOptions(self_collision=False), {}, "ground-only-preview"),
    (EvaluationOptions(anchor=True), {}, "anchored-chassis"),
    (EvaluationOptions(), {"interference": True}, "self-interference"),
    (EvaluationOptions(), {"full_cycle": False}, "incomplete-motor-cycle"),
    (EvaluationOptions(), {"fallen": True}, "excessive-tilt"),
    (EvaluationOptions(), {"closure_ok": False}, "joint-closure-error"),
    (EvaluationOptions(), {"distance": 0}, "nonpositive-forward-motion"),
    (EvaluationOptions(), {"logical_feet": 1}, "logical-foot-count-out-of-range"),
    (EvaluationOptions(), {"logical_feet": 0}, "logical-foot-count-out-of-range"),
    (EvaluationOptions(), {"logical_feet": 7}, "logical-foot-count-out-of-range"),
    (EvaluationOptions(), {"logical_feet": None}, "unknown-logical-foot-count"),
    (EvaluationOptions(), {"logical_feet": 3}, None),
    (EvaluationOptions(), {"logical_feet": 5}, None),
    (EvaluationOptions(), {"logical_feet": 6}, None),
])
def test_preview_interference_and_partial_cycles_cannot_receive_walking_fitness(options, changes, reason):
    measured = dict(full_cycle=True, interference=False, fallen=False, closure_ok=True, distance=0.01, logical_feet=2)
    reasons = ranking_rejections(options, **{**measured, **changes})
    assert reasons == ([] if reason is None else [reason])


@pytest.mark.parametrize("command", ["evaluate", "batch"])
def test_cli_returns_failure_for_interference_and_batch_never_ranks_it(tmp_path, monkeypatch, capsys, command):
    monkeypatch.setattr(cli, "check", lambda *args: object())
    monkeypatch.setattr(cli, "evaluate", lambda *args: {
        "status": "interference",
        "metrics": {"eligibleForRanking": True, "speedMetersPerSecond": 1},
    })
    source = tmp_path / "sweep"
    (source / "candidate-000").mkdir(parents=True)
    (source / "sweep.json").write_text(json.dumps({
        "schemaVersion": 1, "generated": [{"directory": "candidate-000"}],
    }), encoding="utf-8")
    output = tmp_path / "results"
    assert cli.main([command, str(source), "--output", str(output),
                     "--ldraw-library", str(tmp_path)]) == 1
    captured = capsys.readouterr()
    assert "ERROR" in captured.err
    if command == "batch":
        result = json.loads((output / "batch.json").read_text(encoding="utf-8"))
        assert result["failures"] == 1
        assert result["rankedCandidates"] == []

@pytest.mark.parametrize("offset", [[0, 0, 0], [600, -300, 900]])
def test_connector_attachment_uses_only_verified_sockets_and_keeps_checked_scene_unchanged(offset):
    from legowalkers_sim.payload import part_mass_center
    offset = np.array(offset)
    frame = box("3703.dat", offset, [10, 10, 10])
    gear = box("3648b.dat", offset+[0, 0, 60], [30, 30, 10])
    axle = box("3707.dat", offset, [6, 6, 80], shaft=True)
    axle.triangles.pop()  # Open LDraw surfaces must not integrate against the world origin.
    frame.ports = [ConnectionPort(PortType.ROUND_HOLE, offset, np.array([0, 0, 1]))]
    gear.ports = [ConnectionPort(PortType.AXLE_HOLE, offset+[0, 0, 60], np.array([0, 0, 1]))]
    parts = [
        {"id": name, "part": geometry.part_id, "body": body,
         "rotation": geometry.transform[:3, :3].tolist()}
        for name, geometry, body in [("frame", frame, "chassis"), ("gear", gear, "shaft"), ("axle", axle, None)]
    ]
    manifest = {"parts": parts, "expected": {"joints": [
        {"connector": "axle", "bodyA": "chassis", "bodyB": "shaft"},
    ], "gears": []}}
    original = PhysicsScene(units=[
        Unit([frame], 0.01, np.array(offset)*.0004),
        Unit([gear], 0.01, (np.array(offset)+[0, 0, 60])*.0004)])
    checked = CheckedModel(manifest, original, {"chassis": 0, "shaft": 1}, {}, {},
                           LDrawBuild("test", [frame, gear, axle]))
    scene, attached, exemptions = prepare_parts(checked)
    assert attached[-1].unit == 1
    assert set(exemptions) == {("axle", "frame"), ("axle", "gear")}
    assert len(original.units[1].bricks) == 1
    assert len(scene.units[1].bricks) == 2
    gear_mass, _ = part_mass_center(CollisionPart("gear", gear, 1))
    expected_mass, _ = part_mass_center(CollisionPart("axle", axle, 1))
    baseline = box("3707.dat", np.zeros(3), [6, 6, 80], shaft=True)
    baseline.triangles.pop()
    baseline_mass, _ = part_mass_center(CollisionPart("axle", baseline, 1))
    assert expected_mass == pytest.approx(baseline_mass)
    assert scene.units[1].mass-gear_mass == pytest.approx(baseline_mass)
    assert original.units[1].mass == .01
    assert np.isfinite(scene.units[1].center_of_mass).all()
    gear.ports = []
    with pytest.raises(AdapterError, match="ambiguous rigid attachment"):
        prepare_parts(checked)


@pytest.mark.parametrize("coupler_x,enabled,expect_contact", [
    (0, True, True), (30, True, False), (0, False, False),
])
def test_rod_catches_parent_child_crossing_without_colliding_with_its_bearing(
        tmp_path: Path, coupler_x, enabled, expect_contact):
    mujoco = pytest.importorskip("mujoco")
    frame = box("3703.dat", [0, 0, 0], [10, 10, 10])
    axle = box("3707.dat", [0, 0, 0], [6, 6, 80], shaft=True)
    coupler = box("32063.dat", [coupler_x, 0, 60], [9, 40, 5])
    parts = [CollisionPart("frame", frame, 0), CollisionPart("axle", axle, 1),
             CollisionPart("coupler", coupler, 2)]
    scene = PhysicsScene(units=[Unit([part.geometry], 0.01) for part in parts])
    root = ET.fromstring("""
    <mujoco>
      <compiler meshdir="meshes"/>
      <asset/>
      <worldbody>
        <geom name="ground" type="plane" size="1 1 0.1"/>
        <body name="unit_0">
          <inertial mass="0.01" pos="0 0 0" diaginertia="0.00001 0.00001 0.00001"/>
          <body name="unit_1">
            <joint type="hinge" axis="0 1 0"/>
            <inertial mass="0.01" pos="0 0 0" diaginertia="0.00001 0.00001 0.00001"/>
            <body name="unit_2">
              <joint type="hinge" axis="0 1 0"/>
              <inertial mass="0.01" pos="0 0 0" diaginertia="0.00001 0.00001 0.00001"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """)
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    report = install_contacts(root, scene, parts, {("axle", "frame"): "verified socket"},
                              mesh_dir, enabled=enabled, anchor=True, friction=1)
    path = tmp_path / "model.xml"
    ET.ElementTree(root).write(path)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rod = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "part_1")
    assert model.geom_type[rod] == mujoco.mjtGeom.mjGEOM_CYLINDER
    assert model.geom_size[rod, 0] == pytest.approx(AXLE_ROD_RADIUS_METERS)
    assert model.geom_size[rod, 1] == pytest.approx(80 * 0.0004)
    assert bool(data.ncon) == expect_contact
    if expect_contact:
        coupler_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "part_2")
        assert all({contact.geom1, contact.geom2} == {rod, coupler_id} for contact in data.contact)
        assert min(contact.dist for contact in data.contact) < -0.001
    assert report["bodyPairExclusions"] == 0
