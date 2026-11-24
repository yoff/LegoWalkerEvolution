from copy import deepcopy
from types import SimpleNamespace
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
import pytest
from lego_technic_sim.mujoco_export import generate_mjcf
from lego_technic_sim.physics.model import GearConstraint, Joint, JointType, Motor, PhysicsScene, Unit

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.collision import to_mujoco, vector_text
from legowalkers_sim.joint_tree import prioritize_shared_pivots


@pytest.fixture
def fixture():
    axis = np.array([0., 0., 1.])
    pivot = np.array([.02, -.01, .03])
    scene = PhysicsScene(
        units=[Unit(mass=.1, center_of_mass=np.array([i*.013, i*.007, -.02]))
               for i in range(5)],
        joints=[Joint(a, b, JointType.REVOLUTE, pivot.copy(), axis.copy())
                for a, b in [(0, 1), (0, 2), (1, 3), (1, 4), (2, 3), (3, 4)]],
        motors=[Motor(0, speed=2, max_torque=.4)],
        gears=[GearConstraint(1, 2, .5, axis.copy(), axis.copy(), pivot.copy())],
    )
    names = {"chassis": 0, "motor": 1, "a": 2, "b": 3, "c": 4}
    declared = [{"id": name, "bodyA": a, "bodyB": b, "axis": axis.tolist(),
                 "position": (pivot/.0004).tolist()}
                for name, a, b in [("ab", "a", "b"), ("ac", "a", "c")]]
    manifest = {"expected": {"joints": declared},
                "motion": {"sharedPivots": [
                    {"id": "shared", "bodies": ["a", "b", "c"], "abstractHinges": ["ab", "ac"]}]}}
    root = ET.fromstring(generate_mjcf(scene))
    for i, unit in enumerate(scene.units):
        body = root.find(f".//body[@name='unit_{i}']")
        for suffix, point in [("pivot", pivot), ("axis", pivot+axis*.008)]:
            ET.SubElement(body, "site", name=f"{suffix}_{i}",
                          pos=vector_text(to_mujoco(point-unit.center_of_mass)))
    return SimpleNamespace(scene=scene, names=names, manifest=manifest, root=root)


def compile_model(root):
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def test_shared_axes_become_exact_tree_hinges_without_locking_rotation(fixture):
    f = fixture
    before = compile_model(f.root)
    original = deepcopy(f.root)
    joint_copy = deepcopy(f.scene.joints)
    report = prioritize_shared_pivots(f.root, f.scene, f.manifest, f.names)
    assert report["hardSharedPivots"] == [{"id": "shared", "joints": [4, 5]}]
    assert report["softSharedPivots"] == []
    assert {0, 1} <= set(report["protectedJoints"])
    assert len(report["treeJoints"]) == 4 and len(report["loopJoints"]) == 2
    assert f.root.find("./equality/connect[@name='loop_4']") is None
    assert f.root.find("./equality/connect[@name='loop_5']") is None
    for index in (4, 5):
        assert f.root.find(f".//joint[@name='joint_{index}']").get("type") == "hinge"
    for tag in ("actuator",):
        assert ET.tostring(f.root.find(tag)) == ET.tostring(original.find(tag))
    assert (ET.tostring(f.root.find("./equality/joint"))
            == ET.tostring(original.find("./equality/joint")))
    for index in range(5):
        for tag in ("inertial", "geom"):
            assert (ET.tostring(f.root.find(f".//body[@name='unit_{index}']/{tag}"))
                    == ET.tostring(original.find(f".//body[@name='unit_{index}']/{tag}")))
    after = compile_model(f.root)
    assert (after.nq, after.nv, after.nbody, after.neq) == (
        before.nq, before.nv, before.nbody, before.neq)
    assert after.body_mass == pytest.approx(before.body_mass)
    for old, new in zip(joint_copy, f.scene.joints, strict=True):
        assert (new.unit_a_index, new.unit_b_index) == (old.unit_a_index, old.unit_b_index)
        assert new.position == pytest.approx(old.position)
        assert new.axis == pytest.approx(old.axis)

    data = mujoco.MjData(after)
    for a, b in [(-2.8, 1.7), (0., 0.), (.4, -1.9), (3.1, 2.8)]:
        for index, value in [(4, a), (5, b)]:
            joint_id = mujoco.mj_name2id(after, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{index}")
            data.qpos[after.jnt_qposadr[joint_id]] = value
        mujoco.mj_forward(after, data)
        for suffix in ("pivot", "axis"):
            sites = [mujoco.mj_name2id(after, mujoco.mjtObj.mjOBJ_SITE, f"{suffix}_{i}")
                     for i in (2, 3, 4)]
            assert data.site_xpos[sites] == pytest.approx(
                np.tile(data.site_xpos[sites[0]], (3, 1)), abs=1e-14)
        assert data.qpos[after.jnt_qposadr[
            mujoco.mj_name2id(after, mujoco.mjtObj.mjOBJ_JOINT, "joint_4")]] == a


def test_no_multi_body_pivots_preserves_original_xml(fixture):
    f = fixture
    f.manifest["motion"]["sharedPivots"][0]["bodies"] = ["a", "b"]
    before = ET.tostring(f.root)
    report = prioritize_shared_pivots(f.root, f.scene, f.manifest, f.names)
    assert report["hardSharedPivots"] == report["softSharedPivots"] == []
    assert ET.tostring(f.root) == before


def test_impossible_shared_axis_stays_soft_with_explicit_reason(fixture):
    f = fixture
    ET.SubElement(f.root.find("actuator"), "motor", joint="joint_2")
    before = ET.tostring(f.root)
    report = prioritize_shared_pivots(f.root, f.scene, f.manifest, f.names)
    assert report["hardSharedPivots"] == []
    assert report["softSharedPivots"][0]["id"] == "shared"
    assert "cycle" in report["softSharedPivots"][0]["reason"]
    assert set(report["protectedJoints"]) == {0, 1, 2}
    assert ET.tostring(f.root) == before


def test_shared_pivot_geometry_mismatch_is_an_explicit_error(fixture):
    f = fixture
    f.scene.joints[5].position += np.array([.01, 0, 0])
    with pytest.raises(AdapterError, match="verified physical joint tree"):
        prioritize_shared_pivots(f.root, f.scene, f.manifest, f.names)


def test_legacy_realization_hinge_ids_are_resolved_by_physical_connector(fixture):
    f = fixture
    for joint in f.manifest["expected"]["joints"]:
        joint["id"] = "physical-"+joint["id"]
        joint["connector"] = "pivot-shared"
    report = prioritize_shared_pivots(f.root, f.scene, f.manifest, f.names)
    assert report["hardSharedPivots"] == [{"id": "shared", "joints": [4, 5]}]
    compile_model(f.root)


def test_required_gear_ancestor_path_and_joint_direction_are_preserved(fixture):
    f = fixture
    ET.SubElement(f.root.find("equality"), "joint", name="extra_gear",
                  joint1="joint_2", joint2="joint_0", polycoef="0 -1 0 0 0")
    report = prioritize_shared_pivots(f.root, f.scene, f.manifest, f.names)
    assert {0, 1, 2} <= set(report["protectedJoints"])
    assert f.root.find(".//body[@name='unit_1']/body[@name='unit_3']/joint[@name='joint_2']") is not None
    compile_model(f.root)
