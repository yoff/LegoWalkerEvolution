from copy import deepcopy
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
import pytest
from lego_technic_sim.mujoco_export import generate_mjcf
from lego_technic_sim.ldraw.model import LDrawPart, Triangle
from lego_technic_sim.physics.gears import detect_gear_meshes
from lego_technic_sim.physics.model import Joint, JointType, Motor, PhysicsScene, Unit

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.spatial_gears import install_bevel_couplings


def fixture(ratio=1., *, carried=True, reverse=False, signs=(1., 1., 1.), side=1.):
    vertical = np.array([0., 0., 1.])
    horizontal = np.array([1., 0., 0.])
    apex = np.array([80., -40., 60.])
    pivot = apex * .0004
    scene = PhysicsScene(
        units=[Unit(mass=.1, center_of_mass=np.array([i*.013, i*.007, -.02]))
               for i in range(4)],
        joints=[
            Joint(0, 1, JointType.REVOLUTE, pivot.copy(), vertical*signs[0]),
            Joint(0, 2, JointType.REVOLUTE, pivot.copy(), vertical*signs[1]),
            Joint(2 if carried else 0, 3, JointType.REVOLUTE, pivot.copy(), horizontal*signs[2]),
        ],
        motors=[Motor(0, speed=2, max_torque=.4)],
    )
    a, b = {1.: (("32270.dat", 12), ("32270.dat", 12)),
            7/6: (("4143.dat", 14), ("32270.dat", 12)),
            6/7: (("32270.dat", 12), ("4143.dat", 14))}[ratio]
    for unit, part, center, rotation in [
        (1, a[0], apex + vertical*(b[1]*1.25*side), np.eye(3)),
        (3, b[0], apex + horizontal*(a[1]*1.25), np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])),
    ]:
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, center
        triangle = Triangle(center.copy(), center+np.array([1., 1., 0.]), center+np.array([0., 0., 1.]))
        scene.units[unit].bricks = [LDrawPart(part, 4, transform, [triangle])]
    scene.gears = detect_gear_meshes(scene)
    assert len(scene.gears) == 1
    if reverse:
        gear = scene.gears[0]
        gear.unit_a_index, gear.unit_b_index = gear.unit_b_index, gear.unit_a_index
        gear.axis_a, gear.axis_b = gear.axis_b, gear.axis_a
        gear.ratio = 1/gear.ratio
    root = ET.fromstring(generate_mjcf(scene, timestep=.001))
    return scene, root


def test_planar_gears_and_no_gears_preserve_xml_exactly():
    scene, root = fixture()
    scene.gears[0].axis_b = scene.gears[0].axis_a.copy()
    before = ET.tostring(root)
    assert install_bevel_couplings(root, scene)["couplings"] == []
    assert ET.tostring(root) == before
    scene.gears = []
    assert install_bevel_couplings(root, scene)["couplings"] == []
    assert ET.tostring(root) == before


@pytest.mark.parametrize("ratio", [6/7, 1., 7/6])
@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("signs", [(1., 1., 1.), (-1., 1., -1.), (1., -1., 1.)])
@pytest.mark.parametrize("side", [-1., 1.])
def test_carrier_relative_constraint_and_reaction_torque(ratio, reverse, signs, side):
    scene, root = fixture(ratio, reverse=reverse, signs=signs, side=side)
    actuator = ET.tostring(root.find("actuator"))
    report = install_bevel_couplings(root, scene)
    assert report["couplings"][0]["carrierUnit"] == 2
    assert ET.tostring(root.find("actuator")) == actuator
    assert root.find("./equality/joint[@name='gear_0']") is None
    assert len(root.findall("./tendon/fixed/joint")) == 3
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.nu == 1
    data = mujoco.MjData(model)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(3)]
    addresses = model.jnt_qposadr[ids]
    velocities = model.jnt_dofadr[ids]
    expected = np.array([side*ratio*signs[0], -side*ratio*signs[1], signs[2]])
    for input_angle, carrier_angle in [(.6, -.2), (-1.1, .9), (2., 2.)]:
        output_angle = -side*ratio*(input_angle-carrier_angle)
        data.qpos[addresses] = np.array([input_angle, carrier_angle, output_angle])/np.array(signs)
        mujoco.mj_forward(model, data)
        assert data.ten_length[0] == pytest.approx(0., abs=1e-12)
        assert data.efc_pos[0] == pytest.approx(0., abs=1e-12)
        assert data.ten_J.reshape(model.ntendon, model.nv)[0, velocities] == pytest.approx(expected)
        velocity = np.array([.8, -.3, -side*ratio*(.8+.3)])/np.array(signs)
        assert np.dot(expected, velocity) == pytest.approx(0., abs=1e-12)
        data.qpos[addresses[2]] += .1
        mujoco.mj_forward(model, data)
        assert abs(data.efc_pos[0]) == pytest.approx(.1, abs=1e-12)


def test_fixed_carrier_uses_two_relative_shaft_angles():
    scene, root = fixture(7/6, carried=False)
    report = install_bevel_couplings(root, scene)
    assert report["couplings"][0]["carrierUnit"] is None
    terms = report["couplings"][0]["terms"]
    assert terms == [{"joint": "joint_0", "coefficient": 7/6},
                     {"joint": "joint_2", "coefficient": 1.}]
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.nu == 1 and model.ntendon == 1


@pytest.mark.parametrize("broken", ["axis", "offset", "hierarchy", "missing-equality", "ratio", "ref",
                                  "no-stock-gear", "pitch-geometry"])
def test_unsupported_or_unwitnessed_coupling_fails_without_partial_xml_edits(broken):
    scene, root = fixture()
    if broken == "axis":
        scene.gears[0].axis_b = np.array([1., 0., .1])
    elif broken == "offset":
        scene.gears[0].position = np.array([0., .01, 0.])
    elif broken == "hierarchy":
        scene.joints[1].unit_a_index = 1
    elif broken == "missing-equality":
        root.remove(root.find("equality"))
    elif broken == "ratio":
        scene.gears[0].ratio = float("nan")
    elif broken == "ref":
        root.find(".//joint[@name='joint_0']").set("ref", ".1")
    elif broken == "no-stock-gear":
        scene.units[1].bricks.clear()
    elif broken == "pitch-geometry":
        scene.units[1].bricks[0].transform[2, 3] += 5
    original = deepcopy(root)
    with pytest.raises(AdapterError):
        install_bevel_couplings(root, scene)
    assert ET.tostring(root) == ET.tostring(original)


def test_one_motor_transmits_torque_and_carrier_reacts_without_another_actuator():
    scene, root = fixture()
    install_bevel_couplings(root, scene)
    chassis = root.find(".//body[@name='unit_0']")
    chassis.remove(chassis.find("freejoint"))
    for geom in root.findall(".//geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    model.opt.gravity[:] = 0
    model.dof_armature[:] = .01
    assert model.nu == 1
    data = mujoco.MjData(model)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(3)]
    positions, velocities = model.jnt_qposadr[ids], model.jnt_dofadr[ids]
    data.ctrl[0] = 2.
    for _ in range(300):
        mujoco.mj_step(model, data)
    assert data.qpos[positions[0]] > .1
    assert abs(data.qpos[positions[1]]) > .01
    assert abs(data.qpos[positions[2]]) > .01
    forces = data.qfrc_constraint[velocities]
    assert abs(forces[0]) > 1e-5
    assert forces[0] == pytest.approx(-forces[1], abs=1e-10)
    assert forces[0] == pytest.approx(forces[2], abs=1e-10)
    assert abs(data.ten_length[0]) < .001
