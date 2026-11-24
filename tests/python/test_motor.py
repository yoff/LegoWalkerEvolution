import math
from xml.etree import ElementTree as ET
from dataclasses import replace

import mujoco
import pytest

from legowalkers_sim import motor
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.evaluation import EvaluationOptions, ranking_rejections


def test_calibration_hits_measured_9v_points_and_conservatively_caps_stall():
    assert motor.torque_at(motor.NO_LOAD_SPEED, motor.NO_LOAD_SPEED) == pytest.approx(0)
    assert motor.torque_at(motor.LOAD_SPEED, motor.NO_LOAD_SPEED) == pytest.approx(.145)
    assert motor.torque_at(0, motor.NO_LOAD_SPEED) == .4
    assert motor.SLOPE * motor.NO_LOAD_SPEED > .4
    definition = motor.definition(motor.MODEL, motor.NO_LOAD_SPEED, .4)
    assert definition["supplyVolts"] == 9
    assert definition["pwmDuty"] == 1
    assert definition["loadedPoint"]["rpm"] == 146
    assert .145 * motor.LOAD_SPEED == pytest.approx(2.21, abs=.01)


def test_low_pwm_reduces_available_torque_and_polarity_is_symmetric():
    request = motor.NO_LOAD_SPEED / 4
    assert motor.torque_at(0, request) == .1
    for speed in (-100, -2, 0, 2, 100):
        assert abs(motor.torque_at(speed, request)) <= .1
        assert motor.torque_at(-speed, -request) == -motor.torque_at(speed, request)
    assert motor.torque_at(request, request) == pytest.approx(0)


@pytest.mark.parametrize("speed,limit", [(math.nan, .4), (0, .4), (1, 0), (1, .41),
                                       (motor.NO_LOAD_SPEED + 1, .4)])
def test_invalid_or_inflated_motor_requests_are_explicit_errors(speed, limit):
    with pytest.raises(AdapterError):
        motor.torque_at(0, speed, limit)


def test_physical_baseline_and_preview_ranking_are_explicit():
    options = EvaluationOptions()
    options.validate()
    assert options.speed == motor.NO_LOAD_SPEED
    assert options.motor_model == motor.MODEL
    preview = replace(options, motor_model="ideal-servo", speed=3)
    reasons = ranking_rejections(preview, full_cycle=True, interference=False, fallen=False,
                                 closure_ok=True, distance=1, logical_feet=4)
    assert reasons == ["ideal-motor-preview"]


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("load", [0, .145])
def test_direct_torque_physics_reaches_measured_loaded_or_unloaded_speed(direction, load):
    root = ET.fromstring("""
      <mujoco><option timestep=".0005" gravity="0 0 0"/>
        <default><joint damping=".01" armature=".001"/></default>
        <worldbody><body><joint name="rotor" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size=".02" mass=".1"/></body></worldbody>
        <actuator><velocity joint="rotor" kv="1"/></actuator>
      </mujoco>""")
    motor.install(root, {"parts": [{"id": "motor", "part": "58121.dat"}],
                         "expected": {"drives": [{"motorPart": "motor"}]}}, EvaluationOptions())
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.dof_damping[0] == 0
    data = mujoco.MjData(model)
    for _ in range(2000):
        data.qfrc_applied[0] = -direction * load
        data.ctrl[0] = motor.torque_at(float(data.qvel[0]), direction * motor.NO_LOAD_SPEED)
        mujoco.mj_step(model, data)
    expected = motor.LOAD_SPEED if load else motor.NO_LOAD_SPEED
    assert data.qvel[0] == pytest.approx(direction * expected, abs=1e-5)
    assert abs(data.actuator_force[0]) <= .4
