import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.evaluation import EvaluationOptions, ranking_rejections
from legowalkers_sim.ground import GroundContacts, GroundPenetrationStats, foot_regions


def manifest():
    return {"feet": [{"id": "foot", "body": "leg", "partInstance": "coupler",
                      "position": [0, 0, 0], "contactRadiusMeters": .004}],
            "parts": [{"id": "coupler", "body": "leg"}, {"id": "axle", "body": None}],
            "groundWheels": ["wheel-left", "wheel-right"]}


def test_only_the_tip_region_of_the_designated_part_is_allowed():
    monitor = GroundContacts(manifest())
    feet = np.array([[.1, .2, .003]])
    monitor.observe("coupler", [.1, .2, 0], -.0001, 1, feet)
    assert monitor.foot_samples == 1 and not monitor.forbidden
    monitor.observe("coupler", [.13, .2, 0], -.001, 2, feet)
    assert monitor.forbidden["coupler"]["firstTime"] == 2
    monitor.observe("axle", [.1, .2, 0], -.002, 3, feet)
    assert "axle" in monitor.forbidden  # Same position is insufficient without the designated part.
    monitor.observe("battery", [.1, .2, 0], .00001, 4, feet)
    assert "battery" not in monitor.forbidden
    monitor.observe("battery", [.1, .2, 0], 0, 5, feet)
    assert "battery" in monitor.forbidden  # Actual touching counts, including zero depth.


def test_penetration_observer_counts_permitted_soles_without_waiving_nonfoot_contact():
    stats = GroundPenetrationStats()
    monitor = GroundContacts(manifest())
    feet = np.array([[0, 0, 0]])
    for part, distance, time in [("coupler", -.0025, 1), ("axle", -.0002, 1),
                                 ("coupler", -.001, 2), ("coupler", .001, 3)]:
        kind = monitor.observe(part, [0, 0, 0], distance, time, feet)
        stats.observe(part, kind, distance, time)
    report = stats.report()
    assert report["maxFootPenetrationMeters"] == .0025
    assert report["maxForbiddenPenetrationMeters"] == .0002
    assert monitor.report()["forbiddenContactDetected"]
    assert next(p for p in report["parts"] if p["kind"] == "foot")["samples"] == 2
    report["parts"][0]["samples"] = -1
    assert stats.report()["parts"][0]["samples"] > 0


def test_wheels_are_allowed_only_if_the_design_explicitly_designates_them():
    feet = np.array([[0, 0, .1]])
    towing = GroundContacts(manifest())
    towing.observe("wheel-left", [0, 0, 0], -.0001, 0, feet)
    assert towing.wheel_samples == 1 and not towing.forbidden
    solo = GroundContacts({**manifest(), "groundWheels": []})
    solo.observe("wheel-left", [0, 0, 0], -.0001, 0, feet)
    assert solo.report()["forbiddenContactDetected"]


def test_forbidden_ground_contact_is_a_qualification_gate_not_a_collision_filter():
    reasons = ranking_rejections(EvaluationOptions(), full_cycle=True, interference=False,
                                 fallen=False, closure_ok=True, distance=.1, forbidden_ground=True, logical_feet=2)
    assert reasons == ["forbidden-ground-contact"]


def test_rigid_wagon_is_not_eligible_even_with_otherwise_legal_motion():
    reasons = ranking_rejections(EvaluationOptions(), full_cycle=True, interference=False,
                                 fallen=False, closure_ok=True, distance=.1, rigid_towing=True, logical_feet=2)
    assert reasons == ["rigid-wagon-hitch"]


def test_legacy_foot_attachment_must_be_unambiguous():
    m = manifest()
    m["feet"][0].pop("partInstance")
    assert foot_regions(m)[0]["partInstance"] == "coupler"
    m["parts"].append({"id": "second-part-on-leg", "body": "leg"})
    with pytest.raises(AdapterError, match="unambiguous"):
        foot_regions(m)


def test_real_mujoco_floor_contacts_allow_feet_and_wheels_but_reject_the_battery():
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
          <geom name="ground" type="plane" size="1 1 .1"/>
          <body pos="0 0 .003"><freejoint/>
            <geom name="coupler" type="sphere" size=".004"/><site name="foot" size=".001"/>
          </body>
          <body pos=".1 0 .009"><freejoint/>
            <geom name="battery" type="box" size=".01 .01 .01"/>
          </body>
          <body pos=".2 0 .009"><freejoint/>
            <geom name="wheel-left" type="cylinder" size=".01 .004" quat=".70710678 .70710678 0 0"/>
          </body>
        </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    monitor = GroundContacts(manifest())
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    for contact in data.contact:
        other = contact.geom2 if contact.geom1 == ground else contact.geom1
        instance = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(other))
        monitor.observe(instance, contact.pos, float(contact.dist), float(data.time), data.site_xpos)
    assert monitor.foot_samples > 0
    assert monitor.wheel_samples > 0
    assert set(monitor.forbidden) == {"battery"}
