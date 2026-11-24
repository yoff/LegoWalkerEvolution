from dataclasses import replace

import pytest

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.cycles import CycleProgress, cycle_plan, planned_options
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import fitness, is_qualified
from test_evolution import result


def mechanism(ratios=(1,)):
    return {"chassis": "ground", "expected": {
        "drives": [{"joint": "motor"}],
        "gears": [{"bodyA": "input", "bodyB": f"shaft-{i}", "ratio": ratio} for i, ratio in enumerate(ratios)],
        "joints": [{"id": "motor", "bodyA": "ground", "bodyB": "input"}]
        + [{"id": f"leg-{i}", "bodyA": f"shaft-{i}", "bodyB": f"foot-{i}"} for i in range(len(ratios))]},
        "feet": [{"id": f"foot-{i}", "body": f"foot-{i}"} for i in range(len(ratios))]}


def options(**values):
    return replace(EvaluationOptions(duration=20, timestep=.002, cycle_aware=True), **values)


def test_repeat_period_is_common_multiple_not_slowest_foot():
    plan = cycle_plan(mechanism((1/4, 1/6)), options(speed=6))
    assert plan["motorTurnsPerCycle"] == 12
    assert plan["durationSeconds"] == pytest.approx(26.134)
    assert plan["requiredCycles"] == 2


def test_cascaded_ratios_and_speedup_have_exact_repeat_period():
    manifest = mechanism((1/3, 1/2))
    manifest["expected"]["gears"][1]["bodyA"] = "shaft-0"
    plan = cycle_plan(manifest, options())
    assert plan["motorTurnsPerCycle"] == 6
    assert plan["feet"]["foot-1"]["shaft-1"] == [1, 6]
    assert cycle_plan(mechanism((2/3,)), options())["motorTurnsPerCycle"] == 3


def test_slow_token_leg_cannot_purchase_unbounded_simulation_time():
    original = cycle_plan(mechanism(), options())
    assert original["durationSeconds"] == 20
    for speed in (3, -3):
        with pytest.raises(AdapterError, match="cap 60"):
            cycle_plan(mechanism((1, 1/1000)), options(speed=speed))
    stalled = CycleProgress(original, 3)
    stalled.observe(1, 0, 0)
    stalled.observe(20, .1, 1)
    assert stalled.report()["completeCycles"] == 0
    assert not stalled.report()["repeatableForwardMotion"]
    assert original["durationSeconds"] == 20


def test_longer_window_cannot_accumulate_distance_into_better_cycle_speed():
    plan = cycle_plan(mechanism(), options())
    for duration in (20, 50):
        trace = CycleProgress(plan, 3)
        for i in range(duration+1):
            trace.observe(i, i/2, i*.001)
        report = trace.report()
        assert report["minimumCycleSpeedMetersPerSecond"] == pytest.approx(.001)
        assert report["repeatableForwardMotion"]
    shove = CycleProgress(plan, 3)
    for i, position in enumerate((0, 1, 1)):
        shove.observe(i+1, i, position)
    assert shove.report()["completeCycles"] == 2
    assert not shove.report()["repeatableForwardMotion"]


def test_reverse_direction_and_interpolated_cycle_boundaries():
    plan = cycle_plan(mechanism(), options())
    trace = CycleProgress(plan, -3)
    trace.observe(1, 0, 0)
    trace.observe(6, -2.5, .01)
    report = trace.report()
    assert report["completeCycles"] == 2
    assert report["cycleIntervals"][0]["seconds"] == pytest.approx(2)
    assert report["minimumCycleSpeedMetersPerSecond"] == pytest.approx(.002)


def test_invalid_or_disconnected_drivetrain_is_not_silently_assumed_direct():
    manifest = mechanism()
    manifest["expected"]["gears"].append({"bodyA": "orphan-a", "bodyB": "orphan-b", "ratio": 1})
    with pytest.raises(AdapterError, match="Every gear train"):
        cycle_plan(manifest, options())
    with pytest.raises(AdapterError, match="finite and positive"):
        cycle_plan(mechanism((0,)), options())
    manifest = mechanism()
    manifest["feet"][0]["body"] = "orphan-foot"
    with pytest.raises(AdapterError, match="no supported motor"):
        cycle_plan(manifest, options())


def test_legacy_options_are_not_rescheduled_and_cycle_results_cannot_fake_qualification():
    legacy = options(cycle_aware=False)
    assert planned_options({}, legacy) == (legacy, None)
    plan = cycle_plan(mechanism(), options())
    measured = result(qualified=True)
    measured["gait"] = {**plan, "completeCycles": 1, "repeatableForwardMotion": True, "cycleCoverage": 1}
    assert not is_qualified(measured)
    assert fitness(measured)[0] == 0
    measured["gait"]["completeCycles"] = 2
    assert is_qualified(measured)


def test_spatial_axes_cannot_reuse_planar_cycle_proof():
    manifest = mechanism()
    manifest["expected"]["joints"][0]["axis"] = [0, 0, 1]
    manifest["expected"]["joints"][1]["axis"] = [0, 0, -2]
    assert cycle_plan(manifest, options())["motorTurnsPerCycle"] == 1
    manifest["expected"]["joints"][1]["axis"] = [1, 0, 0]
    with pytest.raises(AdapterError, match="moving-frame repeat period"):
        cycle_plan(manifest, options())
    manifest["expected"]["joints"][1]["axis"] = [0, 0, 0]
    with pytest.raises(AdapterError, match="finite nonzero"):
        cycle_plan(manifest, options())
