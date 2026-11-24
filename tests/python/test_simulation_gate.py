"""Small synthetic physics fixtures only; no LEGO trials or campaign writes."""

from copy import deepcopy
from dataclasses import replace
import json
import math
import multiprocessing
import os
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from legowalkers_sim import cli, evaluation
from legowalkers_sim.bridge import AdapterError, write_json
from legowalkers_sim.cli import parser
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import Archive
from legowalkers_sim.parallel import TrialPool, attempt_summary
from legowalkers_sim.simulation_gate import GateAttemptLimit, SimulationPrefixGate
from test_evolution import genome, protocol, result


@pytest.fixture
def rig(monkeypatch):
    counts = {"prepare": 0, "load": 0, "data": 0, "step": 0}
    state = SimpleNamespace(counts=counts, data=[], step_data=[], fault=None)
    load = mujoco.MjModel.from_xml_path
    allocate = mujoco.MjData
    forward = mujoco.mj_forward
    step = mujoco.mj_step

    def prepare(checked, output, options):
        counts["prepare"] += 1
        xml = output / "model.xml"
        xml.write_text(f"""<mujoco>
          <option timestep="{options.timestep}" gravity="0 0 0"/>
          <worldbody>
            <geom name="ground" type="plane" size="1 1 .1" contype="0" conaffinity="0"/>
            <body name="unit_0" pos="0 0 1">
              <freejoint/>
              <geom name="frame" type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/>
              <site name="pivot_a"/><site name="foot_0" pos=".02 0 -.01"/>
              <site name="foot_1" pos="-.02 0 -.01"/>
              <body name="unit_1">
                <joint name="joint_0" type="hinge" axis="0 0 1"/>
                <geom name="rotor" type="sphere" pos=".05 0 0" size=".01" mass=".01"
                      contype="0" conaffinity="0"/>
                <site name="pivot_b"/>
              </body>
            </body>
          </worldbody>
          <actuator><velocity joint="joint_0" kv="1" forcelimited="true" forcerange="-.4 .4"/></actuator>
        </mujoco>""")
        return xml, [("pivot_a", "pivot_b")], ["foot_0", "foot_1"], {
            "geometryInstances": {"frame": "frame", "rotor": "rotor"}}

    def from_xml(path):
        counts["load"] += 1
        return load(path)

    def data(model):
        counts["data"] += 1
        value = allocate(model)
        state.data.append(value)
        return value

    def advance(model, value):
        counts["step"] += 1
        state.step_data.append(id(value))
        step(model, value)

    def update(model, value):
        forward(model, value)
        if state.fault is not None:
            state.fault(model, value)

    monkeypatch.setattr(evaluation, "prepare_xml", prepare)
    monkeypatch.setattr(mujoco, "MjModel", SimpleNamespace(from_xml_path=from_xml))
    monkeypatch.setattr(mujoco, "MjData", data)
    monkeypatch.setattr(mujoco, "mj_step", advance)
    monkeypatch.setattr(mujoco, "mj_forward", update)
    state.checked = SimpleNamespace(
        manifest={
            "grammarVersion": "technic-motion-v3", "ldrawSha256": "0" * 64,
            "genome": {"speed": 3, "towWagon": False},
            "motion": {"logicalFeet": [{"id": "left"}, {"id": "right"}]},
            "expected": {"drives": [{"joint": "motor"}],
                         "joints": [{"id": "motor", "bodyA": "chassis", "bodyB": "rotor"}]},
            "feet": [{"id": name, "body": "chassis", "partInstance": "frame"} for name in ("left", "right")],
        },
        report={"status": "compatible"}, joint_indices={"motor": 0},
        scene=SimpleNamespace(units=[None, None]))
    return state


def options(**values):
    return replace(EvaluationOptions(duration=.03, settle=.004, timestep=.001,
                                     capture_poses=True, short_gate=.005,
                                     motor_model="ideal-servo", speed=3), **values)


def test_survivor_is_the_exact_ungated_run_with_one_preparation_model_and_data(rig, tmp_path):
    baseline = evaluation.evaluate(rig.checked, tmp_path / "baseline", options(short_gate=0))
    before = dict(rig.counts)
    handoffs = []

    def passed():
        assert rig.counts["step"] - before["step"] == 9
        handoffs.append(id(rig.data[-1]))

    measured = evaluation.evaluate(rig.checked, tmp_path / "gated", options(), on_gate_passed=passed)
    assert {key: rig.counts[key] - before[key] for key in before} == {
        "prepare": 1, "load": 1, "data": 1, "step": 30}
    assert len(set(rig.step_data[30:])) == 1
    assert handoffs == [id(rig.data[1])]
    np.testing.assert_array_equal(rig.data[0].qpos, rig.data[1].qpos)
    np.testing.assert_array_equal(rig.data[0].qvel, rig.data[1].qvel)
    for key in ("status", "metrics", "groundContact", "groundPenetration", "collision", "control"):
        assert measured[key] == baseline[key]
    for name in ("model.xml", "trajectory.json", "poses.json", "ground-penetration.json"):
        assert (tmp_path / "gated" / name).read_bytes() == (tmp_path / "baseline" / name).read_bytes()
    report = measured["simulationGate"]
    assert report["status"] == "passed" and report["reasons"] == []
    assert report["simulatedSteps"] == 9 and report["remainingSteps"] == 21
    assert report["measurementSeconds"] == .005
    assert not measured["collision"]["fullMotorCycle"]
    assert "incomplete-motor-cycle" in measured["metrics"]["rankingRejectionReasons"]
    assert json.loads((tmp_path / "gated/simulation-gate.json").read_text()) == report
    assert json.loads((tmp_path / "gated/ground-penetration.json").read_text()) == measured["groundPenetration"]
    assert not (tmp_path / "baseline/simulation-gate.json").exists()


def fault(kind, active):
    def inject(model, data):
        tick = round(data.time / model.opt.timestep)
        if not active(tick):
            return
        if kind == "tilt":
            chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "unit_0")
            data.xmat[chassis].reshape(3, 3)[2, 2] = math.cos(math.pi / 3 + .01)
        elif kind == "closure":
            left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "pivot_a")
            right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "pivot_b")
            data.site_xpos[right] = data.site_xpos[left] + [.00201, 0, 0]
        elif kind == "warning":
            data.warning.number[0] = 1
        else:
            data.qvel[0] = math.nan
    return inject


@pytest.mark.parametrize("kind,reason", [("tilt", "excessive-tilt"), ("closure", "joint-closure-error")])
def test_failure_aborts_on_the_first_irreversible_measured_tick(rig, tmp_path, kind, reason):
    rig.fault = fault(kind, lambda tick: tick == 6)
    with pytest.raises(AdapterError, match=reason):
        evaluation.evaluate(rig.checked, tmp_path / "failed", options())
    assert rig.counts == {"prepare": 1, "load": 1, "data": 1, "step": 6}
    assert (tmp_path / "failed/ground-penetration.json").is_file()
    report = json.loads((tmp_path / "failed/simulation-gate.json").read_text())
    assert report["status"] == "rejected" and report["reasons"] == [reason]
    assert report["simulatedSteps"] == 6 and report["remainingSteps"] == 24
    assert not (tmp_path / "failed/result.json").exists()
    assert (tmp_path / "failed/trajectory.json").exists()


@pytest.mark.parametrize("kind", ["tilt", "closure"])
def test_warmup_tilt_and_closure_do_not_become_measured_failures(rig, tmp_path, kind):
    rig.fault = fault(kind, lambda tick: tick <= 4)
    baseline = evaluation.evaluate(rig.checked, tmp_path / "baseline", options(short_gate=0))
    measured = evaluation.evaluate(rig.checked, tmp_path / "gated", options())
    assert measured["simulationGate"]["status"] == "passed"
    assert measured["metrics"] == baseline["metrics"]
    assert not measured["metrics"]["fallen"] and measured["metrics"]["closureWithinTolerance"]
    assert rig.counts["step"] == 60


def test_closure_diagnosis_identifies_first_joint_failure_and_motor_phase(rig, tmp_path):
    rig.fault = fault("closure", lambda tick: tick == 6)
    with pytest.raises(AdapterError, match="joint-closure-error"):
        evaluation.evaluate(rig.checked, tmp_path, options())
    diagnosis = json.loads((tmp_path / "closure-diagnosis.json").read_text())
    report = json.loads((tmp_path / "simulation-gate.json").read_text())
    assert report["diagnostics"]["closureDiagnosis"] == diagnosis
    first = diagnosis["firstExceeded"]
    assert first["joint"] == "motor"
    assert (first["bodyA"], first["bodyB"]) == ("chassis", "rotor")
    assert first["timeSeconds"] == pytest.approx(.006)
    assert first["errorMeters"] == pytest.approx(.00201)
    assert first["separationVectorMeters"] == pytest.approx([.00201, 0, 0])
    assert first["motorAnglesRadians"] == pytest.approx(rig.data[0].qpos[-1:])
    assert first["motorPhaseDegrees"] == pytest.approx(
        (np.degrees(rig.data[0].qpos[-1:]) % 360).tolist())
    assert diagnosis["jointPeaks"][0] == first
    assert "not a proven physical disconnection" in diagnosis["interpretation"]


def test_completed_closure_diagnosis_excludes_settling_and_preserves_peak(rig, tmp_path):
    rig.fault = fault("closure", lambda tick: tick <= 4 or tick == 15)
    measured = evaluation.evaluate(rig.checked, tmp_path, options())
    diagnosis = measured["closureDiagnosis"]
    assert diagnosis == json.loads((tmp_path / "closure-diagnosis.json").read_text())
    assert diagnosis["firstExceeded"]["timeSeconds"] == pytest.approx(.015)
    assert diagnosis["jointPeaks"][0]["errorMeters"] == measured["metrics"]["maxClosureErrorMeters"]
    assert measured["simulationGate"]["diagnostics"]["closureDiagnosis"]["firstExceeded"] is None


def test_cycle_aware_survivor_keeps_the_same_complete_window_and_gait_progress(rig, tmp_path):
    manifest = rig.checked.manifest
    manifest["chassis"] = "chassis"
    manifest["expected"].update(
        joints=[{"id": "motor", "bodyA": "chassis", "bodyB": "rotor"}], gears=[])
    for foot in manifest["feet"]:
        foot["body"] = "rotor"
    baseline = evaluation.evaluate(rig.checked, tmp_path / "baseline", options(cycle_aware=True, short_gate=0))
    measured = evaluation.evaluate(rig.checked, tmp_path / "gated", options(cycle_aware=True))
    assert measured["simulationGate"]["status"] == "passed"
    assert measured["options"]["duration"] > options().duration
    assert measured["gait"] == baseline["gait"]
    assert measured["metrics"] == baseline["metrics"]
    assert len(rig.step_data) == 2 * round(measured["options"]["duration"] / options().timestep)
    assert (tmp_path / "gated/trajectory.json").read_bytes() == (tmp_path / "baseline/trajectory.json").read_bytes()


@pytest.mark.parametrize("kind", ["tilt", "closure"])
def test_later_failures_keep_the_original_full_run_rules(rig, tmp_path, kind):
    rig.fault = fault(kind, lambda tick: tick == 15)
    measured = evaluation.evaluate(rig.checked, tmp_path, options())
    assert measured["simulationGate"]["status"] == "passed"
    assert not measured["metrics"]["eligibleForRanking"]
    assert rig.counts["step"] == 30
    assert measured["metrics"]["fallen"] if kind == "tilt" else not measured["metrics"]["closureWithinTolerance"]


@pytest.mark.parametrize("kind,reason", [("warning", "numerical warnings"), ("nonfinite", "Non-finite")])
def test_existing_numerical_fail_fast_is_not_suppressed(rig, tmp_path, kind, reason):
    rig.fault = fault(kind, lambda tick: tick == 2)
    with pytest.raises(AdapterError, match=reason):
        evaluation.evaluate(rig.checked, tmp_path, options())
    assert rig.counts["step"] == 2
    assert not (tmp_path / "result.json").exists()


@pytest.mark.parametrize("kind", ["self-interference", "forbidden-ground-contact"])
def test_contact_failures_count_even_during_settling_and_preserve_evidence(tmp_path, kind):
    penetrations = {("a", "b"): {"parts": ["a", "b"], "depthMeters": .00010001, "time": 0}} if kind == "self-interference" else {}
    ground = {"forbiddenContactDetected": kind == "forbidden-ground-contact",
              "forbiddenContacts": [{"part": "frame", "firstTime": 0}] if kind == "forbidden-ground-contact" else []}
    frames = [{"time": 0}]
    before = deepcopy((penetrations, ground, frames))
    gate = SimulationPrefixGate(tmp_path, 2, .001, 1000, 20000)
    with pytest.raises(AdapterError, match=kind):
        gate.observe(0, 0, 0, 0, penetrations, ground, frames)
    assert (penetrations, ground, frames) == before
    assert gate.report["measurementSeconds"] == 0
    assert gate.report["simulatedSteps"] == 0 and gate.report["remainingSteps"] == 20000
    assert gate.report["diagnostics"]["groundContact"] == ground
    assert not (tmp_path / "result.json").exists()


def test_exact_existing_thresholds_pass_and_short_trials_clip_the_prefix(tmp_path):
    gate = SimulationPrefixGate(tmp_path, 2, .001, 1, 3)
    gate.observe(3, .003, math.pi / 3, .002,
                 {("a", "b"): {"parts": ["a", "b"], "depthMeters": .0001}},
                 {"forbiddenContactDetected": False}, [])
    assert gate.report["status"] == "passed"
    assert gate.report["plannedPrefixSeconds"] == .003
    assert gate.report["remainingSteps"] == 0


@pytest.mark.parametrize("seconds", [-1, 61, math.inf, math.nan, False, "2", .0015])
def test_gate_rejects_invalid_settings(seconds):
    with pytest.raises(AdapterError, match="short_gate"):
        options(short_gate=seconds).validate()


@pytest.mark.parametrize("command", ["evaluate", "batch"])
def test_standalone_gate_is_explicit_and_configurable(command):
    args = [command, "manifest.json", "--output", "result"]
    assert parser().parse_args(args).short_gate == 0
    assert parser().parse_args(args + ["--short-gate", "2"]).short_gate == 2


@pytest.mark.parametrize("command,exit_code", [("evaluate", 1), ("batch", 1)])
def test_cli_executes_the_configured_gate_and_surfaces_rejection(rig, tmp_path, monkeypatch, capsys, command, exit_code):
    rig.fault = fault("tilt", lambda tick: tick == 6)
    monkeypatch.setattr(cli, "check", lambda *args: rig.checked)
    source = tmp_path / "source"
    (source / "candidate-001").mkdir(parents=True)
    write_json(source / "sweep.json", {"schemaVersion": 1, "generated": [{"directory": "candidate-001"}]})
    output = tmp_path / "output"
    assert cli.main([command, str(source), "--output", str(output), "--ldraw-library", str(tmp_path),
                     "--duration", ".03", "--settle", ".004", "--timestep", ".001", "--short-gate", ".005",
                     "--motor-model", "ideal-servo", "--speed", "3"]) == exit_code
    assert "Short simulation gate rejected" in capsys.readouterr().err
    assert rig.counts["step"] == 6
    target = output / "candidate-001" if command == "batch" else output
    assert json.loads((target / "simulation-gate.json").read_text())["status"] == "rejected"
    assert not (target / "result.json").exists()
    assert (target / "rejection.json").exists() and not (target / "error.json").exists()
    if command == "batch":
        summary = json.loads((output / "batch.json").read_text())
        assert summary["failures"] == 0 and summary["rejections"] == 1 and summary["rankedCandidates"] == []


def rejected_or_complete(checked, output, options, *, on_gate_passed=None):
    if getattr(checked, "hold", None) is not None and not checked.hold.wait(timeout=30):
        raise AdapterError("Prefix fixture timed out.")
    if getattr(checked, "crash", "") == "before":
        os._exit(7)
    if getattr(checked, "reject", False):
        gate = SimulationPrefixGate(output, options.short_gate, .001, 1000, 20000)
        gate.observe(1, .001, 0, 0, {}, {"forbiddenContactDetected": True}, [])
    if on_gate_passed:
        on_gate_passed()
    if getattr(checked, "hold_after", None) is not None and not checked.hold_after.wait(timeout=30):
        raise AdapterError("Full simulation fixture timed out.")
    if getattr(checked, "crash", "") == "after":
        os._exit(7)
    value = result(qualified=True)
    write_json(output / "result.json", value)
    return value


@pytest.mark.parametrize("workers", [1, 2])
def test_rejection_spends_exactly_one_trial_and_has_no_partial_fitness(tmp_path, workers):
    archive = Archive(tmp_path)
    archive.initialize(protocol(), 2, 42, False)
    try:
        births = [archive.record_birth(genome(), None, "test", [], "generated")[0] for _ in range(2)]
        with TrialPool(archive, workers, rejected_or_complete) as pool:
            pool.submit(births[0], SimpleNamespace(reject=True), options(short_gate=2))
            pool.submit(births[1], SimpleNamespace(reject=False), options(short_gate=2))
        assert archive.counts() == {"rejected": 1, "complete": 1}
        assert pool.failures == 0 and pool.rejections == 1
        rejected = archive.db.execute("SELECT * FROM evaluations WHERE birth=?", (births[0],)).fetchone()
        assert rejected["result"] is None and "Short simulation gate rejected" in rejected["error"]
        assert len(archive.outcomes()) == 1 and archive.best()["birth"] == births[1]
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 2
        assert not (tmp_path / "evaluations/0001/result.json").exists()
        assert json.loads((tmp_path / "evaluations/0001/simulation-gate.json").read_text())["status"] == "rejected"
        page = (tmp_path / "history" / f"{births[0]:04d}.html").read_text()
        assert "Short simulation gate" in page and "evaluations/0001/simulation-gate.json" in page
        assert "Simulation rejection" in page and "Trajectory" in page
    finally:
        archive.db.close()


def replacement_protocol():
    value = protocol()
    value["execution"] = {"simulationGateAccounting": {
        "version": "prefix-before-trial-v1", "maxAttemptsPerFullSimulation": 3}}
    return value


@pytest.mark.parametrize("workers", [1, 2])
@pytest.mark.parametrize("survives", [False, True])
def test_replacement_cap_preserves_allowance_and_an_explicit_new_round_can_resume(tmp_path, workers, survives):
    archive = Archive(tmp_path)
    archive.initialize(replacement_protocol(), 1, 42, False)
    try:
        with TrialPool(archive, workers, rejected_or_complete) as pool:
            for index in range(3):
                birth = archive.record_birth(genome(), None, "test", [], "generated")[0]
                pool.submit(birth, SimpleNamespace(reject=index < 2 or not survives), options())
                pool.drain()
                assert sum(archive.counts().values()) == int(survives and index == 2)
            assert pool.exhausted is not survives
            assert pool.failures == 0 and pool.rejections == (2 if survives else 3)
            if not survives:
                with pytest.raises(GateAttemptLimit, match="cap reached"):
                    pool.submit(birth, SimpleNamespace(), options())
        assert len(archive.outcomes()) == int(survives)
        assert archive.db.execute("SELECT budget FROM run").fetchone()[0] == 1
        assert attempt_summary(archive.db)["pausedAtLimit"] is not survives
        from legowalkers_sim.gallery import publish
        publish(archive)
        page = (tmp_path / "index.html").read_text()
        assert "Cheap simulation gate" in page
        assert ("Paused at cheap-attempt cap" in page) is not survives
        rejected = (tmp_path / "history/0001.html").read_text()
        assert "No full-simulation slot consumed" in rejected
        assert "simulation-attempts/0001/simulation-gate.json" in rejected
        if not survives:
            with TrialPool(archive, workers, rejected_or_complete) as resumed:
                assert not resumed.exhausted
                birth = archive.record_birth(genome(), None, "test", [], "generated")[0]
                resumed.submit(birth, SimpleNamespace(), options())
            assert archive.counts() == {"complete": 1}
            assert archive.db.execute("SELECT COUNT(*) FROM simulation_gate_rounds").fetchone()[0] == 2
        assert (tmp_path / "evaluations/0001/result.json").exists()
        assert not (tmp_path / "simulation-attempts" / ("0003" if survives else "0004")).exists()
        if workers > 1:
            assert (tmp_path / "evaluations/0001.log").exists()
            assert archive.db.execute("SELECT log FROM trial_workers").fetchone()[0] == "evaluations/0001.log"
    finally:
        archive.db.close()


def test_pending_prefixes_cannot_oversubscribe_remaining_full_slots(tmp_path):
    archive = Archive(tmp_path)
    archive.initialize(replacement_protocol(), 2, 42, False)
    hold = multiprocessing.get_context("spawn").Event()
    try:
        with TrialPool(archive, 4, rejected_or_complete) as pool:
            try:
                for _ in range(2):
                    birth = archive.record_birth(genome(), None, "test", [], "generated")[0]
                    pool.submit(birth, SimpleNamespace(hold=hold), options())
                assert pool.pending_gates == 2 and pool.full and not pool.exhausted
                assert archive.counts() == {}
                with pytest.raises(AdapterError, match="pool is full"):
                    pool.submit(birth, SimpleNamespace(), options())
                with pytest.raises(AdapterError, match="still running"):
                    archive.recover()
            finally:
                hold.set()
        assert archive.counts() == {"complete": 2}
        assert attempt_summary(archive.db)["attempts"] == 2
    finally:
        archive.db.close()


@pytest.mark.parametrize("phase,counts", [("before", {}), ("after", {"failed": 1})])
def test_worker_crash_spends_full_slot_only_after_promotion(tmp_path, phase, counts):
    archive = Archive(tmp_path)
    archive.initialize(replacement_protocol(), 1, 42, False)
    try:
        with TrialPool(archive, 2, rejected_or_complete) as pool:
            birth = archive.record_birth(genome(), None, "test", [], "generated")[0]
            pool.submit(birth, SimpleNamespace(crash=phase), options())
        assert pool.failures == 1 and pool.rejections == 0
        assert archive.counts() == counts
        diagnostic = "simulation-attempts" if phase == "before" else "evaluations"
        assert (tmp_path / diagnostic / "0001/error.json").exists()
        assert "code 7" in archive.db.execute("SELECT error FROM simulation_attempts").fetchone()[0]
    finally:
        archive.db.close()


@pytest.mark.parametrize("promoted", [False, True])
def test_interruption_keeps_only_promoted_full_slots_spent(tmp_path, promoted):
    archive = Archive(tmp_path)
    archive.initialize(replacement_protocol(), 1, 42, False)
    hold = multiprocessing.get_context("spawn").Event()
    try:
        with pytest.raises(KeyboardInterrupt):
            with TrialPool(archive, 2, rejected_or_complete) as pool:
                birth = archive.record_birth(genome(), None, "test", [], "generated")[0]
                pool.submit(birth, SimpleNamespace(**{"hold_after" if promoted else "hold": hold}), options())
                pid = pool.jobs[0].process.pid
                if promoted:
                    pool.reap(block=True)
                    assert pool.jobs[0].trial == 1
                raise KeyboardInterrupt
        assert archive.counts() == ({"interrupted": 1} if promoted else {})
        error = archive.db.execute("SELECT error FROM simulation_attempts").fetchone()[0]
        assert ("Trial remains spent" in error) is promoted
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
        archive.recover()
    finally:
        archive.db.close()


def test_real_evaluation_handoff_uses_one_model_and_moves_self_contained_evidence(rig, tmp_path, monkeypatch):
    original = evaluation.prepare_xml

    def prepare(checked, output, settings):
        prepared = original(checked, output, settings)
        meshes = output / "meshes"
        meshes.mkdir()
        (meshes / "proof.obj").write_text(
            "v 0 0 0\nv .01 0 0\nv 0 .01 0\nv 0 0 .01\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n")
        root = ET.parse(prepared[0])
        root.getroot().insert(0, ET.Element("compiler", meshdir="meshes"))
        assets = ET.SubElement(root.getroot(), "asset")
        ET.SubElement(assets, "mesh", name="proof", file="proof.obj")
        root.write(prepared[0])
        return prepared

    monkeypatch.setattr(evaluation, "prepare_xml", prepare)
    archive = Archive(tmp_path)
    archive.initialize(replacement_protocol(), 1, 42, False)
    try:
        with TrialPool(archive, 1, evaluation.evaluate) as pool:
            birth = archive.record_birth(genome(), None, "test", [], "generated")[0]
            pool.submit(birth, rig.checked, options())
        assert rig.counts == {"prepare": 1, "load": 1, "data": 1, "step": 30}
        assert archive.counts() == {"complete": 1}
        assert (tmp_path / "evaluations/0001/model.xml").exists()
        assert json.loads((tmp_path / "evaluations/0001/result.json").read_text())["simulationGate"]["status"] == "passed"
        assert not (tmp_path / "simulation-attempts/0001").exists()
        assert mujoco.MjModel.from_xml_path(str(tmp_path / "evaluations/0001/model.xml")).nmesh == 1
    finally:
        archive.db.close()


@pytest.mark.parametrize("promoted,completed", [(False, False), (True, False), (True, True)])
def test_recovery_preserves_promotion_boundary_and_recovers_moved_evidence(tmp_path, promoted, completed):
    from legowalkers_sim.evolution import now

    archive = Archive(tmp_path)
    archive.initialize(replacement_protocol(), 1, 42, False)
    try:
        birth, design = archive.record_birth(genome(), None, "test", [], "generated")
        with archive.db:
            archive.db.execute("INSERT INTO simulation_gate_rounds VALUES (1,1,3,?,'running')", (now(),))
            archive.db.execute("""INSERT INTO simulation_attempts(id,birth,round,slot,status,started)
                VALUES (1,?,1,1,'running',?)""", (birth, now()))
        source = tmp_path / "simulation-attempts/0001"
        write_json(source / "simulation-gate.json", {"status": "passed" if promoted else "undecided"})
        source.with_suffix(".log").write_text("worker evidence")
        if promoted:
            trial = archive.reserve(birth, attempt=1)
            if completed:
                value = result(qualified=True)
                value["compatibility"] = {"ldrawSha256": "fixture-proof"}
                write_json(source / "result.json", value)
                write_json(tmp_path / "designs" / design / "candidate/manifest.json",
                           {"ldrawSha256": value["compatibility"]["ldrawSha256"]})
        archive.recover()
        assert archive.counts() == ({"complete" if completed else "interrupted": 1} if promoted else {})
        attempt = archive.db.execute("SELECT * FROM simulation_attempts").fetchone()
        assert attempt["status"] == ("complete" if completed else "interrupted")
        assert bool(attempt["trial"]) is promoted
        if promoted:
            assert not source.exists()
            assert (tmp_path / "evaluations/0001/simulation-gate.json").exists()
            assert (tmp_path / "evaluations/0001.log").read_text() == "worker evidence"
    finally:
        archive.db.close()
