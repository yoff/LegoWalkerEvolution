import importlib.util
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import numpy as np
from lego_technic_sim.ldraw.model import LDrawPart, Triangle
from lego_technic_sim.ldraw.parser import LDrawParser
from lego_technic_sim.physics.connection_ports import ConnectionPort, PortType
from lego_technic_sim.physics.connectors import is_connector
from lego_technic_sim.physics.unit_builder import _find_port_connections, build_units_and_joints
from lego_technic_sim.physics.model import Unit

from legowalkers_sim.bridge import AdapterError, ConstructionConnectivityError, JointPreservationError, verify_construction_connectivity, verify_joint_preservation, write_json
from test_bridge import scene_data

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
spec = importlib.util.spec_from_file_location("construct_packaging", ROOT / "scripts/construct_packaging.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)
sys.path.pop(0)

@pytest.mark.parametrize("reduction,proof_search", [(2, True), (5, False)])
def test_motor_reduction_requires_a_supported_new_proof_scope(tmp_path, reduction, proof_search):
    with pytest.raises(AdapterError, match="Motor reduction requires proof search and must be 1 or 5"):
        planner.run(tmp_path / "missing", tmp_path / "output", tmp_path, tmp_path,
                    proof_search=proof_search, motor_reduction=reduction)

def connectivity_fixture():
    manifest, build, _ = scene_data()
    manifest["parts"][1]["body"] = "chassis"
    manifest["parts"][2]["part"] = build.parts[2].part_id = "4459.dat"
    axis = np.array([0., 1., 0.])
    for index in (0, 1):
        build.parts[index].ports = [ConnectionPort(PortType.ROUND_HOLE, build.parts[index].position.copy(), axis)]
    build.parts[2].triangles = [Triangle(np.array([-4., -10., -4.]), np.array([4., 30., 4.]), np.array([4., -10., -4.]))]
    vector = lambda point: dict(zip(("x", "y", "z"), point))
    bond = {"connector": "pin", "partA": "beam-a", "partB": "beam-b",
            "positionA": vector([0., 0., 0.]), "positionB": vector([0., 20., 0.]),
            "axisA": vector(axis), "axisB": vector(axis)}
    proof = {"scope": "frozen", "goals": [
        {"id": "rigid-path-beam-b", "kind": "rigid-path-to-frame", "subject": "beam-b"},
        {"id": "library-connectivity", "kind": "exact-rigid-connectivity", "subject": "chassis"}],
        "steps": [{"goal": "frame", "rule": "retain-ground-scaffold", "parts": ["beam-a"], "bonds": []},
                  {"goal": "bracket", "rule": "two-beam-covered-pin-bracket", "parts": ["beam-b", "pin"], "bonds": []},
                  {"goal": "bonds-bracket", "rule": "coaxial-friction-pin-port-bonds", "parts": [], "bonds": [bond]}]}
    return proof, manifest, build


@pytest.mark.parametrize("rule", ["two-beam-covered-pin-bracket", "three-beam-covered-pin-bracket", "multi-beam-covered-pin-bracket"])
def test_exact_connections_emit_a_rigid_path_witness(rule):
    proof, manifest, build = connectivity_fixture()
    proof["steps"][1]["rule"] = rule
    result = verify_construction_connectivity(proof, manifest, build)
    assert result["status"] == "pass"
    assert result["verifiedGoals"] == ["library-connectivity"]
    assert result["paths"] == [{"part": "beam-b", "frame": "beam-a",
                                "edges": [{"from": "beam-a", "to": "beam-b", "connector": "pin"}]}]

@pytest.mark.parametrize("rule", ["two-beam-covered-pin-bracket", "three-beam-covered-pin-bracket", "multi-beam-covered-pin-bracket"])
def test_every_bracket_pin_requires_an_explicit_rigid_bond(rule):
    proof, manifest, build = connectivity_fixture()
    proof["steps"][1]["rule"] = rule
    proof["steps"][2]["bonds"] = []
    with pytest.raises(AdapterError, match="omitted a bracket pin's rigid bond"):
        verify_construction_connectivity(proof, manifest, build)


def preservation_fixture():
    _, manifest, build = connectivity_fixture()
    manifest["parts"][1]["body"] = "link"
    manifest["parts"][2].update(id="pivot-shared", part="3673.dat")
    build.parts[2].part_id = "3673.dat"
    manifest["grammarVersion"] = "technic-motion-v1"
    manifest["motion"] = {"sharedPivots": [{"id": "shared", "bodies": ["chassis", "link"]}]}
    manifest["expected"]["joints"][0]["connector"] = "pivot-shared"
    goal = {"id": "preserve-pivot-shared", "kind": "exact-joint-preservation", "subject": "pivot-shared",
            "requires": ["frame", "walking", "drive"]}
    proof = {"scope": "frozen", "goals": [goal],
             "steps": [{"goal": "frame", "parts": ["beam-a"]},
                       {"goal": "walking", "parts": ["beam-b", "pivot-shared"]},
                       {"goal": "drive", "parts": [], "introduced": [goal["id"]]}]}
    return proof, manifest, build


def test_joint_contact_feedback_does_not_focus_the_entire_merged_chassis():
    pivot = {"connector": "pivot", "status": "reject", "unexpectedParts": ["gear"],
             "mergedParts": ["rail", "motor"], "splitParts": [],
             "expectedBodies": ["chassis", "link"], "mergedBodies": ["chassis"],
             "splitBodies": [], "failures": ["unexpected gear engagement"]}
    error = JointPreservationError({"pivots": [pivot]}, None)
    assert error.repair_hints["kind"] == "joint-port-engagement"
    assert error.repair_hints["parts"] == ["gear", "pivot"]
    assert error.joint_preservation["pivots"][0]["mergedParts"] == ["rail", "motor"]


def test_joint_preservation_is_explicitly_verified_in_the_complete_assembly():
    proof, manifest, build = preservation_fixture()
    evidence = verify_joint_preservation(proof, manifest, build, build_units_and_joints(build))
    assert evidence["status"] == "pass"
    assert evidence["verifiedGoals"] == ["preserve-pivot-shared"]


def test_adding_a_gear_invalidates_an_earlier_pivot_proof():
    proof, manifest, build = preservation_fixture()
    assert verify_joint_preservation(proof, manifest, build, build_units_and_joints(build))["status"] == "pass"
    transform = np.eye(4)
    transform[:3, 3] = [0, 10, 0]
    gear = LDrawPart("3648b.dat", 7, transform, build.parts[0].triangles)
    gear.ports = [ConnectionPort(PortType.AXLE_HOLE, gear.position.copy(), np.array([0., 1., 0.]))]
    build.parts.append(gear)
    manifest["parts"].append({"id": "new-gear", "part": "3648b.dat", "color": 7,
                              "position": [0, 10, 0], "rotation": np.eye(3).tolist(), "body": "shaft"})
    proof["steps"][2]["parts"] = ["new-gear"]
    evidence = verify_joint_preservation(proof, manifest, build, build_units_and_joints(build))
    assert evidence["status"] == "reject"
    assert evidence["verifiedGoals"] == []
    assert evidence["pivots"][0]["unexpectedParts"] == ["new-gear"]
    error = JointPreservationError(evidence, None)
    assert error.repair_hints["kind"] == "joint-port-engagement"
    assert error.repair_hints["parts"] == ["new-gear", "pivot-shared"]


@pytest.mark.parametrize("invalid", ["omitted", "unowned", "missing-context"])
def test_joint_obligations_cannot_omit_a_pivot_its_owner_or_a_physical_dependency(invalid):
    proof, manifest, build = preservation_fixture()
    if invalid == "omitted":
        proof["goals"] = []
    elif invalid == "unowned":
        proof["steps"][2]["introduced"] = []
    else:
        proof["goals"][0]["requires"] = ["walking", "drive"]
    with pytest.raises(AdapterError):
        verify_joint_preservation(proof, manifest, build, build_units_and_joints(build))


@pytest.mark.parametrize("changed", ["axis", "position", "merged-body", "split-body"])
def test_joint_preservation_rejects_changed_axes_positions_and_rigid_partitions(changed):
    proof, manifest, build = preservation_fixture()
    scene = build_units_and_joints(build)
    if changed == "axis":
        scene.joints[0].axis = np.array([1., 0., 0.])
    elif changed == "position":
        scene.joints[0].position += np.array([.001, 0., 0.])
    elif changed == "merged-body":
        scene.units = [Unit([build.parts[0], build.parts[1]])]
        scene.joints = []
    else:
        transform = np.eye(4)
        transform[:3, 3] = [40, 20, 0]
        extra = LDrawPart("32523.dat", 7, transform, build.parts[0].triangles)
        build.parts.append(extra)
        manifest["parts"].append({"id": "link-extra", "part": "32523.dat", "color": 7,
                                  "position": [40, 20, 0], "rotation": np.eye(3).tolist(), "body": "link"})
        proof["steps"][1]["parts"].append("link-extra")
        scene.units.append(Unit([extra]))
    evidence = verify_joint_preservation(proof, manifest, build, scene)
    assert evidence["status"] == "reject"
    assert evidence["verifiedGoals"] == []
    if changed == "split-body":
        assert evidence["pivots"][0]["splitParts"] == ["link-extra"]
        assert JointPreservationError(evidence, None).repair_hints["kind"] == "rigid-body-partition"


def test_coaxial_bond_lost_to_connector_capacity_is_not_a_connection_proof():
    proof, manifest, build = connectivity_fixture()
    transform = np.eye(4)
    transform[:3, 3] = [0, 10, 0]
    competitor = LDrawPart("32523.dat", 7, transform, build.parts[0].triangles)
    competitor.ports = [ConnectionPort(PortType.ROUND_HOLE, competitor.position.copy(), np.array([0., 1., 0.]))]
    build.parts.append(competitor)
    manifest["parts"].append({"id": "competitor", "part": "32523.dat", "color": 7,
                              "position": [0, 10, 0], "rotation": np.eye(3).tolist(), "body": "other"})
    result = verify_construction_connectivity(proof, manifest, build)
    assert result["status"] == "reject"
    assert result["verifiedGoals"] == []
    assert result["disconnectedParts"] == ["beam-b"]
    assert result["failedBonds"][0]["failedParts"] == ["beam-b"]
    assert {"part": "competitor", "kind": "rigid"} in result["failedBonds"][0]["engagements"]
    error = ConstructionConnectivityError(result)
    assert error.repair_hints["parts"] == ["beam-a", "beam-b", "pin"]


@pytest.mark.parametrize("stock", ["32002.dat", "3673.dat", "32556.dat"])
def test_freely_spinning_pin_cannot_witness_a_rigid_path(stock):
    proof, manifest, build = connectivity_fixture()
    manifest["parts"][2]["part"] = build.parts[2].part_id = stock
    with pytest.raises(AdapterError, match="free bearings"):
        verify_construction_connectivity(proof, manifest, build)

@pytest.mark.parametrize("stock, low, high", [("3673.dat", -10., 30.), ("32556.dat", -20., 40.)])
def test_two_and_three_l_smooth_pins_preserve_articulated_engagements(stock, low, high):
    _, _, build = connectivity_fixture()
    pin = build.parts[2]
    pin.part_id = stock
    pin.triangles = [Triangle(np.array([-4., low, -4.]), np.array([4., high, 4.]), np.array([4., low, -4.]))]
    assert _find_port_connections(pin, [(0, build.parts[0]), (1, build.parts[1])]) == [
        (0, "revolute"), (1, "revolute")]

def test_real_motor_side_pin_rigidly_engages_a_perpendicular_block(tmp_path):
    library = os.environ.get("LDRAW_LIBRARY")
    if not library:
        pytest.skip("Set LDRAW_LIBRARY for actual motor mount geometry.")
    model = tmp_path / "side-mount.ldr"
    model.write_text(
        "0 Motor side mount\n"
        "1 7 0 0 0 1 0 0 0 1 0 0 0 1 58121.dat\n"
        "1 7 70 0 10 1 0 0 0 1 0 0 0 1 6536.dat\n"
        "1 8 60 0 10 1 0 0 0 1 0 0 0 1 4459.dat\n")
    build = LDrawParser(parts_dir=Path(library).resolve()).parse_build(model)
    assert _find_port_connections(build.parts[2], [(0, build.parts[0]), (1, build.parts[1])]) == [
        (0, "rigid"), (1, "rigid")]


def test_every_new_chassis_piece_requires_an_explicit_path_obligation():
    proof, manifest, build = connectivity_fixture()
    proof["goals"] = proof["goals"][1:]
    with pytest.raises(AdapterError, match="every new chassis part"):
        verify_construction_connectivity(proof, manifest, build)


def test_nine_l_axle_is_supported_by_simulator_and_bound_into_oracle_scope(monkeypatch, tmp_path):
    assert is_connector("60485.dat")
    write_json(tmp_path / "realization.json", {"bodies": [], "mounts": [], "connections": []})
    seen = []

    def fingerprint(library, parts):
        seen.extend(parts)
        return "recursive-dependency-hash"

    monkeypatch.setattr(planner, "library_fingerprint", fingerprint)
    monkeypatch.setattr(planner.screening, "_selected_protocol", lambda *a: {"version": "test"})
    scope = planner._oracle_scope({"parent": str(tmp_path), "drives": []}, tmp_path)
    assert "60485.dat" in seen
    assert "6536.dat" in seen
    assert len(scope) == 64


def test_reconstruction_failure_is_not_a_mass_or_viability_success(monkeypatch, tmp_path):
    result = {
        "results": [{"status": "rejected", "simulationSeconds": 0, "error": "partition mismatch"}],
        "counts": {"dynamicTrials": 0, "reconstructionAttempts": 1},
    }
    monkeypatch.setattr(planner.screening, "screen_batch", lambda *a, **kw: result)
    report, spent, mass = planner._screen(tmp_path, {"genotypeId": "a"}, tmp_path / "check",
                                          tmp_path, None, tmp_path)
    assert report["status"] == "rejected"
    assert spent == 1
    assert mass == {"status": "unavailable-reconstruction-rejected"}


def test_offline_screen_cannot_advance_dynamics(monkeypatch, tmp_path):
    result = {"results": [{"simulationSeconds": 1}], "counts": {"dynamicTrials": 1, "reconstructionAttempts": 1}}
    monkeypatch.setattr(planner.screening, "screen_batch", lambda *a, **kw: result)
    with pytest.raises(AdapterError, match="advanced dynamics"):
        planner._screen(tmp_path, {"genotypeId": "a"}, tmp_path / "check", tmp_path, None, tmp_path)


def test_reconstructed_mass_and_phase_balance_are_wired_without_rechecking(monkeypatch, tmp_path):
    candidate = tmp_path / "candidate"
    candidate.mkdir()
    for name in ("manifest.json", "program.json", "realization.json"):
        (candidate / name).write_text("{}")
    checked = SimpleNamespace(report={"partsDependencySha256": "parts"})
    monkeypatch.setattr(planner.screening, "check", lambda *a, **kw: checked)
    calls = []

    def screen(*args, **kwargs):
        calls.append("screen")
        planner.screening.check(candidate / "manifest.json", tmp_path)
        return {"results": [{"status": "ready-for-dynamics", "simulationSeconds": 0}],
                "counts": {"dynamicTrials": 0, "reconstructionAttempts": 1},
                "protocol": {"payloadProfileSha256": "payload"}}

    monkeypatch.setattr(planner.screening, "screen_batch", screen)
    monkeypatch.setattr(planner, "mass_properties", lambda c, p: [{"id": "frame", "massKg": .5}])

    def balance(command, **kwargs):
        calls.append("balance")
        write_json(Path(command[-1]), {"meanHeightLdu": 20, "worstMarginLdu": -1, "outsideSupportSamples": 2})
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(planner.subprocess, "run", balance)
    _, spent, mass = planner._screen(tmp_path, {"directory": "candidate", "genotypeId": "a"},
                                     tmp_path / "check", tmp_path, None, tmp_path / "dotnet")
    assert calls == ["screen", "balance"]
    assert spent == 1
    assert mass["totalMassKg"] == .5
    assert mass["phaseBalance"]["worstMarginLdu"] == -1
    assert json.loads((tmp_path / "check/mass-properties.json").read_text())["parts"][0]["id"] == "frame"


@pytest.mark.parametrize("relation", ["same", "inside", "ancestor"])
def test_construction_keeps_combined_inputs_immutable(tmp_path, relation):
    source = tmp_path / "combined"
    output = {"same": source, "inside": source / "new", "ancestor": tmp_path}[relation]
    with pytest.raises(AdapterError, match="separate"):
        planner.run(source, output, tmp_path, tmp_path, 1)


def test_feedback_candidates_share_one_reconstruction_budget(monkeypatch, tmp_path):
    combined, parent, output = tmp_path / "combined", tmp_path / "parent", tmp_path / "result"
    parent.mkdir()
    for name in ("program.json", "realization.json", "manifest.json"):
        (parent / name).write_text("{}")
    write_json(combined / "combined-input.json", {"parent": str(parent)})
    input_hash = planner.digest(combined / "combined-input.json")
    write_json(combined / "plans/combined-plan.json", {
        "inputSha256": input_hash, "plans": [
            {"id": prefix+"-target", "driveId": prefix,
             "metrics": {"clearanceScore": 1, "worstMarginLdu": -1, "meanHeightLdu": 1}}
            for prefix in ("root-05-interior-rearward-output", "root-04-interior-rearward-output")]})
    generations, screens = [], []

    def generate(arguments, folder, dotnet):
        generations.append(arguments[0])
        folder.mkdir(parents=True)
        return {"rejected": [], "generated": [{"directory": "candidate", "genotypeId": str(len(generations))}]}

    def screen(source, entry, check, *args):
        screens.append(source)
        status = "rejected" if len(screens) == 1 else "ready-for-dynamics"
        report = {"status": status}
        if status == "rejected":
            report["repairHints"] = {"kind": "rigid-body-partition", "parts": ["link"], "bodies": ["shaft"]}
        return report, 1, {"status": "unavailable-reconstruction-rejected"}

    monkeypatch.setattr(planner, "_generate", generate)
    monkeypatch.setattr(planner, "_screen", screen)
    result = planner.run(combined, output, tmp_path, tmp_path, max_checks=2)
    assert generations == ["--input", "--input", "--repair-parent"]
    assert len(screens) == result["reconstructionAttempts"] == 2
    assert result["ready"] == 1
    assert result["physicsTrials"] == result["liveContributions"] == 0
    assert result["outcomes"][0]["unverifiedCandidates"][0]["reason"] == "shared-check-budget-exhausted"
    assert result["outcomes"][1]["evaluations"][0]["report"]["status"] == "ready-for-dynamics"


def test_proof_continuation_persists_scoped_evidence_and_shared_budget(monkeypatch, tmp_path):
    combined, parent, output = tmp_path / "combined", tmp_path / "parent", tmp_path / "proof"
    parent.mkdir()
    for name in ("program.json", "realization.json", "manifest.json"):
        (parent / name).write_text("{}")
    write_json(combined / "combined-input.json", {"parent": str(parent), "drives": []})
    write_json(combined / "plans/combined-plan.json", {
        "inputSha256": planner.digest(combined / "combined-input.json"),
        "plans": [{"id": prefix, "driveId": prefix,
                   "metrics": {"clearanceScore": 1, "worstMarginLdu": -1, "meanHeightLdu": 1}}
                  for prefix in ("root-05-interior-rearward-output", "root-04-interior-rearward-output")]})
    monkeypatch.setattr(planner, "_oracle_scope", lambda *args: "oracle")
    calls = []

    def generate(arguments, folder, dotnet):
        calls.append(arguments[0])
        resume = arguments[0] == "--resume"
        accepted = False
        if resume:
            verdict = json.loads(Path(arguments[3]).read_text())
            evidence = Path(verdict["evidencePath"])
            assert planner.digest(evidence) == verdict["evidenceSha256"]
            assert verdict["scope"] == "request-and-oracle"
            accepted = json.loads(evidence.read_text())["status"] == "ready-for-dynamics"
        entry = {"directory": "candidate-01", "assemblyId": f"assembly-{len(calls)}", "genotypeId": f"gene-{len(calls)}"}
        batch = {"rejected": [], "generated": [] if accepted else [entry]}
        write_json(folder / "search-checkpoint.json", {"state": {
            "scope": "request-and-oracle", "pending": None if accepted else {"steps": []},
            "status": "oracle-accepted" if accepted else "awaiting-oracle", "visited": len(calls),
            "pruned": 0, "discarded": 0, "nogoods": [{}] if resume else [], "rejections": [],
        }})
        return batch

    checked = []

    def screen(source, entry, check, *args):
        checked.append(entry["genotypeId"])
        status = "rejected" if len(checked) == 1 else "ready-for-dynamics"
        return {"status": status, "simulationSeconds": 0}, 1, {"status": "unavailable-reconstruction-rejected"}

    monkeypatch.setattr(planner, "_proof_generate", generate)
    monkeypatch.setattr(planner, "_screen", screen)
    result = planner.run(combined, output, tmp_path, tmp_path, 2, proof_search=True)
    assert calls == ["--input", "--input", "--resume", "--resume"]
    assert result["reconstructionAttempts"] == 2
    assert len(set(checked)) == 2
    assert result["physicsTrials"] == result["liveContributions"] == 0
    assert result["outcomes"][0]["proofSearch"]["nogoods"] == 1
    assert result["outcomes"][0]["unverifiedCandidates"][0]["reason"] == "shared-check-budget-exhausted"
    assert result["outcomes"][1]["proofSearch"]["status"] == "oracle-accepted"


def test_proof_generation_cannot_export_without_pending_obligation(monkeypatch, tmp_path):
    write_json(tmp_path / "candidates.json", {"counts": {"dynamicTrials": 0}, "generated": [{"directory": "candidate"}]})
    write_json(tmp_path / "search-checkpoint.json", {"state": {"pending": None}})
    monkeypatch.setattr(planner.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=0))
    with pytest.raises(AdapterError, match="disagree"):
        planner._proof_generate([], tmp_path, tmp_path)


def test_proof_generation_checks_unique_stock_accounting(monkeypatch, tmp_path):
    proof = {"scope": "frozen", "goals": [{"id": "mount", "requires": [], "external": False}],
             "steps": [{"goal": "mount", "parts": ["x", "x"], "dependencies": []}]}
    write_json(tmp_path / "candidates.json", {
        "counts": {"dynamicTrials": 0}, "generated": [{"directory": "candidate", "assemblyId": "a", "genotypeId": "g"}]})
    write_json(tmp_path / "search-checkpoint.json", {
        "oracleScope": "oracle", "state": {
            "pending": proof, "visited": 1, "maximumStates": 64, "frontier": [], "maximumFrontier": 4}})
    write_json(tmp_path / "candidate/construction-proof.json", proof)
    write_json(tmp_path / "candidate/realization.json", {
        "bodies": [{"body": "frame", "parts": [{"id": "x"}]}], "mounts": [], "connections": []})
    monkeypatch.setattr(planner.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=0))
    with pytest.raises(AdapterError, match="uniquely account"):
        planner._proof_generate(["--oracle-scope", "oracle"], tmp_path, tmp_path)


def test_proof_generation_uses_nested_walker_manifest_identity(monkeypatch, tmp_path):
    proof = {"scope": "frozen", "goals": [
        {"id": "mount", "requires": [], "external": False}, {"id": "oracle", "requires": [], "external": True}],
        "steps": [{"goal": "mount", "parts": ["x"], "dependencies": []}]}
    batch = {"counts": {"dynamicTrials": 0},
             "generated": [{"directory": "candidate", "assemblyId": "a", "genotypeId": "g"}]}
    write_json(tmp_path / "candidates.json", batch)
    write_json(tmp_path / "search-checkpoint.json", {
        "oracleScope": "oracle", "state": {
            "pending": proof, "visited": 1, "maximumStates": 64, "frontier": [], "maximumFrontier": 4}})
    write_json(tmp_path / "candidate/construction-proof.json", proof)
    write_json(tmp_path / "candidate/realization.json", {
        "bodies": [{"body": "frame", "parts": [{"id": "x"}]}], "mounts": [], "connections": []})
    model = tmp_path / "candidate/model.ldr"
    model.write_text("0 Proof fixture\n")
    write_json(tmp_path / "candidate/manifest.json", {
        "motion": {"assemblyId": "a", "genotypeId": "g"}, "ldrawSha256": planner.digest(model)})
    monkeypatch.setattr(planner.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=0))
    assert planner._proof_generate(["--oracle-scope", "oracle"], tmp_path, tmp_path) == batch
