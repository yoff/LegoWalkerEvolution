"""Replay a complete hinged-foot offspring without stepping physics or touching a campaign."""

import hashlib
import json
from pathlib import Path
import subprocess

import mujoco
import pytest

from legowalkers_sim.screening import screen_batch


ROOT = Path(__file__).resolve().parents[2]
FILES = ("program.json", "realization.json", "manifest.json", "model.ldr")


def test_whole_walker_loop_replays_and_reconstructs_without_stepping(tmp_path, monkeypatch):
    dotnet = str(ROOT / ".tools/dotnet/dotnet")
    cli = str(ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")

    def run(*args):
        return subprocess.run([dotnet, *args], cwd=ROOT, check=True,
                              capture_output=True, text=True)

    run(cli, "realize-walker", "--feet", "3", "--steps", "72",
        "--output", str(tmp_path / "source"))
    parent = tmp_path / "source/candidate-001"
    before = {name: (parent / name).read_bytes() for name in FILES}
    output = tmp_path / "witness"
    run("fsi", str(ROOT / "scripts/whole_walker_loop.fsx"),
        "--parent", str(parent), "--output", str(output))
    witness = json.loads((output / "witness.json").read_text())
    assert witness["operation"] == "add-hinged-foot"
    assert witness["parentBodies"] == 14
    assert witness["childBodies"] == 16
    assert witness["logicalFeet"] == 3
    assert witness["dynamicTrials"] == 0
    assert witness["sourceParent"] == str(parent)
    assert {item["file"]: item["sha256"] for item in witness["sourceFiles"]} == {
        name: hashlib.sha256(data).hexdigest() for name, data in before.items()}

    replay = tmp_path / "replay"
    run(cli, "propose-walker", "--parent", str(parent), "--output", str(replay),
        "--seed", str(witness["seed"]), "--attempt", str(witness["attempt"]),
        "--limit", "1", "--steps", "72")
    proposal = json.loads((replay / "candidates.json").read_text())
    assert proposal["generated"][0]["genotypeId"] == witness["childGenotypeId"]
    assert proposal["mutationSelection"]["selectedFamily"] == "add-hinged-foot"
    for name in FILES:
        assert (replay / "candidate-001" / name).read_bytes() == (output / "candidate-001" / name).read_bytes()

    def forbidden(*args, **kwargs):
        pytest.fail("A construction witness must never step physics.")

    for name in ("mj_step", "mj_step1", "mj_step2"):
        monkeypatch.setattr(mujoco, name, forbidden)
    report = screen_batch(output, tmp_path / "checked", ROOT / ".tools/ldraw", max_checks=1)
    assert report["counts"]["dynamicTrials"] == 0
    assert report["counts"]["ready"] == 1
    result = report["results"][0]
    assert result["status"] == "ready-for-dynamics"
    assert result["compatibility"]["bodies"] == 16
    assert result["compatibility"]["motors"] == 1
    assert result["loadOnly"]["simulationSeconds"] == result["loadOnly"]["dynamicTrials"] == 0
    assert result["loadOnly"]["maxClosureErrorMeters"] <= 0.002
    assert result["loadOnly"]["initialSelfInterference"] == []
    assert before == {name: (parent / name).read_bytes() for name in FILES}
