"""Native construction holdout checks; zero dynamics and no campaign writes."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import mujoco
import pytest

from legowalkers_sim.bridge import AdapterError


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def checker():
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        spec = importlib.util.spec_from_file_location("native_loop_check", ROOT / "scripts/check_parallelogram_benchmark.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.path.remove(str(ROOT / "scripts"))


@pytest.fixture(scope="module")
def native_output(tmp_path_factory):
    output = tmp_path_factory.mktemp("native-parallelogram") / "generated"
    subprocess.run([str(ROOT / ".tools/dotnet/dotnet"), "fsi",
                    str(ROOT / "scripts/parallelogram_benchmark.fsx"), "--output", str(output)],
                   cwd=ROOT, check=True, capture_output=True, text=True)
    return output


def test_both_native_l_orientations_and_depth_sides_reconstruct_without_stepping(checker, native_output, tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("A construction benchmark must never step physics.")

    for name in ("mj_step", "mj_step1", "mj_step2"):
        monkeypatch.setattr(mujoco, name, forbidden)
    output = tmp_path / "checked"
    report = checker.verify(native_output, output, ROOT / ".tools/ldraw")
    assert report["status"] == "pass"
    assert report["dynamicTrials"] == report["simulationSeconds"] == 0
    assert len(report["results"]) == 4
    for result in report["results"]:
        assert result["status"] == "pass"
        assert result["reconstruction"]["bodies"] == 8
        assert result["reconstruction"]["joints"] == 10
        assert result["reconstruction"]["motors"] == 1
        assert len(result["connectorEngagement"]) == 12
        assert all(contact["fullSocketWalls"] and contact["droppedContacts"] == 0
                   for contact in result["connectorEngagement"])
        assert result["loadOnly"]["simulationSeconds"] == result["loadOnly"]["dynamicTrials"] == 0
        assert result["loadOnly"]["initialSelfInterference"] == []
    assert json.loads((output / "check.json").read_text()) == report
    with pytest.raises(AdapterError, match="must be empty"):
        checker.verify(native_output, output, ROOT / ".tools/ldraw")


def test_native_check_refuses_to_write_within_immutable_input(checker, native_output):
    with pytest.raises(AdapterError, match="outside the immutable benchmark"):
        checker.verify(native_output, native_output / "checks", ROOT / ".tools/ldraw")
