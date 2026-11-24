import json
import os
from pathlib import Path

import numpy as np
import pytest

from legowalkers_sim.bridge import LDU_TO_METERS, check
from legowalkers_sim.cycles import cycle_plan
from legowalkers_sim.evaluation import EvaluationOptions
from legowalkers_sim.evolution import Generator
from legowalkers_sim.screening import _load_only
from legowalkers_sim.collision import prepare_parts
from legowalkers_sim.payload import apply_payloads, load_profile

ROOT = Path(__file__).parents[2]
OPERATIONS = ("lengthen-foot", "bend-foot", "gear-reduction", "duplicate-branch", "remove-branch",
              "battery-layout", "receiver-layout", "motor-layout")


@pytest.fixture(scope="module")
def physical_gallery(tmp_path_factory):
    library = os.environ.get("LDRAW_LIBRARY")
    if not library:
        pytest.skip("Set LDRAW_LIBRARY for physical mutation reconstruction.")
    directory = tmp_path_factory.mktemp("physical-mutations") / "witnesses"
    generator = Generator(ROOT / ".tools/dotnet/dotnet",
                          ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    generator.invoke("mutation-gallery", "--output", directory)
    return directory, Path(library)


@pytest.mark.integration
@pytest.mark.parametrize("operation", OPERATIONS)
def test_physical_mutations_reconstruct_and_load_without_stepping(physical_gallery, tmp_path, monkeypatch, operation):
    import mujoco
    monkeypatch.setattr(mujoco, "mj_step", lambda *args, **kwargs: pytest.fail("Mutation checks must not step physics."))
    directory, library = physical_gallery
    manifest = directory / operation / "lego-child/manifest.json"
    checked = check(manifest, library)
    report = _load_only(checked, tmp_path)
    assert report["simulationSeconds"] == 0 and report["dynamicTrials"] == 0
    assert report["maxClosureErrorMeters"] < 1e-5
    assert report["initialSelfInterference"] == [], json.dumps(report, indent=2)
    data = json.loads(manifest.read_text())
    if operation in ("battery-layout", "receiver-layout", "motor-layout"):
        layout = json.loads((tmp_path / "mass-layout.json").read_text())
        scene, parts, _ = prepare_parts(checked)
        scene, _ = apply_payloads(scene, parts, checked.manifest, load_profile())
        total = sum(unit.mass for unit in scene.units)
        center = sum(unit.mass*unit.center_of_mass for unit in scene.units)/total/LDU_TO_METERS
        assert layout["totalMassKg"] == pytest.approx(total, abs=1e-12)
        assert np.allclose(layout["centerOfMassLdu"], center, atol=1e-9, rtol=0)
        assert layout["dynamicQualification"] is False
        if operation == "motor-layout":
            geometry = {part.instance: part.geometry for part in parts}
            def depth(name):
                return np.array([v[2] for tri in geometry[name].triangles for v in (tri.v0, tri.v1, tri.v2)])
            bracket = depth("motor-bracket")
            rear_rails = [part.instance for part in parts if part.instance.startswith("rail-120-0-")]
            assert rear_rails
            assert all(bracket.max() <= depth(name).min()+1e-6 for name in rear_rails)
            assert all(depth(f"bush-{index}-100").max() <= bracket.min()+1e-6 for index in (3, 4, 5))
    if operation in ("lengthen-foot", "bend-foot"):
        changes = json.loads((directory / operation / "mutation.json").read_text())["changed"]
        change = next(item for item in changes if item["path"].endswith("/contact"))
        foot = change["path"].split("/")[1]
        assert change["after"][0] > 80
        trajectory = json.loads((directory / operation / "child/motion.json").read_text())
        assert min(next(item["position"][1] for item in frame["feet"] if item["id"] == foot)
                   for frame in trajectory["frames"]) > 0
    if operation == "gear-reduction":
        plan = cycle_plan(data, EvaluationOptions(cycle_aware=True))
        assert plan["motorTurnsPerCycle"] == 5
        assert 21 < plan["durationSeconds"] < 60
        assert any(gear["ratio"] == .2 for gear in data["expected"]["gears"])
    if operation == "duplicate-branch":
        assert data["motion"]["footCount"] == 4
    if operation == "remove-branch":
        assert data["motion"]["footCount"] == 3
