import importlib.util
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError, LDU_TO_METERS
from legowalkers_sim.collision import CollisionPart
from legowalkers_sim.payload import load_profile
from test_collision import box

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("plan_packaging", ROOT / "scripts/plan_packaging.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)


def test_mass_export_covers_connectors_and_transforms_payload_center(monkeypatch):
    battery = box("58119.dat", [50, 0, 0], [100, 100, 100])
    battery.transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    pin = box("6558.dat", [50, 0, 0], [10, 10, 10])
    parts = [CollisionPart("payload-battery", battery, 0), CollisionPart("mount", pin, 0)]
    checked = SimpleNamespace(body_indices={"chassis": 0}, manifest={
        "parts": [{"id": part.instance} for part in parts],
        "payloads": [{"partInstance": "payload-battery", "product": "8881"}],
    })
    monkeypatch.setattr(planner, "prepare_parts", lambda _, *, profile=None: (None, parts, None))
    profile = load_profile()
    profile["components"]["8881"]["centerLdu"] = [10, 0, 0]
    records = planner.mass_properties(checked, profile)
    assert len(records) == 2
    assert all(item["body"] == "chassis" for item in records)
    battery_record = next(item for item in records if item["id"] == "payload-battery")
    assert battery_record["massKg"] == profile["components"]["8881"]["massKg"]
    assert list(battery_record["center"].values()) == pytest.approx([50, 10, 0])
    assert battery_record["source"] == "payload-profile-estimate"
    pin_record = next(item for item in records if item["id"] == "mount")
    assert pin_record["massKg"] > 0
    assert pin_record["source"] == "ABS-mesh-estimate"


def test_unknown_connector_owner_is_an_explicit_error(monkeypatch):
    part = CollisionPart("mount", box("6558.dat", [0, 0, 0], [10, 10, 10]), 9)
    checked = SimpleNamespace(body_indices={"chassis": 0}, manifest={
        "parts": [{"id": "mount"}], "payloads": [],
    })
    monkeypatch.setattr(planner, "prepare_parts", lambda _, *, profile=None: (None, [part], None))
    with pytest.raises(AdapterError, match="body owner"):
        planner.mass_properties(checked, load_profile())


@pytest.mark.parametrize("relation", ["same", "inside", "ancestor"])
def test_experiment_cannot_write_into_or_around_immutable_parent(tmp_path, relation):
    parent = tmp_path / "input" / "candidate"
    output = {"same": parent, "inside": parent / "plans", "ancestor": tmp_path / "input"}[relation]
    with pytest.raises(AdapterError, match="separate"):
        planner.run(parent, output, tmp_path, tmp_path / "dotnet")


def test_fsharp_phase_balance_and_geometry_self_tests():
    result = subprocess.run([
        str(ROOT / ".tools/dotnet/dotnet"), "fsi",
        str(ROOT / "scripts/packaging_planner.fsx"), "--self-test",
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr + result.stdout
    assert "Packaging self-tests passed" in result.stdout
