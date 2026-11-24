import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError
from test_collision import box

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
spec = importlib.util.spec_from_file_location("plan_combined", ROOT / "scripts/plan_combined.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)
sys.path.pop(0)


def placement(geometry, instance="constructed-gear-0"):
    return {
        "id": instance, "part": geometry.part_id, "color": geometry.color,
        "position": planner.vector(geometry.position),
        "rotation": {axis: planner.vector(geometry.transform[:3, i]) for i, axis in enumerate("xyz")},
    }


def test_actual_gear_mesh_is_inside_rotation_invariant_envelope():
    geometry = box("3648b.dat", [100, 20, 40], [10, 12, 4])
    geometry.transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    record, envelope = planner.prepared_part(placement(geometry), geometry, "shaft", "gear")
    assert record["massKg"] > 0
    assert envelope["radius"] == pytest.approx(np.hypot(10, 12))
    vertices = np.array([v for tri in geometry.triangles for v in (tri.v0, tri.v1, tri.v2)])
    low, high = np.array(list(envelope["low"].values())), np.array(list(envelope["high"].values()))
    assert np.all(vertices >= low-1e-10)
    assert np.all(vertices <= high+1e-10)


def test_stock_axle_capsule_uses_mesh_length_and_cross_section():
    geometry = box("3705.dat", [50, 10, 20], [8, 8, 80])
    geometry.transform[:3, :3] = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]
    record, envelope = planner.prepared_part(placement(geometry, "axle"), geometry, "root", "shaft")
    assert record["body"] == "root"
    assert list(envelope["segmentStart"].values()) == pytest.approx([50, 10, -60])
    assert list(envelope["segmentEnd"].values()) == pytest.approx([50, 10, 100])
    assert envelope["radius"] == pytest.approx(np.hypot(8, 8))


def test_mesh_mass_is_translation_independent():
    first = box("3648b.dat", [0, 0, 0], [10, 10, 4])
    moved = box("3648b.dat", [280, -60, -40], [10, 10, 4])
    a, _ = planner.prepared_part(placement(first), first, "root", "gear")
    b, _ = planner.prepared_part(placement(moved), moved, "root", "gear")
    assert a["massKg"] == pytest.approx(b["massKg"], rel=1e-12)
    assert list(b["center"].values()) == pytest.approx([280, -60, -40])


def test_placement_mismatch_is_explicit():
    geometry = box("3648b.dat", [0, 0, 0], [10, 10, 4])
    source = placement(geometry)
    source["position"]["x"] = 1
    with pytest.raises(AdapterError, match="placement mismatch"):
        planner.prepared_part(source, geometry, "root", "gear")


def test_unknown_geometry_kind_is_explicit():
    geometry = box("3648b.dat", [0, 0, 0], [10, 10, 4])
    with pytest.raises(AdapterError, match="geometry kind"):
        planner.prepared_part(placement(geometry), geometry, "root", "unknown")


@pytest.mark.parametrize("relation", ["same", "inside", "ancestor"])
def test_combination_keeps_inputs_immutable(tmp_path, relation):
    transmissions = tmp_path / "transmissions"
    output = {"same": transmissions, "inside": transmissions / "plans", "ancestor": tmp_path}[relation]
    with pytest.raises(AdapterError, match="separate"):
        planner.run(transmissions, tmp_path / "mass" / "mass-properties.json", output, tmp_path, tmp_path)


def test_fsharp_combined_self_tests():
    result = subprocess.run([
        str(ROOT / ".tools/dotnet/dotnet"), "fsi", str(ROOT / "scripts/combined_planner.fsx"), "--self-test",
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr + result.stdout
    assert "Combined self-tests passed" in result.stdout
    assert "warning FS" not in result.stderr
