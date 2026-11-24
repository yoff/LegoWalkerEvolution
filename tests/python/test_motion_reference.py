"""Optional independent comparison against the original pinned Walker1 model."""

import hashlib
import json
import os
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np
import pytest

from legowalkers_sim.evolution import ROOT


@pytest.mark.integration
def test_walker1_motion_program_matches_upstream_leg_joint_graph(tmp_path):
    source = os.environ.get("LEGOWALKERS_WALKER1_REFERENCE")
    library = os.environ.get("LDRAW_LIBRARY")
    if not source or not library:
        pytest.skip("Set LEGOWALKERS_WALKER1_REFERENCE and LDRAW_LIBRARY for upstream Walker1 comparison.")
    source = Path(source)
    assert hashlib.sha256(source.read_bytes()).hexdigest() == "022ab8b0776a51013bba1767e9247a4a82be6f148804364b36dcf5ad1978b97e"
    from lego_technic_sim.ldraw.parser import LDrawParser
    from lego_technic_sim.physics.unit_builder import build_units_and_joints

    build = LDrawParser(parts_dir=Path(library)).parse_build(source)
    scene = build_units_and_joints(build)
    parts = {
        "ground": ("64178.dat", [0, 0, 0]),
        "crank": ("60483.dat", [-100, 0, -60]),
        "upper": ("32523.dat", [-80, -80, -80]),
        "crossbar": ("32316.dat", [-80, 0, -80]),
        "brace": ("32316.dat", [-50, -40, -100]),
        "long-leg": ("32525.dat", [-74, 18, -100]),
        "short-leg": ("32524.dat", [-104, 58, -100]),
        "sole": ("3701.dat", [-68, 106, -80]),
    }
    mapping = {}
    for name, (part_id, position) in parts.items():
        matches = [index for index, unit in enumerate(scene.units)
                   if any(part.part_id == part_id and np.allclose(part.position, position) for part in unit.bricks)]
        assert len(matches) == 1, (name, matches)
        mapping[matches[0]] = name
    actual = Counter(tuple(sorted((mapping[joint.unit_a_index], mapping[joint.unit_b_index])))
                     for joint in scene.joints
                     if joint.unit_a_index in mapping and joint.unit_b_index in mapping)
    output = tmp_path / "motion"
    subprocess.run([str(ROOT / ".tools/dotnet/dotnet"),
                    str(ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll"),
                    "motion", "--example", "walker1", "--steps", "72", "--output", str(output)],
                   check=True, capture_output=True, text=True)
    program = json.loads((output / "program.json").read_text())["program"]
    expected = Counter(tuple(sorted((hinge["a"]["body"], hinge["b"]["body"]))) for hinge in program["hinges"])
    assert actual == expected and sum(actual.values()) == 10
    assert set(mapping.values()) == {body["id"] for body in program["bodies"]}
    assert len(scene.motors) == 1
