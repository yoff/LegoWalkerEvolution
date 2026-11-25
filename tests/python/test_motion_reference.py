"""Optional independent comparison against the original pinned Walker1 model."""

import hashlib
import json
import math
import os
import shutil
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np
import pytest

from legowalkers_sim.evolution import ROOT


@pytest.mark.integration
def test_stock_jansen_search_returns_socket_fitted_closed_motion_with_measured_stance(tmp_path):
    local_dotnet = ROOT / ".tools/dotnet/dotnet"
    dotnet = str(local_dotnet) if local_dotnet.is_file() else shutil.which("dotnet")
    core = ROOT / "src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"
    if not dotnet or not core.is_file():
        pytest.skip("Build the Debug F# core and provide dotnet for the stock Jansen search.")
    output = tmp_path / "search"
    subprocess.run([dotnet, "fsi", str(ROOT / "scripts/jansen_search.fsx"), "--output", str(output)],
                   cwd=ROOT, check=True, capture_output=True, text=True, timeout=120)
    report = json.loads((output / "search.json").read_text())
    motion = json.loads((output / "motion/motion.json").read_text())
    frames = motion["frames"]
    cycle = frames[:(len(frames)-1)//2]
    feet = np.asarray([frame["feet"][0]["position"] for frame in cycle])
    low = feet[:, 1].max()
    stance = feet[feet[:, 1] >= low-report["thresholds"]["stanceBandLdu"]]
    measured = {
        "stride": float(np.ptp(feet[:, 0])), "lift": float(np.ptp(feet[:, 1])),
        "stanceFraction": len(stance)/len(feet), "stanceTravel": float(np.ptp(stance[:, 0])),
    }
    for name, value in measured.items():
        assert value == pytest.approx(report["chosen"]["metrics"][name], abs=1e-8)
    assert measured["stride"] >= 40 and 10 <= measured["lift"] <= 60
    assert measured["stanceFraction"] >= .25 and measured["stanceTravel"] >= 40
    assert report["verifiedFinalists"] > 0
    for frame in (frames[0], frames[len(cycle)], frames[-1]):
        assert np.allclose(frame["feet"][0]["position"], frames[0]["feet"][0]["position"], atol=1e-5)
    assert max(frame["maxResidualLdu"] for frame in frames) < 1.01e-7

    def vector(value):
        return np.asarray([value[axis] for axis in "xyz"])

    # Independently match the exported rigid triangles to actual molded-part sockets.
    sockets = {
        "32526.dat": [np.array([0., 0., 20.*n]) for n in range(5)]
                     + [np.array([20.*n, 0., 80.]) for n in (1, 2)],
        "32271.dat": [np.array([0., 0., 20.*n]) for n in range(7)]
                     + [np.array([16., 0., 132.]), np.array([32., 0., 144.])],
    }
    stock = {part["body"]: part for part in report["parts"]}
    for body in motion["program"]["bodies"]:
        if body["id"] not in ("upper-triangle", "foot-triangle"):
            continue
        part = stock[body["id"]]
        rotation = np.column_stack([vector(part["rotation"][axis]) for axis in "xyz"])
        position = vector(part["position"])
        angle = body["initial"]["angle"]
        c, s = math.cos(angle), math.sin(angle)
        body_rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        for site in body["sites"]:
            target = vector(body["initial"]["origin"]) + body_rotation @ vector(site["position"])
            assert any(np.linalg.norm(position + rotation @ socket - target) < 1e-6
                       for socket in sockets[part["part"]])


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
