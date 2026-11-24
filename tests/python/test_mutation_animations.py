import copy
import importlib.util
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location("mutation_animations", ROOT / "scripts/mutation_animations.py")
renderer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(renderer)


def document():
    return {"mode": "planar-kinematics-not-dynamics",
            "program": {"bodies": [{"id": "beam", "grounded": False,
                                    "sites": [{"position": {"x": 0, "y": 0}}, {"position": {"x": 20, "y": 0}}]}],
                        "hinges": [], "feet": [{"id": "foot"}]},
            "frames": [{"maxResidualLdu": 0, "poses": [{"body": "beam", "origin": [0, 0, 0], "angle": angle}],
                        "feet": [{"id": "foot", "position": [20, 0, 0]}]} for angle in (0, 3.14, 6.28)]}


def test_animated_comparisons_have_synchronized_frames_and_explicit_limitations():
    value = renderer.animation(document(), document(), "move <hinge>")
    tree = ET.fromstring(value)
    ns = {"s": "http://www.w3.org/2000/svg"}
    animations = tree.findall(".//s:animate", ns)
    assert len(animations) == 4
    assert all(node.attrib["dur"] == "4s" and node.attrib["repeatCount"] == "indefinite" for node in animations)
    assert "not dynamic simulation" in value and "not a walking qualification" in value
    assert "&lt;hinge&gt;" in value
    assert renderer.animation(document(), document(), "test") == renderer.animation(document(), document(), "test")


def test_unsolved_or_mismatched_frames_are_rejected():
    bad = document()
    bad["frames"][0]["maxResidualLdu"] = .01
    with pytest.raises(ValueError, match="unsolved"):
        renderer.animation(document(), bad, "test")
    bad = document()
    bad["frames"].append(copy.deepcopy(bad["frames"][-1]))
    with pytest.raises(ValueError, match="same sampling"):
        renderer.animation(document(), bad, "test")


@pytest.mark.parametrize("operation", ["move-hinge-site", "input-phase", "branch-phase", "add-dyad", "remove-dyad",
                                     "duplicate-branch", "remove-branch", "lengthen-foot", "bend-foot", "gear-reduction",
                                     "battery-layout", "receiver-layout", "motor-layout"])
def test_mutation_gallery_contains_playable_vector_animation(operation):
    path = ROOT / "tests/visualizations" / f"mutation-{operation}.svg"
    tree = ET.fromstring(path.read_text())
    animations = tree.findall(".//{http://www.w3.org/2000/svg}animate")
    frames = 360 if operation == "gear-reduction" else 72
    assert len(animations) == frames*2
    assert all(len(animation.attrib["values"].split(";")) == frames for animation in animations)


def test_reduction_illustration_labels_actual_gear_sizes():
    value = (ROOT / "tests/visualizations/mutation-gear-reduction.svg").read_text()
    assert ">8T</text>" in value and ">40T</text>" in value and ">24T</text>" in value


@pytest.mark.parametrize("operation", ["battery-layout", "receiver-layout", "motor-layout"])
def test_mass_layout_illustrations_show_the_mounted_hardware(operation):
    value = (ROOT / "tests/visualizations" / f"mutation-{operation}.svg").read_text()
    assert ">Battery</text>" in value and ">Motor</text>" in value
    assert "Schematic hardware" in value
    if operation == "receiver-layout":
        assert ">Receiver</text>" in value
