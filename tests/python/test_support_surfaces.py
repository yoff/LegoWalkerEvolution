from copy import deepcopy

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.ground import GroundContacts, foot_regions
from legowalkers_sim.support_surfaces import support_surfaces


def manifest(kind):
    points = ([[0, 0, -10], [0, 0, 10]] if kind == "capsule"
              else [[-40, 24, -10], [-40, 24, 10], [40, 24, -10], [40, 24, 10]])
    return {
        "feet": [{"id": str(i), "logicalFoot": "foot", "body": "shoe", "partInstance": "sole",
                  "position": point, "contactRadiusMeters": .004 if kind == "capsule" else .0016}
                 for i, point in enumerate(points)],
        "parts": [{"id": "sole", "body": "shoe"}], "groundWheels": [],
        "motion": {"logicalFeet": [{"id": "foot", "body": "shoe", "contactShape": kind,
                                   "regions": [str(i) for i in range(len(points))]}]},
    }


@pytest.mark.parametrize("kind", ["capsule", "rectangle"])
def test_only_opted_in_bounded_support_fills_the_gaps_between_anchors(kind):
    source = manifest(kind)
    feet = np.array([foot["position"] for foot in source["feet"]]) * .0004
    position = feet.mean(axis=0) + [0, -.001, 0]
    legacy = deepcopy(source)
    del legacy["motion"]["logicalFeet"][0]["contactShape"]
    old = GroundContacts(legacy)
    old.observe("sole", position, -.0001, 0, feet)
    assert old.forbidden
    bounded = GroundContacts(source)
    bounded.observe("sole", position, -.0001, 0, feet)
    assert bounded.foot_samples == 1 and not bounded.forbidden
    bounded.observe("rocker", position, -.0001, 0, feet)
    bounded.observe("sole", position + [.03, 0, 0], -.0001, 0, feet)
    assert set(bounded.forbidden) == {"rocker", "sole"}


def test_surface_is_rigid_transform_invariant_and_does_not_accept_the_shoe_top():
    source = manifest("rectangle")
    surface, = support_surfaces(source, foot_regions(source))
    points = np.array([foot["position"] for foot in source["feet"]]) * .0004
    assert not surface.contains([0, 0, 0], points)
    rotation = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    translation = np.array([.3, -.5, .12])
    candidate = points.mean(axis=0) + [0, .001, 0]
    assert surface.contains(candidate @ rotation.T + translation, points @ rotation.T + translation)


@pytest.mark.parametrize("change", ["part", "body", "radius", "duplicate", "degenerate", "skew", "nonplanar"])
def test_malformed_support_surface_fails_explicitly(change):
    source = manifest("rectangle")
    if change == "part":
        source["feet"][0]["partInstance"] = "other"
    elif change == "body":
        source["feet"][0]["body"] = "other"
    elif change == "radius":
        source["feet"][0]["contactRadiusMeters"] = .003
    elif change == "duplicate":
        source["motion"]["logicalFeet"][0]["regions"][1] = "0"
    elif change == "degenerate":
        source["feet"][1]["position"] = source["feet"][0]["position"]
    elif change == "skew":
        source["feet"][2]["position"][2] = 1
    else:
        source["feet"][3]["position"][1] = 25
    with pytest.raises(AdapterError):
        GroundContacts(source)


def test_exact_surface_radius_passes_but_larger_distance_does_not():
    source = manifest("capsule")
    surface, = support_surfaces(source, foot_regions(source))
    points = np.array([foot["position"] for foot in source["feet"]]) * .0004
    assert surface.contains([.004, 0, 0], points)
    assert not surface.contains([.004000001, 0, 0], points)


def paired_manifest():
    source = manifest("rectangle")
    inner = deepcopy(source["feet"])
    for foot in inner:
        foot["id"] = "inner-" + foot["id"]
        foot["partInstance"] = "inner-sole"
        foot["position"][2] += 40
    logical = source["motion"]["logicalFeet"][0]
    logical["surfaces"] = [
        {"contactShape": "rectangle", "regions": logical["regions"][:]},
        {"contactShape": "rectangle", "regions": [foot["id"] for foot in inner]},
    ]
    logical["regions"] += [foot["id"] for foot in inner]
    source["feet"] += inner
    source["parts"].append({"id": "inner-sole", "body": "shoe"})
    return source


def test_parallel_soles_are_separate_surfaces_not_a_rectangle_across_the_gap():
    source = paired_manifest()
    feet = np.array([foot["position"] for foot in source["feet"]]) * .0004
    contacts = GroundContacts(source)
    assert len(contacts.surfaces) == 2
    assert contacts.observe("inner-sole", feet[4:].mean(axis=0), -.0001, 0, feet) == "foot"
    assert contacts.observe("sole", feet[:4].mean(axis=0), -.0001, 0, feet) == "foot"
    gap = feet.mean(axis=0)
    assert contacts.observe("sole", gap, -.0001, 0, feet) == "forbidden"
    assert contacts.observe("short-leg", feet[:4].mean(axis=0), -.0001, 0, feet) == "forbidden"


@pytest.mark.parametrize("change", ["duplicate", "omitted", "part", "body"])
def test_parallel_sole_surface_ownership_is_validated(change):
    source = paired_manifest()
    logical = source["motion"]["logicalFeet"][0]
    if change == "duplicate":
        logical["surfaces"][1] = deepcopy(logical["surfaces"][0])
    elif change == "omitted":
        logical["surfaces"].pop()
    elif change == "part":
        for foot in source["feet"][4:]:
            foot["partInstance"] = "sole"
    else:
        source["motion"]["logicalFeet"][0]["surfaces"][1]["body"] = "unrelated-body"
    with pytest.raises(AdapterError):
        GroundContacts(source)
