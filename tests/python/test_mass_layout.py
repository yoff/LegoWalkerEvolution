import copy
from types import SimpleNamespace

import numpy as np
import pytest
from lego_technic_sim.physics.model import PhysicsScene, Unit

from legowalkers_sim.bridge import AdapterError, LDU_TO_METERS
from legowalkers_sim.collision import CollisionPart
from legowalkers_sim.mass_layout import analyze_layout
from legowalkers_sim.payload import apply_payloads, install_inertia, load_profile, part_mass_center
from test_collision import box


def fixture():
    geometry = box("beam.dat", [0, 0, 0], [100, 100, 100])
    checked = SimpleNamespace(manifest={
        "feet": [{"id": str(i), "position": [x, 100, z], "contactRadiusMeters": 0}
                 for i, (x, z) in enumerate([(-60, -60), (60, -60), (60, 60), (-60, 60)])]})
    scene = PhysicsScene(units=[
        Unit([geometry], mass=1, center_of_mass=np.zeros(3)),
        Unit([geometry], mass=3, center_of_mass=np.array([.02, 0, 0]))])
    return checked, scene, [CollisionPart("beam", geometry, 0)], load_profile()


def test_mass_weighted_center_signed_support_margin_and_height():
    report = analyze_layout(*fixture())
    assert report["totalMassKg"] == 4
    assert report["centerOfMassLdu"] == pytest.approx([37.5, 0, 0])
    assert report["heightAboveLowestFootMeters"] == pytest.approx(.04)
    assert report["supportHalfPlaneMarginMeters"] == pytest.approx(.009)
    assert report["warnings"] == []
    assert report["dynamicQualification"] is False


def test_inertia_includes_mass_distribution_and_parallel_axis_terms():
    report = analyze_layout(*fixture())
    inertia = report["inertia"]
    intrinsic = 4 * (2 * .08 ** 2) / 12
    parallel = 1 * .015 ** 2 + 3 * .005 ** 2
    expected = np.diag([intrinsic, intrinsic + parallel, intrinsic + parallel])
    assert np.array(inertia["tensorAtCenterOfMassLDrawKgMetersSquared"]) == pytest.approx(expected)
    assert inertia["principalMomentsKgMetersSquared"] == pytest.approx(np.linalg.eigvalsh(expected))
    assert inertia["radiiOfGyrationMeters"] == pytest.approx(np.sqrt(np.linalg.eigvalsh(expected) / 4))
    assert len(inertia["initialSupportAxes"]) == 4
    for edge in inertia["initialSupportAxes"]:
        axis = np.array(edge["axisLDraw"])
        a = edge["edgeXZMeters"][0]
        offset = np.array([.015 - a[0], -.04, -a[1]])
        expected_edge = axis @ expected @ axis + 4 * (offset @ offset - (offset @ axis) ** 2)
        assert edge["inertiaKgMetersSquared"] == pytest.approx(expected_edge)
        assert edge["gravityMomentMagnitudeNm"] == pytest.approx(
            4 * 9.81 * abs(edge["signedSupportMarginMeters"]))
    assert inertia["dynamicQualification"] is False
    assert "neither universally" in inertia["ranking"]


def test_equal_centers_of_mass_can_have_different_tipping_inertia():
    reports = []
    for spread in (.002, .02):
        checked, scene, parts, profile = fixture()
        scene.units[0].center_of_mass = np.array([-3 * spread, 0, 0])
        scene.units[1].center_of_mass = np.array([spread, 0, 0])
        reports.append(analyze_layout(checked, scene, parts, profile))
    assert reports[0]["centerOfMassLdu"] == pytest.approx(reports[1]["centerOfMassLdu"])
    first = np.array(reports[0]["inertia"]["tensorAtCenterOfMassLDrawKgMetersSquared"])
    second = np.array(reports[1]["inertia"]["tensorAtCenterOfMassLDrawKgMetersSquared"])
    assert second[0, 0] == pytest.approx(first[0, 0])
    assert second[1, 1] > first[1, 1] and second[2, 2] > first[2, 2]


def test_payload_compound_inertia_is_reused_in_ldraw_axes():
    checked, scene, parts, profile = fixture()
    tensor = np.diag([.01, .02, .03])
    report = analyze_layout(checked, scene, parts, profile, inertia_tensors={0: tensor})
    no_override = analyze_layout(checked, scene, parts, profile)
    delta = (np.array(report["inertia"]["tensorAtCenterOfMassLDrawKgMetersSquared"])
             - np.array(no_override["inertia"]["tensorAtCenterOfMassLDrawKgMetersSquared"]))
    assert delta == pytest.approx(np.diag([.01, .03, .02]) - np.eye(3) * (2 * .08 ** 2 / 12))


@pytest.mark.parametrize("tensor", [np.diag([1., 1., 3.]), np.diag([float("nan"), 1., 1.]),
                                    np.array([[1., .1, 0.], [0., 1., 0.], [0., 0., 1.]])])
def test_invalid_layout_inertia_is_explicitly_rejected(tensor):
    with pytest.raises(AdapterError, match="Mass-layout inertia"):
        analyze_layout(*fixture(), inertia_tensors={0: tensor})


def test_mesh_mass_estimate_is_translation_invariant_even_for_open_ldraw_meshes():
    first = box("open.dat", [0, 0, 0], [10, 10, 10])
    moved = box("open.dat", [280, 0, 150], [10, 10, 10])
    first.triangles = first.triangles[2:]
    moved.triangles = moved.triangles[2:]
    mass, center = part_mass_center(CollisionPart("first", first, 0))
    other_mass, other_center = part_mass_center(CollisionPart("moved", moved, 0))
    assert other_mass == pytest.approx(mass, rel=1e-12)
    assert other_center == pytest.approx(center+np.array([280, 0, 150])*LDU_TO_METERS)

@pytest.mark.parametrize("offset", [[0, 0, 0], [331, 132, 200], [-650, 270, -200]])
def test_prepared_structural_units_use_part_local_mass_not_upstream_world_origin(offset):
    from lego_technic_sim.ldraw.model import LDrawBuild
    from lego_technic_sim.physics.mesh_properties import ABS_DENSITY_KG_PER_M3, mesh_volume_and_com
    from legowalkers_sim.bridge import CheckedModel
    from legowalkers_sim.collision import prepare_parts
    geometry = box("3710.dat", np.array(offset), [40, 4, 10])
    geometry.triangles = geometry.triangles[2:]
    volume, center = mesh_volume_and_com(geometry.triangles)
    original = PhysicsScene(units=[Unit([geometry], volume*ABS_DENSITY_KG_PER_M3, center)])
    manifest = {"parts": [{"id": "plate", "part": "3710.dat", "body": "sole"}],
                "expected": {"joints": [], "gears": []}}
    checked = CheckedModel(manifest, original, {"sole": 0}, {}, {}, LDrawBuild("test", [geometry]))
    scene, parts, _ = prepare_parts(checked)
    mass, normalized = part_mass_center(parts[0])
    assert scene.units[0].mass == pytest.approx(mass)
    assert scene.units[0].center_of_mass == pytest.approx(normalized)
    assert original.units[0].mass == volume*ABS_DENSITY_KG_PER_M3
    assert original.units[0].center_of_mass == pytest.approx(center)


def test_oppositely_wound_components_are_rejected_before_near_zero_volume_division():
    first = box("cancelled.dat", [-30, 0, 0], [10, 10, 10])
    other = box("other.dat", [30, 0, 0], [10, 10, 10])
    from lego_technic_sim.ldraw.model import Triangle
    first.triangles += [Triangle(t.v0, t.v2, t.v1) for t in other.triangles]
    with pytest.raises(AdapterError, match="Cancelled signed mesh volume: broken"):
        part_mass_center(CollisionPart("broken", first, 0))


def test_finite_outside_mass_center_is_rejected(monkeypatch):
    import legowalkers_sim.payload as payload
    geometry = box("beam.dat", [0, 0, 0], [10, 10, 10])
    monkeypatch.setattr(payload, "mesh_volume_and_com", lambda ts: (1e-9, np.array([1e12, 0, 0])))
    with pytest.raises(AdapterError, match="component center outside physical geometry: broken"):
        part_mass_center(CollisionPart("broken", geometry, 0))


@pytest.mark.parametrize("tensor", [
    np.diag([1., 1., 3.]), np.diag([0., 1., 1.]), np.diag([float("nan"), 1., 1.]),
    np.array([[1., .1, 0.], [0., 1., 0.], [0., 0., 1.]]),
])
def test_nonphysical_inertia_cannot_reach_export(tensor):
    with pytest.raises(AdapterError, match="inertia tensor"):
        install_inertia({}, {0: tensor})


def test_sampled_support_uses_all_foot_paths_and_never_proves_stance():
    checked, scene, parts, profile = fixture()
    checked.manifest["footPaths"] = [{"id": str(i), "points": [[x*2, 120, z*2]]}
                                    for i, (x, z) in enumerate([(-60, -60), (60, -60), (60, 60), (-60, 60)])]
    report = analyze_layout(checked, scene, parts, profile)
    assert report["supportEnvelopeSource"] == "sampled-swept-foot-regions"
    assert report["heightAboveLowestFootMeters"] == pytest.approx(.048)
    assert "not simultaneous stance" in report["gate"]["diagnostics"][1]


def test_outside_support_and_high_com_are_diagnostics_not_static_balance_rejections():
    checked, scene, parts, profile = fixture()
    for foot in checked.manifest["feet"]:
        foot["position"][0] /= 60
        foot["position"][2] /= 60
    report = analyze_layout(checked, scene, parts, profile)
    assert report["supportHalfPlaneMarginMeters"] < 0
    assert len(report["warnings"]) == 2
    assert report["gate"]["outcome"] == "warning"


@pytest.mark.parametrize("mass,center", [(0, [0, 0, 0]), (float("nan"), [0, 0, 0]),
                                       (1, [float("inf"), 0, 0]), (1, [1, 0, 0])])
def test_unphysical_mass_or_center_is_rejected(mass, center):
    checked, scene, parts, profile = fixture()
    scene.units[0].mass = mass
    scene.units[0].center_of_mass = np.array(center)
    with pytest.raises(AdapterError, match="Mass-layout"):
        analyze_layout(checked, scene, parts, profile)


def test_payload_profile_centers_transform_with_the_real_part_and_match_the_physics_scene():
    checked, _, _, profile = fixture()
    geometry = box("58119.dat", [50, 0, 0], [100, 100, 100])
    geometry.transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    profile = copy.deepcopy(profile)
    profile.pop("id", None)
    profile["components"]["8881"]["centerLdu"] = [10, 0, 0]
    parts = [CollisionPart("payload-battery", geometry, 0)]
    checked.manifest["payloads"] = [{"partInstance": "payload-battery", "product": "8881"}]
    scene = PhysicsScene(units=[Unit([geometry], mass=1, center_of_mass=np.zeros(3))])
    scene, _ = apply_payloads(scene, parts, checked.manifest, profile)
    report = analyze_layout(checked, scene, parts, profile)
    assert report["payloadProfile"] is None
    assert len(report["payloadProfileSha256"]) == 64
    assert report["totalMassKg"] == profile["components"]["8881"]["massKg"]
    assert report["centerOfMassLdu"] == pytest.approx([50, 10, 0])
    assert report["centerOfMassLdu"] == pytest.approx(scene.units[0].center_of_mass/LDU_TO_METERS)
    assert report["components"][0]["centerLdu"] == pytest.approx([50, 10, 0])
    profile["components"]["8881"]["centerLdu"] = [1000, 0, 0]
    with pytest.raises(AdapterError, match="component center"):
        analyze_layout(checked, scene, parts, profile)
