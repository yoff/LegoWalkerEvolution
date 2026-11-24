"""Derived mass and conservative support-envelope diagnostics; never steps physics."""

import hashlib
import json

import numpy as np

from .bridge import AdapterError, LDU_TO_METERS
from .collision import prepare_parts, to_mujoco
from .payload import apply_payloads, load_profile, part_mass_center

VERSION = "mass-layout-v2"


def _hull(points):
    points = sorted(set(map(tuple, points)))
    if len(points) < 3:
        return points

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

    sides = []
    for sequence in (points, reversed(points)):
        side = []
        for p in sequence:
            while len(side) > 1 and cross(side[-2], side[-1], p) <= 0:
                side.pop()
            side.append(p)
        sides.append(side[:-1])
    return sides[0]+sides[1]


def _inertia_layout(scene, center, feet, tensors):
    tensor = np.zeros((3, 3))
    transform = to_mujoco(np.eye(3)).T
    for index, unit in enumerate(scene.units):
        vertices = np.array([v for brick in unit.bricks for triangle in brick.triangles
                             for v in (triangle.v0, triangle.v1, triangle.v2)]) * LDU_TO_METERS
        if not len(vertices) or not np.isfinite(vertices).all():
            raise AdapterError(f"Mass-layout inertia requires finite geometry for unit_{index}.")
        size = np.ptp(vertices, axis=0)
        own = transform.T @ tensors[index] @ transform if index in tensors else np.diag(
            unit.mass / 12 * (np.sum(size * size) - size * size))
        if (own.shape != (3, 3) or not np.isfinite(own).all()
                or not np.allclose(own, own.T, rtol=0, atol=1e-12)):
            raise AdapterError(f"Mass-layout inertia is invalid for unit_{index}.")
        own_moments = np.linalg.eigvalsh(own)
        if (own_moments < 0).any() or own_moments[-1] > own_moments[:2].sum() + 1e-12:
            raise AdapterError(f"Mass-layout inertia is nonphysical for unit_{index}.")
        offset = unit.center_of_mass - center
        tensor += own + unit.mass * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
    moments = np.linalg.eigvalsh(tensor)
    if not np.isfinite(tensor).all() or (moments <= 0).any():
        raise AdapterError("Mass-layout inertia must be finite and positive definite.")
    total = sum(unit.mass for unit in scene.units)
    ground_y = max(foot["position"][1] * LDU_TO_METERS + foot.get("contactRadiusMeters", 0)
                   for foot in feet)
    stance = [foot for foot in feet if ground_y - (
        foot["position"][1] * LDU_TO_METERS + foot.get("contactRadiusMeters", 0)) <= LDU_TO_METERS]
    hull = _hull([(foot["position"][0] * LDU_TO_METERS, foot["position"][2] * LDU_TO_METERS)
                  for foot in stance])
    axes = []
    if len(hull) >= 3:
        for a, b in zip(hull, hull[1:] + hull[:1]):
            axis = np.array([b[0] - a[0], 0, b[1] - a[1]])
            axis /= np.linalg.norm(axis)
            offset = center - np.array([a[0], ground_y, a[1]])
            inertia = float(axis @ tensor @ axis + total * (
                np.dot(offset, offset) - np.dot(offset, axis) ** 2))
            margin = float((axis[0] * offset[2] - axis[2] * offset[0]))
            axes.append({"edgeXZMeters": [list(a), list(b)], "axisLDraw": axis.tolist(),
                         "inertiaKgMetersSquared": inertia, "signedSupportMarginMeters": margin,
                         "gravityMomentMagnitudeNm": abs(total * 9.81 * margin)})
    return {
        "tensorAtCenterOfMassLDrawKgMetersSquared": tensor.tolist(),
        "principalMomentsKgMetersSquared": moments.tolist(),
        "radiiOfGyrationMeters": np.sqrt(moments / total).tolist(),
        "initialSupportAxes": axes,
        "source": "Rigid-group uniform-box estimate; payload compound tensors reused where available.",
        "axisScope": "Nominal initial lowest-foot envelope, not measured active stance contacts.",
        "ranking": "Diagnostic only: neither universally smaller nor larger inertia guarantees better walking.",
        "dynamicQualification": False,
    }


def analyze_layout(checked, scene, parts, profile, *, inertia_tensors=None):
    masses = np.array([unit.mass for unit in scene.units])
    centers = np.array([unit.center_of_mass for unit in scene.units])
    if (not len(masses) or not np.isfinite(masses).all() or (masses <= 0).any()
            or centers.shape != (len(masses), 3) or not np.isfinite(centers).all()):
        raise AdapterError("Mass-layout gate requires finite positive body masses and finite centers.")
    total = float(masses.sum())
    center = (masses[:, None]*centers).sum(axis=0)/total
    if not np.isfinite(total) or not np.isfinite(center).all():
        raise AdapterError("Mass-layout total mass or center is non-finite.")
    bounds = []
    payloads = {item["partInstance"]: profile["components"][item["product"]]
                for item in checked.manifest.get("payloads", [])}
    components = []
    for part in parts:
        vertices = np.array([v for triangle in part.geometry.triangles
                             for v in (triangle.v0, triangle.v1, triangle.v2)])*LDU_TO_METERS
        if not len(vertices) or not np.isfinite(vertices).all():
            raise AdapterError(f"Invalid mass-layout geometry: {part.instance}")
        lower, upper = vertices.min(axis=0), vertices.max(axis=0)
        bounds.append((lower, upper))
        if part.instance in payloads or part.geometry.part_id == "58121.dat":
            mass, component_center = part_mass_center(part, payloads.get(part.instance))
            if np.any(component_center < lower-1e-6) or np.any(component_center > upper+1e-6):
                raise AdapterError(f"Mass-layout component center lies outside its physical geometry: {part.instance}")
            components.append({"partInstance": part.instance, "massKg": mass,
                               "centerLdu": (component_center/LDU_TO_METERS).tolist(),
                               "source": "payload-profile-estimate" if part.instance in payloads else "ABS-mesh-estimate"})
    if not bounds:
        raise AdapterError("Mass-layout gate requires actual part geometry.")
    lower = np.min([lo for lo, _ in bounds], axis=0)
    upper = np.max([hi for _, hi in bounds], axis=0)
    if np.any(center < lower-1e-6) or np.any(center > upper+1e-6):
        raise AdapterError("Mass-layout center lies outside the complete physical assembly.")
    feet = {f["id"]: f for f in checked.manifest["feet"]}
    paths = checked.manifest.get("footPaths", [])
    reach_source = "sampled-swept-foot-regions" if paths else "initial-foot-regions-only"
    samples = [(feet[path["id"]], point) for path in paths for point in path["points"]] if paths else [
        (foot, foot["position"]) for foot in feet.values()]
    if not samples:
        raise AdapterError("Mass-layout gate requires at least one foot contact sample.")
    points, ground_y = [], []
    for foot, point in samples:
        radius = foot.get("contactRadiusMeters", 0)/LDU_TO_METERS
        points.extend((point[0]+dx*radius, point[2]+dz*radius) for dx in (-1, 1) for dz in (-1, 1))
        ground_y.append(point[1]*LDU_TO_METERS+foot.get("contactRadiusMeters", 0))
    if not np.isfinite(points).all() or not np.isfinite(ground_y).all():
        raise AdapterError("Mass-layout gate requires finite foot contact samples.")
    hull = _hull(points)
    projection = center[[0, 2]]/LDU_TO_METERS
    margin = None
    warnings = []
    if len(hull) >= 3:
        margins = []
        for a, b in zip(hull, hull[1:]+hull[:1]):
            edge = np.subtract(b, a)
            offset = projection-a
            margins.append(float((edge[0]*offset[1]-edge[1]*offset[0])/np.linalg.norm(edge))*LDU_TO_METERS)
        margin = min(margins)
        if margin < 0:
            warnings.append("COM projection is outside the sampled swept-foot envelope; dynamic leaning is not ruled out.")
    else:
        warnings.append("Degenerate support envelope; static balance cannot be inferred.")
    height = max(ground_y)-float(center[1])
    spans = np.ptp(np.array(points), axis=0)*LDU_TO_METERS
    if height > float(min(spans)):
        warnings.append("COM height exceeds the narrower swept-foot span; this is a tipping-risk heuristic, not a rejection.")
    return {
        "version": VERSION, "payloadProfile": profile.get("id"),
        "payloadProfileSha256": hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
        "totalMassKg": total,
        "centerOfMassLdu": (center/LDU_TO_METERS).tolist(),
        "heightAboveLowestFootMeters": height, "sweptFootHullXZLdu": hull,
        "supportEnvelopeSource": reach_source, "supportHalfPlaneMarginMeters": margin,
        "components": components, "warnings": warnings, "dynamicQualification": False,
        "inertia": _inertia_layout(scene, center, list(feet.values()), inertia_tensors or {}),
        "gate": {"gate": "mass-layout", "outcome": "warning" if warnings else "pass",
                 "code": "estimated-mass-and-support-envelope",
                 "diagnostics": [f"Mass={total:.6g} kg; COM height={height:.6g} m; swept support margin={margin}.",
                                 "Support uses an optimistic union of sampled foot reach, not simultaneous stance contacts."]+warnings},
    }


def mass_layout_gate(checked, payload_profile=None):
    profile = load_profile(payload_profile)
    scene, parts, _ = prepare_parts(checked, profile=profile)
    scene, tensors = apply_payloads(scene, parts, checked.manifest, profile)
    return analyze_layout(checked, scene, parts, profile, inertia_tensors=tensors)
