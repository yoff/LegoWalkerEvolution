"""Measured stance diagnostics and advisory, frozen-motion mass-layout comparisons."""

import math

import numpy as np

from .bridge import AdapterError
from .mass_layout import _hull

VERSION = "measured-stance-support-v1"


def _coordinates(value):
    try:
        return np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise AdapterError("Stance geometry requires finite numeric coordinates.") from error


def _finite_number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def outside_distance(point, polygon):
    """Distance to a convex contact hull; empty support has no finite distance."""
    point = _coordinates(point)
    if point.shape != (2,) or not np.isfinite(point).all():
        raise AdapterError("Support distances require finite horizontal points.")
    polygon = contact_hull(polygon)
    if not len(polygon):
        return None
    points = np.asarray(polygon, dtype=float)
    if len(points) == 1:
        return float(np.linalg.norm(point-points[0]))
    edges = np.roll(points, -1, axis=0)-points
    offsets = point-points
    cross = edges[:, 0]*offsets[:, 1]-edges[:, 1]*offsets[:, 0]
    if len(points) >= 3 and np.all(cross >= -1e-14):
        return 0.
    lengths = np.sum(edges*edges, axis=1)
    fractions = np.clip(np.sum(offsets*edges, axis=1)/lengths, 0, 1)
    closest = points+fractions[:, None]*edges
    return float(np.min(np.linalg.norm(point-closest, axis=1)))


class StanceSupport:
    def __init__(self, start_time, *, anchored=False):
        if not _finite_number(start_time) or start_time < 0:
            raise AdapterError("Stance measurement needs a finite nonnegative start time.")
        self.previous = start_time
        self.anchored = anchored
        self.measured = self.unsupported = self.no_support = 0.
        self.current_interval = self.longest = 0.
        self.maximum_distance = None
        self.distance_time = 0.

    def observe(self, time, center, polygon):
        if not _finite_number(time) or time <= self.previous:
            raise AdapterError("Stance samples require strictly increasing finite times.")
        center = _coordinates(center)
        if self.anchored:
            raise AdapterError("Anchored previews cannot measure free-walking stance support.")
        if center.shape != (3,) or not np.isfinite(center).all():
            raise AdapterError("Stance samples require a finite CoM and finite planar contact points.")
        distance = outside_distance(center[:2], polygon)
        duration = time-self.previous
        self.previous = time
        self.measured += duration
        unsupported = distance is None or distance > 1e-9
        if unsupported:
            self.unsupported += duration
            self.current_interval += duration
            self.longest = max(self.longest, self.current_interval)
        else:
            self.current_interval = 0.
        if distance is None:
            self.no_support += duration
        else:
            self.maximum_distance = max(self.maximum_distance or 0., distance)
            self.distance_time += duration*distance

    def report(self):
        return {
            "version": VERSION,
            "status": "not-applicable" if self.anchored else ("measured" if self.measured else "not-measured"),
            "applicability": "anchored-chassis" if self.anchored else "free-walking",
            "measurementSeconds": self.measured,
            "unsupportedSeconds": self.unsupported,
            "unsupportedFraction": self.unsupported/self.measured if self.measured else None,
            "longestUnsupportedSeconds": self.longest if self.measured else None,
            "noSupportSeconds": self.no_support,
            "maximumOutsideDistanceMeters": self.maximum_distance,
            "outsideDistanceTimeMetersSeconds": self.distance_time if self.measured else None,
            "supportRegion": "convex-hull-of-permitted-load-bearing-contact-manifolds",
            "distanceWithoutContacts": "undefined; counted as unsupported and reported separately",
            "scope": "measurement interval after settling; sampled at every physics timestep",
            "boundaryToleranceMeters": 1e-9,
            "qualificationGate": False,
            "limitations": [
                "CoM projection is not a dynamic-stability certificate; momentum can bridge unsupported intervals.",
                "Support uses actual contact-manifold points, not foot-center points or the swept-foot envelope.",
                "Point and line contacts remain degenerate; no artificial foot area is added.",
                "Forbidden chassis or hardware contacts never enlarge the permitted support region.",
                "A rejected prefix is incomplete evidence, not a full gait or reusable fitness.",
            ],
        }


def contact_hull(contacts):
    points = _coordinates(contacts)
    if points.shape == (0,):
        return []
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise AdapterError("Support contacts must be finite horizontal points.")
    return [list(point) for point in _hull(points)]


def compare_shift(trace, shift_ldu):
    """Hold recorded contacts/motion fixed; change only the estimated mass center."""
    if (not isinstance(trace, dict) or trace.get("version") != VERSION
            or not isinstance(trace.get("frames"), list) or not trace["frames"] or "startTime" not in trace):
        raise AdapterError("Mass-layout guidance requires recorded stance samples.")
    shift = _coordinates(shift_ldu)
    if shift.shape != (3,) or not np.isfinite(shift).all():
        raise AdapterError("Counterfactual mass shifts must be finite three-vectors.")
    from .bridge import LDU_TO_METERS
    from .collision import to_mujoco
    local = to_mujoco(shift*LDU_TO_METERS)
    monitor = StanceSupport(trace["startTime"])
    for frame in trace["frames"]:
        if not isinstance(frame, dict) or not {
                "time", "chassisRotation", "centerOfMassMeters", "supportPolygonXYMeters"} <= frame.keys():
            raise AdapterError("Recorded stance frames require time, CoM, chassis rotation and support points.")
        rotation = _coordinates(frame["chassisRotation"])
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7, rtol=0)
                or not math.isclose(float(np.linalg.det(rotation)), 1., abs_tol=1e-7)):
            raise AdapterError("Stance comparison needs proper chassis rotations.")
        center = _coordinates(frame["centerOfMassMeters"])
        if center.shape != (3,) or not np.isfinite(center).all():
            raise AdapterError("Recorded stance frames require finite three-vector mass centers.")
        center = center+rotation@local
        monitor.observe(frame["time"], center, frame["supportPolygonXYMeters"])
    return monitor.report()


def score(report):
    if report["status"] != "measured":
        raise AdapterError("Unmeasured stance evidence cannot rank mass-layout suggestions.")
    return (report["unsupportedFraction"], report["longestUnsupportedSeconds"],
            report["noSupportSeconds"], report["maximumOutsideDistanceMeters"]
            if report["maximumOutsideDistanceMeters"] is not None else math.inf,
            report["outsideDistanceTimeMetersSeconds"])


def repair_mass_layout(generator, parent, output, feedback, hints, limit):
    """Select existing F# hardware edits, never invent geometry or simulation evidence."""
    import json
    from pathlib import Path
    from .bridge import load_manifest, write_json
    from .discovery import POLICY

    if limit not in (1, 2):
        raise AdapterError("Stance-guided repairs permit one or two realizations.")
    archive = feedback.parent.parent.resolve()
    if not isinstance(hints.get("supportTrace"), str) or not hints["supportTrace"]:
        raise AdapterError("Stance repairs require an archive-relative support trace.")
    relative = Path(hints["supportTrace"])
    trace_path = (archive / relative).resolve()
    if relative.is_absolute() or not trace_path.is_relative_to(archive):
        raise AdapterError("Stance repair evidence must stay inside its archive.")
    trace = json.loads(trace_path.read_text())
    layout = json.loads(trace_path.with_name("mass-layout.json").read_text())
    manifest = load_manifest(parent / "manifest.json")
    if trace.get("ldrawSha256") != manifest["ldrawSha256"]:
        raise AdapterError("Recorded stance evidence does not match the parent's physical model.")
    baseline = compare_shift(trace, [0, 0, 0])
    if not isinstance(layout, dict):
        raise AdapterError("Stance mass-layout evidence must be an object.")
    total = layout.get("totalMassKg")
    if not _finite_number(total) or total <= 0:
        raise AdapterError("Mass-layout guidance requires a positive measured mass.")
    components = layout.get("components")
    if (not isinstance(components, list) or any(
            not isinstance(c, dict) or not isinstance(c.get("partInstance"), str)
            or not _finite_number(c.get("massKg")) or not 0 < c["massKg"] <= total for c in components)):
        raise AdapterError("Stance mass-layout evidence requires named components with positive finite masses.")
    masses = {c["partInstance"]: c["massKg"] for c in components}
    if len(masses) != len(components):
        raise AdapterError("Stance mass-layout component identities must be unique.")
    catalogue = generator.manual_edits(parent)
    ranked = []
    for edit in catalogue["edits"]:
        part = {"battery-layout": "payload-battery", "receiver-layout": "payload-receiver"}.get(edit["operation"])
        if part is None or not edit.get("motionUnchanged") or part not in masses:
            continue
        component = part.removeprefix("payload-")
        changes = [c for c in edit["changed"] if c["path"] == f"hardware/{component}/position"]
        if len(changes) != 1:
            continue
        mass = masses[part]
        before, after = (_coordinates(changes[0][key]) for key in ("before", "after"))
        if before.shape != (3,) or after.shape != (3,) or not np.isfinite([before, after]).all():
            raise AdapterError("Hardware-layout edits require finite position vectors.")
        shift = (after-before)*(mass/total)
        comparison = compare_shift(trace, shift)
        if score(comparison) < score(baseline):
            ranked.append((score(comparison), edit["id"], comparison, shift.tolist()))
    ranked.sort(key=lambda item: (item[0], item[1]))
    if ranked:
        _, selected, comparison, shift = ranked[0]
        summary = generator.apply_edit(parent, output, selected, limit=limit)
        summary.update(operation="repair-realization", constructionVersion=POLICY["construction"]["version"])
        summary["stanceGuidance"] = {
            "version": VERSION, "selectedEdit": selected, "estimatedComShiftLdu": shift,
            "before": baseline, "frozenMotionAfter": comparison,
            "authority": "advisory-selection-only; exact reconstruction and a fresh authorized trial remain mandatory",
            "limitations": [
                "Recorded motion and contact timings are held fixed; a changed mass layout changes the real dynamics.",
                "CoM shift estimates use the moved payload mass; support-hardware mass changes are not modeled here.",
                "Only existing, motion-preserving F# hardware edits are considered.",
            ],
        }
    else:
        summary = {
            "schemaVersion": 1, "gateVersion": "motion-gates-v1",
            "parentGenotypeId": manifest["motion"]["genotypeId"],
            "mechanismId": manifest["motion"]["mechanismId"],
            "constructionVersion": POLICY["construction"]["version"], "operation": "repair-realization",
            "changed": [{"path": "construction/stance-support", "before": [], "after": []}],
            "generated": [], "rejected": [],
            "gates": [{"gate": "lego-realizability", "outcome": "reject",
                       "code": "no-supported-improving-mass-layout",
                       "diagnostics": ["No supported motion-preserving hardware edit improves the recorded-stance comparison. "
                                       "Custom mounts may need a builder upgrade; no repair or walking success is inferred."]}],
            "counts": {"proposals": 1, "realizationAttempts": 0, "rejected": 0,
                       "duplicates": 0, "exported": 0, "dynamicTrials": 0},
        }
    write_json(output / "candidates.json", summary)
    return summary
