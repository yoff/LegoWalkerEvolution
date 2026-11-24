"""Keep physical ground collisions, but reject support outside designated regions."""

import numpy as np

from .bridge import AdapterError


class GroundPenetrationStats:
    """Observe solver overlap without changing contact classification or qualification."""

    def __init__(self):
        self.peaks = {}

    def observe(self, instance, kind, distance, time):
        if distance > 0:
            return
        if kind not in ("foot", "wheel", "forbidden") or not np.isfinite([distance, time]).all():
            raise AdapterError("Ground penetration diagnostics require a classified finite contact.")
        key = (kind, instance)
        if key not in self.peaks:
            self.peaks[key] = {"part": instance, "kind": kind, "samples": 0,
                               "maxPenetrationMeters": 0.0, "peakTime": time}
        peak = self.peaks[key]
        peak["samples"] += 1
        if -distance > peak["maxPenetrationMeters"]:
            peak.update(maxPenetrationMeters=-distance, peakTime=time)

    def report(self):
        parts = [dict(peak) for _, peak in sorted(self.peaks.items())]
        return {
            "schemaVersion": 1, "version": "ground-penetration-v1",
            "scope": "all observed physical floor contacts, including settling",
            "maxFootPenetrationMeters": max((p["maxPenetrationMeters"] for p in parts if p["kind"] == "foot"), default=0.0),
            "maxForbiddenPenetrationMeters": max((p["maxPenetrationMeters"] for p in parts if p["kind"] == "forbidden"), default=0.0),
            "parts": parts,
        }


def foot_regions(manifest):
    regions = []
    for index, foot in enumerate(manifest["feet"]):
        instance = foot.get("partInstance")
        if instance is None:
            candidates = [part["id"] for part in manifest["parts"] if part["body"] == foot["body"]]
            if len(candidates) != 1:
                raise AdapterError(f"Legacy foot {foot['id']} has no unambiguous part attachment.")
            instance = candidates[0]
        regions.append({"partInstance": instance, "site": index,
                        "radiusMeters": foot.get("contactRadiusMeters", .004)})
    return regions


class GroundContacts:
    def __init__(self, manifest):
        self.regions = foot_regions(manifest)
        self.surfaces = []
        if any("contactShape" in foot for foot in manifest.get("motion", {}).get("logicalFeet", [])):
            from .support_surfaces import support_surfaces
            self.surfaces = support_surfaces(manifest, self.regions)
        self.wheels = set(manifest.get("groundWheels", []))
        self.forbidden = {}
        self.foot_samples = 0
        self.wheel_samples = 0

    def observe(self, instance, position, distance, time, feet):
        if distance > 0:
            return
        if instance in self.wheels:
            self.wheel_samples += 1
            return "wheel"
        for surface in self.surfaces:
            if instance == surface.part and surface.contains(position, feet):
                self.foot_samples += 1
                return "foot"
        for region in self.regions:
            if (instance == region["partInstance"]
                    and np.linalg.norm(np.asarray(position)-feet[region["site"]]) <= region["radiusMeters"]):
                self.foot_samples += 1
                return "foot"
        if instance not in self.forbidden:
            self.forbidden[instance] = {
                "part": instance, "firstTime": time, "samples": 0,
                "maxPenetrationMeters": 0.0, "peakTime": time,
                "firstPositionMeters": np.asarray(position).tolist(),
            }
        entry = self.forbidden[instance]
        entry["samples"] += 1
        if -distance > entry["maxPenetrationMeters"]:
            entry["maxPenetrationMeters"] = -distance
            entry["peakTime"] = time
            entry["peakPositionMeters"] = np.asarray(position).tolist()
        return "forbidden"

    def report(self):
        return {
            "policy": "designated-foot-regions-and-wagon-wheels",
            "contactDistanceThresholdMeters": 0.0,
            "footRegions": self.regions, "designatedWheels": sorted(self.wheels),
            "footContactSamples": self.foot_samples, "wheelContactSamples": self.wheel_samples,
            "forbiddenContactDetected": bool(self.forbidden),
            "forbiddenContacts": sorted(self.forbidden.values(), key=lambda item: item["firstTime"]),
        }
