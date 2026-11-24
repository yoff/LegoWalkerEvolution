"""Opt-in bounded support surfaces, anchored to sites on one physical foot part."""

from dataclasses import dataclass

import numpy as np

from .bridge import AdapterError

VERSION = "bounded-support-surfaces-v1"


@dataclass(frozen=True)
class SupportSurface:
    kind: str
    part: str
    sites: tuple[int, ...]
    radius: float

    def contains(self, position, feet):
        points = np.asarray(feet)[list(self.sites)]
        delta = np.asarray(position) - points[0]
        edge = points[1] - points[0]
        closest = points[0] + np.clip(delta @ edge / (edge @ edge), 0, 1) * edge
        if self.kind == "rectangle":
            across = points[2] - points[0]
            closest += np.clip(delta @ across / (across @ across), 0, 1) * across
        return bool(np.linalg.norm(np.asarray(position) - closest) <= self.radius)


def support_surfaces(manifest, regions):
    feet = manifest["feet"]
    by_id = {foot["id"]: index for index, foot in enumerate(feet)}
    result = []
    for logical in manifest.get("motion", {}).get("logicalFeet", []):
        if "contactShape" not in logical:
            continue
        surfaces = logical.get("surfaces", [logical])
        if not isinstance(surfaces, list) or not 1 <= len(surfaces) <= 2:
            raise AdapterError("A logical foot needs one or two explicit physical support surfaces.")
        used = set()
        parts = set()
        for surface in surfaces:
            if not isinstance(surface, dict):
                raise AdapterError("Physical support surfaces require shape and region declarations.")
            kind = surface.get("contactShape")
            count = {"capsule": 2, "rectangle": 4}.get(kind)
            names = surface.get("regions")
            if (count is None or not isinstance(names, list) or len(names) != count
                    or any(not isinstance(name, str) or name not in by_id for name in names)
                    or len(set(names)) != count or used.intersection(names)):
                raise AdapterError("Bounded support needs distinct capsule anchors or rectangle corners.")
            used.update(names)
            sites = tuple(by_id[name] for name in names)
            patches = [feet[index] for index in sites]
            selected = [regions[index] for index in sites]
            if (len({region["partInstance"] for region in selected}) != 1
                    or len({region["radiusMeters"] for region in selected}) != 1
                    or selected[0]["partInstance"] in parts
                    or surface.get("body", logical["body"]) != logical["body"]
                    or any(patch["body"] != logical["body"] or patch.get("logicalFoot") != logical["id"] for patch in patches)):
                raise AdapterError("A bounded support surface must belong to one distinct physical foot part, body and radius.")
            parts.add(selected[0]["partInstance"])
            points = np.array([patch["position"] for patch in patches], dtype=float)
            edge = points[1] - points[0]
            if not np.isfinite(points).all() or np.linalg.norm(edge) <= 1e-6:
                raise AdapterError("Bounded support anchors must be finite and distinct.")
            if kind == "rectangle":
                across = points[2] - points[0]
                if (np.linalg.norm(across) <= 1e-6
                        or abs(edge @ across) > 1e-8 * np.linalg.norm(edge) * np.linalg.norm(across)
                        or not np.allclose(points[3], points[1] + across, rtol=0, atol=1e-6)):
                    raise AdapterError("Sole anchors must form an ordered, nondegenerate rectangle.")
            result.append(SupportSurface(kind, selected[0]["partInstance"], sites, selected[0]["radiusMeters"]))
        declared = logical.get("regions", [])
        if len(declared) != len(used) or set(declared) != used:
            raise AdapterError("Physical support surfaces must cover their logical foot's regions exactly once.")
    return result
