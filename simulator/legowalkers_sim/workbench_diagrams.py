"""Presentation-only projected LDraw silhouettes; no physics or scene construction."""

import copy
import json
import warnings
from functools import lru_cache
from pathlib import Path

import numpy as np
from lego_technic_sim.ldraw.parser import LDrawParser

from .bridge import AdapterError, library_fingerprint, load_manifest
from .mass_layout import _hull


def project(points, view):
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or not np.isfinite(values).all():
        raise AdapterError("Diagram geometry must contain finite 3D points.")
    if view == "front":
        return values[:, :2]
    return np.column_stack((values[:, 0] + .45 * values[:, 2],
                            values[:, 1] - .3 * values[:, 2]))


@lru_cache(maxsize=128)
def _vertices(library, part, dependency):
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        triangles = LDrawParser(parts_dir=Path(library)).parse_part(Path(library) / "parts" / part)
    if caught:
        raise AdapterError("Diagram geometry warnings: " + "; ".join(str(item.message) for item in caught))
    if not triangles:
        raise AdapterError(f"No diagram geometry for {part}.")
    return np.unique([vertex for triangle in triangles for vertex in (triangle.v0, triangle.v1, triangle.v2)], axis=0)


@lru_cache(maxsize=512)
def _outlines(library, part, dependency, rotation):
    vertices = _vertices(library, part, dependency) @ np.asarray(rotation).reshape(3, 3).T
    return {view: np.asarray(_hull(project(vertices, view))) for view in ("front", "oblique")}


def model_diagram(candidate, library):
    manifest = load_manifest(candidate / "manifest.json")
    library = library.resolve()
    dependencies = {part: library_fingerprint(library, [part]) for part in {p["part"] for p in manifest["parts"]}}
    return copy.deepcopy(_model_diagram(json.dumps(manifest, sort_keys=True, allow_nan=False),
                                       str(library), tuple(sorted(dependencies.items()))))


@lru_cache(maxsize=16)
def _model_diagram(document, library, dependency_items):
    manifest = json.loads(document)
    dependencies = dict(dependency_items)
    parts = []
    for source in manifest["parts"]:
        rotation = tuple(v for row in source["rotation"] for v in row)
        outlines = _outlines(str(library), source["part"], dependencies[source["part"]], rotation)
        parts.append({"id": source["id"], "part": source["part"],
                      "placement": [round(float(value), 6) for value in [
                          *source["position"], *rotation]],
                      "color": source["color"], "body": source["body"],
                      **{view: np.round(outlines[view] + project([source["position"]], view)[0], 4).tolist()
                         for view in ("front", "oblique")}})
    def marker(item):
        return {"id": item["id"], **{view: project([item["position"]], view)[0].round(4).tolist()
                                    for view in ("front", "oblique")}}
    return {"parts": parts,
            "joints": [marker(joint) for joint in manifest["expected"]["joints"]],
            "feet": [marker(foot) for foot in manifest["feet"]],
            "paths": [{"id": path["id"], **{view: project(path["points"], view).round(4).tolist()
                                          for view in ("front", "oblique")}}
                      for path in manifest.get("footPaths", [])]}
