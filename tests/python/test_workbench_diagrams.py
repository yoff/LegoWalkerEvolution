import json
import warnings
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim import workbench_diagrams as diagrams
from legowalkers_sim.bridge import AdapterError


def test_real_mesh_silhouettes_keep_whole_model_depth_and_annotation_positions(tmp_path, monkeypatch):
    manifest = {
        "parts": [{"id": "beam", "part": "32525.dat", "color": 1, "body": "leg",
                   "position": [100, 0, 40], "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}],
        "modelFile": "model.ldr", "expected": {"joints": [{"id": "hinge", "position": [100, 0, 40]}]},
        "feet": [{"id": "toe", "position": [140, 0, 40]}],
        "footPaths": [{"id": "toe", "points": [[140, 0, 40], [130, 10, 40]]}],
    }
    triangle = SimpleNamespace(v0=np.array([0, 0, 0]), v1=np.array([40, 0, 0]),
                               v2=np.array([0, 10, 0]))
    monkeypatch.setattr(diagrams, "load_manifest", lambda path: manifest)
    monkeypatch.setattr(diagrams, "library_fingerprint", lambda *args: "test")
    parsed = []
    def parse(path):
        parsed.append(path)
        return [triangle]
    diagrams._vertices.cache_clear()
    diagrams._outlines.cache_clear()
    diagrams._model_diagram.cache_clear()
    monkeypatch.setattr(diagrams, "LDrawParser", lambda **kwargs: SimpleNamespace(parse_part=parse))
    value = diagrams.model_diagram(tmp_path, tmp_path)
    part, = value["parts"]
    assert part["front"] == [[100, 0], [140, 0], [100, 10]]
    assert part["oblique"] == [[118, -12], [158, -12], [118, -2]]
    assert value["joints"][0]["oblique"] == [118, -12]
    assert value["feet"][0]["front"] == [140, 0]
    assert value["paths"][0]["oblique"] == [[158, -12], [148, -2]]
    json.dumps(value, allow_nan=False)
    cached = diagrams.model_diagram(tmp_path, tmp_path)
    assert cached == value
    assert diagrams._model_diagram.cache_info().hits == 1
    cached["parts"][0]["front"][0][0] = -999
    assert diagrams.model_diagram(tmp_path, tmp_path)["parts"][0]["front"][0] == [100, 0]
    manifest["parts"][0]["position"][0] += 20
    moved = diagrams.model_diagram(tmp_path, tmp_path)
    assert moved["parts"][0]["front"][0] == [120, 0]
    assert len(parsed) == 1
    monkeypatch.setattr(diagrams, "library_fingerprint", lambda *args: "changed-library")
    diagrams.model_diagram(tmp_path, tmp_path)
    assert len(parsed) == 2
    assert diagrams._model_diagram.cache_info().misses == 3


@pytest.mark.parametrize("points", [[], [[0, 1]], [[0, float("nan"), 2]], [[0, 1, float("inf")]]])
def test_invalid_diagram_geometry_is_not_silently_rendered(points):
    with pytest.raises(AdapterError, match="finite 3D"):
        diagrams.project(points, "front")


@pytest.mark.parametrize("warning", [False, True])
def test_missing_geometry_and_parser_warnings_are_explicit(tmp_path, monkeypatch, warning):
    def parse(path):
        if warning:
            warnings.warn("missing primitive", UserWarning)
        return []
    diagrams._vertices.cache_clear()
    monkeypatch.setattr(diagrams, "LDrawParser", lambda **kwargs: SimpleNamespace(parse_part=parse))
    with pytest.raises(AdapterError, match="warnings" if warning else "No diagram geometry"):
        diagrams._vertices(str(tmp_path), "missing.dat", "test")
