import importlib.util
import json
from argparse import Namespace
from html.parser import HTMLParser
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError

SPEC = importlib.util.spec_from_file_location("preview", Path(__file__).parents[2] / "scripts" / "preview.py")
assert SPEC is not None and SPEC.loader is not None
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)


class Media(HTMLParser):
    def __init__(self):
        super().__init__()
        self.sources = []

    def handle_starttag(self, tag, attrs):
        if tag in ("img", "source"):
            self.sources.append(dict(attrs)["src"])


def test_presets_have_no_cycles_and_separate_render_rate_from_pose_cache():
    assert preview.PRESETS["draft"]["engine"] == "BLENDER_WORKBENCH"
    assert preview.PRESETS["preview"]["engine"] == "BLENDER_EEVEE_NEXT"
    assert all(settings["engine"] != "CYCLES" for settings in preview.PRESETS.values())
    assert all(settings["fps"] <= preview.SAMPLE_FPS for settings in preview.PRESETS.values())
    draft, standard, quality = (preview.PRESETS[name] for name in ("draft", "preview", "quality"))
    assert draft["width"] < standard["width"] < quality["width"]
    assert draft["samples"] < standard["samples"] < quality["samples"]


@pytest.fixture
def page_inputs(tmp_path):
    assets = tmp_path / "rendered model"
    pages = tmp_path / "pages"
    assets.mkdir()
    pages.mkdir()
    result = {
        "metrics": {"motorTurns": [1.662557], "maxClosureErrorMeters": 0.000321915},
        "collision": {"maxSelfPenetrationMeters": 0.000003426, "penetrationToleranceMeters": 0.0001,
                      "axleRodRadiusMeters": 0.0012},
        "options": {"duration": 20, "torque": 0.4, "speed": 3},
        "compatibility": {"parts": 98, "bodies": 24},
    }
    (assets / "metadata.json").write_text(json.dumps({
        "result": result, "manifest": str(tmp_path / "candidate" / "manifest.json"),
        "evaluation": str(tmp_path / "evaluation"),
    }))
    (assets / "render-info.json").write_text(json.dumps({
        "fps": 10, "label": "Workbench", "preset": "draft",
    }))
    for path in [assets / "walker-v2.png", assets / "walker-v2-anchored.mp4",
                 pages / "walker-free.mp4", pages / "walker-anchored.mp4"]:
        path.write_bytes(b"test media")
    return Namespace(assets=assets, pages=pages)


def test_both_pages_show_current_metrics_and_embed_media_only_in_preview(page_inputs):
    preview.publish(page_inputs)
    for name, embedded in [("index.html", False), ("preview.html", True)]:
        html = (page_inputs.pages / name).read_text()
        assert "@@" not in html
        assert "1.66" in html and "0.00343 mm" in html
        assert "Workbench draft" in html and "10 fps" in html
        assert "historical-previews" in html and "known self-interference" in html
        assert "not a demonstration of walking" in html
        assert "../rendered%20model/walker-v2-anchored.mp4" in html
        parser = Media()
        parser.feed(html)
        assert len(parser.sources) == 4
        assert all(source.startswith("data:") == embedded for source in parser.sources)


def test_missing_new_animation_does_not_overwrite_existing_pages(page_inputs):
    path = page_inputs.pages / "preview.html"
    path.write_text("existing preview")
    (page_inputs.assets / "walker-v2-anchored.mp4").unlink()
    with pytest.raises(AdapterError, match="Missing preview media"):
        preview.publish(page_inputs)
    assert path.read_text() == "existing preview"


def test_republish_uses_actual_render_settings_without_replaying_physics(page_inputs, monkeypatch):
    def unexpected_replay(_):
        pytest.fail("Publishing render settings must not rerun physics.")

    monkeypatch.setattr(preview, "replay", unexpected_replay)
    (page_inputs.assets / "render-info.json").write_text(json.dumps({
        "fps": 15, "label": "EEVEE", "preset": "preview",
    }))
    preview.publish(page_inputs)
    html = (page_inputs.pages / "preview.html").read_text()
    assert "15 fps" in html and "EEVEE preview" in html


def test_static_lego_render_needs_no_evaluation_or_physics(tmp_path, monkeypatch):
    manifest = {"parts": [{"id": "beam", "part": "32523.dat", "color": 7}],
                "modelFile": "model.ldr", "ldrawSha256": "a"*64}
    triangle = SimpleNamespace(v0=np.array([0, 0, 0]), v1=np.array([20, 0, 0]), v2=np.array([0, 20, 0]))
    build = SimpleNamespace(parts=[SimpleNamespace(part_id="32523.dat", triangles=[triangle])])
    monkeypatch.setattr(preview, "load_manifest", lambda path: manifest)
    monkeypatch.setattr(preview, "library_fingerprint", lambda *args: "b"*64)
    monkeypatch.setattr(preview, "LDrawParser",
                        lambda **kwargs: SimpleNamespace(parse_build=lambda path: build))
    args = Namespace(manifest=tmp_path/"manifest.json", library=tmp_path,
                     assets=tmp_path/"still", preset="preview")
    preview.prepare_still(args)
    metadata = json.loads((args.assets/"metadata.json").read_text())
    assert metadata["renderKind"] == "assembly-still"
    assert "result" not in metadata and "evaluation" not in metadata
    geometry = json.loads((args.assets/"geometry.json").read_text())
    assert geometry[0]["id"] == "beam" and geometry[0]["body"] == "assembly"
    assert min(point[2] for point in geometry[0]["vertices"]) == pytest.approx(.025)
    with pytest.raises(AdapterError, match="no evaluated physics"):
        preview.replay(args)
    with pytest.raises(AdapterError, match="must be empty"):
        preview.prepare_still(args)


def test_static_render_rejects_missing_or_mismatched_actual_geometry():
    manifest = {"parts": [{"id": "beam", "part": "32523.dat"}]}
    with pytest.raises(AdapterError, match="part count"):
        preview.render_vertices(manifest, SimpleNamespace(parts=[]))
    with pytest.raises(AdapterError, match="Unexpected render part"):
        preview.render_vertices(manifest, SimpleNamespace(parts=[SimpleNamespace(part_id="other.dat")]))
