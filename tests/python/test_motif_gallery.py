import copy
import importlib.util
import json
import re
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.request import Request, urlopen
from xml.etree import ElementTree as ET

import pytest

from legowalkers_sim.dashboard import DashboardHandler
from legowalkers_sim.gallery import motif_gallery
from test_dashboard import controller

ROOT = Path(__file__).parents[2]
ASSETS = ROOT / "tests/visualizations/motifs"
SPEC = importlib.util.spec_from_file_location("motif_illustrations", ROOT / "scripts/motif_illustrations.py")
renderer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(renderer)


def example():
    return {"title": "Pin <test>", "caption": "Same midpoint",
            "before": [{"id": "pin", "part": "32002.dat", "changed": True, "ports": [],
                        "shaft": [{"x": 0, "y": 0, "z": -20}, {"x": 0, "y": 0, "z": 10}]}],
            "after": [{"id": "pin", "part": "32002.dat", "changed": True, "ports": [],
                       "shaft": [{"x": 0, "y": 0, "z": 10}, {"x": 0, "y": 0, "z": -20}]}]}


def test_motif_schematic_is_deterministic_escaped_and_explicitly_not_physics():
    value = renderer.illustration(example())
    ET.fromstring(value)
    assert value == renderer.illustration(example())
    assert "Pin &lt;test&gt;" in value and "30 LDU" in value
    assert "Not a LEGO mesh" in value and "identical planar motion" in value
    assert value.count('marker-end="url(#direction)"') == 2
    paths = [node for node in ET.fromstring(value).iter()
             if node.tag.endswith("path") and node.attrib.get("stroke-width") == "10"]
    before, after = [[float(number) for number in re.findall(r"-?\d+\.\d+", path.attrib["d"])] for path in paths]
    assert after == pytest.approx([before[2]+600, before[3], before[0]+600, before[1]])


def test_renderer_rejects_missing_parts_and_nonfinite_positions():
    bad = example()
    bad["after"][0]["id"] = "other"
    with pytest.raises(ValueError, match="same part instances"):
        renderer.illustration(bad)
    bad = copy.deepcopy(example())
    bad["after"][0]["shaft"][0]["z"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        renderer.illustration(bad)


def test_gallery_covers_catalogue_and_illustrates_supported_parameters():
    data = json.loads((ASSETS / "catalogue.json").read_text())
    assert {e["motif"] for e in data["examples"]} == {m["id"] for m in data["builder"]["catalogue"]}
    assert {e["id"] for e in data["examples"]} == {"shorter-pin", "reversed-pin", "layer-clearance"}
    for item in data["examples"]:
        assert item["beforeGenotypeId"] != item["afterGenotypeId"]
        image = (ASSETS / f'{item["id"]}.svg').read_text()
        ET.fromstring(image)
        assert 'viewBox="0 0 1200 420"' in image and "schematic" in image
    page = motif_gallery()
    assert "When it applies" in page and "What stays unchanged" in page and "Regression evidence" in page
    assert "planar motion" in page and "collision screening" in page
    assert "40 to 30 LDU" in page and "+10 LDU" in page
    assert page.count("Open full-size diagram") == len(data["examples"])
    assert "min-width:720px" in page and "scroll diagrams sideways" in page


def test_gallery_http_routes_and_head_are_read_only(controller):
    with ThreadingHTTPServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = controller
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            for path in ("/motif-gallery.html", "/motif-illustrations/shorter-pin.svg",
                         "/motif-illustrations/reversed-pin.svg", "/motif-illustrations/layer-clearance.svg",
                         "/motif-illustrations/catalogue.json", "/mutation-gallery.html"):
                with urlopen(base+path) as response:
                    assert response.status == 200 and response.read()
            with urlopen(Request(base+"/motif-gallery.html", method="HEAD")) as response:
                assert response.status == 200 and response.read() == b""
            assert controller.status()["used"] == controller.status()["budget"] == 0
            assert controller.launched == []
        finally:
            server.shutdown()
            thread.join(timeout=5)
