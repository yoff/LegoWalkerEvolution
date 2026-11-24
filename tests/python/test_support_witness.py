import hashlib
from itertools import product
import json
import sqlite3
from xml.etree import ElementTree as ET

import mujoco
import pytest

from legowalkers_sim.bridge import AdapterError, write_json
from legowalkers_sim.support_witness import witness


@pytest.mark.parametrize("shoe", [False, True])
def test_static_witness_fills_only_support_gaps_without_stepping_or_modifying_sources(tmp_path, monkeypatch, shoe):
    archive = tmp_path / "archive"
    source = archive / "evaluations/0001"
    source.mkdir(parents=True)
    with sqlite3.connect(archive / "archive.sqlite3") as db:
        db.executescript("""CREATE TABLE births(id INTEGER,design TEXT);
            CREATE TABLE evaluations(id INTEGER,birth INTEGER);
            INSERT INTO births VALUES(1,'design'); INSERT INTO evaluations VALUES(1,1);""")
    ranges = ((-.016, .016), (-.004, .004), (0, .0096)) if shoe else (
        (-.0036, .0036), (-.004, .004), (-.02, .02))
    points = [(x, y, 0) for x in ranges[0] for y in ranges[1]] if shoe else [(0, 0, -.02)]
    root = ET.Element("mujoco")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "mesh", name="hull",
                  vertex=" ".join(str(v) for point in product(*ranges) for v in point))
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "geom", name="ground", type="plane", size="1 1 .1")
    body = ET.SubElement(world, "body", pos="0 0 .05")
    ET.SubElement(body, "freejoint")
    ET.SubElement(body, "geom", name="part", type="mesh", mesh="hull")
    for index, point in enumerate(points):
        ET.SubElement(body, "site", name=f"foot_{index}", pos=" ".join(map(str, point)))
    ET.ElementTree(root).write(source / "model.xml")
    feet = [{"id": str(i), "logicalFoot": "foot", "partInstance": "sole", "body": "leg",
             "position": list(point), "contactRadiusMeters": .0016 if shoe else .004}
            for i, point in enumerate(points)]
    write_json(archive / "designs/design/candidate/manifest.json", {
        "feet": feet, "parts": [{"id": "sole", "body": "leg", "part": "3701.dat" if shoe else "32525.dat",
                                "rotation": [[1, 0, 0], [0, 0, -1], [0, 1, 0]]}]})
    write_json(source / "collision-model.json", {"geometryInstances": {"part": "sole"}})
    write_json(source / "simulation-gate.json", {
        "diagnostics": {"groundContact": {"forbiddenContacts": [{"part": "sole"}]}}})

    def fingerprint():
        return {str(path.relative_to(archive)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in archive.rglob("*") if path.is_file()}

    def forbidden_step(*args, **kwargs):
        pytest.fail("Static witnesses must never step physics.")

    monkeypatch.setattr(mujoco, "mj_step", forbidden_step)
    before = fingerprint()
    output = tmp_path / "witness"
    report = witness(archive, 1, output)
    assert report["legacy"]["forbiddenContactDetected"]
    assert not report["bounded"]["forbiddenContactDetected"]
    assert report["bounded"]["footContactSamples"] > 0
    assert {item["part"] for item in report["negativePredicateProbes"]["forbiddenContacts"]} == {
        "sole", "non-foot-rocker"}
    assert report["campaignTrials"] == report["simulatedSteps"] == report["simulationSeconds"] == 0
    assert report["groundFixture"] == ("localized-underside-probe" if shoe else "plane")
    assert json.loads((output / "witness.json").read_text()) == report
    assert fingerprint() == before
    with pytest.raises(AdapterError, match="empty directory outside"):
        witness(archive, 1, output)
    with pytest.raises(AdapterError, match="outside"):
        witness(archive, 1, archive / "witness")
