import json
from pathlib import Path
import re
import sys
from xml.etree import ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import build_builder_grammar as guide


@pytest.fixture
def inputs(tmp_path):
    archive, witnesses = tmp_path / "archive", tmp_path / "witnesses"
    candidate = archive / "designs" / guide.GENOTYPE / "candidate"
    media = candidate.parent / "render"
    evaluation = archive / "evaluations/0001"
    for directory in (candidate, media, evaluation, witnesses):
        directory.mkdir(parents=True)
    result = {
        "metrics": {"eligibleForRanking": True, "rankingRejectionReasons": [],
                    "forwardMeters": .44774045962648534, "simulatedMassKg": .515834,
                    "maxClosureErrorMeters": .0007846208287261761},
        "motorModel": {"version": "pf-xl-torque-speed-v2"},
        "groundContact": {"forbiddenContactDetected": False},
        "gait": {"completeCycles": 5},
    }
    (evaluation / "result.json").write_text(json.dumps(result))
    (candidate / "manifest.json").write_text(json.dumps({"parts": [{"id": "test"}]}))
    (candidate / "model.ldr").write_text("0 Test-only model\n")
    (media / "metadata.json").write_text(json.dumps({"evaluation": str(evaluation), "result": result}))
    (media / "walker-v2.png").write_bytes(b"test-only PNG asset")
    (media / "walker-v2-anchored.mp4").write_bytes(b"test-only video asset")
    (witnesses / "gallery.json").write_text(json.dumps({
        "mode": "construction-schematic-not-dynamics", "examples": [{
            "id": "pin", "title": "Shorter pin", "caption": "Matched catalogue intervals",
            "before": [{"id": "pin", "part": "3673.dat", "changed": True,
                        "ports": [], "shaft": [{"x": 0, "y": 0, "z": -20}, {"x": 0, "y": 0, "z": 20}]}],
            "after": [{"id": "pin", "part": "32002.dat", "changed": True,
                       "ports": [], "shaft": [{"x": 0, "y": 0, "z": -20}, {"x": 0, "y": 0, "z": 10}]}],
        }]}))
    return archive, witnesses, result


def test_guide_publishes_all_illustrated_rules_local_media_and_honest_limits(tmp_path, inputs):
    archive, witnesses, _ = inputs
    output = tmp_path / "guide"
    before = {str(p): p.read_bytes() for root in (archive, witnesses) for p in root.rglob("*") if p.is_file()}
    page = guide.publish(output, archive, witnesses)
    text = page.read_text()
    assert len(re.findall(r'<article id="rule-', text)) == 13
    assert 'id="rule-level-feet"' in text
    assert "Away is preferred, not mandatory" in text
    assert "inward/swapped-arm layouts remain legal alternatives" in text
    assert "{{" not in text and 'id="gear-demo"' in text and 'id="sweep-demo"' in text
    assert "0.448" in text and "0.785" in text
    assert "NOT qualified" in text and "Not wired yet" in text and "near the torque cap" in text
    assert "no external scripts" in text and "not a second grammar" in text
    for image in re.findall(r"<svg\b.*?</svg>", text, re.S):
        ET.fromstring(image)
    assert 'url(#direction-pin)' in text and 'id="direction-pin"' in text
    assert not re.search(r'(?:src|href)="https?://', text)
    provenance = json.loads((output / "guide-provenance.json").read_text())
    for name, digest in provenance["assetSha256"].items():
        assert guide.digest(output / "assets" / name) == digest
    for name, digest in provenance["sourceSha256"].items():
        assert guide.digest(guide.ROOT / name) == digest
        assert guide.digest(output / "assets/sources" / (name + ".txt")) == digest
    assert before == {name: Path(name).read_bytes() for name in before}
    with pytest.raises(ValueError, match="fresh output"):
        guide.publish(output, archive, witnesses)


@pytest.mark.parametrize("problem", ["not-qualified", "unsafe-contact", "wrong-motor", "wrong-media"])
def test_guide_rejects_unqualified_or_mismatched_evidence_before_writing(tmp_path, inputs, problem):
    archive, witnesses, result = inputs
    path = archive / "evaluations/0001/result.json"
    if problem == "not-qualified":
        result["metrics"]["eligibleForRanking"] = False
    elif problem == "unsafe-contact":
        result["groundContact"]["forbiddenContactDetected"] = True
    elif problem == "wrong-motor":
        result["motorModel"]["version"] = "historical-ideal-servo"
    else:
        result["metrics"]["forwardMeters"] = 9
    path.write_text(json.dumps(result))
    with pytest.raises(ValueError, match="qualified walker|provenance"):
        guide.publish(tmp_path / "guide", archive, witnesses)
    assert not (tmp_path / "guide").exists()


def test_guide_source_anchors_are_real_and_unique():
    ids = [rule[0] for rule in guide.RULES]
    assert len(ids) == len(set(ids))
    for rule in guide.RULES:
        assert rule[-2] in (guide.ROOT / rule[-3]).read_text()
    for image in guide.diagrams().values():
        ET.fromstring(image)


def test_guide_refuses_output_inside_immutable_evidence(inputs):
    archive, witnesses, _ = inputs
    with pytest.raises(ValueError, match="separate"):
        guide.publish(archive / "guide", archive, witnesses)
