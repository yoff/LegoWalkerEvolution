"""Family constraints survive the generator/manifest/Workbench boundary without implying fitness."""

import json
import subprocess
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

from legowalkers_sim.bridge import AdapterError, load_manifest
from legowalkers_sim.evolution import ROOT
from legowalkers_sim.evolution import Generator
from legowalkers_sim.leg_family_migration import convert, verify_metadata_only


def generate(*args):
    subprocess.run(
        [str(ROOT / ".tools/dotnet/dotnet"),
         str(ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll"), *map(str, args)],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )


@pytest.fixture(scope="module")
def shared(tmp_path_factory):
    directory = tmp_path_factory.mktemp("shared-legs") / "source"
    generate("realize-walker", "--feet", 4, "--steps", 72, "--output", directory)
    return directory / "candidate-001"


def test_new_founders_export_one_shared_family_and_separate_gait_edits(shared):
    manifest = load_manifest(shared / "manifest.json")
    family, = manifest["genome"]["legFamilies"]
    assert len(family["instances"]) == 4
    assert [i["mirrored"] for i in family["instances"]] == [False, True, False, True]
    assert manifest["genome"] == json.loads((shared / "realization.json").read_text())
    result = subprocess.run(
        [str(ROOT / ".tools/dotnet/dotnet"),
         str(ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll"),
         "walker-edits", "--parent", str(shared), "--steps", "72"],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )
    edits = json.loads(result.stdout)
    operations = {e["operation"] for e in edits["edits"]}
    assert {"shared-leg-lengthen-foot", "split-leg-family", "gait-half-cycle"} <= operations
    assert "branch-phase" not in operations
    assert edits["dynamicTrials"] == 0


def test_legacy_export_and_explicit_adoption_preserve_physical_assembly(tmp_path):
    source, target = tmp_path / "source", tmp_path / "shared"
    generate("realize-walker", "--feet", 2, "--leg-policy", "independent",
             "--steps", 72, "--output", source)
    original = source / "candidate-001"
    before = {p.name: p.read_bytes() for p in original.iterdir() if p.is_file()}
    old = load_manifest(original / "manifest.json")
    assert "legFamilies" not in old["genome"]
    generate("share-walker-legs", "--parent", original, "--output", target,
             "--feet", "leg-0-0,leg-0-1", "--steps", 72)
    new = load_manifest(target / "candidate-001/manifest.json")
    batch = json.loads((target / "candidates.json").read_text())
    assert {"schemaVersion", "gateVersion", "mechanismId", "gates", "generated", "rejected", "counts"} <= batch.keys()
    assert batch["gateVersion"] == "motion-gates-v1"
    assert batch["counts"] == {"proposals": 1, "realizationAttempts": 1, "rejected": 0,
                               "duplicates": 0, "exported": 1, "dynamicTrials": 0}
    assert new["motion"]["assemblyId"] == old["motion"]["assemblyId"]
    assert new["motion"]["mechanismId"] == old["motion"]["mechanismId"]
    assert new["motion"]["genotypeId"] != old["motion"]["genotypeId"]
    assert (target / "candidate-001/model.ldr").read_bytes() == before["model.ldr"]
    assert {p.name: p.read_bytes() for p in original.iterdir() if p.is_file()} == before


def test_family_metadata_is_optional_but_not_unvalidated_json(shared, tmp_path):
    manifest = json.loads((shared / "manifest.json").read_text())
    manifest["genome"]["legFamilies"][0]["instances"][0]["phase"] = "half a turn"
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    with pytest.raises(AdapterError, match="schema"):
        load_manifest(path)


def test_automatic_conversion_is_verified_cached_and_preserves_originals(tmp_path):
    source = tmp_path / "source"
    generate("realize-walker", "--feet", 4, "--leg-policy", "independent",
             "--steps", 72, "--output", source)
    original = {str(p.relative_to(source)): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    generator = Generator(ROOT / ".tools/dotnet/dotnet",
                          ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")
    target = convert(source, tmp_path / "converted", generator)
    old = load_manifest(source / "candidate-001/manifest.json")
    new = load_manifest(target / "candidate-001/manifest.json")
    assert len(new["genome"]["legFamilies"][0]["instances"]) == 4
    verify_metadata_only(old, new)
    assert new["motion"]["genotypeId"] != old["motion"]["genotypeId"]
    assert new["motion"]["logicalFeet"] == old["motion"]["logicalFeet"]
    assert {str(p.relative_to(source)): p.read_bytes() for p in source.rglob("*") if p.is_file()} == original
    report = json.loads((target / "leg-family-migration.json").read_text())
    assert report["designs"][0]["ldrawByteIdentical"]
    assert report["designs"][0]["unmatched"] == []
    assert report["dynamicTrials"] == 0
    assert convert(source, tmp_path / "converted", generator) == target
    assert convert(target, tmp_path / "another-cache", generator) == target


@pytest.mark.parametrize("field", ["speed", "feet", "parts"])
def test_conversion_verification_rejects_non_sharing_changes(shared, field):
    before = load_manifest(shared / "manifest.json")
    before["genome"].pop("legFamilies")
    after = json.loads(json.dumps(before))
    after["genome"]["legFamilies"] = []
    after["motion"]["genotypeId"] = "f"*64
    if field == "speed":
        after["genome"]["speed"] += 1
    elif field == "feet":
        after["motion"]["logicalFeet"][0]["regions"] = ["different-contact"]
    else:
        after["parts"][0]["position"][0] += 1
    with pytest.raises(AdapterError, match="physical/evaluation metadata"):
        verify_metadata_only(before, after)


@pytest.mark.parametrize("changed", ["parts", "program", "ldraw", "speed"])
def test_conversion_refuses_changed_generated_physics_without_publishing_a_cache(tmp_path, changed):
    from legowalkers_sim.bridge import write_json
    from test_native_episode import native_fixture

    source, _ = native_fixture(tmp_path)
    original = {str(p.relative_to(source)): p.read_bytes() for p in source.rglob("*") if p.is_file()}
    binary = tmp_path / "compiler"
    binary.mkdir()
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        (binary / name).write_bytes(b"fixture")

    def invoke(*args):
        destination = Path(args[args.index("--output")+1]) / "candidate-001"
        shutil.copytree(source / "candidate-0", destination)
        manifest = json.loads((destination / "manifest.json").read_text())
        manifest["genome"]["legFamilies"] = []
        manifest["motion"]["genotypeId"] = "f"*64
        if changed == "parts":
            manifest["parts"][0]["position"][0] += 1
        elif changed == "speed":
            manifest["genome"]["speed"] += 1
        elif changed == "program":
            (destination / "program.json").write_text('{"changed":true}')
        else:
            (destination / manifest["modelFile"]).write_text("0 changed geometry\n")
        write_json(destination / "manifest.json", manifest)
        write_json(destination.parent / "leg-family-conversion.json",
                   {"status": "converted", "families": [], "unmatched": []})

    generator = SimpleNamespace(command=["dotnet", str(binary / "LegoWalkers.Cli.dll")], invoke=invoke)
    expected = {"parts": "realization choices differ", "program": "changed the physical design",
                "ldraw": "LDraw checksum mismatch", "speed": "motor choice differs"}
    with pytest.raises(AdapterError, match=expected[changed]):
        convert(source, tmp_path / "cache", generator)
    assert not list((tmp_path / "cache").iterdir())
    assert {str(p.relative_to(source)): p.read_bytes() for p in source.rglob("*") if p.is_file()} == original
