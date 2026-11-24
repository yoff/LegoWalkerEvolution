"""Verified, geometry-preserving conversion of copied legacy seed batches."""

import hashlib
import json
from pathlib import Path
import shutil
import tempfile

from .bridge import AdapterError, load_manifest, write_json

POLICY = {
    "version": "legacy-shared-leg-conversion-v1",
    "matching": "Exact normalized motion, stock-part layout and drive rate; groups of at least two legs.",
    "preserves": "Geometry, controller, contact semantics, phase, mirroring, original files and ancestry.",
    "existingConstraints": "Preserve explicit families, splits and deliberately unbound instances.",
    "unmatched": "Leave independent and report why; never approximate or force symmetry.",
    "authorization": "No dynamic trials or extra authorization.",
}


def verify_metadata_only(before, after):
    """Only newly added sharing constraints and their genotype identity may differ."""
    normalized = json.loads(json.dumps(after))
    if before["genome"].get("legFamilies") is None:
        normalized["genome"].pop("legFamilies", None)
        if "legFamilies" in before["genome"]:
            normalized["genome"]["legFamilies"] = None
        normalized["motion"]["genotypeId"] = before["motion"]["genotypeId"]
    if normalized != before:
        raise AdapterError("Leg-family conversion changed physical/evaluation metadata or existing constraints.")


def convert(source, cache, generator):
    from .native_episode import seed_batches

    source = Path(source).resolve()
    batches = seed_batches(source)
    marker = source / "leg-family-migration.json"
    if marker.is_file() and json.loads(marker.read_text()).get("version") == POLICY["version"]:
        return source
    if generator is None:
        raise AdapterError("Leg-family conversion requires the current pinned generator.")
    fingerprint = hashlib.sha256(Path(__file__).read_bytes())
    for path in sorted(source.rglob("*")):
        if path.is_file():
            fingerprint.update(str(path.relative_to(source)).encode()+b"\0"+path.read_bytes()+b"\0")
    for name in ("LegoWalkers.Cli.dll", "LegoWalkers.Core.dll"):
        fingerprint.update(Path(generator.command[1]).with_name(name).read_bytes())
    cache = Path(cache).resolve()
    target = cache / fingerprint.hexdigest()
    if target.exists():
        seed_batches(target)
        return target
    cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="leg-families-", dir=cache) as temporary:
        temporary = Path(temporary)
        rebuilt = temporary / "inputs"
        shutil.copytree(source, rebuilt)
        records = []
        for batch, summary, _ in batches:
            relative = batch.relative_to(source)
            updated = json.loads(json.dumps(summary))
            for index, entry in enumerate(summary["generated"]):
                candidate = batch / entry["directory"]
                before = load_manifest(candidate / "manifest.json")
                report = {"status": "existing-constraints-preserved",
                          "families": before["genome"].get("legFamilies"), "unmatched": []}
                after = json.loads(json.dumps(before))
                if before["genome"].get("legFamilies") is None:
                    output = temporary / f"convert-{len(records)}"
                    generator.invoke("migrate-walker-legs", "--parent", candidate, "--output", output,
                                     "--steps", str(before["motion"]["cycleSteps"]))
                    generated = output / "candidate-001"
                    current = load_manifest(generated / "manifest.json")
                    report = json.loads((output / "leg-family-conversion.json").read_text())
                    genome = json.loads(json.dumps(current["genome"]))
                    genome.pop("legFamilies", None)
                    original = json.loads(json.dumps(before["genome"]))
                    original.pop("legFamilies", None)
                    if (genome != original
                            or current["motion"]["assemblyId"] != before["motion"]["assemblyId"]
                            or current["motion"]["mechanismId"] != before["motion"]["mechanismId"]
                            or json.loads((generated / "program.json").read_text())
                            != json.loads((candidate / "program.json").read_text())
                            or (generated / current["modelFile"]).read_bytes()
                            != (candidate / before["modelFile"]).read_bytes()
                            or any(current.get(key) != before.get(key) for key in ("parts", "payloads", "expected"))):
                        raise AdapterError(f"Leg-family conversion changed the physical design at {candidate}.")
                    if report["status"] == "converted":
                        after["genome"] = current["genome"]
                        after["motion"]["genotypeId"] = current["motion"]["genotypeId"]
                        destination = rebuilt / relative / entry["directory"]
                        write_json(destination / "manifest.json", after)
                        shutil.copy2(generated / "realization.json", destination / "realization.json")
                        load_manifest(destination / "manifest.json")
                verify_metadata_only(before, after)
                updated["generated"][index]["genotypeId"] = after["motion"]["genotypeId"]
                records.append({
                    "batch": str(relative), "directory": entry["directory"],
                    "sourceDesign": before["motion"]["genotypeId"], "targetDesign": after["motion"]["genotypeId"],
                    "status": report["status"], "families": report["families"], "unmatched": report["unmatched"],
                    "sourceManifestSha256": hashlib.sha256((candidate / "manifest.json").read_bytes()).hexdigest(),
                    "ldrawByteIdentical": True, "physicalMetadataIdentical": True, "dynamicTrials": 0,
                })
            write_json(rebuilt / relative / "candidates.json", updated)
        write_json(rebuilt / "leg-family-migration.json", {
            "schemaVersion": 1, **POLICY, "source": str(source), "designs": records, "dynamicTrials": 0})
        seed_batches(rebuilt)
        rebuilt.rename(target)
    return target
