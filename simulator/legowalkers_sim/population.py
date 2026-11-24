"""Bounded, diversity-preserving parents derived from immutable observations."""

from collections import Counter
import json
from pathlib import Path
import random

from .bridge import AdapterError
from .evolution import fitness, is_qualified


POLICY = {
    "version": "native-population-v1",
    "capacity": 12,
    "tournamentSize": 2,
    "randomParentProbability": .2,
    "retention": "Protect each founder family, qualified category champions and the best prototype; "
                 "then prefer distinct structural/layout bins before filling by fitness.",
    "diversity": "Foot category, body/connection counts, part inventory, coarse motor/battery positions and foot definitions.",
}


def family_map(archive):
    roots = list(archive.db.execute(
        "SELECT id,design FROM births WHERE parent IS NULL AND status!='duplicate' ORDER BY id"))
    metadata = archive.directory / "upgrade.json"
    origins = {}
    if metadata.exists():
        for parent in json.loads(metadata.read_text())["parents"]:
            if "founder" in parent:
                source, birth = parent["founder"]
                origins[parent.get("targetDesign", parent["sourceDesign"])] = (str(Path(source).resolve()), birth)
    anchors, groups = {}, {}
    for root in roots:
        origin = origins.get(root["design"], (str(archive.directory), root["id"]))
        groups[root["id"]] = anchors.setdefault(origin, root["id"])
    return groups


def family(archive, birth, groups):
    root = archive.founder(birth)
    if root not in groups:
        raise AdapterError("Population birth belongs to an unavailable founder family.")
    return groups[root]


def _quality(row):
    return fitness(json.loads(row["result"])) if row["result"] is not None else (-1,)


def _qualified(row):
    return row["result"] is not None and is_qualified(json.loads(row["result"]))


def _descriptor(row):
    genome = json.loads(row["genome"])
    parts = [part for body in genome["bodies"] for part in body["parts"]]
    inventory = tuple(sorted(Counter(part["part"] for part in parts).items()))
    layout = tuple(sorted(
        (part["id"], tuple(round(part["position"][axis] / 20) for axis in ("x", "y", "z")))
        for part in parts if part["id"] in ("motor", "payload-battery")))
    feet = tuple(sorted((foot["part"], foot["depth"]) for foot in genome["feet"]))
    return row["category"], len(genome["bodies"]), len(genome["connections"]), inventory, layout, feet


def retain(rows):
    ranked = sorted(rows, key=_quality, reverse=True)
    unique = {}
    for row in ranked:
        unique.setdefault(row["design"], row)
    ranked = list(unique.values())
    selected = {}

    def keep(row):
        selected.setdefault(row["design"], row)

    families, champions = set(), set()
    for row in ranked:
        if row["family"] not in families:
            keep(row)
            families.add(row["family"])
        if _qualified(row) and row["category"] not in champions:
            keep(row)
            champions.add(row["category"])
    prototype = next((row for row in ranked if row["result"] is not None and not _qualified(row)), None)
    if prototype is not None:
        keep(prototype)
    capacity = POLICY["capacity"]
    if len(selected) > capacity:
        raise AdapterError(f"Population capacity {capacity} cannot preserve all founder families and protected elites.")
    bins = {_descriptor(row) for row in selected.values()}
    for row in ranked:
        if len(selected) == capacity:
            break
        descriptor = _descriptor(row)
        if descriptor not in bins:
            keep(row)
            bins.add(descriptor)
    for row in ranked:
        if len(selected) == capacity:
            break
        keep(row)
    return list(selected.values())


def members(archive):
    groups = family_map(archive)
    return retain([{**row, "family": family(archive, row["birth"], groups)} for row in archive.outcomes()])


def choose(parents, seed, proposal):
    if not parents:
        raise AdapterError("Native discovery has no measured population parent in this founder family.")
    rng = random.Random(f'{POLICY["version"]}:{seed}:{proposal}')
    uniform = rng.random() < POLICY["randomParentProbability"]
    contestants = rng.sample(parents, 1 if uniform else min(POLICY["tournamentSize"], len(parents)))
    parent = max(contestants, key=_quality)
    return parent, {
        "policy": POLICY["version"],
        "mode": "random" if uniform else "tournament",
        "populationBirths": [row["birth"] for row in parents],
        "contestants": [row["birth"] for row in contestants],
        "selectedBirth": parent["birth"],
        "proposalIndex": proposal,
    }
