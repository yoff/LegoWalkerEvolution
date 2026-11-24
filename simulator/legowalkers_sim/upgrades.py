"""Prepare a separate versioned experiment, preserving the original archive."""

import contextlib
import json
from pathlib import Path
import shutil
import socket
import sqlite3
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen

from .bridge import AdapterError, write_json
from .evolution import Archive, ROOT, now


def _origin(directory, birth, seen=()):
    from .campaign import database
    directory = Path(directory).resolve()
    if (directory, birth) in seen:
        raise AdapterError("Upgrade ancestry contains a cycle.")
    with database(directory) as db:
        row = db.execute("""
            WITH RECURSIVE ancestry(id,parent,design) AS (
                SELECT id,parent,design FROM births WHERE id=?
                UNION ALL
                SELECT b.id,b.parent,b.design FROM births b JOIN ancestry a ON b.id=a.parent
            )
            SELECT id,design FROM ancestry WHERE parent IS NULL
        """, (birth,)).fetchone()
        if row is None:
            raise AdapterError("Unknown birth while resolving its founder neighborhood.")
        root, design = row["id"], row["design"]
    metadata = directory / "upgrade.json"
    if metadata.exists():
        previous = json.loads(metadata.read_text())
        parent = next((p for p in previous["parents"] if p.get("targetDesign", p["sourceDesign"]) == design), None)
        if parent is not None:
            return _origin(parent.get("sourceArchive", previous["sourceArchive"]), parent["sourceBirth"],
                           (*seen, (directory, birth)))
    return str(directory), root


def representatives(sources):
    sources = list(sources)
    latest = Archive(sources[-1])
    try:
        population = latest.protocol().get("nativeDiscovery", {}).get("population")
    finally:
        latest.db.close()
    if population:
        return _population_seeds(sources)
    from .evolution import fitness
    selected = {}
    for directory in sources:
        archive = Archive(directory)
        try:
            if not archive.protocol().get("nativeDiscovery"):
                raise AdapterError("Site upgrades require native discovery versions.")
            roots = archive.db.execute("SELECT id,design FROM births WHERE parent IS NULL AND status!='duplicate'").fetchall()
            outcomes = archive.outcomes()
            for root in roots:
                choices = [row for row in outcomes if archive.founder(row["birth"]) == root["id"]]
                row = max(choices, key=lambda r: fitness(json.loads(r["result"]))) if choices else {
                    "birth": root["id"], "design": root["design"]}
                key = _origin(directory, root["id"])
                if key not in selected or choices or not selected[key]["measured"]:
                    selected[key] = {**row, "sourceArchive": str(Path(directory).resolve()),
                                     "measured": bool(choices), "founder": list(key)}
        finally:
            archive.db.close()
    if not selected:
        raise AdapterError("No native founder designs are available for an update.")
    return list(selected.values())


def _population_seeds(sources):
    from .campaign import definition, reusable
    from .population import retain

    latest = Path(sources[-1]).resolve()
    target = definition(latest)
    candidates = []
    for directory in reversed(sources):
        directory = Path(directory).resolve()
        if directory != latest and not reusable(directory, target, target_source=latest):
            continue
        archive = Archive(directory)
        try:
            rows = archive.outcomes()
            if directory == latest:
                rows += [dict(row) for row in archive.db.execute("""
                    SELECT b.id AS birth,b.design,d.category,d.genome,NULL AS result FROM births b
                    JOIN designs d ON d.id=b.design WHERE b.parent IS NULL AND b.status='generated'
                    AND NOT EXISTS(SELECT 1 FROM evaluations e WHERE e.birth=b.id)
                    AND NOT EXISTS(SELECT 1 FROM reused_evaluations r WHERE r.birth=b.id)
                    ORDER BY b.id
                """)]
            metadata = archive.directory / "upgrade.json"
            historical = ({parent.get("targetDesign", parent["sourceDesign"]): parent.get("historicalMeasurement", True)
                           for parent in json.loads(metadata.read_text())["parents"]} if metadata.exists() else {})
            for row in rows:
                origin = _origin(directory, row["birth"])
                candidates.append({**row, "family": origin, "founder": list(origin),
                                   "sourceArchive": str(directory),
                                   "measured": row["result"] is not None or historical.get(row["design"], False)})
        finally:
            archive.db.close()
    contributed = _contribution_seeds(latest)
    retained = retain([*candidates, *contributed])
    if not {row["family"] for row in contributed} <= {row["family"] for row in retained}:
        raise AdapterError("Population update cannot retain the founder ancestry of every active contribution.")
    if not retained:
        raise AdapterError("No compatible population or unevaluated founder seeds are available for an update.")
    return retained


def _contribution_seeds(directory):
    marker = directory / "campaign-version.json"
    if not marker.exists():
        return []
    campaign = Path(json.loads(marker.read_text())["campaign"]).resolve()
    registry = campaign / "user-workbench/workbench.sqlite3"
    if not registry.exists():
        return []
    with contextlib.closing(sqlite3.connect(f'{registry.as_uri()}?mode=ro', uri=True)) as db:
        entries = db.execute("""SELECT DISTINCT n.source_archive,n.source_birth FROM contributions c
            JOIN nodes n ON n.id=c.node WHERE c.active=1 ORDER BY n.source_archive,n.source_birth""").fetchall()
    seeds = []
    for source, birth in entries:
        if not (Path(source) / "archive.sqlite3").is_file():
            raise AdapterError("An active contribution's founder archive is unavailable.")
        archive = Archive(source)
        try:
            row = archive.db.execute("""SELECT b.id AS birth,b.design,d.category,d.genome FROM births b
                JOIN designs d ON d.id=b.design WHERE b.id=?""", (birth,)).fetchone()
            if row is None:
                raise AdapterError("An active contribution's founder birth is unavailable.")
            origin = _origin(source, birth)
            seeds.append({**dict(row), "result": None, "family": origin, "founder": list(origin),
                          "sourceArchive": str(Path(source).resolve()), "measured": False})
        finally:
            archive.db.close()
    return seeds


def design_aliases(directory):
    metadata = Path(directory) / "upgrade.json"
    if not metadata.exists():
        return {}
    previous = json.loads(metadata.read_text())
    return {parent["sourceDesign"]: parent.get("targetDesign", parent["sourceDesign"])
            for parent in previous["parents"]}


def prepare(directory, *, target=None, sources=None, generator=None, additional_parents=()):
    directory = Path(directory).resolve()
    extra = list(additional_parents)
    unique = {}
    for row in [*extra, *representatives(sources or [directory])]:
        unique.setdefault(row["design"], row)
    rows = list(unique.values())
    from .discovery import POLICY
    if target is None:
        number = 1
        while True:
            target = directory.parent / f"{directory.name}-upgrade-{number:04d}"
            try:
                target.mkdir()
                break
            except FileExistsError:
                number += 1
    else:
        target = Path(target).resolve()
        target.mkdir(parents=True)
    seeds = target / "seeds"
    seeds.mkdir()
    parents = []
    for index, row in enumerate(rows):
        name = f"neighborhood-{index+1}"
        batch = seeds / name
        source = Path(row["sourceArchive"]) / "designs" / row["design"] / "candidate"
        destination = batch / "candidate-001"
        shutil.copytree(source, destination)
        manifest = json.loads((source / "manifest.json").read_text())
        gates = json.loads((source / "gates.json").read_text())
        write_json(batch / "candidates.json", {
            "schemaVersion": 1, "gateVersion": "motion-gates-v1",
            "mechanismId": manifest["motion"]["mechanismId"],
            "gates": gates[:3], "generated": [{"directory": "candidate-001",
                "genotypeId": row["design"], "assemblyId": manifest["motion"]["assemblyId"]}],
            "rejected": [], "counts": {"proposals": 1, "realizationAttempts": 1,
                "rejected": 0, "duplicates": 0, "exported": 1, "dynamicTrials": 0}})
        parents.append({"seed": name, "sourceBirth": row["birth"], "sourceDesign": row["design"],
                        "sourceArchive": row["sourceArchive"], "founder": row["founder"],
                        "historicalMeasurement": row["measured"]})
    descriptor = {"schemaVersion": 1, "batches": [parent["seed"] for parent in parents]}
    if extra:
        extra_ids = {row["design"] for row in extra}
        descriptor["requalificationBatches"] = [parent["seed"] for parent in parents
                                               if parent["sourceDesign"] in extra_ids]
    write_json(seeds / "seeds.json", descriptor)
    if generator is not None:
        from .leg_family_migration import convert
        from .native_episode import refresh_seed_exports
        seeds = refresh_seed_exports(seeds, target / "input-exports", generator)
        seeds = convert(seeds, target / "leg-family-inputs", generator)
        for parent in parents:
            batch = json.loads((seeds / parent["seed"] / "candidates.json").read_text())
            parent["targetDesign"] = batch["generated"][0]["genotypeId"]
    unique = {}
    for parent in parents:
        unique.setdefault(parent.get("targetDesign", parent["sourceDesign"]), parent)
    if extra or len(unique) != len(parents):
        final = target / "prepared-seeds"
        shutil.copytree(seeds, final)
        parents = list(unique.values())
        descriptor["batches"] = [parent["seed"] for parent in parents]
        if extra:
            descriptor["requalificationBatches"] = [name for name in descriptor["requalificationBatches"]
                                                   if name in descriptor["batches"]]
            if len(descriptor["requalificationBatches"]) != len(extra):
                raise AdapterError("Requalification conversion merged distinct requested genotypes; refusing an incomplete queue.")
        write_json(final / "seeds.json", descriptor)
        seeds = final
    write_json(target / "upgrade.json", {
        "schemaVersion": 1, "sourceArchive": str(directory), "created": now(),
        "parents": parents, "policy": "gait-cycles-v1",
        "mutationVersion": POLICY["mutationVersion"],
        "population": POLICY["population"],
        "note": "Designs carried forward; measurements require compatibility checks. Zero trials authorized."})
    return target, seeds


def launch(controller):
    target, seeds = prepare(controller.directory)
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    command = [sys.executable, "-m", "legowalkers_sim.dashboard",
               "--output", str(target), "--candidate-batch", str(seeds), "--discover", "--cycle-aware",
               "--port", str(port), "--episode-trials", str(controller.episode_trials),
               "--seed", str(controller.seed), "--ldraw-library", str(controller.renderer.library)]
    for flag in ("--dotnet", "--generator", "--payload-profile"):
        if flag in controller.worker_args:
            command += [flag, controller.worker_args[controller.worker_args.index(flag)+1]]
    if "--generator" not in command:
        command += ["--generator", str(ROOT / "src/LegoWalkers.Cli/bin/Debug/net10.0/LegoWalkers.Cli.dll")]
    if controller.renderer.blender:
        command += ["--blender", str(controller.renderer.blender)]
    log = target / "dashboard.log"
    with log.open("wb") as stream:
        process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
    url = f"http://127.0.0.1:{port}/"
    deadline = time.monotonic()+30
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise AdapterError(f"Upgraded dashboard failed to start. See {log}. Original experiment is unchanged.")
        try:
            with urlopen(url+"api/status", timeout=1) as response:
                state = json.load(response)
            if state["budget"] != 0 or state["running"]:
                raise AdapterError("Upgraded dashboard unexpectedly authorized trials.")
            write_json(target / "dashboard.json", {"url": url, "pid": process.pid})
            return {"url": url, "archive": target.name, "budget": 0}
        except (URLError, TimeoutError):
            time.sleep(.1)
    process.terminate()
    process.wait(timeout=10)
    raise AdapterError(f"Upgraded dashboard did not become responsive. See {log}. Original experiment is unchanged.")
