"""Reuse immutable measurements only across identical evaluation conditions."""

import ast
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3

from .bridge import AdapterError, library_fingerprint, load_manifest, write_json
from .evolution import ROOT, canonical, identity

PHYSICS_SOURCES = ("bridge.py", "collision.py", "evaluation.py", "motor.py", "payload.py", "ground.py", "schema.py", "mass_layout.py")

_GATE_ADDITIONS = {
    ast.dump(ast.parse(statement).body[0], include_attributes=False)
    for statement in (
        "from .simulation_gate import SimulationPrefixGate, validate_short_gate",
        "from copy import deepcopy",
        "short_gate: float = 0.0",
        "validate_short_gate(self.short_gate, self.timestep)",
        "gate = SimulationPrefixGate(output, options.short_gate, options.timestep, settle_steps, steps)",
        "gate = SimulationPrefixGate(output, options.short_gate, options.timestep, settle_steps, steps, on_pass=on_gate_passed)",
        """if gate.active:
    gate.observe(0, float(data.time), max_tilt, max_closure, deepcopy(penetrations), deepcopy(ground_contacts.report()), frames)""",
        """if gate.active:
    gate.observe(step + 1, float(data.time), max_tilt, max_closure, deepcopy(penetrations), deepcopy(ground_contacts.report()), frames)""",
        """if options.short_gate:
    result["simulationGate"] = gate.report""",
    )
}

_SUPPORT_ADDITIONS = {
    ast.dump(ast.parse(statement).body[0], include_attributes=False)
    for statement in (
        "self.surfaces = []",
        """if any("contactShape" in foot for foot in manifest.get("motion", {}).get("logicalFeet", [])):
    from .support_surfaces import support_surfaces
    self.surfaces = support_surfaces(manifest, self.regions)""",
        """for surface in self.surfaces:
    if instance == surface.part and surface.contains(position, feet):
        self.foot_samples += 1
        return""",
        """WALKER_MANIFEST_SCHEMA["properties"]["motion"]["properties"]["logicalFeet"]["items"]["properties"]["contactShape"] = {"enum": ["capsule", "rectangle"]}""",
    )
}


class _WithoutOptionalSupport(ast.NodeTransformer):
    def visit(self, node):
        if ast.dump(node, include_attributes=False) in _SUPPORT_ADDITIONS:
            return None
        return super().visit(node)


class _WithoutSimulationGate(ast.NodeTransformer):
    """Remove only the exact read-only observer additions, not physics statements."""

    def visit(self, node):
        if ast.dump(node, include_attributes=False) in _GATE_ADDITIONS:
            return None
        return super().visit(node)

    def visit_FunctionDef(self, node):
        if (node.name == "evaluate" and len(node.args.kwonlyargs) == 1
                and node.args.kwonlyargs[0].arg == "on_gate_passed"
                and node.args.kwonlyargs[0].annotation is None
                and len(node.args.kw_defaults) == 1
                and isinstance(node.args.kw_defaults[0], ast.Constant)
                and node.args.kw_defaults[0].value is None):
            node.args.kwonlyargs = []
            node.args.kw_defaults = []
        return self.generic_visit(node)


_PARSER_CACHE_CLASS_SHA256 = "491ab61c04f68735f7db8e34d5bf23a0e9d7acde9f5b7706dfcfb77f5bbcbec9"
_PARSER_CACHE_ARGUMENT = ast.dump(ast.parse(
    "def check(*, parser_cache: PartParserCache | None = None): pass"
).body[0].args.kwonlyargs[0], include_attributes=False)
_PARSER_CACHE_PARSE = ast.parse("""
try:
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        model = path.parent / manifest["modelFile"]
        parser = (LDrawParser(parts_dir=library) if parser_cache is None else
                  parser_cache.parser_for(model, library, [part["part"] for part in manifest["parts"]]))
        build = parser.parse_build(model)
        scene = build_units_and_joints(build)
    parsed = True
finally:
    if not parsed and parser_cache is not None:
        parser_cache.invalidate()
""").body[0]
_UNCACHED_PARSE = """
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    build = LDrawParser(parts_dir=library).parse_build(path.parent / manifest["modelFile"])
    scene = build_units_and_joints(build)
"""
_PARSER_CACHE_WARNING = ast.parse("""
if caught:
    if parser_cache is not None:
        parser_cache.invalidate()
    raise AdapterError("Simulator warnings: " + "; ".join(str(item.message) for item in caught))
""").body[0]
_UNCACHED_WARNING = """
if caught:
    raise AdapterError("Simulator warnings: " + "; ".join(str(item.message) for item in caught))
"""


class _WithoutOptionalParserCache(ast.NodeTransformer):
    """Prove only the exact pilot cache's default-None path is unchanged."""

    def visit_ClassDef(self, node):
        digest = hashlib.sha256(ast.dump(node, include_attributes=False).encode()).hexdigest()
        if node.name == "PartParserCache" and digest == _PARSER_CACHE_CLASS_SHA256:
            return None
        return node

    def visit_FunctionDef(self, node):
        if (node.name != "check" or len(node.args.kwonlyargs) != 1
                or ast.dump(node.args.kwonlyargs[0], include_attributes=False) != _PARSER_CACHE_ARGUMENT
                or len(node.args.kw_defaults) != 1
                or not isinstance(node.args.kw_defaults[0], ast.Constant)
                or node.args.kw_defaults[0].value is not None):
            return node
        replacements = {
            ast.dump(ast.parse("parsed = False").body[0], include_attributes=False): None,
            ast.dump(_PARSER_CACHE_PARSE, include_attributes=False): _UNCACHED_PARSE,
            ast.dump(_PARSER_CACHE_WARNING, include_attributes=False): _UNCACHED_WARNING,
        }
        statements = [ast.dump(statement, include_attributes=False) for statement in node.body]
        if any(statements.count(pattern) != 1 for pattern in replacements):
            return node
        node.args.kwonlyargs = []
        node.args.kw_defaults = []
        body = []
        for statement, key in zip(node.body, statements, strict=True):
            if key not in replacements:
                body.append(statement)
            elif replacements[key] is not None:
                body.extend(ast.parse(replacements[key]).body)
        node.body = body
        return node


def evaluation_signature(protocol):
    """Physics/scoring identity; assembly-specific dependencies are verified on import."""
    required = ("simulatorRevision", "versions", "evaluation",
                "payloadProfile", "qualification", "controller")
    if any(key not in protocol for key in required):
        raise AdapterError("Cannot reuse a result without a complete evaluation protocol.")
    sources = protocol.get("sourceSha256", {})
    names = [f"simulator/legowalkers_sim/{name}" for name in PHYSICS_SOURCES]
    optional_support = "simulator/legowalkers_sim/support_surfaces.py"
    if optional_support in sources:
        names.append(optional_support)
    if protocol.get("evaluation", {}).get("cycle_aware"):
        names.append("simulator/legowalkers_sim/cycles.py")
    if any(name not in sources for name in names):
        raise AdapterError("Cannot reuse results without all physics implementation fingerprints.")
    conditions = {key: protocol[key] for key in required}
    conditions["evaluation"] = {key: value for key, value in protocol["evaluation"].items() if key != "short_gate"}
    return identity({**conditions,
                     "physicsSources": {name: sources[name] for name in names}})


def compatible_evaluation(original, target, source, *, manifest=None, target_source=None):
    from .evolution import evidence_invalidation
    if evidence_invalidation(original) or evidence_invalidation(target):
        return False
    target_root = Path(target_source) / "protocol-source" if target_source is not None else ROOT
    adjusted = {**original, "sourceSha256": dict(original.get("sourceSha256", {}))}
    legacy_support = manifest is not None and not any(
        "contactShape" in foot for foot in manifest.get("motion", {}).get("logicalFeet", []))
    support_name = "simulator/legowalkers_sim/support_surfaces.py"
    if original.get("sourceSha256", {}).get(support_name) != target.get("sourceSha256", {}).get(support_name):
        if not legacy_support or support_name not in target.get("sourceSha256", {}):
            return False
        if hashlib.sha256((target_root / support_name).read_bytes()).hexdigest() != target["sourceSha256"][support_name]:
            raise AdapterError("Support implementation snapshot changed; cannot prove measurement compatibility.")
        adjusted["sourceSha256"][support_name] = target["sourceSha256"][support_name]
    for filename in ("bridge.py", "evaluation.py", "ground.py", "schema.py"):
        name = f"simulator/legowalkers_sim/{filename}"
        if adjusted["sourceSha256"].get(name) == target.get("sourceSha256", {}).get(name):
            continue
        if filename in ("ground.py", "schema.py") and not legacy_support:
            return False
        roots = (source / "protocol-source", target_root)
        trees = []
        for root, protocol in zip(roots, (original, target), strict=True):
            content = (root / name).read_bytes()
            if hashlib.sha256(content).hexdigest() != protocol["sourceSha256"][name]:
                raise AdapterError(f"Physics-source snapshot changed: {name} in {root}; "
                                   "cannot prove measurement compatibility.")
            tree = ast.parse(content)
            if filename == "bridge.py":
                tree = _WithoutOptionalParserCache().visit(tree)
                tree.body = [node for node in tree.body
                             if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) or node.name != "_validate_walker_metadata"]
            elif filename == "evaluation.py":
                tree = _WithoutSimulationGate().visit(tree)
            else:
                tree = _WithoutOptionalSupport().visit(tree)
            trees.append(ast.dump(tree, include_attributes=False))
        if trees[0] != trees[1]:
            return False
        adjusted["sourceSha256"][name] = target["sourceSha256"][name]
    return evaluation_signature(adjusted) == evaluation_signature(target)


def reuse_observations(archive, library):
    from .upgrades import design_aliases
    from .leg_family_migration import verify_metadata_only

    paths = archive.protocol().get("reuseArchives", [])
    if not paths:
        return
    target_signature = evaluation_signature(archive.protocol())
    aliases = design_aliases(archive.directory)
    for path in paths:
        source = Path(path).resolve()
        if archive.db.execute("SELECT 1 FROM reused_evaluations WHERE source_archive=?", (str(source),)).fetchone():
            continue
        db = sqlite3.connect(f"file:{source / 'archive.sqlite3'}?mode=ro", uri=True)
        db.row_factory = sqlite3.Row
        imported = 0
        try:
            for row in db.execute("""
                SELECT e.*,b.design,p.definition FROM evaluations e JOIN births b ON b.id=e.birth
                JOIN protocols p ON p.id=e.protocol WHERE e.status='complete' ORDER BY e.id
            """):
                design = aliases.get(row["design"], row["design"])
                if design in archive.protocol().get("requalificationCandidates", []):
                    continue
                birth = archive.db.execute("""
                    SELECT b.id FROM births b WHERE b.design=? AND b.parent IS NULL
                    AND NOT EXISTS(SELECT 1 FROM evaluations e WHERE e.birth=b.id)
                    AND NOT EXISTS(SELECT 1 FROM reused_evaluations r WHERE r.birth=b.id)
                """, (design,)).fetchone()
                if birth is None:
                    continue
                original = source / "designs" / row["design"] / "candidate"
                candidate = archive.directory / "designs" / design / "candidate"
                before, after = load_manifest(original / "manifest.json"), load_manifest(candidate / "manifest.json")
                if design != row["design"]:
                    verify_metadata_only(before, after)
                    if ((original / "program.json").read_bytes() != (candidate / "program.json").read_bytes()
                            or (original / before["modelFile"]).read_bytes() != (candidate / after["modelFile"]).read_bytes()):
                        raise AdapterError("Converted observation inputs changed motion or LDraw geometry.")
                elif canonical(before) != canonical(after):
                    raise AdapterError("Reused observations require an identical complete physical/evaluation manifest.")
                target_source = archive.directory if (archive.directory / "protocol-source").is_dir() else None
                if not compatible_evaluation(json.loads(row["definition"]), archive.protocol(), source, manifest=after,
                                             target_source=target_source):
                    raise AdapterError("Archived measurement uses different physics or evaluation conditions.")
                result = json.loads(row["result"])
                if result.get("simulationGate", {}).get("status") == "rejected":
                    raise AdapterError("A short-gate rejection is not a complete reusable measurement.")
                if result["compatibility"]["ldrawSha256"] != after["ldrawSha256"]:
                    raise AdapterError("Reused measurement has a mismatched physical assembly checksum.")
                measured = source / "evaluations" / f"{row['id']:04d}" / "result.json"
                if not measured.exists() or canonical(json.loads(measured.read_text())) != canonical(result):
                    raise AdapterError("Source measurement differs from its archived result file.")
                dependencies = library_fingerprint(Path(library).resolve(), [part["part"] for part in after["parts"]])
                if result["compatibility"].get("partsDependencySha256") != dependencies:
                    raise AdapterError("Reused measurement has missing or changed recursive part dependencies.")
                destination = archive.directory / "observations" / f"{birth['id']:04d}"
                write_json(destination / "result.json", result)
                write_json(destination / "source-protocol.json", json.loads(row["definition"]))
                write_json(destination / "provenance.json", {
                    "sourceArchive": str(source), "sourceEvaluation": row["id"], "sourceProtocol": row["protocol"],
                    "resultSha256": hashlib.sha256(canonical(result).encode()).hexdigest(),
                    "evaluationSignature": target_signature, "partsDependencySha256": dependencies,
                    "newDynamicTrials": 0})
                with archive.db:
                    archive.db.execute("INSERT INTO reused_evaluations VALUES (?,?,?,?,?,?,?)",
                                       (birth["id"], str(source), row["id"], row["protocol"], target_signature,
                                        canonical(result), row["completed"]))
                for folder in ("render", "assembly-render"):
                    png = original.parent / folder / "walker-v2.png"
                    if png.exists():
                        target = candidate.parent / folder / png.name
                        target.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(png, target)
                        break
                imported += 1
        finally:
            db.close()
        if not imported:
            raise AdapterError(f"No compatible unused founder measurement was found in {source}.")
