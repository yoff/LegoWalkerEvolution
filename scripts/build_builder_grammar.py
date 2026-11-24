"""Publish an offline illustrated construction grammar; never run dynamics or change the campaign."""

import argparse
import hashlib
import html
import json
from pathlib import Path
import re
import shutil

from motif_illustrations import illustration

ROOT = Path(__file__).resolve().parents[1]
CORE = "src/LegoWalkers.Core/"
GENOTYPE = "bdaabc9a9fa6ca7bdfe371c8293d8615caf2effcdc19bc1cc7a6d67691642cc2"


def svg(title, content):
    return (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 560 240" role="img">'
        f"<title>{html.escape(title)}</title>{content}</svg>"
    )


def beam(x1, y1, x2, y2, color="#879cba"):
    return (
        f'<path d="M{x1},{y1} L{x2},{y2}" stroke="{color}" stroke-width="19" stroke-linecap="round"/>'
        + "".join(f'<circle cx="{x1+(x2-x1)*n/4}" cy="{y1+(y2-y1)*n/4}" r="4" fill="#152338"/>'
                  for n in range(5))
    )


def label(x, y, text, color="#d7e3f5"):
    return f'<text x="{x}" y="{y}" fill="{color}" font-size="14">{html.escape(text)}</text>'


def diagrams():
    frame = svg("Retain the frame; replace only explicitly obsolete packaging",
                beam(60, 80, 280, 80) + beam(60, 165, 280, 165)
                + beam(100, 80, 100, 165) + beam(240, 80, 240, 165)
                + '<path d="M325 100 H480 V160 H325 Z" stroke="#ffbf69" fill="none" stroke-dasharray="6 5"/>'
                + label(65, 210, "Retained scaffold") + label(330, 190, "Packaging to rebuild"))
    pins = svg("Two socket walls inside a stock pin's shaft interval",
               '<g fill="#879cba"><rect x="175" y="60" width="45" height="110" rx="8"/>'
               '<rect x="265" y="60" width="45" height="110" rx="8"/></g>'
               '<path d="M130 115 H355" stroke="#56d9bf" stroke-width="16" stroke-linecap="round"/>'
               '<path d="M130 185 V198 H355 V185" fill="none" stroke="#56d9bf"/>'
               + label(175, 35, "Coaxial sockets") + label(143, 224, "Stock shaft must cover both walls"))
    bearings = svg("A supported axle with two bearing centers and one gear plane",
                   '<path d="M70 125 H475" stroke="#56d9bf" stroke-width="12"/>'
                   '<g fill="#879cba"><rect x="145" y="85" width="20" height="80" rx="6"/>'
                   '<rect x="340" y="85" width="20" height="80" rx="6"/></g>'
                   '<rect x="240" y="55" width="30" height="140" rx="6" fill="#ffbf69"/>'
                   + label(83, 40, "Bearing") + label(315, 40, "Bearing")
                   + label(205, 222, "Gear clearance on both sides"))
    bonds = svg("Every added beam has an explicit rigid path back to the frame",
                beam(55, 160, 190, 80) + beam(190, 80, 330, 160) + beam(330, 160, 470, 80)
                + '<g fill="#56d9bf"><circle cx="55" cy="160" r="9"/><circle cx="190" cy="80" r="9"/>'
                '<circle cx="330" cy="160" r="9"/><circle cx="470" cy="80" r="9"/></g>'
                + label(25, 207, "Frame") + label(195, 40, "Witnessed friction-pin bonds")
                + label(407, 207, "Mount target"))
    layer = svg("Move a passive link in depth, not its planar pivot positions",
                beam(70, 150, 245, 150) + beam(100, 110, 275, 110, "#ffbf69")
                + '<path d="M245 150 L275 110" stroke="#56d9bf" stroke-width="3" stroke-dasharray="5 4"/>'
                + label(70, 200, "Inherited layer") + label(100, 65, "Alternate layer: +/-10 or +/-20 LDU")
                + label(310, 140, "Keep pivot coverage"))
    motor = svg("A motor must attach through its actual side sockets",
                '<rect x="215" y="55" width="170" height="115" rx="18" fill="#879cba"/>'
                '<path d="M385 112 H440" stroke="#56d9bf" stroke-width="12"/>'
                + beam(100, 70, 100, 180) + beam(100, 180, 235, 180)
                + '<path d="M235 180 V135" stroke="#56d9bf" stroke-width="9"/>'
                '<circle cx="235" cy="135" r="7" fill="#152338"/>'
                + label(233, 37, "58121.dat XL motor") + label(55, 222, "Bracket -> corner -> side socket"))
    accounting = svg("Every physical part has exactly one producer",
                     '<g fill="#233e5d" stroke="#56d9bf"><rect x="45" y="75" width="140" height="65" rx="12"/>'
                     '<rect x="215" y="75" width="130" height="65" rx="12"/>'
                     '<rect x="375" y="75" width="135" height="65" rx="12"/></g>'
                     + label(70, 114, "Produce parts") + label(236, 114, "Name bodies")
                     + label(391, 114, "Record goals")
                     + label(70, 190, "Unique IDs + dependencies + replayable choices"))
    level = svg("Preferred L: long arm upright and short branch away from grounded arm",
                beam(460, 170, 280, 170)
                + '<path d="M280 50 V170 H220" fill="none" stroke="#ffbf69" stroke-width="19" stroke-linecap="round" stroke-linejoin="round"/>'
                + '<g fill="#152338"><circle cx="280" cy="50" r="4"/><circle cx="280" cy="170" r="4"/>'
                '<circle cx="220" cy="170" r="4"/></g>'
                '<path d="M240 205 H165 L175 198 M165 205 L175 212" fill="none" stroke="#56d9bf" stroke-width="3"/>'
                + label(300, 58, "Long arm upright") + label(310, 147, "Grounded arm")
                + label(22, 174, "Short branch / toe") + label(98, 231, "Away is preferred, not mandatory"))
    return {"frame": frame, "pins": pins, "bearings": bearings, "bonds": bonds,
            "layer": layer, "motor": motor, "accounting": accounting, "level": level}


# Readable summaries of executable rules, not another executable construction grammar.
RULES = [
    ("frame", "01", "Retain the load-bearing context", "Foundation",
     "Frame → RetainedScaffold − ExplicitObsoleteParts",
     "Start with the source rails, bridges and grounded anchors. Rebuild the packaging, not an imaginary frame.",
     "Only explicitly obsolete parts are removed. Moving links retain their declared motion and pivot obligations.",
     "This production does not design an arbitrary new chassis or certify its strength.",
     CORE + "WalkerConstructionTheory.fs", 'resolve proof byGoal["frame"]', "frame"),
    ("stock", "02", "Fit real sockets, not stretched beams", "Stock",
     "Beam → 3L | 5L | 7L | 11L",
     "The packaging support stock is 32523, 32316, 32524 and 32525. Their endpoint spans are 40, 80, 120 and 200 LDU.",
     "A rigid placement must match catalogue hole positions and axes. One stud is 20 LDU (8 mm); L is overall nominal length, not endpoint span.",
     "These are the support production's stocks, not the complete LEGO parts catalogue.",
     CORE + "WalkerPackaging.fs", 'let private stock', "pins"),
    ("connectors", "03", "Choose a pin that covers the stack", "Connections",
     "RigidJoint → CoaxialSockets FrictionPin CoveredWalls",
     "4459.dat is the 2L friction pin; 6558.dat is the 3L alternative. Longer stock enables legal stacks and axial depth changes.",
     "The pin must cover both modeled socket walls, respect contact capacity and avoid overlapping another connector. Support socket centers are at most 40 LDU apart.",
     "A long pin is not an unlimited bridge. Modeled engagement does not prove retention or assembly order.",
     CORE + "WalkerPackaging.fs", 'let private pinAlong', "pins"),
    ("pivot", "04", "Preserve articulation", "Connections",
     "RevoluteConnection → CompatibleSockets SmoothPin AxialPlacement",
     "Use 32002.dat (short 2L), 3673.dat (2L), or 32556.dat (3L) for the supported smooth-pin recipes. Driven bearings use axles.",
     "A moving pivot must connect exactly its declared two or three bodies with N−1 hinges. The compiler rejects rigid friction pins at moving pivots.",
     "The short 32002 shaft interval is asymmetric: −20..10 LDU. Nominal part length is not interchangeable with usable modeled shaft length.",
     CORE + "WalkerRealization.fs", 'A moving pivot cannot', "pins"),
    ("supports", "05", "Reach the mount through legal supports", "Packaging",
     "Support → TwoBeamRoute | ExtendedRoute(3..5)",
     "Try the shortest feasible beam count first. Extra 3L beams can retreat around occupied anchors before extending toward the target.",
     "Extensions use 40-LDU socket spans and legal 3L friction-pin joints. Each connector is checked; anchor selection and route counts are bounded.",
     "More beams are offered only when shorter routes have no feasible alternatives. This is not an exhaustive search of every possible bracket.",
     CORE + "WalkerPackaging.fs", 'let supportRoutes', "bonds"),
    ("mounts", "06", "Mount actual hardware", "Packaging",
     "Hardware → BatteryMount ReceiverMount MotorSideBracket",
     "Build around the real 58119 battery box, 58123 receiver and 58121 motor. Keep their physical masses in evaluation.",
     "Exactly one motor is required. Targets use supported orientations; XL motor mounting uses actual front side sockets and a corner connector.",
     "Motor setbacks are bounded by the request's tolerance. Choosing a tolerance or reduction goal is still external direction.",
     CORE + "WalkerPackaging.fs", 'let private validateDrive', "motor"),
    ("bearings", "07", "Support stock axles and compound shafts", "Drive",
     "SupportedShaft → StockAxle Bearing Gear Bearing",
     "Place ordinary bearing centers 20 LDU from gear planes; the larger 40T gear requires 30 LDU. Internal compound bearings can fit between two gears.",
     "Internal gear separation must cover both clearance gaps plus at least 20 LDU between bearing centers. Supported axle stock is 4L, 7L, 8L, 9L, 10L (3737.dat) and 12L; full socket-wall reach is required.",
     "Bearings and bushes do not automatically certify axial retention, stiffness or hardware durability.",
     CORE + "WalkerPackaging.fs", 'let bearingPlanes', "bearings"),
    ("gears", "08", "Require a physical mesh for every transmission", "Drive",
     "SpurMesh → MatchingTeeth Ratio CommonPlane PitchSpacing",
     "Supported spur gears have 8, 24 or 40 teeth. For external meshes, output/input ratio is −Na/Nb and center spacing is (Na+Nb) × 1.25 LDU.",
     "Exactly one physical gear pair must realize each transmission. Shared mesh validation rejects impossible drive alternatives before support search.",
     "A symbolic ratio without the matching LEGO gears is not a realized transmission.",
     CORE + "Domain.fs", 'let matchesSpurMesh', "bearings"),
    ("reduction", "09", "Gear down physically; do not turn up torque", "Experimental",
     "Root24:24 → Root8:40 + RetimedMotion + RebuiltSupports",
     "The explicit packaging request can introduce a 5:1 root stage: ratio magnitude ÷5, reference clock ×5, initial output phase preserved.",
     "The fresh request moves the root mesh plane +80 LDU and preserves downstream 24T meshes. All geometry, support and collision checks remain required.",
     "This packaging layout is NOT qualified: reduced-routing v8 rejected four layouts for interference. Slow/stalled dynamics do not yet automatically request this proof search.",
     CORE + "WalkerConstructionTheory.fs", 'let withMotorReduction', "bearings"),
    ("bonds", "10", "Every added part creates an obligation", "Proof",
     "AddedBeam → PhysicalPart + RigidBond + RigidPathToFrame",
     "A bridge is not accepted merely because it visually touches another beam. Each bracket pin needs an explicit bond between the correct endpoint parts.",
     "New bridge beams introduce their own frame-path goals. The exact library checks every new chassis part and every bracket friction-pin bond.",
     "The mount must bind to the FINAL bridge beam. Local proof completion cannot close external connectivity or pivot-preservation obligations.",
     CORE + "WalkerConstructionTheory.fs", 'let private additionalBeamGoals', "bonds"),
    ("repair", "11", "Repair link layers and connector stacks together", "Feedback",
     "Assembly → LayeredLinks JointStacks",
     "Initial or moving contact pairs automatically focus alternatives. Prefer connector-only fixes; otherwise move an implicated passive link by ±10/±20 LDU and refit compatible pins.",
     "An implicated axle can use shorter supported stock or a new axial position, but must retain every originally reached part. Protected driven/chassis bodies are not freely shifted.",
     "Exact membership, socket-wall coverage and fresh dynamics remain mandatory. Two-link/state/layout/stack and repair-depth bounds still apply; a repair is not a collision-free certificate.",
     CORE + "WalkerConnectionGrammar.fs", 'let productions', "layer"),
    ("proof", "12", "Make every choice replayable", "Proof",
     "Production → StableChoice + Dependencies + Parts + NewGoals",
     "The builder records rule choices, unique physical part IDs, body provenance, costs and obligation dependencies in a derivation.",
     "The dependency graph is acyclic. Bond witnesses depend on all referenced producers. Scope binds the request, source, builder and external oracle.",
     "A completed local derivation is a candidate, not a qualified walker. External reconstruction and fresh dynamics decide admission and walking success.",
     CORE + "ConstructionSearch.fs", 'let validate', "accounting"),
    ("level-feet", "13", "Prefer an outward L without banning alternatives", "Feet",
     "PassiveLeg → Dyad + UpperParallelogram + LevelShoeParallelogram",
     "Replace one independent passive leg while retaining the motor, transmission, carried hardware and foot identity. Prefer the actual molded L bend at the corner, long arm upright and short branch away from the grounded arm.",
     "Check outward direction throughout sampled motion. Prefer retained ground anchors, the existing dyad assembly branch and the full long-arm span before other supported sockets/branches. Preferred layouts lead the four-choice menu, with one slot reserved for another admissible orientation. Already-flat nonpreferred legs can be rebuilt with fresh anchor identifiers.",
     "40 LDU spacing and inward/swapped-arm layouts remain legal alternatives. Orientation is a collision-risk heuristic, not a proof: exact reconstruction, clearance and fresh calibrated walking still decide success.",
     CORE + "WalkerLevelFeet.fs", 'let attemptsWithLimit', "level"),
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_link(output, source, needle):
    path = ROOT / source
    lines = path.read_text().splitlines()
    line = next((i for i, text in enumerate(lines, 1) if needle in text), None)
    if line is None:
        raise ValueError(f"Source anchor no longer exists: {source}: {needle}")
    relative = f"assets/sources/{source}.txt"
    return f'<a href="{html.escape(relative, quote=True)}">{html.escape(path.name)} · line {line}</a>'


def rule_cards(output):
    drawings = diagrams()
    cards = []
    for key, number, title, category, production, description, guards, limitation, source, anchor, drawing in RULES:
        special = {
            "connectors": '<div id="pin-demo" class="demo" aria-label="Interactive pin coverage"></div>',
            "supports": '<div id="support-demo" class="demo" aria-label="Interactive support routes"></div>',
            "gears": '<div id="gear-demo" class="demo" aria-label="Interactive spur gearing"></div>',
        }.get(key)
        cards.append(
            f'<article id="rule-{key}" class="rule" data-category="{category}">'
            f'<div class="rule-head"><span class="number">{number}</span><span class="badge">{category}</span></div>'
            f'<h3>{html.escape(title)}</h3><code class="production">{html.escape(production)}</code>'
            f'<p>{html.escape(description)}</p>'
            f'<figure>{special or drawings[drawing]}<figcaption>Explanatory schematic; not LEGO mesh or collision evidence.</figcaption></figure>'
            f'<dl><dt>Must remain true</dt><dd>{html.escape(guards)}</dd>'
            f'<dt>What this does not prove</dt><dd>{html.escape(limitation)}</dd></dl>'
            f'<details><summary>Executable source</summary>{source_link(output, source, anchor)}</details></article>'
        )
    return "\n".join(cards)


def publish(output, archive, witnesses):
    output, archive, witnesses = output.resolve(), archive.resolve(), witnesses.resolve()
    if output.exists():
        raise ValueError("Use a fresh output directory to preserve an existing guide.")
    if (output == ROOT or ROOT.is_relative_to(output) or archive.is_relative_to(output)
            or witnesses.is_relative_to(output) or output.is_relative_to(archive) or output.is_relative_to(witnesses)):
        raise ValueError("Guide output must be separate from its immutable source inputs.")
    candidate = archive / "designs" / GENOTYPE / "candidate"
    result_path = archive / "evaluations/0001/result.json"
    result = json.loads(result_path.read_text())
    manifest = json.loads((candidate / "manifest.json").read_text())
    if (not result["metrics"]["eligibleForRanking"] or result["metrics"]["rankingRejectionReasons"]
            or result["groundContact"]["forbiddenContactDetected"]
            or result["motorModel"]["version"] != "pf-xl-torque-speed-v2"):
        raise ValueError("The guide requires the measured calibrated qualified walker.")
    media = archive / "designs" / GENOTYPE / "render"
    metadata = json.loads((media / "metadata.json").read_text())
    if Path(metadata["evaluation"]).resolve() != result_path.parent or metadata["result"]["metrics"] != result["metrics"]:
        raise ValueError("Rendered animation provenance does not match the qualification result.")
    gallery_path = witnesses / "gallery.json"
    gallery = json.loads(gallery_path.read_text())
    if gallery["mode"] != "construction-schematic-not-dynamics":
        raise ValueError("Expected catalogue port/shaft schematic witnesses.")
    versions = {}
    sources = {source for *_, source, anchor, drawing in RULES}
    sources.update({CORE + "WalkerBuilder.fs", "simulator/legowalkers_sim/discovery.py",
                    "simulator/legowalkers_sim/bridge.py", "scripts/construct_packaging.py",
                    "scripts/build_builder_grammar.py", "scripts/builder_grammar.html",
                    "scripts/builder_grammar.js", "scripts/motif_illustrations.py"})
    for name in ("WalkerPackaging", "WalkerConstructionTheory", "ConstructionSearch", "WalkerConnectionGrammar"):
        text = (ROOT / CORE / f"{name}.fs").read_text()
        versions[name] = re.search(r'let version = "([^"]+)"', text).group(1)
    cards = rule_cards(output)
    witnesses_html = []
    for example in gallery["examples"]:
        image = illustration(example).replace('id="direction"', f'id="direction-{example["id"]}"')
        image = image.replace("url(#direction)", f'url(#direction-{example["id"]})')
        witnesses_html.append(
            f'<figure class="witness">{image}<figcaption>{html.escape(example["title"])}. '
            'Real catalogue-port/shaft witness; not a LEGO mesh or physical qualification.</figcaption></figure>'
        )
    output.mkdir(parents=True)
    assets = output / "assets"
    assets.mkdir()
    copies = {
        "walker.png": media / "walker-v2.png",
        "walker.mp4": media / "walker-v2-anchored.mp4",
        "model.ldr": candidate / "model.ldr",
        "qualification.json": result_path,
    }
    for name, source in copies.items():
        shutil.copy2(source, assets / name)
    shutil.copy2(Path(__file__).with_name("builder_grammar.js"), assets / "builder_grammar.js")
    substitutions = {
        "RULES": cards, "WITNESSES": "\n".join(witnesses_html),
        "VERSIONS": " · ".join(html.escape(value) for value in versions.values()),
        "FORWARD": f'{result["metrics"]["forwardMeters"]:.3f}',
        "CYCLES": str(result["gait"]["completeCycles"]),
        "MASS": f'{result["metrics"]["simulatedMassKg"]:.3f}',
        "PARTS": str(len(manifest["parts"])),
        "CLOSURE": f'{result["metrics"]["maxClosureErrorMeters"]*1000:.3f}',
        "ARCHIVE": html.escape(archive.name),
        "CONSTRUCTION_SOURCE": source_link(output, "scripts/construct_packaging.py", 'while pending:'),
        "SEARCH_SOURCE": source_link(output, CORE + "ConstructionSearch.fs", 'let next'),
        "FEEDBACK_SOURCE": source_link(output, "simulator/legowalkers_sim/discovery.py", 'def backwards_only'),
        "POLICY_SOURCE": source_link(output, CORE + "WalkerBuilder.fs", 'let defaultPolicy'),
    }
    template = Path(__file__).with_name("builder_grammar.html").read_text()
    required = set(re.findall(r"\{\{([A-Z_]+)\}\}", template))
    if required != set(substitutions):
        raise ValueError("Guide template and publisher placeholders differ.")
    page = re.sub(r"\{\{([A-Z_]+)\}\}", lambda match: substitutions[match[1]], template)
    (output / "index.html").write_text(page)
    inputs = {name: digest(ROOT / name) for name in sorted(sources)}
    for name in sorted(sources):
        target = assets / "sources" / (name + ".txt")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, target)
    provenance = {
        "schemaVersion": 1, "mode": "documentation-not-construction-or-dynamics",
        "versions": versions, "sourceSha256": inputs, "archive": str(archive),
        "genotype": GENOTYPE, "mediaProvenance": str(media / "metadata.json"),
        "witnesses": str(witnesses), "witnessSha256": digest(gallery_path),
        "assetSha256": {name: digest(assets / name) for name in (*copies, "builder_grammar.js")},
        "sourceSnapshot": "assets/sources/<source path>.txt",
        "rules": [{"id": key, "category": category, "source": source}
                  for key, number, title, category, production, description, guards, limitation, source, anchor, drawing in RULES],
    }
    (output / "guide-provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return output / "index.html"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--archive", type=Path, default=ROOT / "output/walker-discovery/versions/v0041")
    parser.add_argument("--witnesses", type=Path, default=ROOT / "output/motif-witnesses-v2")
    args = parser.parse_args()
    print(publish(args.output, args.archive, args.witnesses))


if __name__ == "__main__":
    main()
