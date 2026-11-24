"""Read-only, dependency-free ancestry diagrams for proposals and physical births."""

import contextlib
from dataclasses import asdict, dataclass, replace
import html
import json
from pathlib import Path
import re
import sqlite3

from .bridge import AdapterError
from .evolution import Archive, category_descriptors, fitness, is_qualified


STATES = {
    "pending": ("Pending candidate", "#9dafc8", "rounded"),
    "generated": ("Produced candidates", "#80baff", "parallelogram"),
    "user-parent": ("User-contributed parent (unmeasured)", "#ce9bfa", "rectangle"),
    "running": ("Running", "#63deef", "circle"),
    "qualified": ("Qualified result", "#83ddac", "circle"),
    "unqualified": ("Unqualified result", "#e9bd78", "hexagon"),
    "invalidated": ("Invalidated simulation evidence", "#a0b1c9", "octagon"),
    "rejected": ("Cheap rejection", "#edb184", "triangle"),
    "duplicate": ("Duplicate", "#ce9bfa", "diamond"),
    "failed": ("Failed", "#fa9aaa", "square"),
    "interrupted": ("Interrupted", "#a0b1c9", "octagon"),
}

STYLE = """
.proposal-forest .forest-scroll{overflow:auto;max-height:650px;border:1px solid #31445d;border-radius:10px;cursor:var(--forest-cursor,zoom-out)}
.proposal-forest svg.forest-svg{width:auto;max-width:none;height:auto;max-height:none;display:block}
.forest-svg a{cursor:pointer}
.forest-svg text{fill:#e2eaf7;font:12px system-ui,sans-serif}
.forest-svg .forest-operation{fill:#b7c6da;font-size:10px;paint-order:stroke;stroke:#182333;stroke-width:4px;stroke-linejoin:round}
.forest-svg .forest-leg-count{font-size:9px}
.forest-svg .forest-badge{fill:#182333;stroke:#e9bd78;stroke-width:1.5}
.forest-svg .forest-badge-icon{fill:none;stroke:#e9bd78;stroke-width:1.5;stroke-linejoin:round}
.forest-svg a[data-champion-link]:hover .forest-badge,.forest-svg a[data-champion-link]:focus .forest-badge{fill:#4b3922;stroke-width:3}
.forest-svg .forest-edge{fill:none;stroke:#7087a5;stroke-width:1.5}
.proposal-forest .forest-node-shape{fill:#182333;stroke:var(--node-color);stroke-width:2.5}
.forest-svg a:hover .forest-node-shape,.forest-svg a:focus .forest-node-shape{fill:#31445d;stroke-width:4}
.forest-svg a:focus{outline:none}
.forest-svg .forest-running-ring{fill:none;stroke:#63deef;stroke-width:2;stroke-dasharray:4 4;animation:forest-pulse 1.2s ease-in-out infinite alternate}
.forest-legend{display:flex;flex-wrap:wrap;gap:8px 18px;margin:16px 0}
.forest-legend span{display:inline-flex;align-items:center;gap:8px;font-size:12px}
.forest-legend svg{width:26px;height:26px;max-height:none}
.forest-detail{scroll-margin-top:20px}
@keyframes forest-pulse{to{opacity:.35}}
@media(prefers-reduced-motion:reduce){.forest-svg .forest-running-ring{animation:none}}
.forest-explorer .forest-toolbar,.forest-explorer .forest-actions{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.forest-explorer .forest-map{overflow:auto;max-height:620px;border:1px solid #48607f;border-radius:10px;margin:12px 0}
.forest-explorer svg{display:block;width:auto;max-width:none;max-height:none}
.forest-explorer .fold-node{cursor:pointer}
.forest-explorer .fold-node rect{fill:#21324b;stroke:#80baff;stroke-width:2}
.forest-explorer .fold-node[data-selected="true"] rect{stroke:#ffd166;stroke-width:3}
.forest-explorer .fold-blob{fill:#293044;stroke:#a8a0cc;stroke-dasharray:6 4}
.forest-explorer text{fill:#e2eaf7;font:12px system-ui,sans-serif}
.forest-explorer .fold-edge{fill:none;stroke:#7087a5;stroke-width:1.5}
.forest-explorer .fold-port{cursor:pointer;fill:#80baff;stroke:#182333;stroke-width:2}
.forest-explorer .fold-port[data-highlight="true"]{fill:#ffd166;stroke:#ffd166}
.forest-explorer [role="button"]:focus{outline:none;stroke:#ffd166;stroke-width:3}
.forest-explorer .forest-inspector{border:1px solid #48607f;border-radius:10px;padding:12px}
.forest-explorer .fold-preview-caption{fill:#b7c6da;font-size:10px}
.forest-explorer .forest-inspector figure{max-width:420px;margin:12px 0}
.forest-explorer .forest-inspector img{width:100%;max-height:260px;object-fit:contain}
"""


def esc(value):
    return html.escape(str(value), quote=True)


@dataclass(frozen=True)
class Node:
    key: str
    parents: tuple[str, ...]
    label: str
    operation: str
    state: str
    href: str
    detail: str
    reused: bool = False
    legs: int | None = None
    representative: str | None = None
    category: str | None = None
    version: str | None = None
    contribution: bool = False
    population: bool = False
    contribution_id: str | None = None
    carry_from: str | None = None
    contribution_name: str | None = None
    contribution_href: str | None = None
    preview: str | None = None
    preview_caption: str | None = None


class _PopulationView:
    """Use the ordinary retention policy without opening or migrating an archive."""

    outcomes = Archive.outcomes
    founder = Archive.founder
    protocol = Archive.protocol

    def __init__(self, db, directory):
        self.db = db
        self.directory = directory


def _leg_count(value):
    if value is not None and (type(value) is not int or value < 0):
        raise AdapterError(f"Forest leg count must be a nonnegative integer, got {value!r}.")
    return value


def _state(status, result=None):
    if status == "complete":
        if result is None:
            raise AdapterError("Completed forest observation has no result.")
        return "qualified" if is_qualified(json.loads(result)) else "unqualified"
    if status in ("running", "failed", "interrupted", "duplicate"):
        return status
    if status == "rejected" or status.endswith("-rejected"):
        return "rejected"
    return "pending"


def nodes(db, prefix=""):
    from .evolution import evidence_invalidation
    from .gallery import display_result, media_directory
    from .screening_reports import rejections

    database_path = db.execute("PRAGMA database_list").fetchone()[2]
    screening = rejections(Path(database_path).parent) if database_path else {}

    births = [dict(row) for row in db.execute(
        "SELECT b.*,d.category FROM births b JOIN designs d ON d.id=b.design ORDER BY b.id")]
    protocol = json.loads(db.execute(
        "SELECT p.definition FROM protocols p JOIN run r ON r.protocol=p.id").fetchone()[0])
    population = set()
    if protocol.get("nativeDiscovery", {}).get("population"):
        from .population import members
        population = {row["birth"] for row in members(_PopulationView(db, Path(database_path).parent))}
    leg_counts = {kind: feet for kind, feet, _, _ in category_descriptors(protocol)}
    observations = list(db.execute("""
        SELECT e.birth,e.result,d.category,d.genome FROM evaluations e
        JOIN births b ON b.id=e.birth JOIN designs d ON d.id=b.design
        WHERE e.status='complete' ORDER BY e.id
    """))
    observations += list(db.execute("""
        SELECT r.birth,r.result,d.category,d.genome FROM reused_evaluations r
        JOIN births b ON b.id=r.birth JOIN designs d ON d.id=b.design ORDER BY r.birth
    """))
    best = {}
    for row in observations:
        measured = display_result(dict(row), protocol)
        if measured.get("evidenceInvalidation"):
            continue
        if row["category"] not in best or fitness(measured) > fitness(best[row["category"]][1]):
            best[row["category"]] = (row["birth"], measured)
    representatives = {birth: "champion" if is_qualified(measured) else "prototype"
                       for birth, measured in best.values()}
    proposals = [dict(row) for row in db.execute("SELECT * FROM native_proposals ORDER BY id")]
    trials = {row["birth"]: dict(row) for row in db.execute("SELECT * FROM evaluations")}
    reused = {row["birth"]: dict(row) for row in db.execute("SELECT * FROM reused_evaluations")}
    proposal_ids = {row["id"] for row in proposals}
    origins = {}
    for row in db.execute("SELECT birth,batch FROM native_candidates"):
        match = re.fullmatch(r"proposals/(\d+)", row["batch"])
        if match and int(match[1]) in proposal_ids:
            origins[row["birth"]] = int(match[1])
    contributions = {}
    if db.execute("SELECT 1 FROM sqlite_master WHERE name='user_pool_entries'").fetchone():
        contributions = {row["contribution"]: json.loads(row["provenance"])
                         for row in db.execute("SELECT contribution,provenance FROM user_pool_entries")}
    result = []
    for birth in births:
        if birth["category"] not in leg_counts:
            raise AdapterError(f"Forest birth has an unknown leg-count category: {birth['category']}.")
        legs = _leg_count(leg_counts[birth["category"]])
        trial = trials.get(birth["id"]) or reused.get(birth["id"])
        status = trial.get("status", "complete") if trial else birth["status"]
        state = _state(status, trial["result"] if trial else None)
        invalidation = evidence_invalidation(protocol)
        if invalidation and (trial or birth["status"] == "simulation-rejected"):
            state = "invalidated"
        if birth["id"] in origins:
            parents = (f'p{origins[birth["id"]]}',)
        else:
            parents = (f'b{birth["parent"]}',) if birth["parent"] is not None else ()
        detail = f'Birth {birth["id"]}: {legs} legs; {birth["operation"]}; {status}.'
        if state == "invalidated":
            detail += " " + invalidation
        if trial and birth["id"] in trials:
            detail += f' Trial {trial["id"]}.'
        if trial and trial.get("error"):
            detail += " " + trial["error"]
        if birth["error"]:
            info = screening.get(birth["design"]) if birth["status"] == "screening-rejected" else None
            detail += " " + (info["detail"] if info else birth["error"])
        if birth["id"] in reused:
            detail += " Reused observation; no new trial."
        contribution_id = None
        if birth["operation"] == "user-contribution":
            for change in json.loads(birth["changes"]):
                if change.get("path") != "user-contribution":
                    continue
                identifiers = change.get("after")
                if (not isinstance(identifiers, list) or len(identifiers) != 1
                        or not isinstance(identifiers[0], str)
                        or not re.fullmatch("[a-f0-9]{32}", identifiers[0])):
                    raise AdapterError(f"Birth {birth['id']} has invalid contribution provenance.")
                contribution_id = identifiers[0]
        contribution = contributions.get(contribution_id, {})
        contribution_name = contribution.get("label")
        contribution_node = contribution.get("node")
        if (contribution_name is not None and not isinstance(contribution_name, str)
                or contribution_node is not None and (
                    not isinstance(contribution_node, str) or not re.fullmatch("[a-f0-9]{32}", contribution_node))):
            raise AdapterError(f"Birth {birth['id']} has invalid saved contribution metadata.")
        preview, preview_caption = None, None
        if database_path:
            directory = Path(database_path).parent
            design = directory / "designs" / birth["design"]
            image = media_directory(design) / "walker-v2.png"
            if image.is_file():
                preview = prefix + image.relative_to(directory).as_posix()
                preview_caption = "Saved LEGO pose; not qualification."
            elif (design / "candidate/cycle.svg").is_file():
                preview = prefix + (design / "candidate/cycle.svg").relative_to(directory).as_posix()
                preview_caption = "Solved motion diagram; not live physics."
        result.append(Node(f'b{birth["id"]}', parents, f'B{birth["id"]}', birth["operation"], state,
                           f'{prefix}history/{birth["id"]:04d}.html', detail, birth["id"] in reused, legs,
                           representatives.get(birth["id"]), birth["category"],
                           contribution=birth["operation"] == "user-contribution",
                           population=birth["id"] in population, contribution_id=contribution_id,
                           contribution_name=contribution_name,
                           contribution_href=f"/workbench.html?node={contribution_node}" if contribution_node else None,
                           preview=preview, preview_caption=preview_caption))
    for proposal in proposals:
        summary = json.loads(proposal["summary"]) if proposal["summary"] else {}
        children = [birth for birth in births if origins.get(birth["id"]) == proposal["id"]]
        status = proposal["status"]
        if status == "generated" and children and all(child["status"] == "duplicate" for child in children):
            status = "duplicate"
        operation = summary.get("operation", "mutation attempt")
        legs = _leg_count(summary.get("footCount"))
        detail = f'Proposal {proposal["id"]}: {operation}; {status}. No simulation trial is implied.'
        detail += f" Proposed leg count: {legs}." if legs is not None else " Leg count not yet recorded."
        if proposal["error"]:
            detail += " " + proposal["error"]
        result.append(Node(f'p{proposal["id"]}', (f'b{proposal["parent"]}',), f'P{proposal["id"]}',
                           operation, "generated" if status == "generated" else _state(status),
                           f'{prefix}forest.html#proposal-{proposal["id"]}', detail, legs=legs))
    return result


@contextlib.contextmanager
def _database(directory):
    path = (Path(directory) / "archive.sqlite3").resolve()
    with contextlib.closing(sqlite3.connect(f"{path.as_uri()}?mode=ro", uri=True)) as db:
        db.row_factory = sqlite3.Row
        db.execute("BEGIN")
        yield db


def _version_key(version, key):
    return f"{key[0]}{version}:{key[1:]}"


def campaign_nodes(versions, active):
    registered = {Path(version["archive"]).resolve(): version["id"] for version in versions}
    result = []
    for version in versions:
        identifier = version["id"]
        directory = Path(version["archive"])
        with _database(directory) as db:
            items = nodes(db, f"/versions/{identifier}/")
            roots = {row["design"]: f'b{row["id"]}' for row in db.execute(
                "SELECT id,design FROM births WHERE parent IS NULL")}
        carried = {}
        metadata = directory / "upgrade.json"
        if metadata.exists():
            provenance = json.loads(metadata.read_text())
            for parent in provenance["parents"]:
                source = Path(parent.get("sourceArchive", provenance["sourceArchive"])).resolve()
                if source not in registered:
                    raise AdapterError(f"Forest carry-forward source is not registered: {source}.")
                design = parent.get("targetDesign", parent["sourceDesign"])
                if design not in roots:
                    raise AdapterError(f"Forest carry-forward design has no root in {identifier}.")
                carried[roots[design]] = (
                    registered[source], f'b{parent["sourceBirth"]}')
        for node in items:
            parents = tuple(_version_key(identifier, parent) for parent in node.parents)
            operation, detail = node.operation, f"{identifier}: {node.detail}"
            if node.key in carried:
                source_version, source_birth = carried[node.key]
                parents = (_version_key(source_version, source_birth),)
                operation = "carry forward"
                detail += (f" Carried forward from {source_version} {source_birth.upper()}; "
                           "version update, not a new mutation or trial.")
            result.append(replace(node, key=_version_key(identifier, node.key), parents=parents,
                                  operation=operation, detail=detail, version=identifier,
                                  representative=node.representative if identifier == active else None,
                                  population=node.population and identifier == active,
                                  carry_from=parents[0] if node.key in carried else None))
    return result


def layout(items):
    """Layer a DAG; shared children keep all incoming parents rather than being copied."""
    remaining = {node.key: node for node in items}
    if len(remaining) != len(items):
        raise AdapterError("Forest node identities must be unique.")
    ranks, positions = {}, {}
    while remaining:
        ready = [node for node in remaining.values() if all(parent in ranks for parent in node.parents)]
        if not ready:
            raise AdapterError("Forest ancestry contains a cycle or missing parent.")
        for node in ready:
            ranks[node.key] = max((ranks[parent] + 1 for parent in node.parents), default=0)
            del remaining[node.key]
    for rank in range(max(ranks.values(), default=-1) + 1):
        layer = [node for node in items if ranks[node.key] == rank]
        layer.sort(key=lambda node: (
            sum(positions[parent][1] for parent in node.parents) / len(node.parents) if node.parents else 0,
            node.key[0], int(node.key[1:]) if node.key[1:].isdigit() else 0, node.key,
        ))
        for index, node in enumerate(layer):
            ideal = sum(positions[parent][1] for parent in node.parents) / len(node.parents) if node.parents else 60
            previous = positions[layer[index - 1].key][1] + 85 if index else 60
            positions[node.key] = (65 + rank * 285, max(ideal, previous))
    return positions


def shape(state):
    kind = STATES[state][2]
    if kind == "circle":
        return '<circle class="forest-node-shape" r="20"/>'
    if kind == "diamond":
        return '<path class="forest-node-shape" d="M0 -23 23 0 0 23 -23 0Z"/>'
    if kind == "triangle":
        return '<path class="forest-node-shape" d="M0 -24 24 20 -24 20Z"/>'
    if kind == "hexagon":
        return '<path class="forest-node-shape" d="M-22 0 -12 -20 12 -20 22 0 12 20 -12 20Z"/>'
    if kind == "parallelogram":
        return '<path class="forest-node-shape" d="M-15 -20 24 -20 15 20 -24 20Z"/>'
    if kind == "octagon":
        return '<path class="forest-node-shape" d="M-10 -22 10 -22 22 -10 22 10 10 22 -10 22 -22 10 -22 -10Z"/>'
    return f'<rect class="forest-node-shape" x="-20" y="-20" width="40" height="40" rx="{8 if kind == "rounded" else 0}"/>'


def diagram(items, tile_prefix=""):
    if not items:
        return '<p class="empty">No proposals or births recorded yet.</p>'
    positions = layout(items)
    width = max(x for x, _ in positions.values()) + 110
    height = max(y for _, y in positions.values()) + 65
    edges, vertices = [], []
    for node in items:
        x, y = positions[node.key]
        if node.parents:
            join = x - 180
            for parent in node.parents:
                px, py = positions[parent]
                middle = (px + 24 + join) / 2
                edges.append(f'<path class="forest-edge" data-parent="{esc(parent)}" data-child="{esc(node.key)}" '
                             f'd="M{px+24:g} {py:g} C{middle:g} {py:g} {middle:g} {y:g} {join:g} {y:g}"/>')
            edges.append(f'<path class="forest-edge" d="M{join:g} {y:g} H{x-26:g}" marker-end="url(#forest-arrow)"/>')
            operation = ("realize" if node.key.startswith("b") and all(parent.startswith("p") for parent in node.parents)
                         else node.operation)
            label = operation if len(operation) <= 28 else operation[:25] + "..."
            edges.append(f'<text class="forest-operation" data-child="{esc(node.key)}" text-anchor="middle" '
                         f'x="{(join+x-26)/2:g}" y="{y-7:g}"><title>{esc(operation)}</title>{esc(label)}</text>')
        state_label, color, _ = STATES[node.state]
        title = f"{node.detail} Outcome: {state_label}."
        ring = '<circle class="forest-running-ring" r="27"/>' if node.state == "running" else ""
        vertices.append(f'<a href="{esc(node.href)}" data-live-key="forest-{esc(node.key)}" '
                        f'data-state="{node.state}" tabindex="0" aria-label="{esc(title)}" '
                        f'transform="translate({x:g} {y:g})" style="--node-color:{color}">'
                        f'<title>{esc(title)}</title>{ring}{shape(node.state)}'
                        + (f'<text class="forest-leg-count" text-anchor="middle" y="-34">{esc(node.version)}</text>'
                           if node.version is not None else "")
                        + f'<text text-anchor="middle" y="-3">{esc(node.label)}</text>'
                        f'<text class="forest-leg-count" text-anchor="middle" y="11">'
                        f'{"?" if node.legs is None else node.legs} {"leg" if node.legs == 1 else "legs"}</text>'
                        + ('<text x="29" y="-16">R</text>' if node.reused else "") + '</a>')
        if node.representative is not None:
            qualified = node.representative == "champion"
            label = "Qualified champion" if qualified else "Best unqualified prototype"
            icon = ('<path class="forest-badge-icon" d="M-5 -7H5V-2Q5 3 0 4Q-5 3 -5 -2Z'
                    'M-5 -5H-8V-2Q-8 1 -4 1M5 -5H8V-2Q8 1 4 1M0 4V8M-4 8H4"/>'
                    if qualified else '<path class="forest-badge-icon" d="M-5 8V-8H6L3 -3H-5"/>')
            vertices.append(f'<a data-live-key="forest-badge-{esc(node.key)}" data-champion-link="{node.representative}" '
                            f'href="{esc(tile_prefix)}index.html#category-{esc(node.category)}" tabindex="0" '
                            f'aria-label="{label}: show {node.legs} leg category tile" '
                            f'transform="translate({x+36:g} {y+19:g})">'
                            f'<title>{label}; click to open and highlight its category tile.</title>'
                            f'<circle class="forest-badge" r="12"/>{icon}</a>')
    return (f'<div class="forest-scroll" tabindex="0" role="region" '
            'aria-label="Scrollable proposal ancestry; click background or press Enter or Space to toggle zoom">'
            f'<svg class="forest-svg" xmlns="http://www.w3.org/2000/svg" width="{width:g}" height="{height:g}" '
            f'style="width:calc({width:g}px * var(--forest-scale,1));height:calc({height:g}px * var(--forest-scale,1))" '
            f'viewBox="0 0 {width:g} {height:g}" role="group" aria-label="Proposal ancestry forest">'
            '<defs><marker id="forest-arrow" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0 10 5 0 10Z" fill="#7087a5"/></marker></defs>'
            + "".join(edges + vertices) + "</svg></div>")


def panel(db, prefix="", *, tile_prefix=None):
    return _panel(nodes(db, prefix), prefix, prefix if tile_prefix is None else tile_prefix, "this version")


def _panel(items, prefix, tile_prefix, scope):
    legend = "".join(f'<span><svg viewBox="-30 -30 60 60" aria-hidden="true" style="--node-color:{color}">'
                     f'{shape(state)}</svg>{label}</span>' for state, (label, color, _) in STATES.items())
    versions = ('<p>Version labels distinguish repeated birth and proposal numbers. '
                '&ldquo;carry forward&rdquo; arrows connect saved update provenance, not new mutations or trials. '
                'Outcomes retain each version\'s rules; only current-version representatives receive badges.</p>'
                if any(node.version is not None for node in items) else "")
    # Include the styling locally so frozen worker pages can display the current read-only view.
    model = json.dumps([asdict(node) for node in items], separators=(",", ":")).replace("<", "\\u003c")
    return ('<!-- proposal-forest:start --><section class="panel proposal-forest" id="proposal-forest">'
            f'<style>{STYLE}</style><h2>Proposal ancestry forest</h2>'
            '<div class="forest-explorer" data-live-key="forest-explorer">'
            '<p>Compact ancestry loads here. The full forest below is also available without JavaScript.</p></div>'
            f'<script type="application/json" class="forest-data">{model}</script>'
            '<details class="forest-full" data-live-key="forest-full"><summary>Full forest (all nodes)</summary>'
            '<p>B = physical birth; P = mutation attempt. Arrows point toward offspring. '
            'Operation labels sit on the arrows. B nodes show physical leg counts; P nodes show proposed counts '
            '(&ldquo;?&rdquo; if not yet recorded). '
            'An attempt may produce several candidates, or be rejected before any robot is born. '
            'R marks a reused measurement, not a new trial. A trophy badge marks a qualified champion; '
            'a flag marks the best unqualified prototype where no champion exists. '
            'Click a badge to open and highlight its category tile. Hover or focus for outcome details; '
            'click a birth for its robot and ancestry, or an attempt for diagnostics.</p>'
            '<p>Click empty space in the view to switch between detail (100%) and overview (40%). '
            'Node and badge links stay clickable. Keyboard: focus the view and press Enter or Space.</p>'
            f'{versions}<p>{len(items)} nodes in {scope}. '
            f'<a href="{prefix}forest.html">Open full forest and diagnostics</a></p>'
            f'<div class="forest-legend">{legend}</div>{diagram(items, tile_prefix)}'
            '</details></section><!-- proposal-forest:end -->')


def diagnostics(db, prefix=""):
    records = []
    for row in db.execute("SELECT * FROM native_proposals ORDER BY id"):
        summary = json.loads(row["summary"]) if row["summary"] else {}
        records.append(f'<article class="panel forest-detail" id="proposal-{row["id"]}">'
                       f'<h2>Proposal {row["id"]}: {esc(summary.get("operation", "mutation attempt"))}</h2>'
                       f'<p>{esc(row["status"])}; parent <a href="{prefix}history/{row["parent"]:04d}.html">'
                       f'birth {row["parent"]}</a>; seed {row["seed"]}; attempt {row["attempt"]}.</p>'
                       + (f'<p class="bad">{esc(row["error"])}</p>' if row["error"] else "")
                       + f'<pre>{esc(json.dumps(summary, indent=2))}</pre></article>')
    return "".join(records)


def snapshot(directory, prefix="", *, details=False, tile_prefix=None):
    with _database(directory) as db:
        return panel(db, prefix, tile_prefix=tile_prefix) + (diagnostics(db, prefix) if details else "")


def campaign_snapshot(versions, active):
    return _panel(campaign_nodes(versions, active), "", "/",
                  f"this experiment across {len(versions)} versions")


def insert(content, forest):
    start, end = "<!-- proposal-forest:start -->", "<!-- proposal-forest:end -->"
    if start in content:
        before, _, rest = content.partition(start)
        _, found, after = rest.partition(end)
        if not found:
            raise AdapterError("Dashboard contains an incomplete proposal forest.")
        return before + forest + after
    anchor = '<section aria-label="Current-version results">'
    if anchor in content:
        return content.replace(anchor, forest + anchor, 1)
    anchor = '<main id="dashboard-content">'
    if anchor in content:
        return content.replace(anchor, anchor + forest, 1)
    return content + forest
