"""Dependency-free, local-file-friendly champion and birth-lineage pages."""

import base64
import html
import json
import os
import re
import sqlite3
import uuid
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import quote

from .cycles import MAX_DURATION, REQUIRED_CYCLES
from .evolution import ROOT, category_descriptors, fitness, foot_count_categories, is_qualified
from .proposal_forest import diagnostics, panel

COLORS = ["#80baff", "#83ddac", "#e9bd78", "#ce9bfa", "#fa9aaa", "#73dbe4",
          "#b5c9ff", "#bddb75", "#ff9d65", "#f4a9e3", "#fccebc", "#65c0c9"]

BIRTH_GENES_STYLE = (
    ".birth-genes>summary{cursor:pointer}"
    ".birth-genes>summary>h2,.birth-genes>summary>h3{display:inline;margin:0}"
)

BIRTH_VISUALS_STYLE = (
    ".birth-visuals{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,360px),1fr));gap:20px}"
    ".birth-visuals figure{margin:0}"
    ".birth-visuals img{aspect-ratio:8/5;max-height:500px;object-fit:contain}"
)

MOTION_STYLE = """
.motion-player canvas{display:block;width:100%;height:300px;background:#182333;border-radius:10px}
.motion-views{display:grid;grid-template-columns:minmax(0,1fr);gap:10px}
.motion-views figure{margin:0;min-width:0}
.motion-views figcaption{font-size:12px;margin:5px 0}
.pipeline-visuals .motion-player canvas{height:200px}
.motion-controls{display:flex;align-items:center;gap:8px;margin:8px 0;flex-wrap:wrap}
.motion-controls button{padding:5px 10px;font-size:13px}
.motion-controls input{flex:1;min-width:70px;width:70px}
.motion-controls output,.motion-player>a{font-size:12px}
.motion-controls select{font-size:12px;max-width:160px}
.motion-depth-note{font-size:12px;margin:6px 0}
.motion-legend{display:flex;flex-wrap:wrap;gap:4px 12px;font-size:12px;margin:6px 0}
"""

CSS = """
:root{color-scheme:dark;font:15px/1.55 system-ui,sans-serif;background:#101723;color:#e2eaf7}
body{max-width:1480px;margin:auto;padding:32px}a{color:#91c7ff}h1{font-size:clamp(30px,4vw,50px);line-height:1.1;margin:12px 0}
h2{margin-top:32px}h3{margin:6px 0}p{color:#b7c6da}.eyebrow{color:#72bbff;letter-spacing:.15em;text-transform:uppercase;font-size:12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,390px),1fr));gap:20px}
.card,.panel{background:#182333;border:1px solid #31445d;border-radius:16px;padding:22px}.panel{margin:20px 0}
.pill{display:inline-block;border-radius:20px;padding:4px 11px;background:#293a52;font-size:12px;margin:3px 3px 3px 0}
.good{background:#153e35;color:#94e9bf}.warn{background:#4b3922;color:#ffd393}.bad{color:#ffb3a8}
.card:target{outline:3px solid #e9bd78;outline-offset:4px;scroll-margin:24px}
.stats{display:flex;gap:30px;flex-wrap:wrap}.stat strong{font-size:27px;display:block}.stat small{color:#b7c6da}
img,video{width:100%;border-radius:10px;background:#21324b}video{margin-top:10px}.empty{min-height:150px;display:grid;place-items:center;color:#9dafc8;border:1px dashed #48607f;border-radius:10px}
dl{display:grid;grid-template-columns:1fr 1fr;gap:5px;margin:14px 0}dt{color:#b7c6da}dd{text-align:right;margin:0}
pre{overflow:auto;background:#101723;padding:15px;border-radius:8px;font-size:12px}table{width:100%;border-collapse:collapse;font-size:13px}p{overflow-wrap:anywhere}
th,td{text-align:left;padding:10px;border-bottom:1px solid #31445d;vertical-align:top}.scroll{overflow:auto}
svg{width:100%;height:auto;max-height:290px}details{margin:12px 0}li{margin:5px 0}.muted{color:#a0b1c9;font-size:13px}
button{background:#294d76;color:#e2eaf7;border:1px solid #72bbff;border-radius:8px;padding:10px 16px;font:inherit;cursor:pointer}button:disabled{opacity:.55;cursor:default}
.active-trial-gallery{grid-template-columns:repeat(auto-fit,minmax(min(100%,260px),1fr));list-style:none;padding:0;margin:16px 0}
.active-trial-card{border:1px solid #48607f;border-radius:12px;padding:16px;margin:0}
.active-trial-card h4{margin:0 0 8px}
.active-trial-card img{aspect-ratio:4/3;object-fit:contain}
.pipeline-summary{display:flex;flex-wrap:wrap;gap:6px}
.pipeline-summary a{text-decoration:none}
.pipeline-board{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,190px),1fr));gap:10px;align-items:start}
.pipeline-lane{border:1px solid #48607f;border-radius:10px;background:#101723;min-width:0;padding:8px;scroll-margin-top:90px}
.pipeline-lane h5{font-size:13px;margin:0 0 8px}
.pipeline-lane ul{list-style:none;padding:0;margin:0;max-height:320px;overflow:auto}
.pipeline-lane li{margin:3px 0}
.pipeline-mini{display:flex;gap:8px;align-items:center;padding:3px;border-radius:5px;font-size:12px;line-height:1.3;text-decoration:none}
.pipeline-mini:hover,.pipeline-mini:focus{background:#293a52;outline:1px solid #91c7ff}
.pipeline-mini-picture{position:relative;flex:0 0 64px;width:64px}
.pipeline-mini img,.pipeline-mini-placeholder{display:block;width:64px;height:44px;object-fit:contain;border-radius:4px}
.pipeline-mini img[data-preview-kind="assembly"]{filter:brightness(1.12) contrast(1.35) saturate(1.1)}
.pipeline-mini-placeholder{display:grid;place-items:center;background:#21324b;color:#b7c6da;font-size:9px}
.pipeline-mini-label{overflow-wrap:anywhere;min-width:0}
.pipeline-outcome{position:absolute;right:-3px;bottom:-3px;display:grid;place-items:center;width:18px;height:18px;border:2px solid #101723;border-radius:50%;font:bold 14px/1 system-ui,sans-serif;background:#36445b;color:#fff}
.pipeline-outcome[data-outcome="qualified"]{background:#176348;color:#c5ffe1}
.pipeline-outcome[data-outcome="rejected"],.pipeline-outcome[data-outcome="failed"]{background:#8c302f;color:#fff0eb}
.pipeline-outcome[data-outcome="unqualified"]{background:#775013;color:#fff2c2}
.pipeline-lane:target,.active-trial-card:target{outline:2px solid #e9bd78;outline-offset:3px}
.pipeline-board-note{font-size:12px}
.pipeline-stage>h4{margin:18px 0 6px}
.pipeline-visuals{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,200px),1fr));gap:12px}
.pipeline-visuals figure{margin:0}
.pipeline-visuals img{aspect-ratio:8/5;max-height:240px;object-fit:contain}
.pipeline-visuals figcaption{font-size:12px;color:#a0b1c9}
.construction-projections{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,240px),1fr));gap:12px}
.construction-projections figure{margin:0}
.construction-projections svg{width:100%;height:260px;background:#101723}
#dashboard-navigation{position:sticky;top:0;z-index:10;display:flex;gap:8px;overflow-x:auto;
background:#101723;border:1px solid #48607f;border-radius:10px;padding:10px;align-items:center}
#dashboard-navigation a{white-space:nowrap;padding:4px 8px}
html{scroll-padding-top:80px}
.motif-example{margin:20px 0}.motif-diagram{display:block;min-width:720px}
""" + BIRTH_GENES_STYLE + BIRTH_VISUALS_STYLE + MOTION_STYLE


def esc(value):
    return html.escape(str(value), quote=True)


def display_result(row, protocol):
    from .evolution import trusted_result
    result = json.loads(row["result"])
    if protocol.get("grammarVersion") != "technic-fourbar-v5" and json.loads(row["genome"]).get("towWagon"):
        result["metrics"]["eligibleForRanking"] = False
        reasons = result["metrics"]["rankingRejectionReasons"]
        if "rigid-wagon-hitch" not in reasons:
            reasons.append("rigid-wagon-hitch")
    return trusted_result(result, protocol)


def relative(path, page):
    return quote(os.path.relpath(path, page.parent).replace(os.sep, "/"), safe="/")


def atomic_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)

def motion_script():
    return ('<script data-motion-player-script>'
            + Path(__file__).with_name("motion_diagrams.js").read_text(encoding="utf-8") + "</script>")


def motion_assets(text):
    style = f"<style data-motion-player-style>{MOTION_STYLE}</style>"
    if "<script data-motion-player-script" in text:
        text = re.sub(r"<script data-motion-player-script>.*?</script>",
                      lambda _: motion_script(), text, flags=re.DOTALL)
        if "<style data-motion-player-style>" in text:
            return re.sub(r"<style data-motion-player-style>.*?</style>",
                          lambda _: style, text, flags=re.DOTALL)
        return text.replace("<script data-motion-player-script>", style + "<script data-motion-player-script>", 1)
    assets = style + motion_script()
    return text.replace("</body>", assets + "</body>", 1) if "</body>" in text else text + assets


def document(title, body, *, dashboard=False):
    controls = """<section class="panel" id="live-controls" aria-label="Episode controls">
<button id="prepare-upgrade" type="button" hidden disabled>Prepare discovery/cycle upgrade</button>
<button id="start-episode" type="button" hidden disabled>Run next episode (12 trials)</button>
<button id="pause-episode" type="button" hidden disabled>Pause episode</button>
<p id="worker-status" role="status">Static snapshot. <a href="http://127.0.0.1:8765/">Open live dashboard</a> for in-place updates and episode controls.</p>
<p id="worker-error" class="bad" role="alert"></p><a id="worker-log" hidden>Worker log</a>
<p id="upgrade-status" role="status"></p>
<h3>Candidate pipeline</h3>
<p class="muted">Initial LEGO assembly previews and solved-motion diagrams are shown together.
Neither is live physics footage. Preview rendering runs automatically in a separate, single-worker queue.</p>
<p id="assembly-preview-status" role="status">Assembly previews connect with the live dashboard.</p>
<p id="assembly-preview-error" class="bad" role="alert"></p>
<a id="assembly-preview-log" hidden>Preview worker log</a>
<button id="retry-assembly-previews" type="button" hidden>Retry failed previews</button>
<div id="candidate-pipeline"><p class="muted">Pipeline stages appear when connected to the live dashboard.</p></div>
<p id="runtime-status"></p><p><a href="/mutation-gallery.html">Illustrated mutation gallery</a> &middot;
<a href="/motif-gallery.html">Construction motif gallery</a> &middot;
<a href="/motif-opportunities.html">Motif opportunities</a></p>
<p><a id="experiment-history" href="/history.html" hidden>Updates &amp; history</a></p>
<p class="muted">Updates preserve earlier results and never authorize simulations.
The stable experiment dashboard carries unused allowance forward and keeps earlier versions read-only.</p></section>""" if dashboard else ""
    if dashboard:
        controls += """<section class="panel" id="qualified-walkers" aria-label="Recently qualified walkers">
<h2>Recently qualified walkers</h2>
<div id="qualified-walkers-content"><p>Recorded qualifications connect with the live dashboard.</p></div>
</section>
<section class="panel" id="animation-rendering" aria-label="Animation rendering">
<h3>Animation rendering</h3>
<button id="start-rendering" type="button" hidden disabled>Start rendering</button>
<button id="stop-rendering" type="button" hidden disabled>Stop rendering</button>
<p id="render-status" role="status">Static snapshot. Rendering controls require the live dashboard.</p>
<p class="muted">Uses saved poses only. No simulation trials are spent. Stop cancels the current render; start resumes unfinished work.</p>
<p id="render-error" class="bad" role="alert"></p><a id="render-log" hidden>Rendering log</a>
<div id="recent-renders" hidden><h4>Recently rendered</h4><ul id="render-links"></ul></div></section>"""
    script = ("<script>" + Path(__file__).with_name("dashboard.js").read_text(encoding="utf-8") + "</script>"
              if dashboard or 'class="forest-scroll"' in body else "")
    script = motion_script() + ('<script data-forest-explorer-script>'
        + Path(__file__).with_name("forest_explorer.js").read_text(encoding="utf-8") + "</script>") + script
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>{esc(title)}</title>
<style>{CSS}</style></head><body>{'<nav id="dashboard-navigation" aria-label="Dashboard sections"></nav>' if dashboard else ''}{controls}<main id="dashboard-content">{body}</main>{script}</body></html>"""


def media_directory(design):
    rendered = design / "render"
    return rendered if (rendered / "walker-v2.png").exists() else design / "assembly-render"


def motif_gallery():
    data = json.loads((ROOT / "tests/visualizations/motifs/catalogue.json").read_text())
    builder = data["builder"]
    cards = []
    for motif in builder["catalogue"]:
        examples = [example for example in data["examples"] if example["motif"] == motif["id"]]
        if not examples:
            raise ValueError(f"No gallery witness for motif {motif['id']}.")
        cards.append(f'<section class="panel" id="{esc(motif["id"])}"><h2>{esc(motif["id"])}</h2>'
                     f'<p>Version {esc(motif["version"])} · {esc(motif["origin"])}</p>')
        for example in examples:
            image = f'/motif-illustrations/{esc(example["id"])}.svg'
            cards.append(f'<figure class="motif-example"><div class="scroll"><a href="{image}">'
                         f'<img class="motif-diagram" loading="lazy" src="{image}" '
                         f'alt="{esc(example["title"])}: matched-scale before and after"></a></div>'
                         f'<figcaption>{esc(example["caption"])} '
                         f'<a href="{image}">Open full-size diagram</a></figcaption></figure>')
        for key, title in (("parameters", "Parameters"), ("preconditions", "When it applies"),
                           ("preserves", "What stays unchanged"), ("limitations", "Limitations"),
                           ("evidence", "Regression evidence")):
            cards.append(f'<h3>{title}</h3><ul>'+''.join(f'<li>{esc(item)}</li>' for item in motif[key])+'</ul>')
        cards.append('</section>')
    return document("Construction motif gallery",
                    '<a href="/">Back to dashboard</a><h1>Construction motif gallery</h1>'
                    f'<p>{esc(builder["catalogueVersion"])} · {len(builder["catalogue"])} parameterized motifs</p>'
                    '<p>Mutations change a robot; construction motifs help realize its motion in LEGO. '
                    'These before/after examples are generated by the actual symbolic rules. '
                    'Gray is unchanged context; amber marks the edited part. Both sides use the same scale. '
                    'On narrow screens, scroll diagrams sideways or open them full-size.</p>'
                    '<p>Projected catalog ports and shaft intervals, not LEGO meshes, an assembly sequence or dynamic simulation. '
                    'The motion is unchanged, so these are construction close-ups rather than cycle animations. '
                    'Compiler-checked examples still require exact reconstruction and collision screening; '
                    'nothing here certifies walking, retention or strength.</p>'
                    '<p><a href="/motif-illustrations/catalogue.json">Catalogue, builder policy and witness identities (JSON)</a> · '
                    '<a href="/motif-opportunities.html">Failure patterns and motif opportunities</a></p>'
                    +''.join(cards))


def mutation_gallery():
    operations = ("move-hinge-site", "input-phase", "branch-phase", "add-dyad", "remove-dyad",
                  "duplicate-branch", "remove-branch", "lengthen-foot", "bend-foot", "gear-reduction",
                  "battery-layout", "receiver-layout", "motor-layout")
    cards = "".join(f'<section class="panel"><h2>{esc(operation)}</h2>'
                    f'<img src="/mutation-animations/mutation-{operation}.svg" alt="{esc(operation)}: parent and child"></section>'
                    for operation in operations)
    return document("Mutation illustrations",
                    '<a href="/">Back to dashboard</a><h1>Mutation illustrations</h1>'
                    '<p>Synchronized solved kinematics, not dynamic simulation or evidence of walking. '
                    'Parent on the left; mutated child on the right. Branch copying/removal changes logical foot count; '
                    'beam outlines and labeled gear sizes come from the physical genomes. '
                    'The reduction illustration spans five motor turns; other illustrations span one.</p>'+cards)


def media(design, page, embedded):
    directory = media_directory(design)
    image = directory / "walker-v2.png"
    video = directory / "walker-v2-anchored.mp4"
    if not image.exists():
        return '<div class="empty">Preview not generated; simulation records remain available.</div>'

    def source(path, mime):
        if embedded:
            return f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode()
        return relative(path, page)

    result = f'<img loading="lazy" alt="Generated LEGO design, initial assembly pose" src="{source(image, "image/png")}">'
    if video.exists():
        result += f'<video controls muted loop playsinline preload="none" poster="{source(image, "image/png")}"><source src="{source(video, "video/mp4")}" type="video/mp4"></video>'
    return result


def birth_preview(design, page, rendered=None):
    diagram = design / "candidate/cycle.svg"
    if rendered is None:
        rendered = media(design, page, True) if (media_directory(design) / "walker-v2.png").is_file() else ""
    figures = []
    if rendered:
        figures.append('<figure>' + rendered + '<figcaption class="muted">LEGO assembly render; '
                       'saved-pose animation when available.</figcaption></figure>')
    if diagram.is_file():
        url = relative(diagram, page)
        image = f'<a href="{url}"><img loading="lazy" src="{url}" alt="Solved motion diagram"></a>'
        if (design / "candidate/program.json").is_file():
            image = motion_player(url, image)
        figures.append('<figure data-motion-diagram>' + image
                       + '<figcaption class="muted">Solved kinematics; not the full LEGO assembly '
                       'or measured physics.</figcaption></figure>')
    return ('<section class="panel"><h2>Saved preview</h2><div class="birth-visuals">'
            + "".join(figures) + "</div></section>") if figures else ""

def motion_player(url, image):
    source = url.removesuffix("cycle.svg") + "motion.json"
    return f'<div class="motion-player" data-motion-source="{source}">{image}</div>'


def birth_previews(text, directory, relative_page):
    match = re.fullmatch(r"history/(\d+)\.html", relative_page)
    if match is None:
        return text
    directory = Path(directory).resolve()
    with sqlite3.connect(f"{(directory / 'archive.sqlite3').as_uri()}?mode=ro", uri=True) as db:
        row = db.execute("SELECT design FROM births WHERE id=?", (int(match[1]),)).fetchone()
    if row is None:
        return text
    design = directory / "designs" / row[0]
    if not (design / "candidate/cycle.svg").is_file():
        return text
    if "data-motion-diagram" in text:
        if 'data-motion-source="' in text or not (design / "candidate/program.json").is_file():
            return text
        return re.sub(r'(<figure data-motion-diagram>)(<a href="([^"]+)">.*?</a>)',
                      lambda match: match[1] + motion_player(match[3], match[2]), text, count=1, flags=re.DOTALL)
    heading = '<section class="panel"><h2>Saved preview</h2>'
    start = text.find(heading)
    if start >= 0:
        end = text.find("</section>", start)
        if end < 0:
            raise ValueError("Birth page contains an incomplete saved-preview section.")
        rendered = text[start + len(heading):end]
        preview = birth_preview(design, directory / relative_page, rendered)
        text = text[:start] + preview + text[end + len("</section>"):]
    else:
        preview = birth_preview(design, directory / relative_page)
        ancestry = '<section class="panel"><h2>Ancestry</h2>'
        if ancestry in text:
            text = text.replace(ancestry, preview + ancestry, 1)
        else:
            text = text.replace("</main>", preview + "</main>", 1)
    if BIRTH_VISUALS_STYLE not in text:
        text = text.replace("</head>", f"<style>{BIRTH_VISUALS_STYLE}</style></head>", 1)
    return text


def preview_first(text):
    preview = '<section class="panel"><h2>Saved preview</h2>'
    ancestry = '<section class="panel"><h2>Ancestry</h2>'
    start, anchor = text.find(preview), text.find(ancestry)
    if start < 0 or anchor < 0 or start < anchor:
        return text
    end = text.find("</section>", start)
    if end < 0:
        raise ValueError("Birth page contains an incomplete saved-preview section.")
    end += len("</section>")
    return text[:anchor] + text[start:end] + text[anchor:start] + text[end:]


def collapsible_genes(text):
    for level, title in (("h3", "Actual gene differences from parent"),
                         ("h2", "Canonical genome (provenance seed excluded)")):
        pattern = rf"(<{level}>{re.escape(title)}</{level}>)(\s*<pre>.*?</pre>)"
        text = re.sub(pattern, r'<details class="birth-genes"><summary>\1</summary>\2</details>',
                      text, flags=re.DOTALL)
    if 'class="birth-genes"' in text and BIRTH_GENES_STYLE not in text:
        text = text.replace("</head>", f"<style>{BIRTH_GENES_STYLE}</style></head>", 1)
    return text


def category_anchors(text):
    text = re.sub(r'<article class="card" data-live-key="([^"]+)">',
                  r'<article class="card" id="category-\1" tabindex="-1" data-live-key="\1">', text)
    if 'id="category-' in text and ".card:target" not in text:
        text = text.replace("</head>", '<style>.card:target{outline:3px solid #e9bd78;outline-offset:4px;'
                            'scroll-margin:24px}</style></head>', 1)
    return text


def links(archive, row, page):
    design = archive.directory / "designs" / row["design"]
    paths = [(design / "candidate/model.ldr", "LDraw"), (design / "candidate/manifest.json", "Manifest"),
             (design / "genome.json", "Genome")]
    if row.get("reused"):
        observation = archive.directory / "observations" / f"{row['birth']:04d}"
        paths += [(observation / "result.json", "Reused result"),
                  (observation / "provenance.json", "Original measurement provenance"),
                  (observation / "source-protocol.json", "Original evaluation protocol")]
    elif row.get("id") is not None or row.get("gateAttempt") is not None:
        evaluation = (archive.directory / "simulation-attempts" / f"{row['gateAttempt']:04d}"
                      if row.get("gateAttempt") is not None
                      else archive.directory / "evaluations" / f"{row['id']:04d}")
        if not evaluation.exists() and row.get("id") is not None and archive.db.execute(
                "SELECT 1 FROM sqlite_master WHERE name='simulation_attempts'").fetchone():
            attempt = archive.db.execute("SELECT id FROM simulation_attempts WHERE trial=?", (row["id"],)).fetchone()
            if attempt is not None:
                evaluation = archive.directory / "simulation-attempts" / f"{attempt['id']:04d}"
        paths += [(evaluation.with_suffix(".log"), "Simulation worker log")]
        paths += [(evaluation / "result.json", "Result"), (evaluation / "model.xml", "MuJoCo"),
                  (evaluation / "poses.json", "Recorded poses"),
                  (evaluation / "simulation-gate.json", "Short simulation gate"),
                  (evaluation / "trajectory.json", "Trajectory"),
                  (evaluation / "stance-support.json", "Measured stance support"),
                  (evaluation / "rejection.json", "Simulation rejection"),
                  (evaluation / "error.json", "Simulation error")]
    return " &middot; ".join(f'<a href="{relative(path, page)}">{label}</a>' for path, label in paths if path.exists())


def metrics(result):
    m = result["metrics"]
    body = f"""<dl><dt>Forward speed</dt><dd>{m['speedMetersPerSecond']*1000:+.3f} mm/s</dd>
<dt>Forward distance</dt><dd>{m['forwardMeters']*1000:+.3f} mm</dd>
<dt>Motor revolutions</dt><dd>{esc(', '.join(f'{turn:.3f}' for turn in m['motorTurns']))}</dd>
<dt>Simulated mass</dt><dd>{m['simulatedMassKg']*1000:.1f} g</dd>
<dt>Maximum tilt</dt><dd>{m['maxTiltDegrees']:.2f} degrees</dd>
<dt>Peak self-penetration</dt><dd>{result['collision']['maxSelfPenetrationMeters']*1000:.4f} mm</dd>
<dt>Joint closure error</dt><dd>{m['maxClosureErrorMeters']*1000:.4f} mm</dd>
<dt>Mechanical work</dt><dd>{m['absoluteMechanicalWorkJoules']:.3f} J</dd></dl>"""
    ground = result.get("groundContact")
    stance = result.get("stanceSupport")
    if stance is not None and stance.get("status") == "measured":
        distance = stance["maximumOutsideDistanceMeters"]
        outside = f"{distance*1000:.3f} mm" if distance is not None else "undefined (no loaded contact)"
        body += (f'<details><summary>Stance support (diagnostic only)</summary><dl>'
                 f'<dt>Time outside permitted support</dt><dd>{stance["unsupportedFraction"]*100:.2f}%</dd>'
                 f'<dt>Longest unsupported interval</dt><dd>{stance["longestUnsupportedSeconds"]:.3f} s</dd>'
                 f'<dt>Maximum distance outside support</dt><dd>{outside}</dd>'
                 f'<dt>Time with no permitted loaded contacts</dt><dd>{stance["noSupportSeconds"]:.3f} s</dd></dl>'
                 '<p>Measured contact-manifold support, not swept foot reach. Brief unsupported intervals are allowed; '
                 'these diagnostics do not change qualification. Distance is undefined while no permitted contact is loaded.</p></details>')
    if ground is not None:
        body += f'<p class="muted">Ground-contact samples: feet {ground["footContactSamples"]}; wheels {ground["wheelContactSamples"]}.</p>'
        if ground["forbiddenContacts"]:
            body += '<details><summary class="bad">Forbidden ground contacts</summary><ul>'
            body += "".join(f'<li>{esc(item["part"])}: first at {item["firstTime"]:.3f} s; peak penetration {item["maxPenetrationMeters"]*1000:.4f} mm</li>'
                            for item in ground["forbiddenContacts"])
            body += "</ul></details>"
    return body


def progress_chart(rows, categories, *, foot_only=False):
    total = max((row["id"] for row in rows), default=1)
    lines = ['<svg viewBox="0 0 900 250" role="img" aria-label="Best tested motor-cycle coverage, not walking fitness">',
             '<path d="M45 15V205H880" stroke="#7390b5" fill="none"/>',
             '<text x="5" y="25" fill="#b7c6da" font-size="12">100%</text>',
             '<text x="15" y="205" fill="#b7c6da" font-size="12">0%</text>',
             '<text x="400" y="240" fill="#b7c6da" font-size="12">Full simulation trials (all categories)</text>']
    legend = []
    for (kind, feet, remote, towing), color in zip(categories, COLORS[:len(categories)], strict=True):
        best, points = 0, []
        for row in rows:
            if row["category"] != kind:
                continue
            result = json.loads(row["result"])
            turns = min(abs(turn) for turn in result["metrics"]["motorTurns"])
            best = max(best, min(1, turns))
            x, y = 45 + 820 * row["id"] / max(1, total), 205 - 185 * best
            points.append(f"{x:.2f},{y:.2f}")
            lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="{color}"><title>{esc(kind)}: {best*100:.1f}% cycle coverage</title></circle>')
        if points:
            lines.append(f'<polyline points="{" ".join(points)}" stroke="{color}" fill="none" stroke-width="2"/>')
        suffix = "" if towing is None else (" / towing" if towing else " / solo")
        label = f"{feet} feet" if foot_only else f'{feet} legs / {"remote" if remote else "wired"}{suffix}'
        legend.append(f'<span class="pill" style="color:{color}">{label}</span>')
    lines.append("</svg>")
    return "".join(lines + legend)


def speed_chart(rows, categories, *, foot_only=False):
    measured = [row for row in rows if is_qualified(json.loads(row["result"]))]
    if not measured:
        return '<div class="empty">No qualified speed history yet. Unqualified displacement is not graphed as walking performance.</div>'
    total = max(1, max(row["id"] for row in rows))
    maximum = max(json.loads(row["result"])["metrics"]["speedMetersPerSecond"] * 1000 for row in measured)
    lines = ['<svg viewBox="0 0 900 250" role="img" aria-label="Best tested qualified forward walking speed in millimeters per second">',
             '<path d="M45 15V205H880" stroke="#7390b5" fill="none"/>',
             f'<text x="0" y="25" fill="#b7c6da" font-size="12">{maximum:.3f}</text>',
             '<text x="25" y="205" fill="#b7c6da" font-size="12">0</text>',
             '<text x="400" y="240" fill="#b7c6da" font-size="12">Full simulation trials (all categories)</text>']
    for (kind, _, _, _), color in zip(categories, COLORS[:len(categories)], strict=True):
        best, points = 0, []
        for row in measured:
            if row["category"] != kind:
                continue
            best = max(best, json.loads(row["result"])["metrics"]["speedMetersPerSecond"] * 1000)
            x, y = 45 + 820 * row["id"] / total, 205 - 185 * best / maximum
            points.append(f"{x:.2f},{y:.2f}")
            lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="{color}"><title>{esc(kind)}: {best:.4f} mm/s</title></circle>')
        if points:
            lines.append(f'<polyline points="{" ".join(points)}" stroke="{color}" fill="none" stroke-width="2"/>')
    return "".join([*lines, "</svg>"])


def publish(archive):
    import fcntl
    with (archive.directory / ".publish.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        _publish(archive)


def _publish(archive):
    generated_at = datetime.now(timezone.utc).isoformat()
    run = archive.db.execute("SELECT * FROM run").fetchone()
    if run is None:
        raise ValueError("Cannot publish an uninitialized archive.")
    protocol = json.loads(archive.db.execute("SELECT definition FROM protocols WHERE id=?", (run["protocol"],)).fetchone()[0])
    categories = category_descriptors(protocol)
    foot_only = foot_count_categories(protocol)
    legacy = "allowedGroundContacts" not in protocol.get("qualification", {})
    rows = archive.outcomes()
    rigid_wagons = protocol.get("grammarVersion") != "technic-fourbar-v5"
    for row in rows:
        row["result"] = json.dumps(display_result(row, protocol))
    births = [dict(row) for row in archive.db.execute("""
        SELECT b.*,d.category,d.genome FROM births b JOIN designs d ON d.id=b.design ORDER BY b.id
    """)]
    from .screening_reports import rejection_html, rejections
    screening = rejections(archive.directory)
    by_birth = {birth["id"]: birth for birth in births}
    evaluations = {row["birth"]: dict(row) for row in archive.db.execute("SELECT * FROM evaluations ORDER BY id")}
    evaluations.update({row["birth"]: row for row in rows if row.get("reused")})
    counts = archive.counts()
    used = sum(counts.values())
    media_errors = {row["design"]: row["error"] for row in archive.db.execute("SELECT * FROM media_errors")}
    forest = panel(archive.db)
    atomic_text(archive.directory / "forest.html",
                document("Proposal ancestry forest", '<a href="index.html">Back to dashboard</a>'
                         + forest + diagnostics(archive.db)))
    for name, embedded in (("index.html", False), ("preview.html", True)):
        page = archive.directory / name
        cards = []
        for kind, feet, remote, towing in categories:
            candidates = [row for row in rows if row["category"] == kind]
            best = max(candidates, key=lambda row: fitness(json.loads(row["result"]))) if candidates else None
            descriptor = f"{feet} logical feet" if foot_only else f"Four-bar &middot; {feet} legs"
            header = f'<span class="eyebrow">{descriptor}</span><h3>{"Remote / onboard AA + IR" if remote else "Wired / external power"}</h3>'
            if towing is not None and not foot_only:
                header += f'<span class="pill">{"Pulls a wagon" if towing else "Solo walker"}</span>'
            if best is None:
                content = '<span class="pill warn">No qualified walker yet</span>'
                latest = next((birth for birth in reversed(births) if birth["category"] == kind), None)
                trial = evaluations.get(latest["id"]) if latest else None
                if latest and (media_directory(archive.directory / "designs" / latest["design"]) / "walker-v2.png").exists():
                    content += media(archive.directory / "designs" / latest["design"], page, embedded)
                    content += '<p class="muted">Initial LEGO assembly render; not measured motion or evidence of walking.</p>'
                    content += links(archive, {"design": latest["design"]}, page)
                if trial is not None and trial["status"] in ("failed", "interrupted"):
                    content += f'<h3 class="bad">Trial {trial["id"]}: {esc(trial["status"])}</h3><p class="bad">{esc(trial["error"])}</p>'
                    content += f'<p><a href="history/{latest["id"]:04d}.html">Failure details and lineage</a></p>'
                    content += links(archive, {"design": latest["design"], "id": trial["id"]}, page)
                else:
                    message = "Awaiting a completed evaluation" if run["status"] == "running" else "No completed evaluation"
                    content += f'<div class="empty">{message}</div>'
            else:
                result = json.loads(best["result"])
                qualified = is_qualified(result)
                qualified_label = "Qualified under legacy ground rules" if legacy else "Best tested qualified walker"
                content = (f'<span class="pill good">{qualified_label}</span>' if qualified else
                           '<span class="pill warn">No qualified walker yet</span><p><strong>Best unqualified prototype</strong> &mdash; not a walking champion.</p>')
                content += media(archive.directory / "designs" / best["design"], page, embedded)
                content += metrics(result)
                if "gait" in result:
                    gait = result["gait"]
                    content += (f'<p class="muted">Gait: {gait["motorTurnsPerCycle"]} motor turns per cycle; '
                                f'{gait["completeCycles"]} complete cycles; repeatable forward motion: '
                                f'{esc(gait["repeatableForwardMotion"])}.</p>')
                g = json.loads(best["genome"])
                if remote:
                    content += f'<p class="muted">Battery: {"wagon" if g.get("batteryOnWagon") else "walker"}; receiver: {"wagon" if g.get("receiverOnWagon") else "walker"}.</p>'
                if not qualified:
                    content += '<p class="bad">' + esc(", ".join(result["metrics"]["rankingRejectionReasons"])) + "</p>"
                content += f'<p><a href="history/{best["birth"]:04d}.html">Lineage / evolution history</a> &middot; design {best["design"][:10]}</p>'
                content += f'<p class="muted">{esc(best["operation"])} &middot; trial {best["id"]}</p>'
                content += links(archive, best, page)
                if best["design"] in media_errors:
                    content += f'<p class="bad">{esc(media_errors[best["design"]])}</p>'
            cards.append(f'<article class="card" id="category-{esc(kind)}" tabindex="-1" data-live-key="{esc(kind)}">{header}{content}</article>')
        table = []
        for birth in reversed(births):
            trial = evaluations.get(birth["id"])
            status = trial["status"] if trial else birth["status"]
            if trial and trial["result"]:
                status += " / " + json.loads(trial["result"])["status"]
            parent = f'<a href="history/{birth["parent"]:04d}.html">birth {birth["parent"]}</a>' if birth["parent"] else "root"
            table.append(f'<tr data-live-key="birth-{birth["id"]}"><td><a href="history/{birth["id"]:04d}.html">birth {birth["id"]}</a></td><td>{esc(birth["category"])}</td><td>{esc(birth["design"][:10])}</td><td>{parent}</td><td>{esc(birth["operation"])}</td><td>{esc(status)}</td></tr>')
        backend = ("The motion-program backend evaluates explicitly supplied complete walkers in 2, 3, 4, 5 and 6-foot bins; topology evolution is not yet connected."
                   if protocol.get("grammarVersion") == "technic-motion-v3" else
                   "The current paired four-bar backend reaches 2, 4 and 6 feet; branching motion programs may also express 3 or 5 feet.")
        proposal_panel = ""
        if protocol.get("nativeDiscovery"):
            population_policy = protocol["nativeDiscovery"].get("population")
            backend = ("Native discovery balances new trials across original founder families and selects from a diverse parent population, "
                       if population_policy else
                       "Native discovery balances new trials across founder neighborhoods and selects each neighborhood's elite, ")
            backend += "with automatic LEGO re-realization of bounded mechanism edits. "
            mutations = protocol["nativeDiscovery"].get("mutations", [])
            if mutations:
                backend += f"Enabled mutations: {esc(', '.join(name.replace('-', ' ') for name in mutations))}. "
            construction = protocol["nativeDiscovery"].get("construction", {})
            if construction.get("maxRepairDepth"):
                backend += ("Rejected assemblies can receive diagnostic-guided pin or passive-link layer repairs "
                            "without changing their motion. ")
            backend += "Arbitrary rewiring is not yet supported."
            attempts = []
            for proposal in archive.db.execute("SELECT * FROM native_proposals ORDER BY id DESC"):
                summary = json.loads(proposal["summary"]) if proposal["summary"] else {}
                name = f"proposals/{proposal['id']:06d}/candidates.json"
                details = f'<a href="{name}">motion / physical gates</a>' if (archive.directory / name).exists() else esc(proposal["error"] or "pending")
                attempts.append(f'<tr data-live-key="motion-{proposal["id"]}"><td>{proposal["id"]}</td>'
                                f'<td><a href="history/{proposal["parent"]:04d}.html">birth {proposal["parent"]}</a></td>'
                                f'<td>{esc(summary.get("operation", "pending"))}</td><td>{esc(proposal["status"])}</td>'
                                f'<td>{details}</td></tr>')
            proposal_kind = "motion/repair" if construction.get("maxRepairDepth") else "motion"
            repair_note = ("Diagnostic repair chains use at most two generations, one alternative per repair, within these same limits. "
                           if construction.get("maxRepairDepth") else "")
            proposal_panel = ('<section class="panel"><h2>Native motion proposals</h2>'
                              f'<p>Per next trial: at most 16 {proposal_kind} proposals, two physical alternatives per motion proposal, '
                              'eight load-only reconstruction checks. Cheap rejections and duplicates spend no dynamic slots. '
                              + repair_note +
                              'Interrupted attempts remain archived; a bound pauses the worker, not a fabricated trial.</p>'
                              '<div class="scroll"><table><thead><tr><th>Proposal</th><th>Parent</th><th>Operator</th>'
                              '<th>Outcome</th><th>Diagnostics</th></tr></thead><tbody>'
                              + "".join(attempts) + '</tbody></table></div></section>')
            if population_policy:
                from .population import members
                parents = members(archive)
                parent_rows = "".join(
                    f'<tr><td><a href="history/{row["birth"]:04d}.html">birth {row["birth"]}</a></td>'
                    f'<td>family {row["family"]}</td><td>{esc(row["category"])}</td>'
                    f'<td>{"qualified" if is_qualified(json.loads(row["result"])) else "prototype"}</td></tr>'
                    for row in parents)
                proposal_panel = (
                    '<section class="panel" id="parent-population"><h2>Parent population</h2>'
                    f'<p>{len(parents)} / {population_policy["capacity"]} measured parents. '
                    'Original founder families share trial allocation; imported members do not become new families. '
                    f'Two-parent tournaments select the better contestant; '
                    f'{100*population_policy["randomParentProbability"]:g}% of selections instead choose a random member '
                    'of the selected family. Qualified champions, family representatives and a promising prototype are protected. '
                    'Remaining places prefer structural/layout diversity, then fitness. '
                    'The population grows from carried-forward seeds as compatible results finish. '
                    'All other results remain archived; proposal diagnostics record each parent-selection decision.</p>'
                    '<div class="scroll"><table><thead><tr><th>Parent</th><th>Founder family</th><th>Category</th>'
                    '<th>Result</th></tr></thead><tbody>' + parent_rows
                    + '</tbody></table></div></section>') + proposal_panel
        timing = "This protocol does not fully specify trial timing; consult the individual evaluation results."
        evaluation = protocol.get("evaluation", {})
        if {"duration", "settle", "timestep"} <= evaluation.keys():
            if evaluation.get("cycle_aware"):
                timing = (f"Each trial: at least {evaluation['duration']:g} simulated seconds, planned for "
                          f"{REQUIRED_CYCLES} nominal gait cycles across all motor-coupled feet, "
                          f"capped at {MAX_DURATION:g} seconds. Over-cap gear trains are rejected. ")
            else:
                timing = f"Each trial: {evaluation['duration']:g} simulated seconds. "
            timing += f"{evaluation['settle']:g} s settling; {1000 * evaluation['timestep']:g} ms timesteps."
        from .parallel import attempt_summary
        cheap = attempt_summary(archive.db)
        gate_panel = ""
        if cheap:
            gate_panel = ('<section class="panel"><h2>Cheap simulation gate</h2>'
                          f'<p>{cheap["attempts"]} prefix attempts; {cheap["counts"].get("rejected", 0)} rejected. '
                          'Only survivors reserve full simulations. Up to '
                          f'{cheap["maxAttemptsPerFullSimulation"]} attempts per unfilled full-simulation slot '
                          'per explicitly started episode; no partial fitness.</p>')
            if cheap["pausedAtLimit"]:
                gate_panel += ('<p><strong>Paused at cheap-attempt cap.</strong> Unused full-simulation allowance '
                               'is retained. Another explicit episode permits a new bounded round; '
                               'it does not refund or repeat historical trials.</p>')
            gate_panel += "</section>"
        body = f"""<header><span class="eyebrow">LegoWalkers / evolution archive</span>
<h1>{len(categories)} {"foot-count categories" if foot_only else "kinds"}. One mechanical drive.</h1>
<p>Best tested representatives, not global optima. Every candidate has a recorded genome, parent, mutation and reproducible trial.</p></header>
{f"<div class='panel'><strong>Active policy:</strong> remote only, onboard battery and receiver, no towing, two to six logical feet. Foot count is a design descriptor, not an ancestry boundary. {backend} Isolated one-foot leg tests cannot qualify as walkers.</div>" if foot_only else ""}
{"<div class='panel warn'><strong>Legacy protocol:</strong> unrestricted ground contacts were permitted in this experiment. These results do not qualify under the new feet-only / wagon-wheel rules.</div>" if legacy else "<div class='panel'><strong>Feet-only support:</strong> only designated bounded foot-contact regions may touch the ground. Any other touching part disqualifies the design, including during settling.</div>" if foot_only else "<div class='panel'><strong>Feet-only support:</strong> only the designated 4 mm foot-tip regions may touch the ground. Towing designs additionally allow their two designated wagon wheels. Any other touching part disqualifies the design, including during settling.</div>"}
{"<div class='panel warn'><strong>Superseded towing rule:</strong> rigid-wagon designs are disqualified under the current rule. Their original trial results are preserved; future wagons require a separate drawbar with two free pitch hinges.</div>" if rigid_wagons and any(item[3] for item in categories) else ""}
<div class="panel stats"><div class="stat"><strong>{used} / {run['budget']}</strong><small>{"full simulations reserved / authorized" if cheap else "trials reserved / total hard budget"}</small></div>
<div class="stat"><strong>{counts.get('complete',0)}</strong><small>completed simulations</small></div>
<div class="stat"><strong>{counts.get('failed',0)+counts.get('interrupted',0)}</strong><small>failed / interrupted trials</small></div>
<div class="stat"><strong>{len(births)}</strong><small>proposals including cheap rejects / duplicates</small></div>
<div class="stat"><strong>{sum(bool(row.get("reused")) for row in rows)}</strong><small>reused observations / zero new trials</small></div>
<div class="stat"><strong>{esc(run['status'])}</strong><small>updated {esc(generated_at)}</small></div></div>
<p class="muted">{timing}
One trial is not one timestep or animation frame. The live dashboard updates in place without reloading; playing previews are preserved. Local-file pages are static snapshots.</p>
{forest}
{gate_panel}
<div class="grid">{"".join(cards)}</div>
<section class="panel"><h2>Evolution progress</h2><h3>Qualified walking speed (mm/s)</h3>
<p>Best tested qualified speed by category under this page's protocol. Unqualified designs cannot add performance points.</p>{speed_chart(rows,categories,foot_only=foot_only)}
<h3>Mechanical progress</h3><p>Best completed motor-cycle coverage in each category, capped at 100%. This is not evidence of walking; collision, closure, tilt and ground-contact gates still apply.</p>{progress_chart(rows,categories,foot_only=foot_only)}</section>
<section class="panel"><h2>Protocol and honest limits</h2>
<p>Protocol <a href="protocol.json">{esc(run['protocol'][:16])}</a>; seed {run['seed']}.
Only observations with matching evaluation conditions can compete here. Reused results retain their original protocol and provenance; search-code changes alone do not invalidate physics measurements. Speed ranks qualified walkers; failed designs cannot take champion slots.</p>
<p>{"Remote-only search; wired designs are retained only in historical archives." if foot_only else "Wired: ideal external power/control, no onboard battery or receiver; tether drag is not simulated."}
{"Remote: actual 58119 / 58123 geometry; all frame, mounting and electronics placements are explicit realization choices." if protocol.get("grammarVersion") == "technic-motion-v3" else "Remote: actual 58119 / 58123 geometry, four battery mounting pins on stud-stacked towers, two receiver mounting pins. Battery fore/aft slot and height, receiver fore/aft slot and side are genes."}
Loaded AA box {protocol['payloadProfile']['components']['8881']['massKg']*1000:.0f} g;
receiver {protocol['payloadProfile']['components']['8884']['massKg']*1000:.0f} g. <strong>These are reference estimates, not measured component weights.</strong>
Electronics centers/inertias, mount rigidity, motor characteristics and plastic masses are approximations.
{"Towing is excluded from this search. Both electronics units remain on the walker." if foot_only else "This archived protocol used a rigid load-bearing drawbar; those towing designs are now disqualified." if rigid_wagons else "Towing designs use separate walker, drawbar and wagon bodies, with two passive transverse pitch hinges and independent wheels; no steering. Remote towing genomes evolve each electronics unit's carrier and mounting position."}
No physical buildability, payload retention or useful gait is certified.</p>
<p><a href="payload-profile.json">Mass profile and sources</a> &middot; <a href="archive.sqlite3">SQLite archive</a> &middot;
<a data-archive-link="render/simulation/preview.html" href="../render/simulation/preview.html">Collision-reference preview</a> &middot;
<a data-archive-link="evolution-pilot/preview.html" href="../evolution-pilot/preview.html">Original paused pilot</a> &middot;
<a data-archive-link="evolution-feet-wagon-pilot/preview.html" href="../evolution-feet-wagon-pilot/preview.html">Feet-only / wagon experiment</a></p></section>
{proposal_panel}
<section class="panel"><h2>All proposals and ancestry</h2><p>Births are distinct from physical design IDs.
Revisited genomes share a design ID, not a fabricated new design; parent links connect births and cannot create ancestry cycles.</p>
<div class="scroll"><table><thead><tr><th>Birth</th><th>Kind</th><th>Design</th><th>Parent</th><th>Operator</th><th>Outcome</th></tr></thead><tbody>{"".join(table)}</tbody></table></div></section>"""
        atomic_text(page, document("LegoWalkers evolution", body, dashboard=True))
    for birth in births:
        page = archive.directory / "history" / f"{birth['id']:04d}.html"
        trial = evaluations.get(birth["id"])
        chain = []
        cursor = birth
        while cursor is not None:
            chain.append(f'<a href="{cursor["id"]:04d}.html">birth {cursor["id"]} / {esc(cursor["design"][:10])}</a>')
            cursor = by_birth.get(cursor["parent"])
        children = [child for child in births if child["parent"] == birth["id"]]
        genome = json.loads(birth["genome"])
        genes = {key: value for key, value in genome.items() if key != "seed"}
        parent_genes = json.loads(by_birth[birth["parent"]]["genome"]) if birth["parent"] else {}
        changes = {key: {"from": parent_genes.get(key), "to": value}
                   for key, value in genes.items() if parent_genes.get(key) != value}
        body = f"""<a href="../preview.html">Back to all categories</a><h1>Birth {birth['id']}</h1>
<p>{esc(birth['category'])} &middot; design {esc(birth['design'])}</p>"""
        body += birth_preview(archive.directory / "designs" / birth["design"], page)
        body += f"""<section class="panel"><h2>Ancestry</h2>{" &rarr; ".join(reversed(chain))}
<p>Operator: {esc(birth['operation'])}; proposal seed: {birth['seed']}; status: {esc(birth['status'])}</p>
<p>Children: {" &middot; ".join(f'<a href="{child["id"]:04d}.html">birth {child["id"]}</a>' for child in children) or "none yet"}</p>
<h3>{"Recorded construction edits" if birth["operation"] == "repair-realization" else "Recorded motion edits"}</h3><pre>{esc(json.dumps(json.loads(birth["changes"]),indent=2))}</pre>
<h3>Actual gene differences from parent</h3><pre>{esc(json.dumps(changes,indent=2))}</pre></section>
<section class="panel"><h2>Canonical genome (provenance seed excluded)</h2><pre>{esc(json.dumps(genes,indent=2))}</pre></section>"""
        candidate = archive.directory / "designs" / birth["design"] / "candidate"
        if (candidate / "program.json").exists():
            prefix = f'../designs/{birth["design"]}/candidate'
            body += (f'<p><a href="{prefix}/program.json">Motion program</a> &middot; '
                     f'<a href="{prefix}/realization.json">Physical realization</a> &middot; '
                     f'<a href="{prefix}/cycle.svg">Solved motion diagram</a></p>')
            origin = archive.db.execute("SELECT batch,directory FROM native_candidates WHERE birth=?",
                                        (birth["id"],)).fetchone()
            if origin is not None:
                construction = archive.directory / origin["batch"] / origin["directory"] / "construction.json"
                if construction.is_file():
                    body += f'<p><a href="{relative(construction, page)}">Construction motif and limits</a></p>'
                builder = archive.directory / origin["batch"] / "builder.json"
                if builder.is_file():
                    body += f'<p><a href="{relative(builder, page)}">Builder policy and versioned motif catalogue</a></p>'
                feedback = archive.directory / origin["batch"] / "feedback.json"
                if feedback.is_file():
                    body += f'<p><a href="{relative(feedback, page)}">Screening feedback used for this repair</a></p>'
                for path, label in [
                    (archive.directory / origin["batch"] / origin["directory"] / "grammar.json", "Connection grammar derivation"),
                    (archive.directory / origin["batch"] / "grammar-search.json", "Bounded grammar search accounting"),
                    (archive.directory / origin["batch"] / "conflicts.json", "Reported contact-pair constraints"),
                ]:
                    if path.is_file():
                        body += f'<p><a href="{relative(path, page)}">{label}</a></p>'
        if birth["error"]:
            info = screening.get(birth["design"]) if birth["status"] == "screening-rejected" else None
            body += (rejection_html(info["detail"], info["report"], "../") if info
                     else f'<section class="panel bad">{esc(birth["error"])}</section>')
        if trial:
            label = "Reused observation (no new trial)" if trial.get("reused") else f'Trial {trial["id"]}'
            body += f'<section class="panel"><h2>{label}: {esc(trial["status"])}</h2>'
            if trial["result"]:
                result = display_result({**trial, "genome": birth["genome"]}, protocol)
                body += metrics(result)
                body += f'<p class="bad">{esc(", ".join(result["metrics"]["rankingRejectionReasons"]))}</p>'
            if trial["error"]:
                body += f'<p class="bad">{esc(trial["error"])}</p>'
            body += links(archive, {**trial, "design": birth["design"]}, page) + "</section>"
        elif archive.db.execute("SELECT 1 FROM sqlite_master WHERE name='simulation_attempts'").fetchone():
            attempt = archive.db.execute("SELECT * FROM simulation_attempts WHERE birth=?", (birth["id"],)).fetchone()
            if attempt is not None:
                body += (f'<section class="panel"><h2>Cheap simulation attempt {attempt["id"]}: '
                         f'{esc(attempt["status"])}</h2><p>No full-simulation slot consumed; no partial fitness.</p>'
                         + links(archive, {"design": birth["design"], "gateAttempt": attempt["id"]}, page) + "</section>")
        atomic_text(page, collapsible_genes(document(f"LegoWalkers birth {birth['id']}", body)))
    progress = {"status": run["status"], "updated": generated_at, "budget": run["budget"], "counts": counts,
                "cheapGate": cheap,
                "proposals": len(births), "protocol": run["protocol"],
                "representatives": {
                    kind: max((row for row in rows if row["category"] == kind),
                              key=lambda row: fitness(json.loads(row["result"])))["design"]
                    for kind in sorted({row["category"] for row in rows})}}
    atomic_text(archive.directory / "progress.json", json.dumps(progress, indent=2) + "\n")
