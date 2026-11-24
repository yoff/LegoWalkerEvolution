import contextlib
import copy
import json
import math
from pathlib import Path
import subprocess
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from legowalkers_sim import motion_diagrams
from legowalkers_sim.bridge import AdapterError
from legowalkers_sim.dashboard import DashboardHandler
from legowalkers_sim.gallery import motion_assets
from legowalkers_sim.campaign import version_html
from legowalkers_sim.workbench import Workbench
from test_campaign import setup


def sample(turns=1):
    program = {"bodies": [{"id": "crank", "grounded": False, "initial": {"origin": {"x": 0, "y": 0, "z": 0}, "angle": 0}, "sites": [
        {"id": "pivot", "position": {"x": 0, "y": 0, "z": 0}},
        {"id": "tip", "position": {"x": 10, "y": 0, "z": 0}}]}],
        "hinges": [], "motor": {"body": "crank", "phase": 0}, "transmissions": [],
        "feet": [{"id": "foot", "port": {"body": "crank", "site": "tip"}, "radius": 10}]}
    return {"schemaVersion": 1, "units": "ldu", "mode": "planar-kinematics-not-dynamics",
            "program": program, "frames": [
                {"inputAngle": n*2*math.pi/72, "maxResidualLdu": 0,
                 "poses": [{"body": "crank", "origin": [0, 0, 0], "angle": n*2*math.pi/72}],
                 "feet": [{"id": "foot", "position": [10*math.cos(n*2*math.pi/72), 10*math.sin(n*2*math.pi/72), 0]}]}
                for n in range(72*turns+1)]}

def test_old_and_readonly_pages_gain_player_assets_without_duplicate_scripts():
    old = '<html><body><div class="motion-player" data-motion-source="../candidate/motion.json"></div></body></html>'
    served = motion_assets(old)
    assert served.count("<script data-motion-player-script>") == 1
    assert ".motion-player canvas" in served
    assert motion_assets(served) == served
    readonly = motion_assets(version_html(served, "/versions/v0030/history/0023.html", readonly=True))
    assert readonly.count("<script data-motion-player-script>") == 1
    assert "Read-only version history" in readonly


def test_previously_embedded_players_receive_current_assets_without_changing_other_scripts():
    old = ('<html><body><script>window.unrelated = true;</script>'
           '<style data-motion-player-style>.outdated{}</style>'
           '<script data-motion-player-script>outdatedPlayer();</script></body></html>')
    served = motion_assets(old)
    assert "outdatedPlayer" not in served and ".outdated" not in served
    assert "window.unrelated = true;" in served
    assert "Depth (oblique)" in served and ".motion-legend{" in served
    assert served.count("<script data-motion-player-script>") == 1
    assert served.count("<style data-motion-player-style>") == 1
    assert motion_assets(served) == served


def test_solver_caches_full_gaits_without_touching_candidate_files(tmp_path, monkeypatch):
    candidate = tmp_path / "candidate"
    candidate.mkdir()
    program = json.dumps({"schemaVersion": 1, "program": sample()["program"]}).encode()
    (candidate / "program.json").write_bytes(program)
    (candidate / "manifest.json").write_text("{}")
    before = {path.name: path.read_bytes() for path in candidate.iterdir()}
    calls = []
    monkeypatch.setattr(motion_diagrams, "load_manifest", lambda path: {"parts": [], "feet": []})
    monkeypatch.setattr(motion_diagrams, "cycle_plan", lambda *args: {"motorTurnsPerCycle": 3})

    def run(command, **kwargs):
        calls.append(command)
        assert command[:3] == ["dotnet", "generator.dll", "motion"]
        assert command[-4:] == ["--steps", "72", "--turns", "3"] and kwargs["timeout"] == 60
        source = Path(command[command.index("--program")+1])
        assert source.read_bytes() == program and source.parent != candidate
        output = Path(command[command.index("--output")+1])
        output.mkdir()
        (output / "motion.json").write_text(json.dumps(sample(3)))
        return SimpleNamespace(returncode=0, stderr="")

    monkeypatch.setattr(motion_diagrams.subprocess, "run", run)
    motion_diagrams._solve.cache_clear()
    try:
        first = motion_diagrams.solved_motion(candidate, ["dotnet", "generator.dll"])
        assert len(first["frames"]) == 217 and first["frames"][-1]["inputAngle"] == 6*math.pi
        assert motion_diagrams.solved_motion(candidate, ["dotnet", "generator.dll"]) == first
        assert len(calls) == 1
        assert {path.name: path.read_bytes() for path in candidate.iterdir()} == before
        assert not Path(calls[0][calls[0].index("--program")+1]).exists()
    finally:
        motion_diagrams._solve.cache_clear()


def test_physical_depths_augment_flat_poses_without_mutating_the_solver_cache(tmp_path, monkeypatch):
    data = sample()
    original = copy.deepcopy(data)
    (tmp_path / "program.json").write_text(json.dumps({"schemaVersion": 1, "program": data["program"]}))
    manifest = {"parts": [{"body": "crank", "position": [0, 0, 10]},
                          {"body": "crank", "position": [0, 0, 30]}],
                "feet": [{"id": "contact-a", "logicalFoot": "foot", "position": [10, 0, -50]},
                         {"id": "contact-b", "logicalFoot": "foot", "position": [10, 0, -30]}]}
    monkeypatch.setattr(motion_diagrams, "load_manifest", lambda path: manifest)
    monkeypatch.setattr(motion_diagrams, "cycle_plan", lambda *args: {"motorTurnsPerCycle": 1})
    monkeypatch.setattr(motion_diagrams, "_solve", lambda *args: data)
    diagram = motion_diagrams.solved_motion(tmp_path, ["generator"])
    assert diagram["displayDepths"] == {"bodies": {"crank": 20}, "feet": {"foot": -40}}
    assert data == original and diagram["frames"] == original["frames"]
    manifest["parts"][0]["position"][2] = 50
    second = motion_diagrams.solved_motion(tmp_path, ["generator"])
    assert second["displayDepths"]["bodies"]["crank"] == 40
    assert diagram["displayDepths"]["bodies"]["crank"] == 20
    assert data == original


@pytest.mark.parametrize("failure", ["exit", "timeout"])
def test_solver_failures_are_explicit_and_not_cached(monkeypatch, failure):
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if failure == "timeout":
            raise subprocess.TimeoutExpired(command, 60)
        return SimpleNamespace(returncode=2, stderr="Constraint closure failed.")

    monkeypatch.setattr(motion_diagrams.subprocess, "run", run)
    motion_diagrams._solve.cache_clear()
    for _ in range(2):
        with pytest.raises(AdapterError, match="60-second|Constraint closure failed"):
            motion_diagrams._solve(b"{}", ("dotnet", "generator.dll"), 1)
    assert len(calls) == 2


def test_full_32_turn_coverage_is_valid_and_longer_cycles_are_rejected(tmp_path, monkeypatch):
    data = sample(32)
    motion_diagrams.validate(data, data["program"], 32)
    assert len(data["frames"]) == 2305
    monkeypatch.setattr(motion_diagrams, "load_manifest", lambda path: {})
    monkeypatch.setattr(motion_diagrams, "cycle_plan", lambda *args: {"motorTurnsPerCycle": 33})
    with pytest.raises(AdapterError, match="one to 32 motor turns"):
        motion_diagrams.solved_motion(tmp_path, ("dotnet", "generator.dll"))


@pytest.mark.parametrize("damage", ["coverage", "phase", "body", "foot", "nonfinite", "boolean", "null", "program"])
def test_invalid_or_incomplete_pose_data_is_never_presented_as_animation(damage):
    data = sample()
    original = copy.deepcopy(data["program"])
    if damage == "coverage":
        data["frames"].pop()
    elif damage == "phase":
        data["frames"][1]["inputAngle"] = 0
    elif damage == "body":
        data["frames"][1]["poses"][0]["body"] = "unknown"
    elif damage == "foot":
        data["frames"][1]["feet"] = []
    elif damage == "nonfinite":
        data["frames"][1]["poses"][0]["origin"][0] = float("nan")
    elif damage == "boolean":
        data["frames"][1]["feet"][0]["position"][0] = True
    elif damage == "null":
        data["frames"][1] = None
    else:
        data["program"]["bodies"][0]["id"] = "changed"
    with pytest.raises(AdapterError, match="Motion diagram"):
        motion_diagrams.validate(data, original, 1)


def test_motion_routes_preserve_archives_and_serve_current_historical_and_workbench_candidates(setup, monkeypatch):
    campaign = setup.create()
    with contextlib.closing(campaign.active.connect()) as db:
        design = db.execute("SELECT design FROM births LIMIT 1").fetchone()[0]
    candidate = campaign.directory / "designs" / design / "candidate"
    (candidate / "program.json").write_text("{}")
    calls = []
    monkeypatch.setattr(motion_diagrams, "solved_motion",
                        lambda path, command: calls.append(path) or sample())
    before = campaign.history()
    saved = {path: path.read_bytes() for path in candidate.iterdir() if path.is_file()}
    with ThreadingHTTPServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = campaign
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base = f"http://127.0.0.1:{server.server_port}"

        def request(path):
            try:
                with urlopen(base + path, timeout=5) as response:
                    return response.status, response.read()
            except HTTPError as error:
                with error:
                    return error.code, error.read()

        try:
            for prefix in ("", "/versions/v0001"):
                code, body = request(f"{prefix}/designs/{design}/candidate/motion.json")
                assert code == 200 and json.loads(body) == sample()
            assert calls == [candidate.resolve(), candidate.resolve()]
            assert request(f"/versions/v9999/designs/{design}/candidate/motion.json")[0] == 404
            assert request(f"/designs/{'f'*64}/candidate/motion.json")[0] == 404
            outside = candidate.parent.parent.parent.parent / "outside"
            outside.mkdir()
            (outside / "program.json").write_text("{}")
            (campaign.directory / "designs" / ("e"*64)).symlink_to(outside, target_is_directory=True)
            assert request(f"/designs/{'e'*64}/candidate/motion.json")[0] == 403
            store = Workbench(campaign.root / "user-workbench")
            node = "a"*32
            with store.connect() as db:
                db.execute("INSERT INTO nodes VALUES(?,?,?,?,?,?,?,?)",
                           (node, "b"*32, None, "draft", "{}", "now", str(campaign.directory), 1))
            workbench_candidate = store.candidate(node)
            workbench_candidate.mkdir(parents=True)
            (workbench_candidate / "program.json").write_text("{}")
            assert request(f"/workbench-assets/{node}/motion.json")[0] == 200
            assert calls[-1] == workbench_candidate
            assert request(f"/workbench-assets/{'c'*32}/motion.json")[0] == 500
        finally:
            server.shutdown()
    after = campaign.history()
    assert (after["used"], after["budget"]) == (before["used"], before["budget"])
    assert setup.launched == []
    assert all(path.read_bytes() == value for path, value in saved.items())


@pytest.mark.parametrize("reduce,fail", [(False, False), (True, False), (False, True)])
def test_player_starts_paused_manual_play_visibility_scrub_reduced_motion_and_removal(reduce, fail):
    script = (Path(__file__).parents[2] / "simulator/legowalkers_sim/motion_diagrams.js").read_text()
    harness = """
const assert = require("node:assert/strict");
const events = new Map(), raf = new Map(); let next = 0, visible, mutation, mediaChange, fetches = 0;
class Element {
    constructor(tag) { this.tag = tag; this.children = []; this.dataset = {}; this.style = {}; this.listeners = {}; this.attributes = {}; this.isConnected = true; }
    append(...nodes) { for (const node of nodes) {node.parent = this; this.children.push(node);} }
    remove() { this.parent.children = this.parent.children.filter(child => child !== this); }
    setAttribute(name, value) { this.attributes[name] = value; }
    addEventListener(name, callback) { this.listeners[name] = callback; }
    removeEventListener(name, callback) { if (this.listeners[name] === callback) delete this.listeners[name]; }
    matches(selector) { return selector === ".motion-player" && this.className === "motion-player"; }
    querySelector(selector) { return this.children.find(child => child.tag === selector || child.matches(selector)) ||
        this.children.map(child => child.querySelector(selector)).find(Boolean); }
    getBoundingClientRect() { return {width: 400, height: 300}; }
    getContext() { return Object.fromEntries(["setTransform","clearRect","beginPath","moveTo","lineTo","stroke",
        "setLineDash","arc","fill","strokeText","fillText"].map(name => [name, () => {}])); }
}
const root = new Element("div"); root.className = "motion-player"; root.dataset.motionSource = "/motion.json";
const staticLink = new Element("a"); staticLink.href = "/cycle.svg"; root.append(staticLink);
let roots = [root];
global.location = {protocol: "http:"};
global.devicePixelRatio = 2;
global.document = {hidden: false, documentElement: {}, createElement: tag => new Element(tag),
    querySelectorAll: () => roots, addEventListener: (name, callback) => events.set(name, callback)};
const reduced = {matches: REDUCED, addEventListener(name, callback) {mediaChange = callback;}};
global.matchMedia = () => reduced;
global.IntersectionObserver = class {constructor(callback) {visible = callback;} observe() {} unobserve() {}};
global.MutationObserver = class {constructor(callback) {mutation = callback;} observe() {}};
global.ResizeObserver = class {observe() {} disconnect() {}};
global.requestAnimationFrame = callback => {raf.set(++next, callback); return next;};
global.cancelAnimationFrame = id => raf.delete(id);
global.fetch = async () => {fetches++; return {ok: !FAILING, headers: {get: () => "application/json"},
    json: async () => FAILING ? {error:"Cannot close saved linkage."} : (DATA)};};
function frame(time) {const [id, callback] = raf.entries().next().value; raf.delete(id); callback(time);}
"""
    assertions = """
(async () => {
    assert.equal(fetches, 0);
    visible([{target: root, isIntersecting: true}]);
    await new Promise(resolve => setImmediate(resolve));
    assert.equal(fetches, 1);
    if (FAILING) {
        const error = root.querySelector("p");
        assert.match(error.textContent, /Cannot close saved linkage.*static overview remains available/);
        assert.equal(error.attributes.role, "alert");
        assert.notEqual(staticLink.hidden, true);
        assert.equal(root.querySelector("canvas"), undefined);
        assert.equal(raf.size, 0);
        return;
    }
    assert.equal(staticLink.hidden, true);
    const button = root.querySelector("button"), slider = root.querySelector("input"), output = root.querySelector("output");
    const projection = root.querySelector("select");
    assert.equal(projection.value, "front");
    assert.equal(projection.children[0].value, "front");
    assert.equal(button.textContent, "Play");
    assert.equal(raf.size, 0);
    visible([{target: root, isIntersecting: false}]);
    visible([{target: root, isIntersecting: true}]);
    document.hidden = true; events.get("visibilitychange")();
    document.hidden = false; events.get("visibilitychange")();
    assert.equal(button.textContent, "Play"); assert.equal(raf.size, 0);
    button.listeners.click();
    frame(1000); frame(1100);
    assert.notEqual(output.textContent, "0 deg");
    button.listeners.click();
    const paused = output.textContent;
    assert.equal(raf.size, 0); assert.equal(button.textContent, "Play");
    assert.equal(output.textContent, paused);
    slider.value = "36"; slider.listeners.input();
    assert.equal(output.textContent, "180 deg"); assert.equal(raf.size, 0);
    for (const view of ["oblique", "front"]) {
        projection.value = view; projection.listeners.change();
        assert.equal(output.textContent, "180 deg");
        assert.equal(button.textContent, "Play"); assert.equal(raf.size, 0);
    }
    button.listeners.click(); assert.equal(raf.size, 1);
    visible([{target: root, isIntersecting: false}]); assert.equal(raf.size, 0);
    visible([{target: root, isIntersecting: true}]); assert.equal(raf.size, 1); assert.equal(fetches, 1);
    document.hidden = true; events.get("visibilitychange")(); assert.equal(raf.size, 0);
    document.hidden = false; events.get("visibilitychange")(); assert.equal(raf.size, 1);
    reduced.matches = true; mediaChange(); assert.equal(raf.size, 0); assert.equal(button.textContent, "Play");
    button.listeners.click(); assert.equal(raf.size, 1);
    const added = new Element("div"); added.className = "motion-player"; added.dataset.motionSource = "/motion.json";
    const addedLink = new Element("a"); addedLink.href = "/cycle.svg"; added.append(addedLink);
    roots.push(added); mutation([{addedNodes:[added], removedNodes:[]}]);
    visible([{target:added, isIntersecting:true}]);
    await new Promise(resolve => setImmediate(resolve));
    assert.equal(added.querySelector("button").textContent, "Play");
    assert.equal(raf.size, 1);
    root.isConnected = false; added.isConnected = false; roots = [];
    mutation([{addedNodes: [], removedNodes: [root]}]); assert.equal(raf.size, 0);
})().catch(error => {console.error(error); process.exitCode = 1;});
"""
    code = (harness.replace("REDUCED", json.dumps(reduce)).replace("DATA", json.dumps(sample()))
            + script + assertions).replace("FAILING", json.dumps(fail))
    completed = subprocess.run(["node", "-e", code], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr[-3000:]
