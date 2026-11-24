const assert = require("node:assert/strict");
const {readFileSync} = require("node:fs");
const {resolve} = require("node:path");
const {test} = require("node:test");
const vm = require("node:vm");

// Only the DOM surface used by workbench selection; no browser, network or physics.
class Element {
  constructor(tag, text = "") {
    this.tagName = tag;
    this.children = [];
    this.dataset = {};
    this.attributes = {};
    this.value = "";
    this.textContent = text;
    this.className = "";
    this.disabled = false;
    this.classList = {
      add: name => this.classList.toggle(name, true),
      remove: name => this.classList.toggle(name, false),
      toggle: (name, enabled) => {
        const names = new Set(this.className.split(" ").filter(Boolean));
        if (enabled) names.add(name); else names.delete(name);
        this.className = [...names].join(" ");
      }
    };
  }
  append(...children) { children.forEach(child => child.parent = this); this.children.push(...children); }
  replaceChildren(...children) {
    this.children = children;
    children.forEach(child => child.parent = this);
    if (this.tagName === "select") this.value = children[0]?.value || "";
  }
  setAttribute(name, value) { this.attributes[name] = value; }
  closest(selector) {
    for (let node = this; node; node = node.parent) {
      if (selector === ".motion-player" && node.className.split(" ").includes("motion-player")) return node;
    }
    return null;
  }
  querySelector(tag) { return descendants(this).find(node => node !== this && node.tagName === tag); }
  scrollIntoView() {}
}

const descendants = element => [element, ...element.children.flatMap(descendants)];
const text = element => [element.textContent, ...element.children.map(text)].join(" ");
const id = digit => digit.repeat(32);
const workspace = id("a"), root = id("1"), middle = id("2"), tip = id("3"), discarded = id("4");
const otherWorkspace = id("b"), otherRoot = id("5"), rejected = id("6");
const script = readFileSync(resolve(__dirname, "../../simulator/legowalkers_sim/workbench.js"), "utf8");
const html = readFileSync(resolve(__dirname, "../../simulator/legowalkers_sim/workbench.html"), "utf8");

function fixture() {
  const node = (nodeId, parent, status = "draft") => ({
    id: nodeId, parent, workspace, status, physical: true, preview: false,
    summary: {operation: parent ? "lengthen-foot" : "fork", changed: []}
  });
  return {
    sources: [], admissions: [], contributions: [], checks: [],
    workspaces: [{id: workspace, label: "Test chain", author: "Test", head: root},
      {id: otherWorkspace, label: "Other", author: "Other test", head: otherRoot}],
    nodes: [node(root, null, "kept"), node(middle, root), node(tip, middle),
      node(discarded, root, "discarded"),
      {...node(otherRoot, null, "kept"), workspace: otherWorkspace},
      {...node(rejected, tip, "rejected"), physical: false}]
  };
}

function start(state = fixture(), url = "http://example.test/workbench.html", behavior = {}) {
  const elements = {};
  for (const match of html.matchAll(/<(\w+)[^>]*\bid="([^"]+)"/g)) elements[match[2]] = new Element(match[1]);
  elements.level.value = "";
  elements["diagram-view"].value = "oblique";
  const requests = [];
  const timers = [];
  const location = {href: url};
  const document = {
    getElementById: name => elements[name],
    createElement: tag => new Element(tag),
    createElementNS: (_namespace, tag) => new Element(tag),
    createTextNode: value => new Element("#text", value),
    querySelectorAll: () => [...new Set(Object.values(elements).flatMap(descendants))]
      .filter(e => ["button", "input", "select", "textarea"].includes(e.tagName))
  };
  const context = vm.createContext({
    document, location, URL,
    history: {replaceState: (_state, _title, url) => { location.href = url.toString(); }},
    localStorage: {getItem: () => "", setItem: () => {}},
    Option: class extends Element {
      constructor(label, value) { super("option", label); this.value = value; }
    },
    setTimeout: callback => timers.push(callback),
    fetch: async (url, options) => {
      let value;
      if (options?.method === "POST") {
        const request = JSON.parse(options.body);
        requests.push(request);
        if (request.action === "contribute") {
          const check = {id: id("d"), node: request.node, label: request.label, started: new Date().toISOString(),
            phase: "checking", status: "running", elapsedSeconds: 2, detail: "Reconstructing parts at time zero.",
            outcomes: [{gate: "geometry", outcome: "warning", source: "saved-generation", code: "saved-envelope-warning"}]};
          state.checks = [check];
          try {
            await behavior.contribute?.(request);
          } catch (error) {
            check.status = "failed"; check.detail = error.message;
            return {ok: false, json: async () => ({error: error.message})};
          }
          state.contributions.push({id: id("c"), node: request.node, label: request.label, author: request.author, active: 1});
          state.nodes.find(node => node.id === request.node).status = "kept";
          check.status = "complete"; check.phase = "admitted"; check.detail = "Admitted without fitness.";
          check.outcomes.push({gate: "simulator-reconstruction", outcome: "pass", source: "admission-check", code: "loaded-at-time-zero"});
        } else if (request.action === "preview") {
          value = await behavior.preview?.(request) ||
            {alternatives: [], summary: {}, parent: {parts: [], paths: [], joints: [], feet: []}};
        } else if (request.action === "apply") {
          const parent = state.nodes.find(n => n.id === request.node);
          const children = ["7", "8"].map(digit => ({
            ...parent, id: id(digit), parent: parent.id, status: "draft",
            summary: {operation: "lengthen-foot", changed: []}
          }));
          state.nodes.push(...children);
          value = {children: children.map(n => n.id)};
        } else if (request.action === "checkout") {
          const node = state.nodes.find(n => n.id === request.node);
          state.workspaces.find(w => w.id === node.workspace).head = node.id;
        } else if (request.action === "rebase") {
          const parent = state.nodes.find(n => n.id === request.node);
          const child = {...parent, id: id("9"), parent: parent.id, status: "kept",
            summary: {operation: "rebase", dynamicTrials: 0, fitnessImported: false}};
          state.nodes.push(child);
          state.workspaces.find(w => w.id === child.workspace).head = child.id;
          value = {workspace: child.workspace, node: child.id};
        }
      } else if (url === "/api/status") value = {csrfToken: "test"};
      else if (url.includes("?checks=")) value = {checks: state.checks};
      else if (url.includes("?edits=")) value = {edits: behavior.edits || [
        {id: "e".repeat(64), operation: "lengthen-foot", level: "coupled", changed: []}
      ]};
      else value = state;
      return {ok: true, json: async () => JSON.parse(JSON.stringify(value || {}))};
    }
  });

  vm.runInContext(script, context);
  const flush = async () => { await new Promise(setImmediate); await new Promise(setImmediate); };
  const buttons = () => descendants(elements["version-tree"]).filter(e => e.tagName === "button");
  return {
    elements, state, requests, location, flush, buttons,
    poll: async () => { await timers.shift()?.(); await flush(); },
    select: async node => {
      const button = buttons().find(b => b.dataset.node === node);
      assert.ok(button, "Version must be navigable in the tree");
      button.onclick();
      await flush();
    }
  };
}

const parameterEdits = [0, 20, 40].map((value, index) => ({
    id: ["a", "b", "c"][index].repeat(64), level: "realization", operation: "receiver-layout",
    changed: [{path: "hardware/receiver/position", before: [80, -34, 180], after: [80 + value, -34, 180]}]
  }));
  function diagramPreview(request) {
    const model = x => ({parts: [{id: "receiver", part: "58123.dat", color: 1, body: "chassis",
      placement: [x, 0, 0], front: [[x, 0], [x + 10, 0], [x, 10]],
      oblique: [[x, 0], [x + 10, 0], [x, 10]]}], paths: [], feet: [], joints: []});
    return {parent: model(0), alternatives: [{diagram: model(parameterEdits.findIndex(e => e.id === request.edit) * 20),
      comparison: {parts: [], motionUnchanged: true}}], summary: {selectedEdit: request.edit}};
  }
  const parameterSelect = page => descendants(page.elements["edit-options"]).find(e => e.dataset.parameterValues);

  test("rebase submits the selected version to a current family and selects its new immutable child", async () => {
    const state = fixture();
    state.activeVersion = "v0002";
    state.sources = [{version: "v0001", birth: 1, label: "Historical"},
      {version: "v0002", birth: 2, label: "Current family"}];
    const page = start(state);
    await page.flush();
    await page.select(middle);
    assert.equal(page.elements["rebase-source"].children.length, 1);
    page.elements["rebase-form"].onsubmit({preventDefault() {}});
    await page.flush();
    assert.deepEqual(page.requests, [{csrfToken: "test", action: "rebase",
      node: middle, version: "v0002", birth: 2}]);
    assert.equal(page.elements.node.value, id("9"));
    assert.match(page.elements.message.textContent, /geometry unchanged.*no fitness imported/);
  });

  test("parameter values share a card, cache previews and apply the exact selected edit", async () => {
    const page = start(fixture(), undefined, {edits: parameterEdits, preview: diagramPreview});
    await page.flush();
    assert.equal(page.elements["edit-options"].children.length, 1);
    assert.equal(parameterSelect(page).children.length, 3);
    parameterSelect(page).value = parameterEdits[1].id;
    parameterSelect(page).onchange();
    await page.flush();
    assert.deepEqual(page.requests.map(r => r.edit), [parameterEdits[1].id, parameterEdits[2].id]);
    assert.match(text(page.elements["edit-options"]), /Cached preview ready/);
    assert.equal(descendants(page.elements["edit-options"]).filter(e => e.tagName === "svg").length, 2);
    parameterSelect(page).value = parameterEdits[2].id;
    parameterSelect(page).onchange();
    await page.flush();
    assert.equal(page.requests.length, 2, "Cached neighbor must need no new preview request");
    page.elements.apply.onclick();
    await page.flush();
    assert.equal(page.requests[2].action, "apply");
    assert.equal(page.requests[2].edit, parameterEdits[2].id);
  });

  test("preview completion from an old node cannot replace the new node's diagrams", async () => {
    let finish;
    const held = new Promise(resolve => {finish = resolve;});
    const page = start(fixture(), undefined, {edits: parameterEdits,
      preview: async request => {await held; return diagramPreview(request);}});
    await page.flush();
    page.elements["preview-option"].onclick();
    await page.flush();
    await page.select(middle);
    finish();
    await page.flush();
    assert.equal(page.elements.node.value, middle);
    assert.equal(descendants(page.elements["edit-options"]).filter(e => e.tagName === "svg").length, 0);
    assert.equal(page.requests.length, 1, "Old previews must not prefetch onto a new node");
  });

  test("different parameter targets and structural edits do not merge", async () => {
    const entries = [
      ...parameterEdits,
      {...parameterEdits[0], id: "d".repeat(64), changed: [
        {path: "hardware/battery/position", before: [80, -34, 180], after: [100, -34, 180]}]},
      {...parameterEdits[0], id: "e".repeat(64), operation: "add-dyad"},
      {...parameterEdits[1], id: "f".repeat(64), operation: "add-dyad"}
    ];
    const page = start(fixture(), undefined, {edits: entries});
    await page.flush();
    assert.equal(page.elements["edit-options"].children.length, 4);
  });

  test("shared leg values keep all instance changes on one parameter card", async () => {
    const entries = parameterEdits.map((edit, index) => ({
      ...edit, operation: "shared-leg-move-hinge-site", level: "motion",
      changed: [0, 1, 2, 3].map(leg => ({
        path: `bodies/leg-${leg}-upper/sites/corner/position`, before: [0, 0], after: [index * 20, 0]
      }))
    }));
    const page = start(fixture(), undefined, {edits: entries});
    await page.flush();
    assert.equal(page.elements["edit-options"].children.length, 1);
    assert.equal(parameterSelect(page).children.length, 3);
    assert.match(text(page.elements["edit-options"]), /shared-leg-move-hinge-site/);
    for (let leg = 0; leg < 4; ++leg) {
      assert.match(text(page.elements["edit-options"]), new RegExp(`leg-${leg}-upper`));
    }
  });

  test("rapid parameter selection skips obsolete queued values and never displays a stale result", async () => {
    let finish;
    const held = new Promise(resolve => {finish = resolve;});
    const page = start(fixture(), undefined, {edits: parameterEdits,
      preview: async request => {if (request.edit === parameterEdits[0].id) await held;}});
    await page.flush();
    page.elements["preview-option"].onclick();
    await page.flush();
    assert.equal(parameterSelect(page).disabled, false);
    assert.equal(page.elements.apply.disabled, true);
    for (const edit of parameterEdits.slice(1)) {
      parameterSelect(page).value = edit.id;
      parameterSelect(page).onchange();
      await page.flush();
    }
    assert.equal(page.requests.length, 1);
    finish();
    await page.flush();
    assert.equal(page.requests[1].edit, parameterEdits[2].id, "Newest selected value takes priority");
    assert.equal(page.elements.edit.value, parameterEdits[2].id);
    assert.equal(page.elements["edit-options"].children[0].dataset.edit, parameterEdits[2].id);
  });

  test("preview errors are visible, retryable and do not block other values", async () => {
    let attempts = 0;
    const page = start(fixture(), undefined, {edits: parameterEdits, preview: async request => {
      if (request.edit === parameterEdits[0].id && ++attempts === 1) {
        throw new Error("Part geometry unavailable");
      }
    }});
    await page.flush();
    page.elements["preview-option"].onclick();
    await page.flush();
    assert.match(text(page.elements["edit-options"]), /Preview failed: Part geometry unavailable/);
    assert.match(page.elements.message.textContent, /Part geometry unavailable/);
    page.elements["preview-option"].onclick();
    await page.flush();
    assert.equal(attempts, 2);
    assert.match(text(page.elements["edit-options"]), /Cached preview ready/);
    assert.equal(page.elements.apply.disabled, false);
  });
test("clearance warnings show first phase and repair scores without claiming physical validation", async () => {
  const report = {suspectPairs: 1, sampledPoses: 97, motorTurnsCovered: 1, refinedIntervals: 8,
    endpointRepeats: true, unmodeledParts: ["pin"], limitations: ["Not a safety certificate."],
    contacts: [{parts: ["receiver", "leg"], clearanceLdu: -3, inputAngleRadians: Math.PI,
      firstInputAngleRadians: Math.PI / 2}]};
  const page = start(fixture(), undefined, {edits: parameterEdits, preview: request => {
    const value = diagramPreview(request);
    value.alternatives[0].clearance = report;
    value.alternatives[0].clearanceRepair = {before: {score: 75}, after: {score: 25}};
    return value;
  }});
  await page.flush();
  page.elements["preview-option"].onclick();
  await page.flush();
  assert.match(text(page.elements["edit-options"]), /Advisory motion clearance: 1 suspect pairs/);
  assert.match(text(page.elements["edit-options"]), /first suspect phase 90.0 degrees/);
  assert.match(text(page.elements["edit-options"]), /score 75.000 -> 25.000/);
  assert.match(text(page.elements["edit-options"]), /not physical collision validation/);
  assert.equal(page.state.nodes.length, fixture().nodes.length);
});

test("tree shows ancestry, highlights selection separately from head and preserves it on reload", async () => {
  const page = start();
  await page.flush();
  const tree = page.elements["version-tree"];
  const item = descendants(tree).find(e => e.tagName === "li" && e.dataset.node === middle);
  assert.ok(descendants(item).some(e => e.tagName === "li" && e.dataset.node === tip));
  assert.match(text(page.elements["contribution-target"]), /original fork: none/);
  await page.select(middle);
  const selected = page.buttons().filter(b => b.attributes["aria-current"] === "true");
  assert.equal(selected.length, 1);
  assert.equal(selected[0].dataset.node, middle);
  assert.match(text(page.buttons().find(b => b.dataset.node === root)), /saved head/);
  assert.equal(page.state.workspaces[0].head, root);
  assert.equal(page.requests.length, 0);
  assert.equal(page.elements["contribute-form"].dataset.node, middle);
  assert.match(text(page.elements["contribution-target"]), /Submitting Version 2/);
  assert.match(text(page.elements["contribution-target"]), /Mutation chain: Version 1.* -> Version 2/);
  assert.equal(new URL(page.location.href).searchParams.get("node"), middle);
  const reload = start(page.state, page.location.href);
  await reload.flush();
  assert.equal(reload.elements.node.value, middle);
  assert.equal(reload.elements["contribute-form"].dataset.node, middle);
  assert.match(reload.elements["contribute-submit"].textContent, /version 2/);
});

test("contribution sends the selected intermediate, never the head or endpoint", async () => {
  const page = start();
  await page.flush();
  await page.select(middle);
  page.elements["contribution-name"].value = "Middle design";
  page.elements["contribute-form"].onsubmit({preventDefault() {}});
  await page.flush();
  const request = page.requests.find(r => r.action === "contribute");
  assert.equal(request.node, middle);
  assert.equal(request.author, "Test");
  assert.equal(page.state.workspaces[0].head, root);
  assert.equal(page.state.contributions[0].node, middle);
  assert.match(text(page.elements["contribution-message"]), /Version 2.*admitted/);
  assert.match(text(page.elements.pool), /Middle design.*Version 2/);
  assert.match(text(page.buttons().find(b => b.dataset.node === middle)), /contributed/);
  await page.select(tip);
  assert.match(page.elements["contribution-message"].textContent, /Version 2.*admitted/);
  const open = descendants(page.elements.pool).find(e => e.tagName === "button" && e.textContent === "Open");
  open.onclick();
  await page.flush();
  assert.equal(page.elements.node.value, middle);
});

test("admission shows stages and prior outcomes while navigation and previews remain available", async () => {
  let finish;
  const paused = new Promise(resolve => finish = resolve);
  const page = start(fixture(), undefined, {contribute: () => paused});
  await page.flush();
  await page.select(middle);
  page.elements["contribution-name"].value = "Exact middle";
  page.elements["contribute-form"].onsubmit({preventDefault() {}});
  await page.flush();
  await page.poll();
  assert.equal(page.elements["contribute-submit"].disabled, true);
  assert.equal(page.elements["preview-option"].disabled, false);
  assert.equal(page.elements.apply.disabled, false);
  assert.equal(page.elements.workspace.disabled, false);
  assert.equal(page.elements.discard.disabled, true);
  assert.match(text(page.elements["contribution-check-items"]), /Saved inputs.*complete.*Reconstruction.*running/);
  assert.match(text(page.elements["contribution-check-items"]), /geometry: warning.*saved construction outcome, not rerun/);
  assert.match(text(page.elements["contribution-check-items"]), new RegExp(middle));
  page.elements["preview-option"].onclick();
  await page.flush();
  assert.ok(page.requests.some(request => request.action === "preview" && request.node === middle));
  await page.select(tip);
  assert.equal(page.elements.discard.disabled, false);
  page.elements["preview-option"].onclick();
  await page.flush();
  const selectedOption = () => descendants(page.elements["edit-options"]).find(node =>
    node.tagName === "button" && node.textContent === "Select this option");
  assert.ok(selectedOption(), "The newly selected version has its own loaded preview");
  finish();
  await page.flush();
  assert.equal(page.state.contributions[0].node, middle);
  assert.equal(page.elements.node.value, tip, "Admission must not steal selection back");
  assert.equal(new URL(page.location.href).searchParams.get("node"), tip);
  assert.ok(selectedOption(), "Background completion must preserve loaded mutation diagrams");
  assert.match(text(page.elements["contribution-check-items"]), /simulator-reconstruction: pass.*this admission check/);
  assert.match(text(page.elements["contribution-message"]), /Version 2.*admitted/);
});

test("failed admission keeps previous outcomes and leaves editing usable", async () => {
  const page = start(fixture(), undefined, {contribute: async () => {throw new Error("Load-only check failed");}});
  await page.flush();
  await page.select(middle);
  page.elements["contribution-name"].value = "Failed check";
  page.elements["contribute-form"].onsubmit({preventDefault() {}});
  await page.flush();
  await page.poll();
  assert.match(text(page.elements["contribution-message"]), /Load-only check failed/);
  assert.match(text(page.elements["contribution-check-items"]), /checking|Reconstruction/);
  assert.match(text(page.elements["contribution-check-items"]), /failed.*geometry: warning/s);
  assert.equal(page.elements["preview-option"].disabled, false);
  assert.equal(page.elements.apply.disabled, false);
  assert.equal(page.state.contributions.length, 0);
});

test("new edit selects its draft, while rejected/discarded versions stay inspectable but not submittable", async () => {
  const page = start();
  await page.flush();
  await page.select(tip);
  page.elements.apply.onclick();
  await page.flush();
  assert.equal(page.requests[0].node, tip);
  assert.equal(page.elements.node.value, id("7"));
  assert.equal(page.elements["contribute-form"].dataset.node, id("7"));
  assert.equal(page.state.nodes.find(n => n.id === id("7")).parent, tip);
  for (const node of [discarded, rejected]) {
    await page.select(node);
    assert.equal(page.elements["contribute-submit"].disabled, true);
    assert.equal(page.elements["contribute-form"].dataset.node, node);
  }
  await page.select(middle);
  assert.equal(page.elements["contribute-submit"].disabled, false);
  assert.equal(page.state.workspaces[0].head, root);
});

test("workspace switching and resume are explicit, while missing URL versions report an error", async () => {
  const page = start();
  await page.flush();
  await page.select(middle);
  page.elements.workspace.value = otherWorkspace;
  page.elements.workspace.onchange();
  await page.flush();
  assert.equal(page.elements.node.value, otherRoot);
  assert.equal(page.elements["contribution-author"].value, "Other test");
  assert.equal(page.buttons().length, 1);
  page.elements.workspace.value = workspace;
  page.elements.workspace.onchange();
  await page.flush();
  await page.select(tip);
  page.elements.resume.onclick();
  await page.flush();
  assert.equal(page.elements.node.value, root);
  assert.match(text(page.elements["contribution-target"]), /original fork/);
  const missing = start(fixture(), `http://example.test/workbench.html?node=${id("f")}`);
  await missing.flush();
  assert.match(missing.elements.message.textContent, /linked version no longer exists/);
  assert.equal(missing.requests.length, 0);
});
