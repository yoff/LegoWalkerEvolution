const assert = require("node:assert/strict");
const {readFileSync} = require("node:fs");
const {resolve} = require("node:path");
const {test} = require("node:test");
const vm = require("node:vm");

const script = readFileSync(resolve(__dirname, "../../simulator/legowalkers_sim/dashboard.js"), "utf8");
const renderCode = script.slice(script.indexOf("    function renderPipeline("), script.indexOf("    async function update("));
const revealCode = script.slice(script.indexOf("    function revealSection("),
  script.indexOf('    addEventListener("hashchange", () => revealSection'));

class Element {
  constructor(tag) {
    this.tagName = tag.toUpperCase();
    this.children = [];
    this.dataset = {};
    this.attributes = {};
    this.textContent = "";
    this.listeners = {};
  }
  append(...children) {
    for (const child of children) { child.parentElement = this; this.children.push(child); }
  }
  setAttribute(name, value) { this.attributes[name] = value; }
  addEventListener(name, listener) { this.listeners[name] = listener; }
  replaceWith(replacement) {
    const siblings = this.parentElement.children;
    siblings[siblings.indexOf(this)] = replacement;
    replacement.parentElement = this.parentElement;
  }
  focus() { this.focused = true; }
  scrollIntoView() { this.scrolled = true; }
}
const all = root => [root, ...root.children.flatMap(all)];
const byId = (root, id) => all(root).find(node => node.id === id);
const make = (key, extra = {}) => ({key, birth: Number(key.split("-")[1]), history: "history/0001.html",
  stage: "screening", name: "Outward L", visuals: {assembly: "designs/exact/assembly-render/walker-v2.png"}, ...extra});

function harness(pipeline) {
  const root = new Element("div");
  const context = vm.createContext({
    document: {createElement: tag => new Element(tag), getElementById: id => byId(root, id)},
    candidatePipeline: root, pipelineNotice: () => null,
    patch: (current, incoming) => {current.children = incoming.children; for (const child of current.children) child.parentElement = current;},
    pipelineCard: () => new Element("li"), startupCard: () => new Element("li"),
    previewStatus: {}, previewError: {}, previewLog: {}, previewRetry: {},
  });
  vm.runInContext(revealCode + renderCode, context);
  context.renderPipeline({campaign: true, version: "v0052", pipeline,
    assemblyPreviews: {available: true, counts: {}}});
  return {root, context};
}

test("overview links to compact stages, thumbnails to scoped details, and empty stages remain navigable", () => {
  const walkers = [make("birth-1"), make("birth-2", {visuals: {diagram: "designs/other/candidate/cycle.svg"}}),
    make("birth-3", {visuals: {}, name: "<script>literal name</script>"})];
  const {root, context} = harness({stages: [
    {id: "screening", label: "Screening", count: 3, items: walkers.slice(0, 1), walkers},
    {id: "ready", label: "Ready for simulation", count: 0, items: [], walkers: []},
  ], recent: [], finished: {count: 0, walkers: []}});
  const summary = all(root).find(node => node.className === "pipeline-summary");
  assert.equal(summary.children[0].href, "#pipeline-board-screening");
  const lane = byId(root, "pipeline-board-screening");
  const rows = all(lane).filter(node => node.className === "pipeline-mini");
  assert.equal(rows.length, 3);
  assert.equal(rows[0].href, "#pipeline-card-screening-birth-1");
  assert.equal(rows[1].href, "/versions/v0052/history/0001.html");
  assert.equal(all(rows[0]).find(node => node.tagName === "IMG").src,
    "/versions/v0052/designs/exact/assembly-render/walker-v2.png");
  assert.equal(all(rows[0]).find(node => node.tagName === "IMG").dataset.previewKind, "assembly");
  assert.equal(all(rows[1]).find(node => node.tagName === "IMG").dataset.previewKind, "diagram");
  assert.equal(all(lane).filter(node => node.className === "pipeline-outcome").length, 0);
  assert.match(all(rows[1]).find(node => node.tagName === "IMG").alt, /Solved diagram/);
  assert(all(rows[2]).some(node => node.textContent === "No preview"));
  assert(all(rows[2]).some(node => node.textContent === "<script>literal name</script>"));
  assert.equal(all(root).filter(node => ["VIDEO", "CANVAS"].includes(node.tagName)).length, 0);
  const headingLink = all(lane).find(node => node.tagName === "A");
  assert.equal(headingLink.href, "#pipeline-screening");
  assert(byId(root, "pipeline-card-screening-birth-1"));
  const empty = byId(root, "pipeline-board-ready");
  const details = empty.parentElement.parentElement;
  assert.equal(details.tagName, "DETAILS");
  assert(!details.open);
  assert(context.revealSection("#pipeline-board-ready"));
  assert(details.open && empty.scrolled && empty.focused);
  assert(context.revealSection("#pipeline-ready"));
  assert(byId(root, "pipeline-ready").scrolled);
  const image = all(rows[0]).find(node => node.tagName === "IMG");
  image.listeners.error();
  assert(!all(rows[0]).includes(image));
  assert(all(rows[0]).some(node => node.textContent === "No preview"));
});

test("startup contributions and finished walkers have compact rows linked to their actual stage", () => {
  const admission = {key: "admit-L", label: "Outward L", kind: "admission", status: "running",
    birth: null, history: null};
  const completed = {key: "already-admitted", label: "Now queued", kind: "admission", status: "complete", birth: 5};
  const preparation = {key: "prepare", label: "Preparing immutable candidates", kind: "startup", status: "complete"};
  const finished = make("birth-33", {name: "Birth 33", stage: "rejected", parentPreview: true});
  const {root, context} = harness({
    startup: {tracked: true, items: [preparation, admission, completed], counts: {complete: 2, running: 1}},
    stages: [], recent: [finished], finished: {count: 1, walkers: [finished]},
  });
  const admissionRows = all(byId(root, "pipeline-board-startup")).filter(node => node.className === "pipeline-mini");
  assert.equal(admissionRows.length, 1);
  assert(byId(root, "pipeline-card-startup-already-admitted"));
  assert.equal(admissionRows[0].href, "#pipeline-card-startup-admit-L");
  assert(byId(root, "pipeline-card-startup-admit-L"));
  const recent = all(byId(root, "pipeline-board-recent")).find(node => node.className === "pipeline-mini");
  assert.equal(recent.href, "#pipeline-card-recent-birth-33");
  assert.match(all(recent).find(node => node.tagName === "IMG").alt, /Parent:/);
  const details = byId(root, "pipeline-recent");
  details.open = false;
  assert(context.revealSection(recent.href));
  assert(details.open);
  assert.equal(byId(root, "pipeline-card-recent-birth-33").focused, true);
});

test("legacy snapshots use their bounded items rather than inventing unseen walkers", () => {
  const item = make("birth-1");
  const {root} = harness({stages: [{id: "queued", label: "Queued", count: 9, items: [item]}], recent: []});
  const rows = all(byId(root, "pipeline-board-queued")).filter(node => node.className === "pipeline-mini");
  assert.equal(rows.length, 1);
  assert(all(root).some(node => node.textContent === "Queued / awaiting processing (9)"));
});

test("initial queue precedes planning and building across overview, board and details", () => {
  const stages = ["planning", "building", "queued", "screening"].map(id => ({
    id, label: id === "queued" ? "Pending / awaiting screening" : id,
    count: 0, items: [], walkers: [],
  }));
  const {root} = harness({stages, recent: []});
  const order = ["queued", "planning", "building", "screening"];
  const summary = all(root).find(node => node.className === "pipeline-summary");
  assert.deepEqual(summary.children.slice(0, 4).map(node => node.href),
    order.map(id => `#pipeline-board-${id}`));
  assert.equal(summary.children[0].textContent, "Queued / awaiting processing: 0");
  assert.deepEqual(all(root).filter(node => order.some(id => node.id === `pipeline-board-${id}`))
    .map(node => node.id), order.map(id => `pipeline-board-${id}`));
  assert.deepEqual(root.children.filter(node => order.some(id => node.id === `pipeline-${id}`))
    .map(node => node.id), order.map(id => `pipeline-${id}`));
  assert(!all(root).some(node => node.textContent.includes("Pending / awaiting screening")));
  assert.deepEqual(stages.map(stage => stage.id), ["planning", "building", "queued", "screening"]);
});

test("admission overflow without a birth links to its stage rather than a null history", () => {
  const admission = make("admission-extra", {
    birth: null, history: null, name: "Contribution still building", visuals: {},
  });
  const {root} = harness({stages: [
    {id: "building", label: "Building", count: 1, items: [], walkers: [admission]},
  ], recent: []});
  const row = all(byId(root, "pipeline-board-building")).find(node => node.className === "pipeline-mini");
  assert.equal(row.href, "#pipeline-building");
  assert(all(row).some(node => node.textContent === admission.name));
  assert(byId(root, "pipeline-building"));
});

test("recent outcome symbols use recorded qualification and distinguish rejection, failure and unknown results", () => {
  const outcomes = [
    {stage: "complete", qualification: {qualified: true}, expected: "qualified", label: "Qualified"},
    {stage: "rejected", expected: "rejected", label: "Rejected"},
    {stage: "screening-rejected", expected: "rejected", label: "Rejected"},
    {stage: "complete", qualification: {qualified: false}, expected: "unqualified", label: "Evaluated, not qualified"},
    {stage: "failed", expected: "failed", label: "Failed"},
    {stage: "interrupted", expected: "interrupted", label: "Interrupted"},
    {stage: "complete", expected: "unknown", label: "No recorded qualification outcome"},
  ];
  const walkers = outcomes.map((extra, index) => make("birth-" + index, extra));
  const {root} = harness({stages: [], recent: walkers, finished: {count: walkers.length, walkers}});
  const rows = all(byId(root, "pipeline-board-recent")).filter(node => node.className === "pipeline-mini");
  for (const [index, row] of rows.entries()) {
    const badge = all(row).find(node => node.className === "pipeline-outcome");
    assert.equal(badge.dataset.outcome, outcomes[index].expected);
    assert.equal(badge.attributes.role, "img");
    assert.equal(badge.attributes["aria-label"], outcomes[index].label);
    assert(row.title.includes(outcomes[index].label));
  }
  const first = all(rows[0]).find(node => node.className === "pipeline-outcome");
  const second = all(rows[1]).find(node => node.className === "pipeline-outcome");
  assert.notEqual(first.textContent, second.textContent);
  const image = all(rows[0]).find(node => node.tagName === "IMG");
  image.listeners.error();
  assert(all(rows[0]).includes(first));
});
