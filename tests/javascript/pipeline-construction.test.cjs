const assert = require("node:assert/strict");
const {readFileSync} = require("node:fs");
const {resolve} = require("node:path");
const {test} = require("node:test");
const vm = require("node:vm");

const script = readFileSync(resolve(__dirname, "../../simulator/legowalkers_sim/dashboard.js"), "utf8");
const cardCode = script.slice(script.indexOf("    function pipelineCard("), script.indexOf("    function startupCard("));
class Element {
  constructor(tag) { this.tagName = tag; this.children = []; this.dataset = {}; this.textContent = ""; }
  append(...children) { this.children.push(...children); }
}
const nodes = root => [root, ...root.children.flatMap(nodes)];
const text = root => nodes(root).map(node => node.textContent).join(" ");
function render(trace) {
  const context = vm.createContext({document: {createElement: tag => new Element(tag)}});
  vm.runInContext(cardCode, context);
  return context.pipelineCard({key: "proposal-3", proposal: 3, birth: 1, category: "4-feet",
    stage: "building", history: "forest.html#proposal-3", visuals: {}, parentPreview: true, construction: trace}, "/versions/v0042/");
}
test("planning trace exposes rejected builder attempts and correctly scoped report links", () => {
  const card = render({tracked: true, operation: "gear-reduction", counts: {
    realizationAttempts: 2, exported: 1, rejected: 1, duplicates: 0},
    events: [{stage: "building", status: "rejected", attempt: 1, detail: "<unsafe> mesh"}],
    report: "proposals/000003/candidates.json"});
  assert.match(text(card), /Planning and builder attempts/);
  assert.match(text(card), /attempt 1: <unsafe> mesh/);
  assert.match(text(card), /Builder attempts: 2; exported: 1; rejected: 1/);
  assert.match(text(card), /not simulation trials/);
  assert.equal(nodes(card).find(node => node.textContent === "Recorded proposal / construction report (JSON)").href,
    "/versions/v0042/proposals/000003/candidates.json");
});
test("legacy proposals explicitly distinguish saved results from live planning", () => {
  const card = render({tracked: false, events: [], operation: null, counts: null, report: null});
  assert.match(text(card), /no live planning trace/);
  assert.match(text(card), /no stages are inferred/);
});
