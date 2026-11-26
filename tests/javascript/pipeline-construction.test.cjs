const assert = require("node:assert/strict");
const {readFileSync} = require("node:fs");
const {resolve} = require("node:path");
const {test} = require("node:test");
const vm = require("node:vm");

const script = readFileSync(resolve(__dirname, "../../simulator/legowalkers_sim/dashboard.js"), "utf8");
const cardCode = script.slice(script.indexOf("    function constructionLive("), script.indexOf("    function startupCard("));
class Element {
  constructor(tag) { this.tagName = tag; this.children = []; this.dataset = {}; this.textContent = ""; }
  append(...children) { this.children.push(...children); }
  setAttribute(name, value) { (this.attributes ||= {})[name] = value; }
}
const nodes = root => [root, ...root.children.flatMap(nodes)];
const text = root => nodes(root).map(node => node.textContent).join(" ");
function render(trace) {
  const context = vm.createContext({document: {createElement: tag => new Element(tag),
    createElementNS: (_, tag) => new Element(tag)}});
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

test("live construction shows actual proof, scoped geometry, conflicts and stopped snapshot status", () => {
  const box = (part, low, high) => ({part, body:"chassis",
    low:{x:low,y:low,z:low}, high:{x:high,y:high,z:high}});
  const live = {workerRunning:false, status:"productions-exhausted", updated:new Date().toISOString(),
    detail:"No witness", authority:"Partial sampled constraints; no physics.", openGoals:[{id:"oracle"}],
    search:{visited:16, maximumStates:512, frontier:1, pruned:3},
    proof:{goals:[{id:"support",kind:"mount",requires:[],external:false},
      {id:"oracle",kind:"exact-admission",requires:["support"],external:true}],
      steps:[{goal:"support",rule:"two-beam-route",choice:"a".repeat(64)}]},
    inspection:{goal:"support",rule:"route-clearance",checked:12,rejected:8,cacheHits:2,
      fixed:[box("frame",0,10)], reserved:[box("moving",-100,100)],
      candidate:[box("<candidate>",20,30)],
      conflicts:[{part:"<candidate>",obstacle:"moving",body:"leg",inputAngleRadians:1.25}]}};
  const card = render({live, tracked:true, events:[], counts:null, operation:null,
    liveReport:"motion-builds/000001/attempt-001/construction-live.json",
    eventLog:"motion-builds/000001/attempt-001/construction-events.jsonl",
    attempts:[{directory:"attempt-003",placement:"inherited",layers:1,status:"duplicate-request",requestId:"a".repeat(64)}]});
  assert.match(text(card), /Saved snapshot; worker stopped/);
  assert.match(text(card), /16\/512 states/);
  assert.match(text(card), /12 placement checks, 8 rejected, 2 route-cache hits/);
  assert.match(text(card), /two-beam-route/);
  assert.match(text(card), /awaiting external verification/);
  assert.match(text(card), /<candidate> \/ moving at input 1.250 rad/);
  assert.match(text(card), /duplicate-request/);
  const svgs = nodes(card).filter(node => node.tagName === "svg");
  assert.equal(svgs.length, 2);
  assert.equal(svgs[0].attributes.viewBox, "-120 -120 240 240");
  const rectangles = nodes(svgs[0]).filter(node => node.tagName === "rect");
  assert.equal(rectangles.length, 3);
  assert.equal(rectangles[2].attributes.x, "20");
  assert.equal(rectangles[2].attributes.fill, "#ff7979");
  assert.equal(rectangles[0].attributes["stroke-dasharray"], "4 3");
  assert.equal(nodes(card).find(node => node.textContent === "Latest proof and constraint snapshot (JSON)").href,
    "/versions/v0042/motion-builds/000001/attempt-001/construction-live.json");
  assert(!nodes(card).some(node => node.tagName === "script"));
});
