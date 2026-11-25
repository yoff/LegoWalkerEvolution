const assert = require("node:assert/strict");
const {readFileSync} = require("node:fs");
const {resolve} = require("node:path");
const {test} = require("node:test");
const vm = require("node:vm");

const html = readFileSync(resolve(__dirname, "../../src/LegoWalkers.Core/MotionPreview.html"), "utf8");
const script = html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];

function preview(sites) {
  const paths = [];
  const context = {
    setTransform() { paths.length = 0; },
    beginPath() { this.path = []; },
    moveTo(x, y) { this.start = [x, y]; },
    lineTo(x, y) { this.path.push([this.start, [x, y]]); this.start = [x, y]; },
    stroke() { paths.push(...this.path); },
    setLineDash() {}, arc() {}, fill() {}, fillText() {}
  };
  const data = {
    program: {bodies: [{id: "body", grounded: false, sites}], feet: []},
    frames: [0, Math.PI / 2, Math.PI, 3 * Math.PI / 2, 2 * Math.PI].map(angle => ({
      inputAngle: angle, maxResidualLdu: 0,
      poses: [{body: "body", origin: [0, 0], angle}], feet: []
    }))
  };
  const elements = Object.fromEntries(["motion-data", "view", "phase", "status", "play", "program"]
    .map(id => [id, {events: {}, addEventListener(name, callback) { this.events[name] = callback; }}]));
  elements["motion-data"].textContent = JSON.stringify(data);
  elements.view.getContext = () => context;
  elements.view.getBoundingClientRect = () => ({width: 480, height: 480});
  vm.runInNewContext(script, {
    document: {getElementById: id => elements[id]},
    devicePixelRatio: 1, requestAnimationFrame() {}, window: {addEventListener() {}}
  });
  return frame => {
    elements.phase.value = frame;
    elements.phase.events.input();
    return paths.map(([a, b]) => [...a, ...b]);
  };
}

test("rigid triangles retain all three edges as their horizontal site order reverses", () => {
  const draw = preview([
    {id: "pivot", position: {x: 0, y: 0}},
    {id: "upper", position: {x: 40, y: 0}},
    {id: "knee", position: {x: 10, y: 30}}
  ]);
  for (let frame = 0; frame < 4; frame++) {
    const edges = draw(frame);
    assert.equal(edges.length, 3);
    const lengths = edges.map(([x, y, u, v]) => Math.hypot(u - x, v - y)).sort((a, b) => a - b);
    const expected = [Math.hypot(10, 30), 40, Math.hypot(30, 30)].map(length => length * 5);
    lengths.forEach((length, index) => assert.ok(Math.abs(length - expected[index]) < 1e-8));
  }
});

test("two-site links remain a single segment", () => {
  const draw = preview([
    {id: "root", position: {x: 0, y: 0}},
    {id: "tip", position: {x: 40, y: 0}}
  ]);
  for (let frame = 0; frame < 4; frame++) assert.equal(draw(frame).length, 1);
});
