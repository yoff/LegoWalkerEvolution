const assert = require("node:assert/strict");
const {readFileSync} = require("node:fs");
const {resolve} = require("node:path");
const {test} = require("node:test");
const vm = require("node:vm");

const script = readFileSync(resolve(__dirname, "../../simulator/legowalkers_sim/motion_diagrams.js"), "utf8");

function motion() {
  const bodies = ["near", "far"].map(id => ({
    id, grounded: false, sites: [
      {id: "pivot", position: {x: 0, y: 0, z: 0}},
      {id: "tip", position: {x: 20, y: 10, z: 0}}
    ]
  }));
  return {
    mode: "planar-kinematics-not-dynamics",
    program: {bodies, hinges: [{a: {body: "near", site: "pivot"}, b: {body: "far", site: "pivot"}}],
      feet: bodies.map(body => ({id: `${body.id}-foot`, port: {body: body.id, site: "tip"}}))},
    displayDepths: {bodies: {near: -100, far: 100}, feet: {"near-foot": -100, "far-foot": 100}},
    frames: [0, Math.PI, 2 * Math.PI].map(angle => ({
      inputAngle: angle,
      poses: bodies.map(body => ({body: body.id, origin: [0, 0, 0], angle})),
      feet: bodies.map(body => ({id: `${body.id}-foot`,
        position: [20 * Math.cos(angle) - 10 * Math.sin(angle), 20 * Math.sin(angle) + 10 * Math.cos(angle), 0]}))
    }))
  };
}

async function player(data = motion(), initialSeek, size = {width: 600, height: 300}) {
  const drawing = {
    paths: [], circles: [], labels: [],
    setTransform() {},
    clearRect() { this.paths = []; this.circles = []; this.labels = []; },
    beginPath() { this.path = []; },
    moveTo(x, y) { this.path.push([x, y]); },
    lineTo(x, y) { this.path.push([x, y]); },
    stroke() { this.paths.push({points: [...this.path], colour: this.strokeStyle}); },
    setLineDash() {},
    arc(x, y, radius) { this.circle = {x, y, radius}; },
    fill() { this.circles.push({...this.circle, colour: this.fillStyle}); },
    strokeText() {},
    fillText(text, x, y) { this.labels.push({text, x, y, colour: this.fillStyle}); }
  };
  class Element {
    constructor(tag) {
      this.tagName = tag; this.children = []; this.dataset = {}; this.style = {};
      this.attributes = {}; this.events = {}; this.isConnected = true; this.textContent = "";
    }
    append(...children) { children.forEach(child => child.parent = this); this.children.push(...children); }
    remove() { this.parent.children = this.parent.children.filter(child => child !== this); }
    setAttribute(name, value) { this.attributes[name] = value; }
    addEventListener(name, callback) { this.events[name] = callback; }
    removeEventListener(name) { delete this.events[name]; }
    fire(name, event = {}) { this.events[name](event); }
    querySelector(tag) { return all(this).find(node => node !== this && node.tagName === tag); }
    getBoundingClientRect() { return size; }
    getContext() {
      if (!this.context) this.context = {...drawing, paths: [], circles: [], labels: []};
      return this.context;
    }
  }
  const all = node => [node, ...node.children.flatMap(all)];
  const root = new Element("div"), link = new Element("a");
  root.dataset.motionSource = "/motion.json"; link.href = "/cycle.svg"; root.append(link);
  if (initialSeek !== undefined) root.dataset.motionSeekAngle = String(initialSeek);
  const requests = new Map(), observers = [], events = {}, reduced = {matches: false, addEventListener() {}};
  let nextRequest = 0;
  const document = {
    hidden: false, documentElement: {},
    createElement: tag => new Element(tag),
    querySelectorAll: () => [root],
    addEventListener: (name, callback) => { events[name] = callback; }
  };
  const context = vm.createContext({
    document, location: {protocol: "http:"}, devicePixelRatio: 1,
    matchMedia: () => reduced,
    fetch: async () => ({ok: true, json: async () => data}),
    requestAnimationFrame: callback => { requests.set(++nextRequest, callback); return nextRequest; },
    cancelAnimationFrame: id => requests.delete(id),
    IntersectionObserver: class {
      constructor(callback) { this.callback = callback; observers.push(this); }
      observe(target) { this.callback([{target, isIntersecting: true}]); }
      unobserve() {}
    },
    ResizeObserver: class { observe() {} disconnect() {} },
    MutationObserver: class { observe() {} }
  });
  vm.runInContext(script, context);
  await new Promise(setImmediate);
  const elements = all(root);
  const controls = {
    view: elements.find(node => node.tagName === "select"),
    button: elements.find(node => node.tagName === "button"),
    slider: elements.find(node => node.tagName === "input"),
    status: elements.find(node => node.tagName === "output"),
    canvas: elements.find(node => node.tagName === "canvas" && node.attributes["aria-label"].startsWith("Animated solved"))
  };
  const text = () => all(root).map(node => node.textContent).join(" ");
  const mainDrawing = controls.canvas?.context;
  const topCanvas = elements.find(node => node.tagName === "canvas" && node !== controls.canvas);
  return {
    ...controls, drawing: mainDrawing || drawing, topCanvas, topDrawing: topCanvas?.context, requests, root, document, text,
    feet: () => mainDrawing.circles.filter(circle => circle.radius === 5),
    visible: value => observers[0].callback([{target: root, isIntersecting: value}]),
    choose: value => { controls.view.value = value; controls.view.fire("change"); }
  };
}

test("top view sits above the linkage and aligns numbered feet across projections and phases", async () => {
  const p = await player();
  assert.ok(p.topCanvas, p.text());
  const topFeet = () => p.topDrawing.circles.filter(circle => circle.radius === 5);
  assert.deepEqual(p.topDrawing.labels.map(label => label.text), p.drawing.labels.map(label => label.text));
  const views = p.root.children.find(node => node.className === "motion-views");
  assert.equal(views.children[0].children[0], p.topCanvas);
  assert.equal(views.children[1].children[0], p.canvas);
  assert.notEqual(topFeet()[0].x, topFeet()[1].x);
  assert.notEqual(topFeet()[0].y, topFeet()[1].y);
  const initial = topFeet().map(foot => foot.x);
  p.slider.value = "1"; p.slider.fire("input");
  assert.notDeepEqual(topFeet().map(foot => foot.x), initial);
  for (const view of ["front", "oblique"]) {
    p.choose(view);
    for (const phase of ["0", "1", "2"]) {
      p.slider.value = phase; p.slider.fire("input");
      assert.deepEqual(topFeet().map(foot => foot.x), p.feet().map(foot => foot.x));
      assert.notEqual(topFeet()[0].y, topFeet()[1].y);
    }
  }
  assert.deepEqual(p.topDrawing.labels.map(label => label.colour), p.drawing.labels.map(label => label.colour));
  assert.match(p.text(), /Aligned top: X \+ 0.45Z/);
  assert.match(p.text(), /horizontally sheared for alignment/);
  p.choose("front");
  assert.match(p.text(), /Top: X forward \/ Z lateral/);
  assert.equal(topFeet()[0].x, topFeet()[1].x);
});

test("physical depths separate coincident planar feet; flat view restores exact overlap", async () => {
  const p = await player();
  assert.ok(p.canvas, p.text());
  assert.equal(p.view.value, "oblique");
  const [far, near] = p.feet();
  assert.notEqual(far.x, near.x);
  assert.notEqual(far.y, near.y);
  assert.notEqual(far.colour, near.colour);
  assert.deepEqual(p.drawing.labels.map(label => label.text), ["2", "1"]);
  for (const foot of [far, near]) {
    assert.ok(p.drawing.paths.some(path => path.colour === foot.colour && path.points.length === 3));
    assert.ok(p.drawing.circles.some(circle => circle.radius === 3 && circle.colour === foot.colour));
  }
  assert.match(p.text(), /near-foot \(layer -100 LDU\)/);
  assert.match(p.text(), /far-foot \(layer 100 LDU\)/);
  p.choose("front");
  const flat = p.feet();
  assert.equal(flat[0].x, flat[1].x);
  assert.equal(flat[0].y, flat[1].y);
  assert.match(p.text(), /compare foot heights here/);
});

test("diagnostic phase seeking pauses at the nearest solved frame and supports lazy loading", async () => {
  const p = await player();
  p.root.fire("motion-seek", {detail: {inputAngleRadians: Math.PI}});
  assert.equal(p.slider.value, 1);
  assert.equal(p.button.textContent, "Play");
  assert.equal(p.status.textContent, "180 deg");
  const queued = await player(motion(), Math.PI);
  assert.equal(queued.slider.value, 1);
  assert.equal(queued.button.textContent, "Play");
  p.root.fire("motion-seek", {detail: {inputAngleRadians: 20}});
  assert.match(p.text(), /outside this solved diagram/);
  assert.equal(p.slider.value, 1);
});

test("changing projection preserves playback, scrubbed phase and pause state", async () => {
  const p = await player();
  assert.equal(p.button.textContent, "Pause");
  p.choose("front");
  assert.equal(p.button.textContent, "Pause");
  assert.equal(p.requests.size, 1);
  p.slider.value = "1"; p.slider.fire("input");
  assert.equal(p.button.textContent, "Play");
  assert.equal(p.status.textContent, "180 deg");
  assert.equal(p.requests.size, 0);
  p.choose("oblique");
  assert.equal(p.slider.value, 1);
  assert.equal(p.status.textContent, "180 deg");
  assert.equal(p.button.textContent, "Play");
  assert.equal(p.requests.size, 0);
});

test("off-screen stopping and visibility recovery retain the selected projection", async () => {
  const p = await player();
  p.choose("front");
  p.visible(false);
  assert.equal(p.requests.size, 0);
  p.visible(true);
  assert.equal(p.view.value, "front");
  assert.equal(p.requests.size, 1);
  assert.equal(p.feet()[0].x, p.feet()[1].x);
});

test("stacked views share horizontal alignment and fit extreme depths at narrow and wide sizes", async () => {
  const data = motion();
  data.displayDepths.bodies = {near: -10000, far: 10000};
  data.displayDepths.feet = {"near-foot": -10000, "far-foot": 10000};
  for (const size of [{width: 180, height: 200}, {width: 600, height: 300}, {width: 1200, height: 300}]) {
    const p = await player(data, undefined, size);
    for (const view of ["oblique", "front"]) {
      p.choose(view);
      const topFeet = p.topDrawing.circles.filter(circle => circle.radius === 5);
      assert.deepEqual(topFeet.map(foot => foot.x), p.feet().map(foot => foot.x));
      for (const foot of [...p.feet(), ...topFeet]) {
        assert.ok(foot.x >= 29 && foot.x <= size.width - 29, JSON.stringify(foot));
        assert.ok(foot.y >= 29 && foot.y <= size.height - 29, JSON.stringify(foot));
      }
    }
  }
});

test("solved 3D coordinates remain usable when no physical-layer augmentation is present", async () => {
  const data = motion();
  delete data.displayDepths;
  for (const frame of data.frames) {
    frame.poses[0].origin[2] = frame.feet[0].position[2] = -100;
    frame.poses[1].origin[2] = frame.feet[1].position[2] = 100;
  }
  const p = await player(data);
  assert.ok(p.canvas, p.text());
  assert.notEqual(p.feet()[0].x, p.feet()[1].x);
});

test("invalid motion retains its static fallback and exposes a readable error", async () => {
  const p = await player({mode: "invalid"});
  assert.equal(p.canvas, undefined);
  assert.match(p.text(), /Animated diagram unavailable: Unsupported solved motion data/);
  assert.notEqual(p.root.querySelector("a").hidden, true);
});
