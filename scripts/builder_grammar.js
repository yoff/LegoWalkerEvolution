(() => {
  "use strict";
  const $ = id => document.getElementById(id);
  const reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
  const svg = content => `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 560 240" role="img">${content}</svg>`;
  const text = (x, y, value, color = "#d7e3f5") => `<text x="${x}" y="${y}" fill="${color}" font-size="14">${value}</text>`;
  const beam = (a, b) => `<path d="M${a} L${b}" fill="none" stroke="#879cba" stroke-width="18" stroke-linecap="round"/>`;
  const circle = (x, y, r, color) => `<circle cx="${x}" cy="${y}" r="${r}" fill="none" stroke="${color}" stroke-width="3"/>`;
  const cards = [...document.querySelectorAll(".rule")];
  $("rule-filter").onchange = event => {
    cards.forEach(card => { card.hidden = !!event.target.value && card.dataset.category !== event.target.value; });
    $("rule-count").textContent = `${cards.filter(card => !card.hidden).length} rules`;
  };

  $("pin-demo").innerHTML = `<div class="controls"><label>Pin stock <select id="pin-stock"><option value="2">4459 · 2L friction pin</option><option value="3">6558 · 3L friction pin</option></select></label><label>Socket-center gap <select id="pin-gap"><option value="20">20 LDU</option><option value="40">40 LDU</option></select></label></div><div id="pin-image"></div><output id="pin-result"></output>`;
  function pins() {
    const length = Number($("pin-stock").value) * 20, gap = Number($("pin-gap").value);
    const pass = length >= gap + 20, color = pass ? "#56d9bf" : "#ffbf69";
    $("pin-image").innerHTML = svg(`<title>Stock friction pin coverage for two modeled socket walls</title><g fill="#879cba"><rect x="${280-gap*1.5-30}" y="55" width="60" height="115" rx="7"/><rect x="${280+gap*1.5-30}" y="55" width="60" height="115" rx="7"/></g><path d="M${280-length*1.5} 112 H${280+length*1.5}" stroke="${color}" stroke-width="15" stroke-linecap="round"/>` + text(145, 205, `${length} LDU stock; ${gap+20} LDU wall span`, color));
    $("pin-result").textContent = pass ? "Covered in this two-wall schematic. Other contacts and connector overlaps still need checking." : "Rejected: stock does not cover both walls. Select a longer pin or change the route.";
  }
  $("pin-stock").onchange = pins; $("pin-gap").onchange = pins; pins();

  $("support-demo").innerHTML = `<div class="controls"><label>Route length <select id="support-count"><option>2</option><option>3</option><option>4</option><option>5</option></select></label></div><div id="support-image"></div><output id="support-result"></output>`;
  function supports() {
    const n = Number($("support-count").value), pts = [[60, 145], [150, 70], [250, 145], [330, 90], [410, 145], [490, 90]];
    let content = "<title>Topological support chain; each new beam adds a rigid-path obligation</title>";
    for (let i = 0; i < n; i++) content += beam(pts[i], pts[i+1]);
    for (let i = 0; i <= n; i++) content += `<circle cx="${pts[i][0]}" cy="${pts[i][1]}" r="7" fill="#56d9bf"/>`;
    content += text(35, 205, "Frame") + text(300, 225, `${n} beams = ${n} frame-path goals`);
    $("support-image").innerHTML = svg(content);
    $("support-result").textContent = "Topology only, not a to-scale legal route. The builder solves catalogue spans and pin coverage; extensions add stock 3L beams, not stretched links.";
  }
  $("support-count").onchange = supports; supports();

  $("gear-demo").innerHTML = `<div class="controls"><label>Root mesh <select id="gear-ratio"><option value="1">24T : 24T · 1:1</option><option value="5">8T : 40T · 5:1</option></select></label><button id="gear-toggle" type="button">Play gears</button></div><div id="gear-image"></div><output id="gear-result"></output>`;
  let gearAngle = 0, gearPlaying = false, sweepPlaying = false, previous = null;
  function gear(cx, radius, angle) {
    return `<g transform="translate(${cx},110) rotate(${angle})"><circle r="${radius}" fill="#243b55" stroke="#ffbf69" stroke-width="5"/><path d="M0 0 H${radius}" stroke="#56d9bf" stroke-width="4"/><circle r="7" fill="#0d1727"/></g>`;
  }
  function gears() {
    const reduction = Number($("gear-ratio").value), input = reduction === 5 ? 8 : 24, output = reduction === 5 ? 40 : 24;
    // Pitch circles share the same scale: 8T + 40T and 24T + 24T both span 60 LDU.
    $("gear-image").innerHTML = svg("<title>Ideal external spur gear pitch circles and counter-rotation, not tooth geometry</title>" +
      gear(170, input*2.5, gearAngle) + gear(290, output*2.5, -gearAngle/reduction) +
      text(70, 215, `${input}T input / ${output}T output · center spacing 60 LDU`));
    $("gear-result").textContent = reduction === 5 ? "Output turns 1/5 as fast, in the opposite direction. Ideal torque multiplication is 5× before losses; motor cap stays 0.4 Nm." : "Equal speed, opposite rotation. These circles show ideal pitch geometry, not a meshing/collision certificate.";
  }
  $("gear-ratio").onchange = gears;
  $("gear-toggle").onclick = () => { gearPlaying = !gearPlaying; $("gear-toggle").textContent = gearPlaying ? "Pause gears" : "Play gears"; };
  gears();

  function sweep(angle) {
    const a = [110, 160], d = [290, 160], theta = angle * Math.PI / 180;
    const b = [a[0]+40*Math.cos(theta), a[1]+40*Math.sin(theta)];
    const dx = d[0]-b[0], dy = d[1]-b[1], distance = Math.hypot(dx, dy);
    const along = (160*160-120*120+distance*distance)/(2*distance);
    const height = Math.sqrt(160*160-along*along);
    const c = [b[0]+along*dx/distance-height*dy/distance, b[1]+along*dy/distance+height*dx/distance];
    const foot = [b[0]+1.15*(c[0]-b[0]), b[1]+1.15*(c[1]-b[1])];
    $("sweep-drawing").innerHTML = '<path d="M65 290 H465" stroke="#30435f"/>' +
      `<circle cx="${a[0]}" cy="${a[1]}" r="52" fill="#ffbf6910" stroke="#ffbf69" stroke-dasharray="6 5"/>` +
      beam(a, d) + beam(a, b) + beam(b, c) + beam(c, d) +
      `<path d="M${c} L${foot}" stroke="#56d9bf" stroke-width="12"/>` +
      '<path d="M55 105 V230 H75" fill="none" stroke="#56d9bf" stroke-width="12"/>' +
      '<path d="M110 90 V130" stroke="#ffbf69" stroke-width="12" stroke-dasharray="4 4"/>' +
      [a,b,c,d].map(([x,y]) => circle(x,y,6,"#0d1727")).join("") +
      text(335, 70, "Amber: crank keepout") + text(335, 95, "Mint: outside support") +
      text(330, 122, "Dashed stub: risky support");
    $("sweep-status").textContent = `${Math.round(angle)}° · advisory sweep`;
  }
  $("sweep-phase").oninput = event => { sweepPlaying = false; $("sweep-toggle").textContent = "Play schematic"; sweep(Number(event.target.value)); };
  $("sweep-toggle").onclick = () => { sweepPlaying = !sweepPlaying; $("sweep-toggle").textContent = sweepPlaying ? "Pause schematic" : "Play schematic"; };
  sweep(0);
  function animate(now) {
    const dt = previous === null ? 0 : Math.min(now-previous, 100) / 1000; previous = now;
    if (!document.hidden && gearPlaying) { gearAngle = (gearAngle+dt*90)%1800; gears(); }
    if (!document.hidden && sweepPlaying) { $("sweep-phase").value = (Number($("sweep-phase").value)+dt*60)%360; sweep(Number($("sweep-phase").value)); }
    requestAnimationFrame(animate);
  }
  // Reduced-motion users get static diagrams until they explicitly press a play control.
  if (!reduced) { gearPlaying = true; $("gear-toggle").textContent = "Pause gears"; }
  requestAnimationFrame(animate);

  let step = -1;
  const messages = [
    "Choose a legal support route within the frozen request and bounds.",
    "Build the witness; record the beams, pin bonds and new frame-path goals.",
    "Exact check reports a support/brace collision. This full witness is rejected.",
    "Use part provenance to reopen the implicated support choice, preserving unrelated decisions.",
    "Try the alternative and check again. In this schematic example it passes load-only checks; walking still needs a fresh trial."
  ];
  $("search-next").onclick = () => {
    step = (step+1)%5;
    document.querySelectorAll(".search-step").forEach((node, index) => {
      node.classList.toggle("active", index === step);
      node.classList.toggle("reject", index === step && step === 2);
    });
    $("search-status").textContent = messages[step];
  };
  $("search-reset").onclick = () => {
    step = -1;
    document.querySelectorAll(".search-step").forEach(node => node.classList.remove("active", "reject"));
    $("search-status").textContent = "Illustrative search trace; no construction or trial is running.";
  };
})();
