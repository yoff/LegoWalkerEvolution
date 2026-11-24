(() => {
  "use strict";
  const $ = id => document.getElementById(id);
  let state, selected, edits = [], token, busy = false, contributing = false, catalogRequest = 0;
  let checks = [], pendingContribution = null;
  const optionPreviews = new Map();
  const previewErrors = new Map();
  let previewRequest = null, previewSelection = 0;
  let contributionWorkspace;
  const element = (tag, text) => {
    const node = document.createElement(tag);
    if (text !== undefined) node.textContent = text;
    return node;
  };
  const message = (text, error = false) => {
    $("message").textContent = text;
    $("message").classList.toggle("error", error);
  };
  async function json(url, options) {
    const response = await fetch(url, options);
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || `Request failed (${response.status})`);
    return value;
  }
  async function action(action, values = {}) {
    if (busy) throw new Error("Wait for the current action to finish.");
    const foreground = action !== "contribute";
    if (!foreground && contributing) throw new Error("A contribution is already being submitted; editing remains available.");
    if (foreground) busy = true;
    else contributing = true;
    updateControls();
    if (foreground) message(action === "preview" ? "Building whole-model option diagrams for the selected version; no branches or simulation steps..." :
      action === "render" ? "Preparing a static LEGO assembly preview; this can take a few minutes..." :
      "Working; no simulation is being authorized...");
    try {
      const status = await json("/api/status");
      token = status.csrfToken;
      return await json("/api/workbench", {method: "POST", headers: {"Content-Type": "application/json"},
        body: JSON.stringify({csrfToken: token, action, ...values})});
    } finally {
      if (foreground) busy = false;
      else contributing = false;
      updateControls();
    }
  }
  function updateControls() {
    for (const control of document.querySelectorAll("button,input,select,textarea")) {
      if (!control.closest(".motion-player")) control.disabled = busy;
    }
    updateContributionButton();
    $("apply").disabled = busy || !!previewRequest;
    for (const control of document.querySelectorAll("button")) {
      if (control.dataset.previewApply) control.disabled = busy || !!previewRequest;
    }
    if (selected && checks.some(check => check.status === "running" && check.node === selected.id)) {
      $("discard").disabled = true;
      $("discard").title = "This exact version is being checked. Other versions can still be edited or discarded.";
    } else $("discard").title = "";
  }
  function options(select, entries, value) {
    select.replaceChildren(...entries.map(([id, label]) => new Option(label, id)));
    if (entries.some(([id]) => id === value)) select.value = value;
  }
  async function refresh(workspace, node) {
    state = await json("/api/workbench");
    checks = state.checks || [];
    renderContributionChecks();
    const params = new URL(location.href).searchParams;
    if (!workspace) {
      const linkedNode = state.nodes.find(n => n.id === params.get("node"));
      if (params.has("node") && !linkedNode) throw new Error("The linked version no longer exists. Remove the node parameter to open the workbench.");
      workspace = linkedNode?.workspace || params.get("workspace");
      node = linkedNode?.id;
      if (workspace && !state.workspaces.some(w => w.id === workspace)) throw new Error("The linked workspace no longer exists.");
    }
    const linkedSource = state.sources.find(s => s.version === params.get("version") && String(s.birth) === params.get("birth"));
    const sourceValue = linkedSource ? `${linkedSource.version}:${linkedSource.birth}` : $("source").value;
    options($("source"), state.sources.map(s => [`${s.version}:${s.birth}`, s.label]), sourceValue);
    options($("rebase-source"), state.sources.filter(s => s.version === state.activeVersion)
      .map(s => [`${s.version}:${s.birth}`, s.label]), $("rebase-source").value);
    options($("workspace"), state.workspaces.map(w => [w.id, `${w.label} - ${w.author}`]), workspace || $("workspace").value);
    populateNodes(node);
    renderPool();
    await showNode();
    updateControls();
  }
  function renderPool() {
    const pool = $("pool");
    pool.replaceChildren();
    if (!state.contributions.length) pool.append(element("p", "No contributions have been admitted yet. Saved drafts appear in the branch selector above."));
    for (const entry of state.contributions) {
      const admission = state.admissions.find(a => a.contribution === entry.id);
      const outcome = admission ? `${admission.admission}; ${admission.measurement}${admission.error ? `: ${admission.error}` : ""}` : "unmeasured; awaiting episode admission";
      const target = state.nodes.find(n => n.id === entry.node);
      const line = element("p", `${entry.label} by ${entry.author} - ${versionLabel(target)} - ${entry.active ? outcome : "withdrawn"}${entry.note ? ` - ${entry.note}` : ""} `);
      const inspect = element("button", "Open");
      inspect.onclick = () => run(async () => {
        $("workspace").value = target.workspace;
        populateNodes(target.id);
        await showNode();
      });
      line.append(inspect);
      if (entry.active) {
        const withdraw = element("button", "Withdraw");
        withdraw.onclick = () => run(async () => {
          await action("withdraw", {contribution: entry.id});
          await refresh($("workspace").value, selected?.id);
          message("Contribution withdrawn for future episode snapshots; existing descendants are preserved.");
        });
        line.append(withdraw);
      }
      pool.append(line);
    }
  }
  function populateNodes(node) {
    const workspace = state.workspaces.find(w => w.id === $("workspace").value);
    const nodes = state.nodes.filter(n => n.workspace === workspace?.id);
    options($("node"), nodes.map(n => [n.id,
      `${versionLabel(n)} [${n.status}]${n.id === workspace.head ? " - saved head" : ""}`]), node || workspace?.head);
  }
  function versionLabel(node) {
    const nodes = state.nodes.filter(n => n.workspace === node.workspace);
    return `Version ${nodes.findIndex(n => n.id === node.id) + 1}: ${node.parent ? node.summary.operation : "original fork"} (${node.id.slice(0, 8)})`;
  }
  function versionTree() {
    const workspace = state.workspaces.find(w => w.id === $("workspace").value);
    const nodes = state.nodes.filter(n => n.workspace === workspace?.id);
    const children = new Map();
    for (const node of nodes) {
      if (!children.has(node.parent)) children.set(node.parent, []);
      children.get(node.parent).push(node);
    }
    const branch = parent => {
      const list = element("ul");
      for (const node of children.get(parent) || []) {
        const item = element("li");
        item.dataset.node = node.id;
        const button = element("button", versionLabel(node));
        button.type = "button";
        button.disabled = busy;
        button.dataset.node = node.id;
        if (node.id === selected?.id) button.setAttribute("aria-current", "true");
        button.title = `${node.id}\n${node.summary.changed ? editLabel(node.summary) : "Original campaign candidate"}`;
        for (const [label, active] of [
          ["Selected", node.id === selected?.id], ["saved head", node.id === workspace.head],
          [node.status, true], ["contributed", state.contributions.some(c => c.node === node.id && c.active)]
        ]) {
          if (!active) continue;
          const badge = element("span", label);
          badge.className = `version-badge${label === "Selected" ? " selected" : ""}`;
          button.append(badge);
        }
        button.onclick = () => run(async () => {
          if (busy) return;
          $("node").value = node.id;
          await showNode();
        });
        item.append(button);
        if (children.has(node.id)) item.append(branch(node.id));
        list.append(item);
      }
      return list;
    };
    $("version-tree").replaceChildren(nodes.length ? branch(null) : element("p", "Fork a candidate to start a version tree."));
  }
  function updateContributionButton() {
    $("contribute-submit").disabled = busy || contributing || checks.some(check => check.status === "running") ||
      !selected?.physical || ["discarded", "rejected"].includes(selected?.status);
  }
  function renderContributionChecks() {
    const running = checks.filter(check => check.status === "running");
    let shown = running.length ? running : checks.slice(0, 1);
    if (pendingContribution && !checks.some(check => check.node === pendingContribution.node &&
      Date.parse(check.started) >= pendingContribution.started - 1000)) {
      shown = [pendingContribution, ...running];
    }
    const panel = $("contribution-check");
    panel.hidden = !shown.length;
    panel.dataset.running = shown.some(check => check.status === "running");
    const items = $("contribution-check-items");
    items.replaceChildren();
    const stages = [["inputs", "Saved inputs and construction outcomes"], ["checking", "Reconstruction and time-zero gates"],
      ["admitting", "Unchanged-version and pool admission"]];
    for (const check of shown) {
      const node = state?.nodes.find(node => node.id === check.node);
      const title = `${check.label} - ${node ? versionLabel(node) : check.node.slice(0, 8)}`;
      const article = element("article");
      article.append(element("h3", title), element("p", `Exact checked version: ${check.node}`));
      const phase = check.phase === "admitted" ? stages.length : stages.findIndex(stage => stage[0] === check.phase);
      const list = element("ol");
      list.className = "check-stages";
      stages.forEach(([id, label], index) => {
        const status = index < phase ? "complete" : index === phase
          ? check.status === "running" ? "running" : check.status : "pending";
        const item = element("li", `${label}: ${status}`);
        item.className = status;
        list.append(item);
      });
      const elapsed = check.elapsedSeconds ?? Math.max(0, (Date.now() - check.started) / 1000);
      const duration = element("p", `${Math.floor(elapsed)} seconds elapsed. ${check.status === "complete"
        ? "Admitted without fitness; no trial spent." : "Not admitted until every required stage completes. No physics trials."}`);
      duration.setAttribute("aria-live", "off");
      article.append(list, element("p", check.detail), duration);
      if (check.screening) {
        const progress = element("ol");
        for (const step of check.screening.checks) {
          const item = element("li", `${step.label}: ${step.status}${step.detail ? ` - ${step.detail}` : ""}`);
          item.className = step.status;
          progress.append(item);
        }
        article.append(element("p", check.screening.detail), progress);
      }
      if (check.outcomes?.length) {
        const outcomes = element("details");
        outcomes.open = true;
        outcomes.append(element("summary", "Previous gate outcomes and diagnostics"));
        const results = element("ul");
        results.className = "check-outcomes";
        for (const gate of check.outcomes) {
          const source = gate.source === "saved-generation" ? "saved construction outcome, not rerun" : "this admission check";
          results.append(element("li", `${gate.gate}: ${gate.outcome} (${source}). ${gate.code || ""}${
            gate.diagnostics?.length ? ` - ${gate.diagnostics.join("; ")}` : ""}`));
        }
        outcomes.append(results);
        article.append(outcomes);
      }
      const open = element("a", "Open the checked version");
      open.href = `/workbench.html?node=${check.node}`;
      open.target = "_blank";
      article.append(open);
      if (check.report) {
        const report = element("a", " Latest static validation report");
        report.href = check.report;
        report.target = "_blank";
        article.append(report);
      }
      items.append(article);
    }
    const announcement = shown.map(check => `${check.label}: ${check.phase}, ${check.status}.`).join(" ");
    if ($("check-announcement").textContent !== announcement) $("check-announcement").textContent = announcement;
  }
  async function pollChecks() {
    try {
      const value = await json("/api/workbench?checks=1");
      if (!Array.isArray(value.checks)) throw new Error("Invalid admission progress response.");
      checks = value.checks;
      $("check-progress-error").textContent = "";
      renderContributionChecks();
      updateControls();
    } catch (error) {
      $("check-progress-error").textContent = `Cannot refresh admission progress: ${error.message}`;
    } finally {
      setTimeout(pollChecks, document.hidden ? 10000 : 2000);
    }
  }
  function contributionTarget() {
    const target = $("contribution-target");
    target.replaceChildren();
    $("contribute-form").dataset.node = selected?.id || "";
    if (!selected) {
      target.textContent = "No version selected.";
      $("contribute-submit").textContent = "Select a version to contribute";
      updateContributionButton();
      return;
    }
    const workspace = state.workspaces.find(w => w.id === selected.workspace);
    target.append(element("strong", `Submitting ${versionLabel(selected)}`),
      element("p", `Workspace: ${workspace.label}. Status: ${selected.status}. Exact version ID: ${selected.id}`));
    const ancestors = [];
    for (let node = selected; node; node = state.nodes.find(n => n.id === node.parent)) ancestors.unshift(versionLabel(node));
    target.append(element("p", `Mutation chain: ${ancestors.join(" -> ")}`));
    if (!selected.parent) {
      const warning = element("p", "This is the original fork: none of your subsequent mutations are included. Select a descendant in the version tree to contribute an edited design.");
      warning.className = "selection-warning";
      target.append(warning);
    }
    if (selected.physical) {
      const alt = `Contribution target: ${versionLabel(selected)} (${selected.preview ? "static assembly" : "kinematic diagram"})`;
      if (selected.preview) {
        const image = element("img");
        image.src = asset(selected.id, "walker-v2.png");
        image.alt = alt;
        target.append(image);
      } else target.append(motionPlayer(selected, alt));
    } else target.append(element("p", "This version has no physical export and cannot be contributed."));
    if (["discarded", "rejected"].includes(selected.status)) target.append(element("p", "Discarded or rejected versions cannot be contributed. Select a retained or draft version."));
    $("contribute-submit").textContent = `Validate and contribute version ${state.nodes.filter(n => n.workspace === selected.workspace).findIndex(n => n.id === selected.id) + 1} (${selected.id.slice(0, 8)})`;
    updateContributionButton();
  }
  function asset(id, name) { return `/workbench-assets/${id}/${name}`; }
  function motionPlayer(node, alt = "Solved motion diagram, not measured dynamics") {
    const player = element("div");
    player.className = "motion-player";
    player.dataset.motionSource = asset(node.id, "motion.json");
    const overview = element("a");
    overview.href = asset(node.id, "cycle.svg");
    const image = element("img");
    image.src = overview.href;
    image.alt = alt;
    overview.append(image);
    player.append(overview);
    return player;
  }
  function visual(node, title) {
    const card = element("article");
    card.className = "card";
    card.append(element("h2", title), element("p", `${node.summary.operation} - ${node.status} - ${node.id.slice(0, 8)}`));
    if (node.summary.comparison) {
      const comparison = node.summary.comparison;
      card.append(element("p", comparison.motionUnchanged ? "Motion program unchanged; physical realization edited." : "Motion program changed; realization updated with it."));
      const details = element("details");
      details.append(element("summary", `${comparison.parts.length} physical parts added, removed or changed`),
        element("pre", JSON.stringify(comparison.parts, null, 2)));
      card.append(details);
    }
    if (node.physical) {
      const visuals = element("div");
      visuals.className = "visuals";
      visuals.append(motionPlayer(node), element("small", "Solved kinematics - not measured walking"));
      if (node.preview) {
        const assembly = element("img");
        assembly.src = asset(node.id, "walker-v2.png");
        assembly.alt = "Static LEGO physical realization";
        visuals.append(assembly);
      }
      card.append(visuals);
      const render = element("button", node.preview ? "Refresh static assembly" : "Render static LEGO assembly");
      render.onclick = () => run(async () => {
        await action("render", {node: node.id});
        await refresh($("workspace").value, selected?.id);
        message("Static assembly preview ready; zero simulation steps.");
      });
      card.append(render, element("p", "Assembly preparation is separate from kinematics and runs zero physics steps."));
      if (node.clearance) {
        const inspect = element("button", "Show prescribed-motion clearance warnings");
        inspect.onclick = () => run(async () => {
          const report = await json(asset(node.id, "motion-clearance.json"));
          const diagnostic = clearanceCard(report, null, node);
          inspect.replaceWith(diagnostic);
        });
        card.append(inspect);
      }
      for (const [name, label] of [["model.ldr", "LDraw model"], ["program.json", "Motion program"], ["realization.json", "Realization genome"], ["manifest.json", "Manifest"], ["contribution-validation.json", "Latest static validation"], ["assembly-preview.log", "Assembly preview log"]]) {
        const link = element("a", `${label} `);
        link.href = asset(node.id, name);
        link.target = "_blank";
        card.append(link, document.createTextNode(" "));
      }
    } else card.append(element("p", "This edit produced no valid physical export. Inspect the gate diagnostics, then return to its parent."));
    return card;
  }
  async function showNode() {
    const request = ++catalogRequest;
    selected = state.nodes.find(n => n.id === $("node").value);
    versionTree();
    contributionTarget();
    updateControls();
    if (selected && contributionWorkspace !== selected.workspace) {
      contributionWorkspace = selected.workspace;
      $("contribution-author").value = state.workspaces.find(w => w.id === selected.workspace).author;
    }
    optionPreviews.clear();
    previewErrors.clear();
    ++previewSelection;
    edits = [];
    filterEdits();
    $("comparison").replaceChildren();
    if (!selected) {
      $("node-info").textContent = "Fork a candidate to begin, or resume an existing workspace.";
      $("evidence").textContent = "";
      return;
    }
    const url = new URL(location.href);
    url.searchParams.set("workspace", selected.workspace);
    url.searchParams.set("node", selected.id);
    history.replaceState(null, "", url);
    const parent = state.nodes.find(n => n.id === selected.parent);
    if (parent) $("comparison").append(visual(parent, "Parent"));
    $("comparison").append(visual(selected, `Selected: ${versionLabel(selected)}`));
    $("node-info").textContent = `Selected for editing and contribution: ${versionLabel(selected)}. ${selected.id === state.workspaces.find(w => w.id === selected.workspace)?.head ? "Also the saved workspace head." : "Saved head is elsewhere; it will not be submitted instead."}`;
    $("evidence").textContent = JSON.stringify(selected.summary, null, 2);
    const source = state.sources.find(s => s.version === new URL(location.href).searchParams.get("version") &&
      String(s.birth) === new URL(location.href).searchParams.get("birth"));
    if (source) $("source").value = `${source.version}:${source.birth}`;
    if (selected.physical && !["discarded", "rejected"].includes(selected.status)) {
      message("Loading applicable motion and realization edits...");
      const value = await json(`/api/workbench?edits=${selected.id}`);
      if (request !== catalogRequest) return;
      edits = value.edits;
      filterEdits();
      message("Workspace ready. Select an edit, preview alternatives, then keep or branch.");
    } else {
      message(`Viewing ${versionLabel(selected)}. This ${selected.status} version cannot be edited or contributed.`);
    }
  }
  function filterEdits() {
    const visible = edits.filter(e => !$("level").value || e.level === $("level").value);
    options($("edit"), visible.map(e => [e.id, editLabel(e)]), $("edit").value);
    $("edit-options").replaceChildren(...editGroups(visible).map(group => {
      const edit = group.find(e => e.id === $("edit").value) || group[0];
      return optionCard(edit, group);
    }));
    updateControls();
  }
  function editGroups(entries) {
    const parameterized = new Set(["input-phase", "branch-phase", "move-hinge-site",
      "lengthen-foot", "bend-foot", "battery-layout", "receiver-layout", "motor-layout"]);
    const groups = new Map();
    for (const edit of entries) {
      // Exact paths keep different sites, stock replacements and mounting recipes separate.
      const key = parameterized.has(edit.operation.replace(/^shared-leg-/, "")) && edit.changed?.length ?
        JSON.stringify([edit.level, edit.operation, edit.changed.map(c => [c.path, c.before, c.after.length])]) : edit.id;
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key).push(edit);
    }
    return [...groups.values()].map(group => group.sort((a, b) => {
      const left = a.changed.flatMap(c => c.after), right = b.changed.flatMap(c => c.after);
      for (let i = 0; i < left.length; ++i) {
        if (left[i] !== right[i]) return left[i] - right[i];
      }
      return a.id.localeCompare(b.id);
    }));
  }
  function editLabel(edit) {
    const number = value => Number(value.toFixed(3)).toString();
    const changes = (edit.changed || []).map(change => {
      if (change.path.includes("/part:")) {
        const [before, after] = change.path.split("/part:")[1].split("->");
        return before === after ? `reposition ${before}` : `${before} -> ${after}`;
      }
      return `${change.path}: ${change.before.map(number).join(", ")} -> ${change.after.map(number).join(", ")}`;
    });
    return `${edit.level ? `[${edit.level}] ` : ""}${edit.operation} - ${changes.join("; ")}`;
  }
  function svgNode(tag, attributes = {}) {
    const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, value);
    return node;
  }
  function geometryChanged(part, other) {
    return !other || part.part !== other.part || part.color !== other.color ||
      part.body !== other.body || JSON.stringify(part.placement) !== JSON.stringify(other.placement);
  }
  function clearanceCard(report, repair, node) {
    const diagnostic = element("details");
    diagnostic.append(element("summary", `Advisory motion clearance: ${report.suspectPairs} suspect pairs`));
    diagnostic.append(element("p", `${report.sampledPoses} prescribed poses over ${report.motorTurnsCovered.toFixed(3)} motor turns; ${report.refinedIntervals} refined intervals. ${report.endpointRepeats ? "Endpoint repeats." : "INCOMPLETE CYCLE COVERAGE."} Thin rods and boxes, not physical collision validation. No physics steps.`));
    if (report.samplingFailure) diagnostic.append(element("p", `INCOMPLETE SAMPLING: ${report.samplingFailure}. No automatic clearance repair selected.`));
    if (repair) diagnostic.append(element("p",
      `Construction comparison on identical poses: score ${repair.before.score.toFixed(3)} -> ${repair.after.score.toFixed(3)}. Lower is only an approximate ranking; remaining warnings and all physical gates still apply.`));
    for (const contact of report.contacts) {
      diagnostic.append(element("p", `${contact.parts.join(" / ")}: first suspect phase ${(contact.firstInputAngleRadians * 180 / Math.PI).toFixed(1)} degrees; minimum modeled gap ${contact.clearanceLdu.toFixed(3)} LDU near ${(contact.inputAngleRadians * 180 / Math.PI).toFixed(1)} degrees.`));
      if (node) {
        const phase = element("button", "Show first suspect phase in this node's motion");
        phase.onclick = () => run(() => {
          const player = [...document.querySelectorAll(".motion-player")].find(root =>
            root.dataset.motionSource === asset(node.id, "motion.json"));
          if (!player) throw new Error("This node's motion diagram is no longer open; reopen the node to inspect its phase.");
          player.dispatchEvent(new CustomEvent("motion-seek", {detail: {inputAngleRadians: contact.firstInputAngleRadians}}));
          player.scrollIntoView({block: "nearest"});
        });
        diagnostic.append(phase);
      }
    }
    diagnostic.append(element("p", `Unmodeled parts: ${report.unmodeledParts.length}. ${report.limitations.join(" ")}`));
    return diagnostic;
  }
  function diagramPair(parent, child, index, groupModels = []) {
    const view = $("diagram-view").value;
    const allPoints = model => [
      ...model.parts.flatMap(p => p[view]), ...model.paths.flatMap(p => p[view]),
      ...model.feet.map(p => p[view]), ...model.joints.map(p => p[view])
    ];
    const points = [...allPoints(parent), ...allPoints(child), ...groupModels.flatMap(allPoints)];
    const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
    const loX = Math.min(...xs), loY = Math.min(...ys);
    const scale = Math.min(300 / Math.max(1, Math.max(...xs) - loX),
      170 / Math.max(1, Math.max(...ys) - loY));
    const xy = p => [10 + (p[0] - loX) * scale, 10 + (p[1] - loY) * scale];
    const pair = element("div");
    pair.className = "diagram-pair";
    for (const [model, other, label, color] of [
      [parent, child, "Before", "#ffba75"], [child, parent, `After - alternative ${index + 1}`, "#72c3ff"]
    ]) {
      const figure = element("figure");
      figure.append(element("figcaption", label));
      const svg = svgNode("svg", {viewBox: "0 0 320 190", role: "img", "aria-label": `${label}, ${view} whole-model diagram`});
      const old = new Map(other.parts.map(p => [p.id, p]));
      const shape = (part, stroke, ghost = false) => {
        const polygon = svgNode("polygon", {
          points: part[view].map(p => xy(p).join(",")).join(" "),
          fill: ghost ? "none" : stroke, "fill-opacity": ".18", stroke,
          "stroke-width": ghost ? ".9" : ".7", "stroke-dasharray": ghost ? "3 2" : "none"
        });
        const title = svgNode("title");
        title.textContent = `${part.id}: ${part.part}${ghost ? " (previous placement)" : ""}`;
        polygon.append(title);
        svg.append(polygon);
      };
      for (const part of model.parts.filter(p => !geometryChanged(p, old.get(p.id)))) shape(part, "#66788f");
      if (model === child) {
        const current = new Map(child.parts.map(p => [p.id, p]));
        for (const part of parent.parts.filter(p => geometryChanged(p, current.get(p.id)))) shape(part, "#ffba75", true);
      }
      for (const part of model.parts.filter(p => geometryChanged(p, old.get(p.id)))) shape(part, color);
      for (const path of model.paths) svg.append(svgNode("polyline", {
        points: path[view].map(p => xy(p).join(",")).join(" "), fill: "none", stroke: "#74e0b0",
        "stroke-width": ".8", "stroke-opacity": ".65"
      }));
      for (const [markers, stroke, radius] of [[model.joints, "#e2eaf7", 1.7], [model.feet, "#74e0b0", 2.5]]) {
        for (const marker of markers) {
          const [cx, cy] = xy(marker[view]);
          const circle = svgNode("circle", {cx, cy, r: radius, stroke, fill: "#101723", "stroke-width": ".8"});
          const title = svgNode("title");
          title.textContent = marker.id;
          circle.append(title);
          svg.append(circle);
        }
      }
      figure.append(svg);
      pair.append(figure);
    }
    return pair;
  }
  function optionCard(edit, group = [edit]) {
    const card = element("article");
    card.className = `edit-option${edit.id === $("edit").value ? " selected" : ""}`;
    card.dataset.edit = edit.id;
    card.append(element("h3", `[${edit.level}] ${edit.operation}`), element("p", editLabel(edit)));
    if (group.length > 1) {
      const label = element("label", `Parameter value (${group.length} exact applicable edits)`);
      const select = element("select");
      select.dataset.parameterValues = "true";
      options(select, group.map(e => [e.id, editLabel(e).split(" - ").slice(1).join(" - ")]), edit.id);
      select.onchange = () => run(() => previewOption(select.value));
      label.append(select);
      card.append(label, element("small", "Discrete generator values, not interpolation. Selecting a value builds its diagram only; it creates no branch."));
    }
    const preview = element("button", optionPreviews.has(edit.id) ? "Select this option" : "Show before/after diagrams");
    preview.onclick = () => run(() => previewOption(edit.id));
    card.append(preview);
    const value = optionPreviews.get(edit.id);
    const status = element("p");
    status.className = "option-status";
    status.setAttribute("role", "status");
    status.setAttribute("aria-live", "polite");
    status.textContent = previewRequest?.node === selected?.id && previewRequest.id === edit.id ?
      "Preparing this value; you can select another while it finishes." :
      previewErrors.get(edit.id) ? `Preview failed: ${previewErrors.get(edit.id)}. Use Show before/after diagrams to retry.` :
      value ? `Cached preview ready. No dynamics checked.${previewRequest ? " Another value is being prepared." : ""}` :
      previewRequest ? "Diagram preparation is serial. This value can be selected next." : "This value has not been previewed.";
    card.append(status);
    if (value) {
      if (value.summary.parentClearance) {
        card.append(element("h4", "Parent diagnostic"), clearanceCard(value.summary.parentClearance, null, selected));
      }
      if (!value.alternatives.length) card.append(element("p", "No physical export: this option was rejected by the construction gates."));
      const groupModels = group.flatMap(e => optionPreviews.get(e.id)?.alternatives.map(a => a.diagram) || []);
      value.alternatives.forEach((alternative, index) => {
        card.append(diagramPair(value.parent, alternative.diagram, index, groupModels));
        const changes = alternative.comparison.parts;
        const moved = changes.filter(p => p.before && p.after && p.before.part === p.after.part).length;
        card.append(element("p", `${changes.length} parts changed (${moved} retained-stock changes). ${alternative.comparison.motionUnchanged ? "Motion program unchanged." : "Motion program changed."}`));
        if (alternative.clearance) card.append(clearanceCard(alternative.clearance, alternative.clearanceRepair));
      });
      const evidence = element("details");
      evidence.append(element("summary", "Construction gates and edit evidence"), element("pre", JSON.stringify(value.summary, null, 2)));
      card.append(evidence);
      const apply = element("button", "Create draft branches from this edit");
      apply.dataset.previewApply = "true";
      apply.onclick = () => run(async () => { $("edit").value = edit.id; await applySelected(); });
      card.append(apply);
    }
    return card;
  }
  function preparePreview(node, id, catalog) {
    const request = {node, id, promise: null};
    previewRequest = request;
    request.promise = (async () => {
      try {
        const status = await json("/api/status");
        const value = await json("/api/workbench", {method: "POST", headers: {"Content-Type": "application/json"},
          body: JSON.stringify({csrfToken: status.csrfToken, action: "preview", node, edit: id})});
        if (catalog === catalogRequest) {
          if (optionPreviews.size >= 24) optionPreviews.delete(optionPreviews.keys().next().value);
          optionPreviews.set(id, value);
          previewErrors.delete(id);
        }
      } catch (error) {
        if (catalog === catalogRequest) {
          previewErrors.set(id, error.message);
          message(`Option preview failed: ${error.message}`, true);
        }
      } finally {
        if (previewRequest === request) previewRequest = null;
        if (catalog === catalogRequest) filterEdits();
        else updateControls();
      }
    })();
    filterEdits();
    return request.promise;
  }
  async function previewOption(id) {
    if (!selected || !id) throw new Error("Select an available edit first.");
    if (busy) throw new Error("Wait for the current action to finish.");
    if (!edits.some(edit => edit.id === id)) throw new Error("This edit is unavailable; refresh the catalog.");
    $("edit").value = id;
    const node = selected.id;
    const selection = ++previewSelection, catalog = catalogRequest;
    filterEdits();
    if (!optionPreviews.has(id) && previewRequest) await previewRequest.promise;
    if (selection !== previewSelection || catalog !== catalogRequest) return;
    if (!optionPreviews.has(id)) {
      message("Building this exact parameter value; other values remain selectable. No branches or simulation steps...");
      await preparePreview(node, id, catalog);
    }
    if (selection !== previewSelection || catalog !== catalogRequest || !optionPreviews.has(id)) return;
    filterEdits();
    const card = [...$("edit-options").children].find(c => c.dataset.edit === id);
    card?.scrollIntoView({block: "nearest"});
    message("Option diagrams ready. No branches created and zero simulation steps.");
    const group = editGroups(edits).find(entries => entries.some(e => e.id === id));
    const index = group.findIndex(e => e.id === id);
    const neighbor = [group[index + 1], group[index - 1]].find(e => e && !optionPreviews.has(e.id) && !previewErrors.has(e.id));
    if (neighbor && !busy && !previewRequest) preparePreview(node, neighbor.id, catalog);
  }
  async function applySelected() {
    if (!selected || !$("edit").value) throw new Error("Select an available edit first.");
    if (previewRequest) throw new Error("Wait for the current diagram preparation to finish before creating branches.");
    const value = await action("apply", {node: selected.id, edit: $("edit").value});
    await refresh(selected.workspace, value.children[0]);
  }
  async function run(fn) {
    try { await fn(); } catch (error) { message(error.message, true); }
  }
  $("fork-form").onsubmit = event => {
    event.preventDefault();
    run(async () => {
      const [version, birth] = $("source").value.split(":");
      const value = await action("fork", {version, birth: Number(birth), label: $("workspace-name").value, author: $("author").value});
      localStorage.setItem("walker-workbench-author", $("author").value);
      await refresh(value.workspace, value.node);
    });
  };
  $("workspace").onchange = () => run(async () => { populateNodes(); await showNode(); });
  $("rebase-form").onsubmit = event => {
    event.preventDefault();
    run(async () => {
      if (!selected?.physical || ["discarded", "rejected"].includes(selected.status)) throw new Error("Select a retained or draft physical version first.");
      if (!$("rebase-source").value) throw new Error("Select a current campaign family first.");
      const [version, birth] = $("rebase-source").value.split(":");
      const value = await action("rebase", {node: selected.id, version, birth: Number(birth)});
      await refresh(value.workspace, value.node);
      message("New rebased version saved: geometry unchanged, campaign ancestry recorded, no fitness imported. Contribute this version after withdrawing any obsolete entry.");
    });
  };
  $("node").onchange = () => run(showNode);
  $("level").onchange = filterEdits;
  $("diagram-view").onchange = filterEdits;
  $("edit").onchange = filterEdits;
  $("preview-option").onclick = () => run(() => previewOption($("edit").value));
  $("apply").onclick = () => run(applySelected);
  for (const name of ["checkout", "keep", "discard"]) {
    $(name).onclick = () => run(async () => {
      if (!selected) throw new Error("Select a node first.");
      await action(name, {node: selected.id});
      await refresh(selected.workspace, name === "discard" ? selected.parent : selected.id);
    });
  }
  $("resume").onclick = () => run(async () => { populateNodes(); await showNode(); });
  $("undo").onclick = () => run(async () => {
    if (!selected?.parent) throw new Error("The fork root has no earlier workspace node.");
    await action("checkout", {node: selected.parent});
    await refresh(selected.workspace, selected.parent);
  });
  $("contribute-form").onsubmit = event => {
    event.preventDefault();
    run(async () => {
      const target = selected;
      if (!target?.physical || ["discarded", "rejected"].includes(target.status)) throw new Error("Select a retained or draft physical version first.");
      if ($("contribute-form").dataset.node !== target.id) throw new Error("The selected version changed; review the contribution target before submitting.");
      const notice = $("contribution-message");
      notice.classList.remove("error");
      const label = versionLabel(target);
      const values = {node: target.id, label: $("contribution-name").value,
        author: $("contribution-author").value, note: $("note").value};
      pendingContribution = {node: target.id, label: values.label, phase: "inputs", status: "running",
        started: Date.now(), detail: "Preparing the exact submitted version. You can continue previewing and branching.",
        outcomes: []};
      renderContributionChecks();
      notice.textContent = `Checking ${label}. Keep evolving: new drafts are separate from this exact submitted version. No physics steps.`;
      try {
        await action("contribute", values);
        const value = await json("/api/workbench");
        state = value;
        checks = value.checks || [];
        selected = state.nodes.find(node => node.id === $("node").value);
        renderPool();
        versionTree();
        contributionTarget();
        renderContributionChecks();
        updateControls();
        notice.textContent = `${label} admitted without a fitness score. At the next explicit episode, it queues for evaluation within the existing trial allowance and becomes available as a parent.`;
        if (!busy) message(notice.textContent);
      } catch (error) {
        notice.textContent = `${label}: ${error.message}`;
        notice.classList.add("error");
        throw error;
      } finally {
        pendingContribution = null;
        renderContributionChecks();
        updateControls();
      }
    });
  };
  $("author").value = localStorage.getItem("walker-workbench-author") || "";
  run(() => refresh());
  pollChecks();
})();
