(() => {
    "use strict";
    function graph(nodes) {
        const byKey = new Map(nodes.map(node => [node.key, node]));
        if (byKey.size !== nodes.length) throw new Error("Duplicate ancestry identity.");
        const children = new Map(nodes.map(node => [node.key, []]));
        const ranks = new Map(), pending = new Map();
        for (const node of nodes) {
            pending.set(node.key, node.parents.length);
            for (const parent of node.parents) {
                if (!byKey.has(parent)) throw new Error("Missing ancestry parent: " + parent);
                children.get(parent).push(node.key);
            }
        }
        const queue = nodes.filter(node => !node.parents.length).map(node => node.key);
        for (let index = 0; index < queue.length; index++) {
            const key = queue[index], node = byKey.get(key);
            ranks.set(key, Math.max(-1, ...node.parents.map(parent => ranks.get(parent))) + 1);
            for (const child of children.get(key)) {
                pending.set(child, pending.get(child) - 1);
                if (!pending.get(child)) queue.push(child);
            }
        }
        if (ranks.size !== nodes.length) throw new Error("Ancestry contains a cycle.");
        return {nodes, byKey, children, ranks};
    }

    function walk(g, starts, direction, allowed) {
        const found = new Set(), queue = [...starts];
        for (let index = 0; index < queue.length; index++) {
            const key = queue[index];
            if (found.has(key) || !g.byKey.has(key) || (allowed && !allowed.has(key))) continue;
            found.add(key);
            queue.push(...(direction === "back" ? g.byKey.get(key).parents : g.children.get(key)));
        }
        return found;
    }

    function fringe(g, mode) {
        const inputs = new Set(g.nodes.filter(node => !node.parents.length || node.contribution).map(node => node.key));
        const outputs = new Set(g.nodes.filter(node => mode === "population"
            ? node.population : node.state === "qualified" && !node.reused).map(node => node.key));
        return {inputs, outputs};
    }

    function versionGroups(g, separate = new Set()) {
        const byId = new Map(), groups = new Map(), aliases = new Map(g.nodes.map(node => [node.key, node.key]));
        const roots = new Map(aliases);
        const compare = (a, b) => (a.version || "").localeCompare(b.version || "", undefined, {numeric: true}) ||
            a.key.localeCompare(b.key, undefined, {numeric: true});
        function find(key) {
            const parent = roots.get(key);
            if (parent !== key) roots.set(key, find(parent));
            return roots.get(key);
        }
        function join(a, b) {
            const left = find(a), right = find(b);
            if (left === right) return;
            if (compare(g.byKey.get(left), g.byKey.get(right)) <= 0) roots.set(right, left);
            else roots.set(left, right);
        }
        for (const node of g.nodes) {
            if (node.carry_from) {
                if (node.parents.length !== 1 || node.parents[0] !== node.carry_from) {
                    throw new Error("Invalid carry-forward provenance: " + node.key);
                }
                join(node.carry_from, node.key);
            }
            if (!node.contribution || !node.contribution_id) continue;
            if (byId.has(node.contribution_id)) join(byId.get(node.contribution_id), node.key);
            else byId.set(node.contribution_id, node.key);
        }
        const related = new Map();
        for (const node of g.nodes) {
            const first = find(node.key);
            if (!related.has(first)) related.set(first, []);
            related.get(first).push(node);
        }
        for (const nodes of related.values()) {
            if (nodes.length < 2) continue;
            nodes.sort(compare);
            const keys = nodes.map(node => node.key), first = keys[0];
            const id = nodes[0].contribution_id || "carry:" + first, collapsed = !separate.has(id);
            const carries = nodes.filter(node => node.carry_from).length;
            const admissions = nodes.filter(node => node.contribution).length;
            const kind = carries ? (admissions > 1 ? "version copies" : "carry-forward copies") : "version admissions";
            groups.set(first, {id, keys, first, collapsed, carries, admissions, kind});
            if (collapsed) for (const key of keys) aliases.set(key, first);
        }
        return {groups, aliases};
    }

    function fold(g, mode, expanded, separate = new Set()) {
        const raw = fringe(g, mode), {groups, aliases} = versionGroups(g, separate);
        const project = keys => new Set([...keys].filter(key => g.byKey.has(key)).map(key => aliases.get(key)));
        const inputs = project(raw.inputs), outputs = project(raw.outputs), revealed = project(expanded);
        for (const group of groups.values()) if (!group.collapsed) {
            for (const key of group.keys) revealed.add(key);
        }
        const visible = new Set([...inputs, ...outputs, ...revealed]);
        const levels = [...new Set([...revealed].filter(key => g.byKey.has(key) &&
            !inputs.has(key) && !outputs.has(key)).map(key => g.ranks.get(key)))].sort((a, b) => a - b);
        const buckets = new Map(), owner = new Map(), entities = [];
        for (const node of g.nodes) {
            if (aliases.get(node.key) !== node.key) {
                owner.set(node.key, aliases.get(node.key));
                continue;
            }
            const rank = g.ranks.get(node.key);
            if (visible.has(node.key)) {
                owner.set(node.key, node.key);
                const layer = outputs.has(node.key) ? levels.length * 2 + 2
                    : inputs.has(node.key) ? 0 : levels.indexOf(rank) * 2 + 2;
                entities.push({key: node.key, layer, node});
            } else {
                let gap = 0;
                while (gap < levels.length && levels[gap] <= rank) gap++;
                const key = "hidden-" + gap;
                if (!buckets.has(key)) buckets.set(key, {key, layer: gap * 2 + 1, members: new Set()});
                buckets.get(key).members.add(node.key);
                owner.set(node.key, key);
            }
        }
        for (const [key, alias] of aliases) if (key !== alias) owner.set(key, owner.get(alias));
        entities.push(...buckets.values());
        const edges = new Map();
        for (const node of g.nodes) for (const parent of node.parents) {
            const from = owner.get(parent), to = owner.get(node.key);
            if (from === to) continue;
            const id = JSON.stringify([from, to]);
            if (!edges.has(id)) edges.set(id, {id, from, to, pairs: []});
            edges.get(id).pairs.push([parent, node.key]);
        }
        const reachesOutput = walk(g, raw.outputs, "back");
        for (const bucket of buckets.values()) {
            bucket.traversal = new Set(g.nodes.filter(node => owner.get(node.key) === bucket.key).map(node => node.key));
            bucket.ended = [...new Set([...bucket.traversal].filter(key =>
                !g.children.get(key).length && !raw.outputs.has(key)).map(key => aliases.get(key)))];
            bucket.offPath = [...bucket.members].filter(key => !reachesOutput.has(key)).length;
        }
        return {inputs, outputs, visible, entities, buckets, edges: [...edges.values()], groups, aliases,
            compressed: [...aliases].filter(([key, alias]) => key !== alias).length};
    }

    function trace(g, key, direction, mode) {
        const starts = Array.isArray(key) ? key : [key];
        if (direction === "back") return walk(g, starts, "back");
        const descendants = walk(g, starts, "forward");
        if (direction === "ends") return descendants;
        const outputs = fringe(g, mode).outputs;
        const reachableOutputs = [...outputs].filter(output => descendants.has(output));
        if (!reachableOutputs.length) return new Set(starts);
        const ancestors = walk(g, reachableOutputs, "back");
        return new Set([...descendants].filter(descendant => ancestors.has(descendant)));
    }

    function relatedPorts(g, bucket, port, ports) {
        const direction = port.direction === "forward" ? "forward" : "back";
        const reachable = walk(g, port.targets, direction, bucket.traversal || bucket.members);
        return ports.filter(other => other.direction !== port.direction &&
            other.targets.some(key => reachable.has(key)));
    }

    function presentation(g, node, group) {
        const members = group ? group.keys.map(key => g.byKey.get(key)) : [node];
        const named = members.find(member => member.contribution_name);
        const source = node.preview ? node : members.find(member => member.preview);
        return {
            name: node.contribution_name || named?.contribution_name,
            contributionHref: node.contribution_href || named?.contribution_href,
            preview: source?.preview,
            caption: source ? `${source.preview_caption || "Saved preview."}${source.key !== node.key
                ? ` From ${source.version || ""} ${source.label}.` : ""}` : null,
        };
    }

    function lines(text, limit) {
        const result = [];
        for (const word of text.split(/\s+/)) {
            const last = result.length - 1;
            if (last >= 0 && result[last].length + word.length + 1 <= limit) result[last] += " " + word;
            else {
                for (let index = 0; index < word.length; index += limit) result.push(word.slice(index, index + limit));
            }
        }
        return result;
    }

    if (typeof module !== "undefined" && module.exports) module.exports = {graph, walk, fringe, versionGroups, fold, trace, relatedPorts, presentation};
    if (typeof document === "undefined") return;
    if (window.legoForestExplorer) return;
    window.legoForestExplorer = true;
    const instances = new Map();
    const html = (tag, text) => {
        const node = document.createElement(tag);
        if (text !== undefined) node.textContent = text;
        return node;
    };
    const svg = (tag, attrs = {}, text) => {
        const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
        for (const [key, value] of Object.entries(attrs)) node.setAttribute(key, value);
        if (text !== undefined) node.textContent = text;
        return node;
    };
    function button(label, action) {
        const node = html("button", label);
        node.type = "button";
        node.addEventListener("click", action);
        return node;
    }
    function activate(node, action) {
        node.setAttribute("role", "button");
        node.setAttribute("tabindex", "0");
        node.addEventListener("click", action);
        node.addEventListener("keydown", event => {
            if (event.key === "Enter" || event.key === " ") {event.preventDefault(); action();}
        });
    }
    function save(state) {
        state.undo.push({expanded: new Set(state.expanded), separate: new Set(state.separate),
            selected: state.selected, mode: state.mode});
    }
    function reveal(state, keys, selected) {
        save(state);
        for (const key of keys) state.expanded.add(key);
        state.selected = selected || state.selected;
        render(state);
    }
    function render(state) {
        const root = state.root, g = state.graph, folded = fold(g, state.mode, state.expanded, state.separate);
        const showInspector = () => {
            const inspector = root.querySelector(".forest-inspector");
            inspector.scrollIntoView({block: "start"});
            inspector.focus({preventScroll: true});
        };
        state.selected = folded.aliases.get(state.selected) || state.selected;
        const scroll = root.querySelector(".forest-map");
        const left = scroll?.scrollLeft || 0, top = scroll?.scrollTop || 0;
        const focus = document.activeElement?.dataset.forestFocus;
        const toolbar = html("div"); toolbar.className = "forest-toolbar";
        const label = html("label", "Outputs: ");
        const select = html("select"); select.setAttribute("aria-label", "Ancestry output fringe");
        for (const [value, text] of [["qualified", "Qualified walkers"], ["population", "Current parent population"]]) {
            const option = html("option", text); option.value = value; select.append(option);
        }
        select.value = state.mode; select.dataset.forestFocus = "mode";
        select.addEventListener("change", () => {save(state); state.mode = select.value; render(state);});
        label.append(select);
        const undo = button("Undo expansion", () => {
            const prior = state.undo.pop();
            if (prior) {Object.assign(state, prior); render(state);}
        });
        undo.disabled = !state.undo.length;
        toolbar.append(label, button("Reset overview", () => {
            save(state); state.expanded.clear(); state.separate.clear(); state.selected = null; render(state);
            root.querySelector(".forest-map").scrollLeft = 0;
            root.querySelector(".forest-map").scrollTop = 0;
            root.closest(".proposal-forest").querySelector(".forest-full").open = false;
        }), undo, button("Full forest", () => {
            const full = root.closest(".proposal-forest").querySelector(".forest-full");
            full.open = true; full.scrollIntoView({block: "start"});
        }));
        const named = new Map();
        for (const node of g.nodes) if (node.contribution && node.contribution_name) {
            const key = folded.aliases.get(node.key);
            if (!named.has(key)) named.set(key, {node: g.byKey.get(key), name: node.contribution_name});
        }
        if (named.size) {
            const label = html("label", "Find contribution: ");
            const find = html("select"); find.setAttribute("aria-label", "Find contribution by name");
            find.dataset.forestFocus = "find-contribution";
            const placeholder = html("option", "Choose a contribution..."); placeholder.value = ""; find.append(placeholder);
            for (const [key, item] of [...named].sort((a, b) => a[1].name.localeCompare(b[1].name))) {
                const option = html("option", `${item.name} (${item.node.version || ""} ${item.node.label})`);
                option.value = key; find.append(option);
            }
            find.value = named.has(state.selected) ? state.selected : "";
            find.addEventListener("change", () => {
                if (!find.value) return;
                reveal(state, [find.value], find.value);
                showInspector();
            });
            label.append(find); toolbar.append(label);
        }
        const groupedCarries = [...folded.groups.values()].some(group => group.collapsed && group.carries);
        const status = html("p", `${folded.visible.size} visible / ${g.nodes.length} recorded nodes; ${g.nodes.length - folded.visible.size - folded.compressed} hidden. ${folded.inputs.size} seeds/contributions; ${folded.outputs.size} outputs.${folded.compressed ? ` ${folded.compressed} ${groupedCarries ? "repeat admissions / carry-forward copies" : "repeated contribution admissions"} grouped under their first occurrence.` : ""}`);
        status.className = "muted";
        const help = html("p", "Find a contribution by name, or recognize its saved LEGO picture. Pictures identify designs, not qualification; motion diagrams are labelled separately. Arrows point to descendants. Fringes are arranged by role, not time: contributions may also have ancestors inside a blob. Click a node for tracing actions; Open details is separate. Click a port inside a blob to reveal one step. Hover or focus a port to highlight only its connected opposite ports. A blob does not imply all inputs connect to all outputs.");
        help.className = "muted";
        const map = html("div"); map.className = "forest-map";
        const layers = [...new Set(folded.entities.map(entity => entity.layer))].sort((a, b) => a - b);
        const positions = new Map(), ports = [], bucketPorts = new Map(), cards = new Map();
        const slots = new Map(layers.map(layer => [layer, 0]));
        for (const entity of folded.entities) {
            const index = slots.get(entity.layer);
            const incoming = folded.edges.filter(edge => edge.to === entity.key).length;
            const outgoing = folded.edges.filter(edge => edge.from === entity.key).length;
            let height = Math.max(130, Math.max(incoming, outgoing) * 24 + 70);
            if (entity.node) {
                const group = [...folded.groups.values()].find(group => group.keys.includes(entity.key));
                const view = presentation(g, entity.node, group);
                view.names = view.name ? lines(view.name, 34) : [];
                view.captions = view.preview ? lines(view.caption, 42) : [];
                cards.set(entity.key, view);
                height = 64 + view.names.length * 16 + (view.preview ? 122 + view.captions.length * 14 : 0);
            }
            positions.set(entity.key, {x: 30 + layers.indexOf(entity.layer) * 340, y: 30 + index, width: 250, height});
            slots.set(entity.layer, index + height + 24);
        }
        const width = Math.max(600, layers.length * 340), height = Math.max(220, ...slots.values()) + 40;
        const drawing = svg("svg", {width, height, viewBox: `0 0 ${width} ${height}`, role: "group", "aria-label": "Folded ancestry graph"});
        const defs = svg("defs"), marker = svg("marker", {id: "fold-arrow", viewBox: "0 0 10 10", refX: 9, refY: 5, markerWidth: 6, markerHeight: 6, orient: "auto"});
        marker.append(svg("path", {d: "M0 0 10 5 0 10Z", fill: "#7087a5"})); defs.append(marker); drawing.append(defs);
        for (const entity of folded.entities) if (entity.members) {
            const p = positions.get(entity.key);
            drawing.append(svg("rect", {x: p.x, y: p.y, width: p.width, height: p.height, rx: 30, class: "fold-blob"}));
            drawing.append(svg("text", {x: p.x + 30, y: p.y + 24}, `${entity.members.size} hidden nodes`));
            drawing.append(svg("text", {x: p.x + 30, y: p.y + 42}, `${entity.offPath} off output paths`));
            const inspect = svg("g", {class: "fold-node", "data-forest-focus": entity.key,
                "aria-label": `Explore ${entity.members.size} hidden nodes`});
            inspect.append(svg("text", {x: p.x + 30, y: p.y + p.height - 16}, "Explore hidden / ended branches"));
            activate(inspect, () => {state.selected = entity.key; render(state);});
            drawing.append(inspect);
        }
        for (const edge of folded.edges) {
            const from = positions.get(edge.from), to = positions.get(edge.to);
            function endpoint(key, p, outgoing) {
                if (!folded.buckets.has(key)) return [p.x + (outgoing ? p.width : 0), p.y + p.height / 2];
                const siblings = folded.edges.filter(other => outgoing ? other.from === key : other.to === key);
                const y = p.y + 60 + siblings.indexOf(edge) * 24;
                const x = p.x + (outgoing ? p.width - 12 : 12);
                const port = {id: edge.id + ":" + key, bucket: key, x, y, direction: outgoing ? "back" : "forward",
                    targets: [...new Set(edge.pairs.map(pair => pair[outgoing ? 0 : 1]))]};
                ports.push(port);
                if (!bucketPorts.has(key)) bucketPorts.set(key, []);
                bucketPorts.get(key).push(port);
                return [x, y];
            }
            const [x1, y1] = endpoint(edge.from, from, true), [x2, y2] = endpoint(edge.to, to, false);
            const middle = (x1 + x2) / 2;
            drawing.append(svg("path", {class: "fold-edge", d: `M${x1} ${y1} C${middle} ${y1} ${middle} ${y2} ${x2} ${y2}`, "marker-end": "url(#fold-arrow)"}));
        }
        for (const entity of folded.entities) if (entity.node) {
            const p = positions.get(entity.key), node = entity.node, view = cards.get(entity.key);
            const admissions = folded.groups.get(node.key);
            const summary = admissions?.collapsed
                ? `${admissions.keys.length} ${admissions.carries ? "version nodes" : "admissions"} (first shown)` : null;
            const group = svg("g", {class: "fold-node", "data-selected": state.selected === node.key,
                "data-forest-focus": node.key, "aria-label": `${view.name ? view.name + ". " : ""}${node.version || ""} ${node.label}: ${node.state}.${summary ? ` ${summary}. Expand version admissions in tracing actions.` : ""} Select tracing actions.`});
            group.append(svg("title", {}, node.detail));
            group.append(svg("rect", {x: p.x, y: p.y, width: p.width, height: p.height, rx: 10}));
            let y = p.y + 23;
            if (view.preview) {
                const image = svg("image", {href: view.preview, x: p.x + 12, y: p.y + 10, width: 226, height: 112,
                    preserveAspectRatio: "xMidYMid meet", role: "img",
                    "aria-label": `${view.name || node.label}: ${view.caption}`});
                group.append(image); y += 122;
                const captions = view.captions.map(text => svg("text",
                    {x: p.x + 12, y, class: "fold-preview-caption"}, text));
                for (const caption of captions) {
                    caption.setAttribute("y", y); group.append(caption); y += 14;
                }
                image.addEventListener("error", () => {
                    image.setAttribute("visibility", "hidden");
                    image.setAttribute("aria-hidden", "true");
                    captions[0].textContent = "Saved preview unavailable.";
                });
            }
            for (const name of view.names) {
                group.append(svg("text", {x: p.x + 12, y}, name)); y += 16;
            }
            group.append(svg("text", {x: p.x + 12, y}, `${node.version || ""} ${node.label} - ${node.state}`));
            group.append(svg("text", {x: p.x + 12, y: y + 22}, summary ||
                (node.operation.length > 27 ? node.operation.slice(0, 24) + "..." : node.operation)));
            activate(group, () => {state.selected = node.key; render(state); showInspector();});
            drawing.append(group);
        }
        for (const port of ports) {
            const node = svg("circle", {cx: port.x, cy: port.y, r: 8, class: "fold-port",
                "data-forest-focus": port.id,
                "aria-label": `Expand ${port.direction === "back" ? "ancestors" : "descendants"}: ${port.targets.length} immediate nodes`});
            port.element = node;
            node.append(svg("title", {}, `${port.direction === "back" ? "Backward" : "Forward"} one step: ${port.targets.map(key => g.byKey.get(key).label).join(", ")}`));
            activate(node, () => reveal(state, port.targets, port.targets[0]));
            const highlight = active => {
                for (const other of [port, ...relatedPorts(g, folded.buckets.get(port.bucket), port, bucketPorts.get(port.bucket))]) {
                    other.element?.setAttribute("data-highlight", String(active));
                }
            };
            node.addEventListener("mouseenter", () => highlight(true));
            node.addEventListener("mouseleave", () => highlight(false));
            node.addEventListener("focus", () => highlight(true));
            node.addEventListener("blur", () => highlight(false));
            drawing.append(node);
        }
        map.append(drawing);
        const inspector = html("div"); inspector.className = "forest-inspector";
        inspector.setAttribute("aria-label", "Ancestry tracing actions");
        inspector.setAttribute("tabindex", "-1");
        inspector.dataset.forestFocus = "inspector";
        const selected = g.byKey.get(state.selected), bucket = folded.buckets.get(state.selected);
        if (selected) {
            const group = [...folded.groups.values()].find(group => group.keys.includes(selected.key));
            const view = presentation(g, selected, group);
            if (view.name) inspector.append(html("h3", view.name));
            if (view.preview) {
                const figure = html("figure"), link = html("a"), image = html("img");
                link.href = view.preview; image.src = view.preview; image.alt = view.caption; image.loading = "lazy";
                const caption = html("figcaption", view.caption + " Open picture for full size.");
                image.addEventListener("error", () => {image.hidden = true; caption.textContent = "Saved preview unavailable.";});
                link.append(image); figure.append(link, caption); inspector.append(figure);
            }
            inspector.append(html("p", selected.detail));
            const link = html("a", "Open birth / proposal details"); link.href = selected.href; inspector.append(link);
            if (view.contributionHref) {
                const row = html("p"), link = html("a", "Open contributed workbench version");
                link.href = view.contributionHref; row.append(link); inspector.append(row);
            }
            const admissions = [...folded.groups.values()].find(group => group.keys.includes(selected.key));
            const selectedKeys = admissions?.collapsed ? admissions.keys : [selected.key];
            if (admissions) {
                inspector.append(html("p", admissions.collapsed
                    ? `${admissions.keys.length} recorded version nodes: ${admissions.admissions} contribution admissions, ${admissions.carries} carry-forward copies. This node shows the first occurrence and its outcome, not a combined trial result. Tracing follows the real paths of all occurrences without joining unrelated version paths.`
                    : `${admissions.keys.length} version-specific nodes are shown separately.`));
                if (admissions.collapsed && folded.outputs.has(selected.key)) {
                    const outputs = fringe(g, state.mode).outputs;
                    const members = admissions.keys.filter(key => outputs.has(key)).map(key => {
                        const node = g.byKey.get(key);
                        return `${node.version || ""} ${node.label}`;
                    });
                    inspector.append(html("p", `Output membership (${state.mode === "population" ? "current population" : "recorded qualification"}) comes from: ${members.join(", ")}. Expand to inspect those version-specific outcomes.`));
                }
                inspector.append(button(`${admissions.collapsed ? "Expand" : "Collapse"} ${admissions.kind} (${admissions.keys.length})`, () => {
                    save(state);
                    if (admissions.collapsed) state.separate.add(admissions.id);
                    else {state.separate.delete(admissions.id); state.selected = admissions.first;}
                    render(state);
                }));
                if (!admissions.collapsed) {
                    const list = html("ul");
                    for (const key of admissions.keys) {
                        const admission = g.byKey.get(key), row = html("li"), link = html("a",
                            `${admission.version || ""} ${admission.label}: ${admission.state}`);
                        link.href = admission.href; row.append(link); list.append(row);
                    }
                    inspector.append(list);
                }
            }
            const actions = html("div"); actions.className = "forest-actions";
            function action(label, keys, id) {
                const control = button(label, () => reveal(state, keys, selected.key));
                control.dataset.forestFocus = id;
                control.disabled = ![...keys].some(key => !folded.visible.has(folded.aliases.get(key)));
                actions.append(control);
            }
            action("Backward one step", selectedKeys.flatMap(key => g.byKey.get(key).parents), "back-step");
            action("Forward one step", selectedKeys.flatMap(key => g.children.get(key)), "forward-step");
            action("Trace to origins", trace(g, selectedKeys, "back", state.mode), "trace-back");
            const descendants = walk(g, selectedKeys, "forward");
            const outputs = fringe(g, state.mode).outputs;
            const hasOutput = [...descendants].some(key => outputs.has(key));
            action(hasOutput ? "Trace to outputs" : "Trace to ends",
                trace(g, selectedKeys, hasOutput ? "forward" : "ends", state.mode), "trace-forward");
            const endings = [...descendants].filter(key => !g.children.get(key).length && !outputs.has(key));
            action(`Show ended branches (${endings.length})`, endings, "endings");
            const collapse = button("Collapse here", () => {
                save(state);
                for (const key of descendants) state.expanded.delete(key);
                state.selected = null; render(state);
            });
            collapse.disabled = ![...descendants].some(key => state.expanded.has(key));
            actions.append(collapse); inspector.append(actions);
            if (!hasOutput) {
                inspector.append(html("p", "No recorded path from this node to the selected outputs. Trace to ends follows complete paths to the currently recorded terminal descendants, including pending ends that may still gain children."));
            }
        } else if (bucket) {
            inspector.append(html("p", `${bucket.members.size} hidden nodes, ${bucket.ended.length} ended branches. Ports reveal immediate boundary nodes without expanding unrelated branches.`));
            const counts = new Map();
            for (const key of bucket.members) {
                const outcome = g.byKey.get(key).state; counts.set(outcome, (counts.get(outcome) || 0) + 1);
            }
            inspector.append(html("p", [...counts].map(([key, count]) => `${key}: ${count}`).join("; ")));
            const endings = button(`Show ended branches (${bucket.ended.length})`, () => reveal(state, bucket.ended, bucket.ended[0]));
            endings.disabled = !bucket.ended.length;
            inspector.append(endings);
        } else {
            inspector.append(html("p", "Select a node or a blob to explore. Hidden history includes rejected, duplicate and ended branches; it is not discarded."));
        }
        if (!folded.outputs.size) inspector.append(html("p", `No ${state.mode === "qualified" ? "qualified walkers" : "current population members"} recorded in this scope.`));
        root.replaceChildren(toolbar, status, help, map, inspector);
        map.scrollLeft = left; map.scrollTop = top;
        if (focus) {
            const controls = [...root.querySelectorAll("[data-forest-focus]")];
            const target = controls.find(node => node.dataset.forestFocus === focus) ||
                controls.find(node => node.dataset.forestFocus === state.selected);
            target?.focus({preventScroll: true});
        }
    }
    function scan() {
        for (const [root] of instances) if (!root.isConnected) instances.delete(root);
        for (const root of document.querySelectorAll(".forest-explorer")) {
            const data = root.parentElement.querySelector(".forest-data")?.textContent;
            if (!data || instances.get(root)?.data === data) continue;
            const state = instances.get(root) ||
                {root, mode: "qualified", expanded: new Set(), separate: new Set(), selected: null, undo: []};
            state.data = data;
            instances.set(root, state);
            try {
                const current = graph(JSON.parse(data));
                state.graph = current;
                render(state);
            } catch (error) {
                const message = html("p", `Compact ancestry unavailable: ${error.message}. Use the full forest below.`);
                message.setAttribute("role", "alert"); message.className = "bad";
                root.replaceChildren(message); console.error("Compact ancestry:", error);
            }
        }
    }
    new MutationObserver(scan).observe(document.documentElement, {subtree: true, childList: true, characterData: true});
    scan();
})();
