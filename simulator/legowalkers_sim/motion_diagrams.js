(() => {
    "use strict";
    if (!["http:", "https:"].includes(location.protocol)) return;
    const players = new Map();
    const reduced = matchMedia("(prefers-reduced-motion: reduce)");
    const colours = ["#94a8c2", "#ffd166", "#ef817a", "#90d6ad", "#c69cf4", "#e4aa67", "#73cbd4", "#cfb8a7"];
    const element = (tag, text) => {
        const node = document.createElement(tag);
        if (text !== undefined) node.textContent = text;
        return node;
    };
    function sync(state) {
        cancelAnimationFrame(state.request);
        state.previous = null;
        if (!state.draw || !state.visible || document.hidden) return;
        state.draw();
        if (!state.playing) return;
        const tick = time => {
            if (state.previous !== null) {
                state.phase = (state.phase + Math.min(time - state.previous, 100) * state.count / state.duration) % state.count;
            }
            state.previous = time;
            state.draw();
            state.request = requestAnimationFrame(tick);
        };
        state.request = requestAnimationFrame(tick);
    }
    async function load(state) {
        if (state.loading) return;
        state.loading = true;
        const root = state.root;
        const note = element("p", "Loading solved linkage poses; no physics trials...");
        note.className = "muted";
        root.append(note);
        try {
            const response = await fetch(root.dataset.motionSource);
            if (!response.ok) {
                const message = response.headers.get("Content-Type")?.includes("application/json")
                    ? (await response.json()).error : `HTTP ${response.status}`;
                throw new Error(message || "Motion data is unavailable.");
            }
            const data = await response.json();
            if (data.mode !== "planar-kinematics-not-dynamics" || !Array.isArray(data.frames) || data.frames.length < 2) {
                throw new Error("Unsupported solved motion data.");
            }
            if (players.get(root) !== state) return;
            const frames = data.frames;
            const bodyIndices = new Map(data.program.bodies.map((body, i) => [body.id, i]));
            const footColours = ["#70b8ff", "#ffd166", "#ef817a", "#90d6ad", "#c69cf4", "#e4aa67", "#73cbd4", "#cfb8a7"];
            const contactColours = new Map(data.program.feet.map((foot, i) =>
                [foot.port.body, footColours[i % footColours.length]]));
            const geometry = frames.map(frame => {
                const poses = new Map(frame.poses.map(pose => [pose.body, pose]));
                return data.program.bodies.map(body => {
                    const pose = poses.get(body.id), c = Math.cos(pose.angle), s = Math.sin(pose.angle);
                    return body.sites.map(site => [
                        pose.origin[0] + c * site.position.x - s * site.position.y,
                        pose.origin[1] + s * site.position.x + c * site.position.y,
                        data.displayDepths?.bodies[body.id] ?? pose.origin[2] + site.position.z
                    ]);
                });
            });
            const trails = data.program.feet.map(foot => frames.map(frame => {
                const p = frame.feet.find(value => value.id === foot.id).position;
                return [p[0], p[1], data.displayDepths?.feet[foot.id] ?? p[2]];
            }));
            const projections = {};
            for (const view of ["front", "oblique", "top-front", "top-oblique"]) {
                const bounds = {left: Infinity, right: -Infinity, top: Infinity, bottom: -Infinity};
                const oblique = view === "oblique" || view === "top-oblique", top = view.startsWith("top-");
                const include = p => {
                    const x = p[0] + (oblique ? .45 * p[2] : 0);
                    const y = top ? p[2] : p[1] - (oblique ? .3 * p[2] : 0);
                    bounds.left = Math.min(bounds.left, x); bounds.right = Math.max(bounds.right, x);
                    bounds.top = Math.min(bounds.top, y); bounds.bottom = Math.max(bounds.bottom, y);
                };
                geometry.forEach(bodies => bodies.forEach(sites => sites.forEach(include)));
                trails.forEach(trail => trail.forEach(include));
                projections[view] = bounds;
            }
            const port = value => {
                const body = bodyIndices.get(value.body);
                return [body, data.program.bodies[body].sites.findIndex(site => site.id === value.site)];
            };
            const hinges = data.program.hinges.map(hinge => [port(hinge.a), port(hinge.b)]);
            const footPorts = data.program.feet.map(foot => port(foot.port));
            const bodyOrder = data.program.bodies.map((_, i) => i).sort((a, b) =>
                geometry[0][b][0][2] - geometry[0][a][0][2]);
            const footOrder = data.program.feet.map((_, i) => i).sort((a, b) => trails[b][0][2] - trails[a][0][2]);
            const canvas = element("canvas");
            canvas.setAttribute("aria-label", "Animated solved linkage and complete foot-path trails; not measured physics");
            const context = canvas.getContext("2d");
            if (!context) throw new Error("Canvas drawing is unavailable.");
            const topCanvas = element("canvas");
            topCanvas.setAttribute("aria-label", "Synchronized top schematic with numbered feet horizontally aligned to the linkage below; height is collapsed");
            const topContext = topCanvas.getContext("2d");
            if (!topContext) throw new Error("Top-view canvas drawing is unavailable.");
            const views = element("div");
            views.className = "motion-views";
            const mainFigure = element("figure"), topFigure = element("figure");
            mainFigure.append(canvas, element("figcaption", "Side / oblique: linkage and foot heights"));
            const topCaption = element("figcaption");
            topFigure.append(topCanvas, topCaption);
            views.append(topFigure, mainFigure);
            const controls = element("div");
            controls.className = "motion-controls";
            const button = element("button", state.playing ? "Pause" : "Play");
            button.type = "button";
            const slider = element("input");
            slider.type = "range"; slider.min = "0"; slider.max = frames.length - 1; slider.step = "1"; slider.value = "0";
            slider.setAttribute("aria-label", "Solved motor phase");
            const status = element("output");
            status.setAttribute("aria-live", "off");
            const overview = element("a", "Static four-phase overview");
            overview.href = root.querySelector("a").href;
            const view = element("select");
            view.setAttribute("aria-label", "Motion diagram projection");
            for (const [value, label] of [["front", "Flat side view"], ["oblique", "Depth (oblique)"]]) {
                const option = element("option", label);
                option.value = value;
                view.append(option);
            }
            view.value = state.view;
            const depthNote = element("p");
            depthNote.className = "motion-depth-note muted";
            const legend = element("div");
            legend.className = "motion-legend";
            data.program.feet.forEach((foot, i) => {
                const label = element("span", `${i + 1}: ${foot.id} (layer ${Math.round(trails[i][0][2])} LDU)`);
                label.style.color = footColours[i % footColours.length];
                legend.append(label);
            });
            state.count = frames.length - 1;
            state.duration = 4000 * (frames[state.count].inputAngle - frames[0].inputAngle) / (2 * Math.PI);
            const drawProjection = (target, context, projection, scale) => {
                const size = target.getBoundingClientRect(), ratio = Math.min(2, devicePixelRatio || 1);
                if (size.width <= 0 || size.height <= 0) return;
                const width = Math.round(size.width * ratio), height = Math.round(size.height * ratio);
                if (target.width !== width || target.height !== height) {
                    target.width = width; target.height = height;
                }
                context.setTransform(ratio, 0, 0, ratio, 0, 0);
                context.clearRect(0, 0, size.width, size.height);
                const bounds = projections[projection], oblique = projection === "oblique" || projection === "top-oblique",
                    top = projection.startsWith("top-");
                const x = p => size.width / 2 + (p[0] + (oblique ? .45 * p[2] : 0) - (bounds.left + bounds.right) / 2) * scale;
                const y = p => size.height / 2 + ((top ? p[2] : p[1] - (oblique ? .3 * p[2] : 0)) - (bounds.top + bounds.bottom) / 2) * scale;
                const index = Math.round(state.phase), frame = frames[index];
                context.lineWidth = 1.5;
                context.globalAlpha = .5;
                for (const i of footOrder) {
                    context.strokeStyle = footColours[i % footColours.length];
                    context.beginPath();
                    trails[i].forEach((p, n) => n ? context.lineTo(x(p), y(p)) : context.moveTo(x(p), y(p)));
                    context.stroke();
                }
                context.globalAlpha = 1;
                context.strokeStyle = "#94a8c277"; context.lineWidth = 1;
                context.beginPath();
                for (const [a, b] of hinges) {
                    const start = geometry[index][a[0]][a[1]], end = geometry[index][b[0]][b[1]];
                    context.moveTo(x(start), y(start)); context.lineTo(x(end), y(end));
                }
                context.stroke();
                for (const i of bodyOrder) {
                    const body = data.program.bodies[i];
                    const sites = geometry[index][i];
                    context.strokeStyle = contactColours.get(body.id) || colours[i % colours.length]; context.fillStyle = context.strokeStyle;
                    context.lineWidth = body.grounded ? 2 : 4; context.setLineDash(body.grounded ? [5, 5] : []);
                    context.beginPath();
                    for (const p of sites.slice(1)) {
                        context.moveTo(x(sites[0]), y(sites[0]));
                        context.lineTo(x(p), y(p));
                    }
                    context.stroke(); context.setLineDash([]);
                    for (const p of sites) {
                        context.beginPath(); context.arc(x(p), y(p), 3, 0, 2 * Math.PI); context.fill();
                    }
                }
                context.font = "bold 12px system-ui";
                for (const i of footOrder) {
                    const p = trails[i][index];
                    const attachment = geometry[index][footPorts[i][0]][footPorts[i][1]];
                    context.fillStyle = footColours[i % footColours.length];
                    context.strokeStyle = context.fillStyle; context.lineWidth = 1;
                    context.beginPath();
                    context.moveTo(x(attachment), y(attachment)); context.lineTo(x(p), y(p)); context.stroke();
                    context.beginPath(); context.arc(x(p), y(p), 5, 0, 2 * Math.PI); context.fill();
                    context.strokeStyle = "#182333"; context.lineWidth = 3;
                    context.strokeText(String(i + 1), x(p) + 7, y(p) - 7);
                    context.fillText(String(i + 1), x(p) + 7, y(p) - 7);
                }
            };
            state.draw = () => {
                const topView = `top-${state.view}`, bounds = projections[state.view], topBounds = projections[topView];
                const size = canvas.getBoundingClientRect(), topSize = topCanvas.getBoundingClientRect();
                const scale = Math.max(0, Math.min(
                    (Math.min(size.width, topSize.width) - 60) / Math.max(1, bounds.right - bounds.left),
                    (size.height - 60) / Math.max(1, bounds.bottom - bounds.top),
                    (topSize.height - 60) / Math.max(1, topBounds.bottom - topBounds.top)));
                drawProjection(topCanvas, topContext, topView, scale);
                drawProjection(canvas, context, state.view, scale);
                topCaption.textContent = state.view === "oblique"
                    ? "Aligned top: X + 0.45Z / Z lateral — matching feet line up below"
                    : "Top: X forward / Z lateral — matching feet line up below";
                depthNote.textContent = state.view === "oblique"
                    ? "Depth schematic: +Z recedes; layers use mean physical depths, not a 3D assembly."
                    : "Flat side view: depth is collapsed; compare foot heights here.";
                depthNote.textContent += " Top schematic above: height is collapsed; horizontal depth offset and scale match the linkage below.";
                if (state.view === "oblique") depthNote.textContent += " The top view is horizontally sheared for alignment, not an undistorted footprint.";
                const index = Math.round(state.phase), frame = frames[index];
                slider.value = index;
                status.textContent = `${Math.round(frame.inputAngle * 180 / Math.PI)} deg`;
            };
            button.addEventListener("click", () => {
                state.playing = !state.playing;
                button.textContent = state.playing ? "Pause" : "Play";
                sync(state);
            });
            slider.addEventListener("input", () => {
                state.phase = Number(slider.value); state.playing = false; button.textContent = "Play"; sync(state);
            });
            view.addEventListener("change", () => {
                state.view = view.value; sync(state);
            });
            state.seek = angle => {
                if (!Number.isFinite(angle) || angle < frames[0].inputAngle || angle > frames.at(-1).inputAngle + 1e-8) {
                    const error = element("p", "Requested diagnostic phase is outside this solved diagram.");
                    error.className = "bad";
                    error.setAttribute("role", "alert");
                    root.append(error);
                    return;
                }
                let nearest = 0;
                frames.forEach((frame, index) => {
                    if (Math.abs(frame.inputAngle - angle) < Math.abs(frames[nearest].inputAngle - angle)) nearest = index;
                });
                state.phase = nearest; state.playing = false; button.textContent = "Play";
                sync(state);
            };
            state.button = button;
            controls.append(button, view, slider, status);
            root.querySelector("a").hidden = true;
            note.remove();
            root.append(views, controls, depthNote, legend, overview);
            state.resize = new ResizeObserver(() => { if (state.visible && !document.hidden) state.draw(); });
            state.resize.observe(canvas);
            state.resize.observe(topCanvas);
            if (root.dataset.motionSeekAngle !== undefined) {
                state.seek(Number(root.dataset.motionSeekAngle));
                delete root.dataset.motionSeekAngle;
            }
            sync(state);
        } catch (error) {
            if (players.get(root) !== state) return;
            note.textContent = `Animated diagram unavailable: ${error.message}. The static overview remains available.`;
            note.className = "bad";
            note.setAttribute("role", "alert");
        }
    }
    const visibility = new IntersectionObserver(entries => {
        for (const entry of entries) {
            const state = players.get(entry.target);
            if (!state) continue;
            state.visible = entry.isIntersecting;
            if (state.visible && !document.hidden) load(state);
            sync(state);
        }
    });
    function scan() {
        for (const [root, state] of players) {
            if (root.isConnected) continue;
            cancelAnimationFrame(state.request);
            state.resize?.disconnect();
            root.removeEventListener("motion-seek", state.onSeek);
            visibility.unobserve(root);
            players.delete(root);
        }
        for (const root of document.querySelectorAll(".motion-player[data-motion-source]")) {
            if (players.has(root)) continue;
            const state = {root, phase: 0, playing: false, view: "front", visible: false, previous: null};
            state.onSeek = event => {
                const angle = event.detail?.inputAngleRadians;
                if (state.seek) state.seek(angle);
                else root.dataset.motionSeekAngle = String(angle);
            };
            root.addEventListener("motion-seek", state.onSeek);
            players.set(root, state);
            visibility.observe(root);
        }
    }
    new MutationObserver(records => {
        if (records.some(record => [...record.addedNodes, ...record.removedNodes].some(node =>
            node.matches?.(".motion-player") || node.querySelector?.(".motion-player")))) scan();
    }).observe(document.documentElement, {childList: true, subtree: true});
    document.addEventListener("visibilitychange", () => {
        for (const state of players.values()) {
            if (state.visible && !document.hidden) load(state);
            sync(state);
        }
    });
    reduced.addEventListener("change", () => {
        if (!reduced.matches) return;
        for (const state of players.values()) {
            state.playing = false;
            if (state.button) state.button.textContent = "Play";
            sync(state);
        }
    });
    scan();
})();
