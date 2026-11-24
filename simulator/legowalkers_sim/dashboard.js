(() => {
    function toggleZoom(view, x, y) {
        const scale = Number(view.style.getPropertyValue("--forest-scale") || 1);
        const next = scale === 1 ? 0.4 : 1;
        const left = (view.scrollLeft + x) / scale;
        const top = (view.scrollTop + y) / scale;
        view.style.setProperty("--forest-scale", next);
        view.style.setProperty("--forest-cursor", next === 1 ? "zoom-out" : "zoom-in");
        view.scrollLeft = left * next - x;
        view.scrollTop = top * next - y;
    }
    document.addEventListener("click", event => {
        const view = event.target.closest?.(".forest-scroll");
        if (!view || event.defaultPrevented || event.button !== 0 ||
            event.ctrlKey || event.metaKey || event.shiftKey || event.altKey ||
            event.target.closest("a, button, input, select, textarea, summary, [role='button']") ||
            !getSelection().isCollapsed) return;
        const bounds = view.getBoundingClientRect();
        toggleZoom(view, event.clientX - bounds.left - view.clientLeft,
            event.clientY - bounds.top - view.clientTop);
    });
    document.addEventListener("keydown", event => {
        if (!event.target.matches?.(".forest-scroll") || event.defaultPrevented ||
            event.ctrlKey || event.metaKey || event.shiftKey || event.altKey ||
            !["Enter", " "].includes(event.key)) return;
        event.preventDefault();
        if (!event.repeat) toggleZoom(event.target, event.target.clientWidth / 2, event.target.clientHeight / 2);
    });
    function prepareCategories(root) {
        for (const card of root.querySelectorAll(".card[data-live-key]")) {
            if (card.querySelector(":scope > details.category")) continue;
            const [label, heading] = card.children;
            const summary = document.createElement("summary");
            summary.style.cursor = "pointer";
            summary.append(label, heading);
            const content = document.createElement("div");
            content.append(...card.childNodes);
            const details = document.createElement("details");
            details.className = "category";
            details.open = true;
            details.style.margin = "0";
            details.append(summary, content);
            card.append(details);
            card.style.alignSelf = "start";
        }
    }
    prepareCategories(document);
    function revealCategory(hash, smooth = false) {
        const card = document.getElementById(hash.slice(1));
        if (!card?.matches(".card[data-live-key]")) return false;
        const details = card.querySelector(":scope > details.category");
        if (details) details.open = true;
        card.focus({preventScroll: true});
        card.scrollIntoView({behavior: smooth && !matchMedia("(prefers-reduced-motion: reduce)").matches
            ? "smooth" : "auto", block: "center"});
        return true;
    }
    revealCategory(location.hash);
    addEventListener("hashchange", () => revealCategory(location.hash, true));
    document.addEventListener("click", event => {
        const link = event.target.closest?.("a[data-champion-link]");
        if (!link || event.button !== 0 || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return;
        const target = new URL(link.getAttribute("href"), location.href);
        const pagePath = path => path.replace(/(?:index|preview)\.html$/, "");
        if (target.origin !== location.origin || pagePath(target.pathname) !== pagePath(location.pathname)) return;
        if (!revealCategory(target.hash, true)) return;
        event.preventDefault();
        location.hash = target.hash;
    });
    document.addEventListener("toggle", event => {
        if (event.target.matches("details.category") && !event.target.open) {
            for (const video of event.target.querySelectorAll("video")) video.pause();
        }
    }, true);
    if (!["http:", "https:"].includes(location.protocol) || !document.getElementById("start-episode")) return;
    if (location.pathname.startsWith("/archives/") || location.pathname.startsWith("/versions/")) return;
    const button = document.getElementById("start-episode");
    const status = document.getElementById("worker-status");
    const error = document.getElementById("worker-error");
    const log = document.getElementById("worker-log");
    const candidatePipeline = document.getElementById("candidate-pipeline");
    const previewStatus = document.getElementById("assembly-preview-status");
    const previewError = document.getElementById("assembly-preview-error");
    const previewLog = document.getElementById("assembly-preview-log");
    const previewRetry = document.getElementById("retry-assembly-previews");
    const renderStart = document.getElementById("start-rendering");
    const renderStop = document.getElementById("stop-rendering");
    const renderStatus = document.getElementById("render-status");
    const renderError = document.getElementById("render-error");
    const renderLog = document.getElementById("render-log");
    const renderLinks = document.getElementById("render-links");
    const recentRenders = document.getElementById("recent-renders");
    const upgrade = document.getElementById("prepare-upgrade");
    const upgradeStatus = document.getElementById("upgrade-status");
    const runtimeStatus = document.getElementById("runtime-status");
    const experimentHistory = document.getElementById("experiment-history");
    let revision = null;
    let token = null;
    let starting = false;
    let deferred = false;
    let running = false;
    let canStart = true;
    let renderingAction = false;
    let recentRendering = null;
    let upgrading = false;
    let pendingSectionHash = location.hash;

    function revealSection(hash) {
        const target = document.getElementById(hash.slice(1));
        if (!target) return false;
        for (let ancestor = target; ancestor; ancestor = ancestor.parentElement) {
            if (ancestor.tagName === "DETAILS") ancestor.open = true;
        }
        target.focus?.({preventScroll: true});
        target.scrollIntoView({block: "start"});
        return true;
    }
    addEventListener("hashchange", () => revealSection(location.hash));
    document.addEventListener("click", event => {
        const link = event.target.closest?.("#dashboard-navigation a, a[data-pipeline-link]");
        if (link && event.button === 0 && !event.ctrlKey && !event.metaKey && !event.shiftKey && !event.altKey) {
            revealSection(link.hash);
        }
    });

    function updateNavigation() {
        const navigation = document.getElementById("dashboard-navigation");
        if (!navigation) return;
        const incoming = navigation.cloneNode(false);
        const targets = [
            [document.getElementById("live-controls"), "Episode / pipeline"],
            [document.getElementById("qualified-walkers"), "Qualified walkers"],
            [document.getElementById("animation-rendering"), "Rendering"],
            [document.getElementById("pipeline-board"), "Walker board"],
        ];
        for (const section of candidatePipeline.querySelectorAll(".pipeline-stage")) {
            section.id = `pipeline-${section.dataset.liveKey}`;
            targets.push([section, section.querySelector("summary,h4").textContent]);
        }
        const names = new Map();
        for (const heading of document.querySelectorAll("#dashboard-content h2")) {
            const name = "section-" + heading.textContent.toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/-$/, "");
            const count = (names.get(name) || 0) + 1;
            names.set(name, count);
            if (!heading.id) heading.id = name + (count > 1 ? `-${count}` : "");
            targets.push([heading, heading.textContent]);
        }
        for (const [target, label] of targets) {
            if (!target) continue;
            const link = document.createElement("a");
            link.href = `#${target.id}`;
            link.textContent = label;
            incoming.append(link);
        }
        patch(navigation, incoming);
        if (pendingSectionHash && revealSection(pendingSectionHash)) pendingSectionHash = "";
    }

    function archiveLinks(root) {
        for (const link of root.querySelectorAll("[data-archive-link]")) {
            link.setAttribute("href", "/archives/" + link.dataset.archiveLink);
        }
    }
    archiveLinks(document);

    function patch(current, incoming) {
        if (current.isEqualNode(incoming)) return current;
        if (current.nodeType !== incoming.nodeType || current.nodeName !== incoming.nodeName) {
            current.replaceWith(incoming);
            return incoming;
        }
        if (current.nodeType !== Node.ELEMENT_NODE) {
            current.textContent = incoming.textContent;
            return current;
        }
        if (current.matches(".forest-explorer")) return current;
        // Keep the existing media element and decoder when its source is unchanged.
        if (current.matches(".motion-player") && current.dataset.motionSource === incoming.dataset.motionSource) return current;
        if (current.tagName === "VIDEO" &&
            current.querySelector("source")?.getAttribute("src") === incoming.querySelector("source")?.getAttribute("src")) return current;
        if (current.matches(".card") && [...current.querySelectorAll("video")].some(video => !video.paused)) {
            deferred = true;
            return current;
        }
        for (const attribute of [...current.attributes]) {
            if (current.tagName === "DETAILS" && attribute.name === "open") continue;
            if (current.matches(".forest-scroll") && attribute.name === "style") continue;
            if (!incoming.hasAttribute(attribute.name)) current.removeAttribute(attribute.name);
        }
        for (const attribute of incoming.attributes) {
            if (current.tagName === "DETAILS" && attribute.name === "open") continue;
            if (current.matches(".forest-scroll") && attribute.name === "style") continue;
            if (current.getAttribute(attribute.name) !== attribute.value) current.setAttribute(attribute.name, attribute.value);
        }
        const keyed = new Map([...current.children].filter(child => child.dataset.liveKey)
            .map(child => [child.dataset.liveKey, child]));
        let cursor = current.firstChild;
        for (const child of [...incoming.childNodes]) {
            const key = child.nodeType === Node.ELEMENT_NODE && child.dataset.liveKey;
            const match = key ? keyed.get(key) : cursor;
            let updated;
            if (match && (!key || match.dataset.liveKey === key)) {
                updated = patch(match, child);
                if (match === cursor) cursor = updated;
            } else {
                updated = child;
            }
            if (updated !== cursor) current.insertBefore(updated, cursor);
            cursor = updated.nextSibling;
        }
        while (cursor) {
            const next = cursor.nextSibling;
            cursor.remove();
            cursor = next;
        }
        return current;
    }

    async function request(url, options = {}) {
        const response = await fetch(url, {cache: "no-store", ...options});
        const value = await response.json();
        if (!response.ok) throw new Error(value.error || `Request failed (${response.status})`);
        return value;
    }

    function pipelineCard(item, prefix, recent = false) {
        const card = document.createElement("li");
        card.className = "active-trial-card";
        card.dataset.liveKey = `pipeline-${prefix}-${item.key}`;
        const heading = document.createElement("h4");
        const link = document.createElement("a");
        link.href = prefix + item.history;
        link.textContent = item.proposal ? `Proposal ${item.proposal} · parent birth ${item.birth}`
            : `${item.trial ? `Trial ${item.trial} · ` : ""}Birth ${item.birth}`;
        heading.append(link);
        const description = document.createElement("p");
        description.className = "muted";
        description.textContent = `${item.category} · ${item.stage}${item.pid ? ` · process ${item.pid}` : ""}`;
        card.append(heading, description);
        if (item.repairs?.length) {
            const repairs = document.createElement("p");
            repairs.dataset.liveKey = "repair-births";
            repairs.append("Repair attempts: ");
            for (const [index, repair] of item.repairs.entries()) {
                if (index) repairs.append(" · ");
                const link = document.createElement("a");
                link.href = prefix + repair.history;
                link.textContent = `Birth ${repair.birth} (${repair.status})`;
                repairs.append(link);
            }
            card.append(repairs);
        }
        screeningChecks(card, item.screening, recent);
        if (item.parentPreview) {
            const note = document.createElement("p");
            note.textContent = "Parent shown below; the child has not been realized yet.";
            card.append(note);
        }
        const visuals = document.createElement("div");
        visuals.className = "pipeline-visuals";
        for (const [kind, url] of Object.entries(item.visuals)) {
            if (!url) continue;
            const figure = document.createElement("figure");
            const imageLink = document.createElement("a");
            imageLink.href = prefix + url;
            const image = document.createElement("img");
            image.src = prefix + url;
            image.alt = `${item.parentPreview ? "Parent: " : ""}${kind === "assembly" ? "LEGO assembly preview" : "Solved motion diagram"} for birth ${item.birth}`;
            imageLink.append(image);
            const caption = document.createElement("figcaption");
            caption.textContent = kind === "assembly" ? "Initial LEGO assembly; not measured motion."
                : "Solved kinematics; not the full LEGO assembly or live physics.";
            if (kind === "diagram") {
                const player = document.createElement("div");
                player.className = "motion-player";
                player.dataset.motionSource = (prefix + url).replace(/cycle\.svg$/, "motion.json");
                player.append(imageLink);
                figure.append(player, caption);
            } else figure.append(imageLink, caption);
            visuals.append(figure);
        }
        if (!visuals.children.length) {
            visuals.className = "empty";
            visuals.textContent = "Preview unavailable; design files remain linked from the birth page.";
        }
        card.append(visuals);
        if (!item.parentPreview && !item.visuals.assembly) {
            const note = document.createElement("p");
            note.className = item.previewError ? "bad" : "muted";
            note.textContent = item.previewError ? `Assembly preview failed: ${item.previewError}`
                : `Assembly preview: ${item.previewStatus || "not queued yet"}.`;
            card.append(note);
        }
        if (item.detail) {
            const detail = document.createElement("p");
            detail.textContent = item.detail;
            card.append(detail);
        }
        if (item.construction) {
            const evidence = document.createElement("details");
            evidence.dataset.liveKey = `construction-${item.key}`;
            const title = document.createElement("summary");
            title.textContent = "Planning and builder attempts";
            evidence.append(title);
            const trace = item.construction;
            const note = document.createElement("p");
            note.className = "muted";
            note.textContent = trace.tracked
                ? "Recorded generator stages and bounded physical realization attempts. These are not simulation trials."
                : "This version has no live planning trace. Only saved proposal results are shown; no stages are inferred.";
            evidence.append(note);
            if (trace.operation) {
                const operation = document.createElement("p");
                operation.textContent = `Selected operation: ${trace.operation}`;
                evidence.append(operation);
            }
            const list = document.createElement("ol");
            for (const event of trace.events) {
                const entry = document.createElement("li");
                entry.textContent = `${event.stage} · ${event.status}${event.attempt ? ` · attempt ${event.attempt}` : ""}: ${event.detail}`;
                list.append(entry);
            }
            evidence.append(list);
            if (trace.counts) {
                const counts = document.createElement("p");
                counts.textContent = `Builder attempts: ${trace.counts.realizationAttempts}; exported: ${trace.counts.exported}; rejected: ${trace.counts.rejected}; duplicates: ${trace.counts.duplicates}.`;
                evidence.append(counts);
            }
            if (trace.report) {
                const report = document.createElement("a");
                report.href = prefix + trace.report;
                report.textContent = "Recorded proposal / construction report (JSON)";
                evidence.append(report);
            }
            card.append(evidence);
        }
        if (item.qualification) {
            const verdict = document.createElement("p");
            const {qualified, rejectionReasons} = item.qualification;
            verdict.className = qualified ? "good" : "bad";
            verdict.textContent = qualified ? "Qualified walker."
                : rejectionReasons.length ? `Not qualified: ${rejectionReasons.join(", ")}.`
                : "Not qualified. See the trial result for details.";
            card.append(verdict);
        }
        if (item.report) {
            const reportLine = document.createElement("p");
            const reportLink = document.createElement("a");
            reportLink.href = prefix + item.report;
            reportLink.textContent = "Detailed screening report (JSON)";
            reportLine.append(reportLink);
            card.append(reportLine);
        }
        if (item.log) {
            const logLine = document.createElement("p");
            const logLink = document.createElement("a");
            logLink.href = prefix + item.log;
            logLink.textContent = "Worker log";
            logLine.append(logLink);
            card.append(logLine);
        }
        return card;
    }

    function screeningChecks(card, screening, collapsed = false) {
        if (!screening) return;
        if (collapsed) {
            const details = document.createElement("details");
            details.dataset.liveKey = "screening-diagnostics";
            const summary = document.createElement("summary");
            summary.textContent = "Screening diagnostics";
            details.append(summary);
            card.append(details);
            card = details;
        }
        const detail = document.createElement("p");
        detail.className = "muted";
        const checks = screening.checks || [];
        const passed = checks.filter(check => check.status === "pass").length;
        detail.textContent = `Checks passed: ${passed}. ${screening.detail}`;
        card.append(detail);
        if (!checks.length) return;
        const list = document.createElement("ol");
        for (const check of checks) {
            const line = document.createElement("li");
            line.textContent = `${check.label}: ${check.status}${check.detail ? ` — ${check.detail}` : ""}`;
            line.className = ["failed", "rejected", "reject", "interrupted"].includes(check.status) ? "bad" : "muted";
            if (check.started) {
                const end = check.completed ? Date.parse(check.completed) : Date.now();
                line.textContent += ` (${Math.max(0, Math.floor((end - Date.parse(check.started)) / 1000))}s)`;
            }
            list.append(line);
        }
        card.append(list);
    }

    function startupCard(item, prefix) {
        const card = document.createElement("li");
        card.className = "active-trial-card";
        card.dataset.liveKey = item.key;
        const heading = document.createElement("h4");
        heading.textContent = item.label;
        const description = document.createElement("p");
        const status = item.status === "complete" ? "completed" : item.status;
        description.textContent = `${item.kind === "admission" ? "Contribution admission" : "Startup"} · ${status}`;
        if (item.elapsedSeconds !== null) {
            const elapsed = Math.floor(item.elapsedSeconds);
            description.textContent += ` · ${Math.floor(elapsed / 60)}m ${elapsed % 60}s elapsed`;
        }
        description.className = ["failed", "rejected", "interrupted"].includes(item.status) ? "bad" : "muted";
        card.append(heading, description);
        screeningChecks(card, item.screening);
        if (item.detail) {
            const detail = document.createElement("p");
            detail.textContent = item.detail;
            card.append(detail);
        }
        if (item.history) {
            const line = document.createElement("p");
            const link = document.createElement("a");
            link.href = prefix + item.history;
            link.textContent = `Continue with birth ${item.birth}`;
            line.append(link);
            card.append(line);
        }
        if (item.log) {
            const line = document.createElement("p");
            const link = document.createElement("a");
            link.href = prefix + item.log;
            link.textContent = "Episode log";
            line.append(link);
            card.append(line);
        }
        return card;
    }

    function pipelineNotice(state) {
        if (!state.pipeline.tracked) {
            return "This frozen version does not record exact screening/preflight stages. Pending is not necessarily idle; detailed tracking applies after the implementation update.";
        }
        if (state.running && !state.pipeline.startup?.tracked && state.pipeline.stages.every(stage => stage.count === 0)) {
            return "The episode worker is running, but no candidate-stage activity is currently recorded. Startup work such as contribution admission happens before candidates enter this pipeline. See the episode log for details.";
        }
        return null;
    }

    function renderQualified(qualified) {
        const current = document.getElementById("qualified-walkers-content");
        if (!current) return;
        const incoming = current.cloneNode(false);
        const note = document.createElement("p");
        note.textContent = qualified.count
            ? `Showing ${qualified.items.length} of ${qualified.count} qualified designs, newest qualification first. Each result uses its source version's rules; historical qualification is not approval under current rules. Reused observations are not counted twice.`
            : "No recorded qualified walkers yet. Screening passes and unqualified prototypes do not count.";
        incoming.append(note);
        const cards = document.createElement("ul");
        cards.className = "grid active-trial-gallery";
        for (const item of qualified.items) {
            const card = pipelineCard(item, item.prefix, true);
            const measurement = document.createElement("p");
            measurement.textContent = `${item.version ? `${item.version} · ` : ""}${item.speedMmPerSecond.toFixed(2)} mm/s · ${item.forwardMeters.toFixed(3)} m forward · measured ${item.updated}`;
            card.append(measurement);
            if (item.video) {
                const line = document.createElement("p");
                const video = document.createElement("a");
                video.href = item.prefix + item.video;
                video.textContent = "Recorded simulation animation";
                line.append(video);
                card.append(line);
            }
            cards.append(card);
        }
        incoming.append(cards);
        patch(current, incoming);
    }

    function renderPipeline(state) {
        const incoming = document.createElement("div");
        incoming.id = "candidate-pipeline";
        const prefix = state.campaign ? `/versions/${state.version}/` : "";
        const startup = state.pipeline.startup;
        const recent = state.pipeline.finished || {
            count: state.pipeline.recent.length, walkers: state.pipeline.recent,
        };
        const stages = [...state.pipeline.stages.map(stage => ({
            ...stage, walkers: stage.walkers || stage.items,
        })), {id: "recent", label: "Recently finished / rejected", count: recent.count,
            walkers: recent.walkers, items: state.pipeline.recent, recent: true}];
        if (startup?.tracked) {
            const admissions = startup.items.filter(item => item.kind === "admission" && item.status !== "complete");
            stages.unshift({id: "startup", label: "Contribution admission", count: admissions.length,
                items: startup.items, walkers: admissions.map(item => ({
                    ...item, name: item.label, stage: item.status, visuals: item.visuals || {},
                }))});
        }
        const summary = document.createElement("nav");
        summary.className = "pipeline-summary";
        summary.setAttribute("aria-label", "Pipeline stage overview");
        for (const stage of stages) {
            const count = document.createElement("a");
            count.className = "pill";
            count.textContent = `${stage.label}: ${stage.count}`;
            count.href = `#pipeline-board-${stage.id}`;
            count.title = `View ${stage.label} on the walker board`;
            count.dataset.pipelineLink = "";
            summary.append(count);
        }
        incoming.append(summary);
        const board = document.createElement("section");
        board.id = "pipeline-board";
        board.dataset.liveKey = "walker-board";
        const boardTitle = document.createElement("h4");
        boardTitle.textContent = "Walker board";
        const boardNote = document.createElement("p");
        boardNote.className = "muted pipeline-board-note";
        boardNote.textContent = "Overview → stage column → walker details. Pictures are saved assembly previews or solved diagrams, not proof of walking. Each walker appears below; detailed stage galleries show a bounded selection. Proposal pictures show the parent, not an unrealized child.";
        const columns = document.createElement("div");
        columns.className = "pipeline-board";
        const empty = document.createElement("details");
        empty.dataset.liveKey = "empty-pipeline-stages";
        const emptyTitle = document.createElement("summary");
        emptyTitle.textContent = `Empty stages (${stages.filter(stage => !stage.count).length})`;
        const emptyColumns = document.createElement("div");
        emptyColumns.className = "pipeline-board";
        for (const stage of stages) {
            const lane = document.createElement("section");
            lane.id = `pipeline-board-${stage.id}`;
            lane.className = "pipeline-lane";
            lane.tabIndex = -1;
            lane.dataset.liveKey = `board-${stage.id}`;
            lane.setAttribute("aria-label", stage.label);
            const title = document.createElement("h5"), zoom = document.createElement("a");
            zoom.href = `#pipeline-${stage.id}`;
            zoom.dataset.pipelineLink = "";
            zoom.textContent = `${stage.label} (${stage.count})`;
            zoom.title = "Open detailed stage view";
            title.append(zoom); lane.append(title);
            const list = document.createElement("ul");
            const detailed = new Set(stage.items.map(item => item.key));
            for (const item of stage.walkers) {
                const row = document.createElement("li"), link = document.createElement("a");
                row.dataset.liveKey = item.key;
                link.className = "pipeline-mini";
                link.href = detailed.has(item.key) ? `#pipeline-card-${stage.id}-${item.key}` : prefix + item.history;
                if (detailed.has(item.key)) link.dataset.pipelineLink = "";
                const name = item.name || (item.proposal ? `Proposal ${item.proposal} (parent B${item.birth})` : `Birth ${item.birth}`);
                link.title = `${name} · ${item.stage}${item.parentPreview ? " · parent picture" : ""}`;
                const picture = item.visuals.assembly || item.visuals.diagram;
                const pictureBox = document.createElement("span");
                pictureBox.className = "pipeline-mini-picture";
                const fallback = document.createElement("span");
                fallback.className = "pipeline-mini-placeholder";
                fallback.textContent = "No preview";
                if (picture) {
                    const image = document.createElement("img");
                    image.src = prefix + picture;
                    image.alt = `${item.parentPreview ? "Parent: " : ""}${item.visuals.assembly ? "Saved LEGO assembly" : "Solved diagram"} for ${name}`;
                    image.loading = "lazy";
                    image.dataset.previewKind = item.visuals.assembly ? "assembly" : "diagram";
                    image.addEventListener("error", () => image.replaceWith(fallback));
                    pictureBox.append(image);
                } else pictureBox.append(fallback);
                if (stage.recent) {
                    let outcome, description, symbol;
                    if (item.stage === "rejected" || item.stage.endsWith("-rejected")) {
                        [outcome, description, symbol] = ["rejected", "Rejected", "\u00d7"];
                    } else if (item.stage === "complete" && item.qualification?.qualified === true) {
                        [outcome, description, symbol] = ["qualified", "Qualified", "\u2713"];
                    } else if (item.stage === "complete" && item.qualification?.qualified === false) {
                        [outcome, description, symbol] = ["unqualified", "Evaluated, not qualified", "!"];
                    } else if (item.stage === "failed") {
                        [outcome, description, symbol] = ["failed", "Failed", "!"];
                    } else if (item.stage === "interrupted") {
                        [outcome, description, symbol] = ["interrupted", "Interrupted", "\u2016"];
                    } else {
                        [outcome, description, symbol] = ["unknown", "No recorded qualification outcome", "?"];
                    }
                    const badge = document.createElement("span");
                    badge.className = "pipeline-outcome";
                    badge.dataset.outcome = outcome;
                    badge.setAttribute("role", "img");
                    badge.setAttribute("aria-label", description);
                    badge.title = description; badge.textContent = symbol;
                    pictureBox.append(badge);
                    link.title += ` \u00b7 ${description}`;
                }
                const label = document.createElement("span");
                label.className = "pipeline-mini-label";
                label.textContent = name;
                link.append(pictureBox, label); row.append(link); list.append(row);
            }
            if (!stage.walkers.length) {
                const note = document.createElement("p");
                note.className = "muted"; note.textContent = "No walkers in this stage.";
                lane.append(note);
            } else lane.append(list);
            (stage.count ? columns : emptyColumns).append(lane);
        }
        empty.append(emptyTitle, emptyColumns);
        board.append(boardTitle, boardNote, columns, empty);
        incoming.append(board);
        const notice = pipelineNotice(state);
        if (notice) {
            const note = document.createElement("p");
            note.className = "muted";
            note.textContent = notice;
            incoming.append(note);
        }
        if (startup?.tracked) {
            const section = document.createElement("details");
            section.dataset.liveKey = "startup";
            section.className = "pipeline-stage";
            section.open = startup.items.some(item => ["queued", "running", "failed", "interrupted"].includes(item.status));
            const title = document.createElement("summary");
            title.textContent = "Episode startup and contribution admission";
            for (const [status, count] of Object.entries(startup.counts)) {
                title.textContent += ` · ${status === "complete" ? "completed" : status}: ${count}`;
            }
            const cards = document.createElement("ul");
            cards.className = "grid active-trial-gallery";
            for (const item of startup.items) {
                const card = startupCard(item, prefix);
                card.id = `pipeline-card-startup-${item.key}`;
                card.tabIndex = -1;
                cards.append(card);
            }
            section.append(title, cards);
            incoming.append(section);
        }
        function group(key, label, items, count, recent = false) {
            const section = document.createElement(recent ? "details" : "section");
            section.id = `pipeline-${key}`;
            section.tabIndex = -1;
            section.className = "pipeline-stage";
            section.dataset.liveKey = key;
            if (recent) section.open = true;
            const title = document.createElement(recent ? "summary" : "h4");
            title.textContent = label + (count > items.length ? ` (showing ${items.length} of ${count})` : "");
            section.append(title);
            const cards = document.createElement("ul");
            cards.className = "grid active-trial-gallery";
            for (const item of items) {
                const card = pipelineCard(item, prefix, recent);
                card.id = `pipeline-card-${key}-${item.key}`;
                card.tabIndex = -1;
                cards.append(card);
            }
            if (!items.length) {
                const note = document.createElement("p");
                note.className = "muted"; note.textContent = "No walkers in this stage.";
                section.append(note);
            }
            section.append(cards);
            incoming.append(section);
        }
        for (const stage of state.pipeline.stages) group(stage.id, stage.label, stage.items, stage.count);
        group("recent", "Recently finished / rejected", state.pipeline.recent, recent.count, true);
        patch(candidatePipeline, incoming);
        const previews = state.assemblyPreviews;
        previewStatus.textContent = !previews.available ? "Assembly previews unavailable: configure a Blender executable."
            : `Automatic assembly previews: ${previews.running ? "worker active" : "idle"}. ${previews.counts.queued || 0} queued; ${previews.counts.rendering || 0} rendering; ${previews.counts.complete || 0} complete; ${previews.counts.failed || 0} failed.`;
        previewError.textContent = previews.error || "";
        previewLog.hidden = !previews.log;
        if (previews.log) previewLog.href = prefix + previews.log;
        previewRetry.hidden = !previews.error && !previews.counts.failed;
        previewRetry.disabled = !previews.available;
    }

    async function update() {
        const state = await request("/api/status");
        token = state.csrfToken;
        running = state.running;
        canStart = state.canStart !== false;
        runtimeStatus.textContent = state.campaign
            ? `Experiment ${state.version}. Simulation capacity: ${state.simulationWorkers}. Screening/preflight capacity: ${state.screeningWorkers ?? 1}. Each episode uses a verified, frozen implementation; updates apply between episodes.`
            : state.pinnedRuntime
            ? "This experiment resumes using its verified, frozen implementation."
            : "This experiment uses its recorded evaluation and search policy.";
        upgrade.hidden = false;
        upgrade.textContent = state.campaign ? "Update implementation" : "Prepare discovery/cycle upgrade";
        upgrade.disabled = upgrading || state.running || Boolean(state.campaign &&
            (!state.updateAvailable || (state.rendering.running && (state.rendering.busy ?? true))));
        experimentHistory.hidden = !state.campaign;
        if (state.campaign && !upgrading) {
            upgradeStatus.textContent = state.updateError ? `Update failed; previous version retained: ${state.updateError}`
                : state.updateAvailable ? "Update available. It will be applied before the next episode."
                : "Latest implementation installed. Updates spend no trials.";
        }
        button.hidden = false;
        button.disabled = starting || state.running || !canStart;
        button.textContent = state.running ? "Episode running" : !canStart ? "Prepared episode finished"
            : state.remaining > 0 ? `Resume episode (${state.remaining} trials remaining)`
            : state.finiteQueue ? `Run prepared episode (${state.nextEpisodeTrials} trial${state.nextEpisodeTrials === 1 ? "" : "s"})`
            : `Run next episode (${state.nextEpisodeTrials} trial${state.nextEpisodeTrials === 1 ? "" : "s"})`;
        status.textContent = state.running
            ? `Episode running in dispatcher ${state.pid}. ${state.activeTrials.length} full simulations${state.cheapGate ? ` and ${state.cheapGate.counts.running ?? 0} cheap prefixes` : ""}; ${state.simulationWorkers} worker slots available. Chat can remain idle or be closed.`
            : `Worker idle. ${state.used} of ${state.budget} authorized ${state.cheapGate ? "full-simulation" : "trial"} slots spent. No episode starts automatically.`;
        if (state.notice && !state.running) status.textContent += ` ${state.notice}`;
        renderPipeline(state);
        renderQualified(state.pipeline.qualified);
        updateNavigation();
        error.textContent = state.error || "";
        log.hidden = !state.log;
        if (state.log) log.href = state.log;
        const rendering = state.rendering;
        renderStart.hidden = renderStop.hidden = false;
        renderStart.disabled = renderingAction || rendering.running || !rendering.available;
        renderStop.disabled = renderingAction || !rendering.running || rendering.status === "stopping";
        renderStatus.textContent = `${rendering.status === "stopping" ? "Stopping" : rendering.running
            ? rendering.current === null ? "Watching for saved trials" : `Rendering trial ${rendering.current}`
            : "Rendering worker stopped"}. ${rendering.complete} complete; ${rendering.queued} queued; ${rendering.failed} failed.`;
        renderError.textContent = rendering.error || (!rendering.available
            ? "Blender is unavailable. Restart the dashboard with --blender PATH." : "");
        renderLog.hidden = !rendering.log;
        if (rendering.log) renderLog.href = rendering.log;
        const recent = JSON.stringify(rendering.recent);
        if (recent !== recentRendering) {
            const incoming = renderLinks.cloneNode(false);
            for (const item of rendering.recent) {
                const entry = document.createElement("li");
                const video = document.createElement("a");
                video.href = item.video;
                video.textContent = `Trial ${item.trial} animation`;
                const history = document.createElement("a");
                history.href = item.history;
                history.textContent = "Result and lineage";
                entry.append(video, " \u00b7 ", history);
                incoming.append(entry);
            }
            patch(renderLinks, incoming);
            recentRenders.hidden = rendering.recent.length === 0;
            recentRendering = recent;
        }
        if (state.revision !== revision) {
            const response = await fetch("index.html", {cache: "no-store"});
            if (!response.ok) throw new Error(`Dashboard update failed (${response.status})`);
            const incoming = new DOMParser().parseFromString(await response.text(), "text/html").getElementById("dashboard-content");
            if (!incoming) throw new Error("Dashboard update has no content region.");
            prepareCategories(incoming);
            archiveLinks(incoming);
            deferred = false;
            patch(document.getElementById("dashboard-content"), incoming);
            revision = deferred ? null : state.revision;
        }
        updateNavigation();
        const forest = document.getElementById("proposal-forest");
        if (forest) {
            const snapshot = await request("/api/forest");
            const incoming = new DOMParser().parseFromString(snapshot.html, "text/html").getElementById("proposal-forest");
            if (!incoming) throw new Error("Proposal forest update has no content region.");
            patch(forest, incoming);
        }
    }

    previewRetry.addEventListener("click", async () => {
        previewRetry.disabled = true;
        try {
            await request("/api/previews/retry", {method: "POST", headers: {"Content-Type": "application/json"},
                body: JSON.stringify({csrfToken: token})});
            await update();
        } catch (failure) {
            previewError.textContent = failure.message;
        } finally {
            previewRetry.disabled = false;
        }
    });

    button.addEventListener("click", async () => {
        starting = true;
        button.disabled = true;
        try {
            await request("/api/episode", {method: "POST", headers: {"Content-Type": "application/json"},
                body: JSON.stringify({csrfToken: token})});
            await update();
        } catch (failure) {
            error.textContent = failure.message;
        } finally {
            starting = false;
            button.disabled = running || !canStart;
        }
    });

    async function renderingControl(action) {
        renderingAction = true;
        renderStart.disabled = renderStop.disabled = true;
        try {
            await request(`/api/render/${action}`, {method: "POST",
                headers: {"Content-Type": "application/json"}, body: JSON.stringify({csrfToken: token})});
            renderingAction = false;
            await update();
        } catch (failure) {
            renderError.textContent = failure.message;
        } finally {
            renderingAction = false;
        }
    }
    renderStart.addEventListener("click", () => renderingControl("start"));
    renderStop.addEventListener("click", () => renderingControl("stop"));
    upgrade.addEventListener("click", async () => {
        upgrading = true;
        upgrade.disabled = true;
        upgradeStatus.textContent = "Preparing and validating the update; no trials will start.";
        try {
            const prepared = await request("/api/upgrade", {method: "POST",
                headers: {"Content-Type": "application/json"}, body: JSON.stringify({csrfToken: token})});
            if (prepared.version) {
                upgradeStatus.textContent = prepared.updated ? `Updated to ${prepared.version}. No trials started.` : "Already up to date.";
                await update();
                return;
            }
            const link = document.createElement("a");
            link.href = prepared.url;
            link.target = "_blank";
            link.rel = "noopener";
            link.textContent = `Open ${prepared.archive} (zero trials authorized)`;
            upgradeStatus.replaceChildren(link);
        } catch (failure) {
            upgradeStatus.textContent = `Upgrade failed: ${failure.message}`;
        } finally {
            upgrading = false;
            upgrade.disabled = running;
        }
    });

    async function poll() {
        try {
            await update();
        } catch (failure) {
            error.textContent = `Live update failed: ${failure.message}. Retrying; this page will not reload.`;
            button.disabled = true;
            renderStart.disabled = renderStop.disabled = true;
            upgrade.disabled = true;
        }
        setTimeout(poll, 2000);
    }
    updateNavigation();
    poll();
})();
