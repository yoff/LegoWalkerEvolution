import json
import threading
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from legowalkers_sim.dashboard import DashboardHandler
from legowalkers_sim.evolution import Generator
from legowalkers_sim import workbench_diagrams
from test_campaign import setup
from test_workbench import Manual


def test_workbench_http_fork_branch_assets_and_authorization_do_not_start_simulation(setup, monkeypatch):
    controller = setup.create()
    manual = Manual()
    monkeypatch.setattr(Generator, "manual_edits", lambda self, parent: manual.manual_edits(parent))
    monkeypatch.setattr(Generator, "apply_edit", lambda self, parent, output, edit: manual.apply_edit(parent, output, edit))
    monkeypatch.setattr(workbench_diagrams, "model_diagram", lambda *args: {
        "parts": [], "feet": [], "joints": [], "paths": []})
    with ThreadingHTTPServer(("127.0.0.1", 0), DashboardHandler) as server:
        server.controller = controller
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"

        def get(path):
            with urlopen(base + path) as response:
                return json.load(response)

        def post(value, origin=None):
            headers = {"Content-Type": "application/json"}
            if origin:
                headers["Origin"] = origin
            with urlopen(Request(base + "/api/workbench", data=json.dumps(value).encode(), headers=headers)) as response:
                return json.load(response)

        try:
            initial = get("/api/workbench")
            assert initial["workspaces"] == [] and initial["sources"]
            source = initial["sources"][0]
            request = {"csrfToken": controller.token, "action": "fork", "version": source["version"],
                       "birth": source["birth"], "label": "HTTP workspace", "author": "<script>not markup</script>"}
            with pytest.raises(HTTPError) as error:
                post({**request, "csrfToken": "wrong"})
            assert error.value.code == 403
            with pytest.raises(HTTPError) as error:
                post(request, "https://example.invalid")
            assert error.value.code == 403
            with pytest.raises(HTTPError) as error:
                post({**request, "unexpected": True})
            assert error.value.code == 409
            with pytest.raises(HTTPError) as error:
                post({"csrfToken": controller.token, "action": []})
            assert error.value.code == 409
            assert get("/api/workbench")["workspaces"] == []
            root = post(request)
            assert get("/api/workbench?edits=" + root["node"])["edits"][1]["level"] == "realization"
            preview_request = {"csrfToken": controller.token, "action": "preview", "node": root["node"], "edit": "a"*64}
            before_preview = get("/api/workbench")
            with pytest.raises(HTTPError) as error:
                post({**preview_request, "csrfToken": "wrong"})
            assert error.value.code == 403
            with pytest.raises(HTTPError) as error:
                post(preview_request, "https://example.invalid")
            assert error.value.code == 403
            preview = post(preview_request)
            assert preview["dynamicTrials"] == 0 and len(preview["alternatives"]) == 1
            assert get("/api/workbench") == before_preview
            child = post({"csrfToken": controller.token, "action": "apply", "node": root["node"], "edit": "a"*64})["children"][0]
            post({"csrfToken": controller.token, "action": "keep", "node": child})
            state = get("/api/workbench")
            assert state["workspaces"][0]["head"] == child and len(state["nodes"]) == 2
            with urlopen(base + f"/workbench-assets/{child}/cycle.svg") as response:
                assert "image/svg+xml" in response.headers["Content-Type"]
                assert b"<svg" in response.read()
            with urlopen(Request(base + f"/workbench-assets/{child}/cycle.svg", method="HEAD")) as response:
                assert response.status == 200 and not response.read()
            with urlopen(base + "/workbench.html") as response:
                page = response.read().decode()
                assert "Breeding workbench" in page and "Editing level" in page
                contribution_form = page.split('id="contribute-form"', 1)[1].split("</form>", 1)[0]
                assert 'id="contribution-author"' in contribution_form
                assert 'id="contribute-submit"' in contribution_form
                assert 'id="version-tree" aria-label="Workspace version tree"' in page
                assert 'id="contribution-target"' in page
                assert 'id="contribution-message"' in page and "User-contributed pool" in page
                assert 'id="rebase-form"' in page and 'id="rebase-source"' in page
                assert "<script>not markup</script>" not in page
            with urlopen(base + "/workbench.js") as response:
                assert response.status == 200 and "javascript" in response.headers["Content-Type"]
            rebase = {"csrfToken": controller.token, "action": "rebase", "node": child,
                      "version": state["activeVersion"], "birth": source["birth"]}
            with pytest.raises(HTTPError) as error:
                post({**rebase, "version": "v9999"})
            assert error.value.code == 409
            rebased = post(rebase)
            assert rebased["summary"]["operation"] == "rebase" and not rebased["summary"]["fitnessImported"]
            state = get("/api/workbench")
            assert next(node for node in state["nodes"] if node["id"] == rebased["node"])["parent"] == child
            assert controller.status()["used"] == 1 and controller.status()["budget"] == 3
            assert setup.launched == []
        finally:
            server.shutdown()
            thread.join(timeout=5)
