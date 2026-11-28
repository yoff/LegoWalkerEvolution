import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest


def test_symbolic_trudge_explains_exhausted_domains_and_preserves_repairs():
    pytest.importorskip("z3", reason="Install the optional [solver] extra to exercise Z3.")
    root = Path(__file__).resolve().parents[2]
    solver = shutil.which("z3") or str(root / ".venv/bin/z3")
    core = Path(os.environ.get(
        "LEGOWALKERS_CORE_DLL",
        str(root / "src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"),
    ))
    result = subprocess.run(
        [str(root / ".tools/dotnet/dotnet"), "fsi", f"--reference:{core}",
         str(root / "examples/trudge-constraints.fsx"), solver],
        cwd=root, text=True, capture_output=True, timeout=30, check=False,
    )
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["unsatCore"] == [{"goal": "scaffold", "choice": "inherited"}]
    assert report["untestedRoutePreserved"]
    assert report["repairInvalidatedOldDomains"]
    assert {item["goal"] for item in report["jointAllocation"]} == {"motor-bearing-support", "battery-support"}
    assert report["sharedSocketReserved"]
    assert report["retainedSocketRespected"]
