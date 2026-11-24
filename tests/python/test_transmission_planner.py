from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def test_fsharp_fresh_transmission_pruning_phase_and_polarity():
    result = subprocess.run([
        str(ROOT / ".tools/dotnet/dotnet"), "fsi",
        str(ROOT / "scripts/transmission_planner.fsx"), "--self-test",
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr + result.stdout
    assert "Transmission self-tests passed" in result.stdout
