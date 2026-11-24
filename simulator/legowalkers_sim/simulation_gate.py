"""Read-only early rejection of an irreversible failure in a trial's prefix."""

import math
from pathlib import Path
import time

from .bridge import AdapterError, write_json


class SimulationGateRejected(AdapterError):
    """An expected scientific rejection, not a simulation worker failure."""


class GateAttemptLimit(AdapterError):
    """The current explicit episode has exhausted its cheap-attempt allowance."""


def validate_short_gate(seconds, timestep):
    if (isinstance(seconds, bool) or not isinstance(seconds, (int, float))
            or not math.isfinite(seconds) or not 0 <= seconds <= 60):
        raise AdapterError("short_gate must be finite and between 0 and 60 seconds; 0 disables it.")
    if not math.isclose(seconds / timestep, round(seconds / timestep), abs_tol=1e-7):
        raise AdapterError("short_gate must be a whole number of timesteps.")


class SimulationPrefixGate:
    def __init__(self, output: Path, seconds, timestep, settle_steps, total_steps, *, on_pass=None):
        self.output = output
        self.seconds = seconds
        self.timestep = timestep
        self.settle_steps = settle_steps
        self.total_steps = total_steps
        self.end_step = min(total_steps, settle_steps + round(seconds / timestep))
        self.active = seconds > 0
        self.report = None
        self.started = time.perf_counter()
        self.on_pass = on_pass

    def observe(self, step, simulated_time, max_tilt, max_closure, penetrations, ground, frames,
                closure_diagnosis=None, stance_diagnosis=None):
        if not self.active:
            return
        maximum_penetration = max((entry["depthMeters"] for entry in penetrations.values()), default=0.0)
        reasons = [reason for invalid, reason in (
            (maximum_penetration > 0.0001, "self-interference"),
            (ground["forbiddenContactDetected"], "forbidden-ground-contact"),
            (max_tilt > math.pi / 3, "excessive-tilt"),
            (max_closure > 0.002, "joint-closure-error"),
        ) if invalid]
        if not reasons and step < self.end_step:
            return
        self.report = {
            "schemaVersion": 1, "version": "simulation-prefix-gate-v1",
            "status": "rejected" if reasons else "passed",
            "drivenSeconds": self.seconds,
            "plannedPrefixSeconds": self.end_step * self.timestep,
            "plannedEvaluationSeconds": self.total_steps * self.timestep,
            "simulatedSeconds": simulated_time, "simulatedSteps": step,
            "measurementSeconds": max(0, step - self.settle_steps) * self.timestep,
            "remainingSteps": self.total_steps - step,
            "prefixWallSeconds": time.perf_counter() - self.started,
            "reasons": reasons,
            "diagnostics": {
                "maxSelfPenetrationMeters": maximum_penetration,
                "selfContacts": sorted(penetrations.values(), key=lambda entry: entry["depthMeters"], reverse=True),
                "maxTiltDegrees": math.degrees(max_tilt),
                "maxClosureErrorMeters": max_closure,
                "groundContact": ground,
            },
        }
        if closure_diagnosis is not None:
            diagnosis = closure_diagnosis()
            self.report["diagnostics"]["closureDiagnosis"] = diagnosis
            write_json(self.output / "closure-diagnosis.json", diagnosis)
        if stance_diagnosis is not None:
            self.report["diagnostics"]["stanceSupport"] = stance_diagnosis()
        write_json(self.output / "simulation-gate.json", self.report)
        self.active = False
        if reasons:
            write_json(self.output / "trajectory.json", {"schemaVersion": 1, "frames": frames})
            raise SimulationGateRejected(
                f"Short simulation gate rejected at t={simulated_time:.6f}: {', '.join(reasons)}. "
                "No partial fitness. See simulation-gate.json."
            )
        if self.on_pass is not None:
            self.on_pass()
