"""Cached presentation-only F# linkage poses; never edits or evaluates a candidate."""

from functools import lru_cache
import json
import math
from pathlib import Path
import subprocess
import tempfile
import threading

from .bridge import AdapterError, load_manifest
from .cycles import cycle_plan
from .evaluation import EvaluationOptions

_lock = threading.Lock()
STEPS = 72


def validate(data, program, turns):
    if (not isinstance(data, dict) or data.get("schemaVersion") != 1 or data.get("units") != "ldu"
            or data.get("mode") != "planar-kinematics-not-dynamics" or data.get("program") != program):
        raise AdapterError("Motion diagram output does not match the saved motion program.")
    frames = data.get("frames")
    if not isinstance(frames, list) or len(frames) != STEPS*turns+1:
        raise AdapterError("Motion diagram output has incomplete gait coverage.")
    bodies = {body["id"] for body in program["bodies"]}
    feet = {foot["id"] for foot in program["feet"]}

    def number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    def vector(value):
        return isinstance(value, list) and len(value) == 3 and all(number(v) for v in value)

    for index, frame in enumerate(frames):
        if (not isinstance(frame, dict) or not number(frame.get("inputAngle")) or
                not math.isclose(frame["inputAngle"], index*2*math.pi/STEPS, abs_tol=1e-9)
                or not number(frame.get("maxResidualLdu"))):
            raise AdapterError("Motion diagram contains invalid solved phase data.")
        poses, contacts = frame.get("poses"), frame.get("feet")
        if (not isinstance(poses, list) or len(poses) != len(bodies)
                or any(not isinstance(pose, dict) for pose in poses)
                or {pose.get("body") for pose in poses} != bodies
                or any(not vector(pose.get("origin")) or not number(pose.get("angle")) for pose in poses)
                or not isinstance(contacts, list) or len(contacts) != len(feet)
                or any(not isinstance(foot, dict) for foot in contacts)
                or {foot.get("id") for foot in contacts} != feet
                or any(not vector(foot.get("position")) for foot in contacts)):
            raise AdapterError("Motion diagram contains invalid solved body or foot positions.")


@lru_cache(maxsize=32)
def _solve(program_bytes, command, turns):
    with tempfile.TemporaryDirectory(prefix="legowalkers-motion-") as temporary:
        root = Path(temporary)
        source, output = root / "program.json", root / "solved"
        source.write_bytes(program_bytes)
        try:
            completed = subprocess.run(
                [*command, "motion", "--program", str(source), "--output", str(output),
                 "--steps", str(STEPS), "--turns", str(turns)],
                capture_output=True, text=True, timeout=60)
        except subprocess.TimeoutExpired as error:
            raise AdapterError("Motion diagram solver exceeded its 60-second presentation limit.") from error
        if completed.returncode:
            raise AdapterError(f"Motion diagram solver failed ({completed.returncode}): {completed.stderr.strip()}")
        data = json.loads((output / "motion.json").read_text())
        validate(data, json.loads(program_bytes)["program"], turns)
        return data


def solved_motion(candidate, command):
    manifest = load_manifest(candidate / "manifest.json")
    turns = cycle_plan(manifest, EvaluationOptions(speed=100, settle=0))["motorTurnsPerCycle"]
    if not 1 <= turns <= 32:
        raise AdapterError("Animated diagrams support full gait cycles of one to 32 motor turns.")
    program = (candidate / "program.json").read_bytes()
    # Serialize cache misses so multiple visible cards cannot spawn a solver herd.
    with _lock:
        data = _solve(program, tuple(map(str, command)), turns)
    bodies, feet = {}, {}
    for part in manifest["parts"]:
        bodies.setdefault(part["body"], []).append(part["position"][2])
    for foot in manifest["feet"]:
        feet.setdefault(foot.get("logicalFoot", foot["id"]), []).append(foot["position"][2])
    return {**data, "displayDepths": {
        "bodies": {body: sum(values)/len(values) for body, values in bodies.items()},
        "feet": {foot: sum(values)/len(values) for foot, values in feet.items()}}}
