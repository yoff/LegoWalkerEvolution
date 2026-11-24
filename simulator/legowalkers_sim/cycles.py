"""Bounded gait coverage derived from the reconstructed physical gear graph."""

from dataclasses import replace
from fractions import Fraction
import math

from .bridge import AdapterError

VERSION = "gait-cycles-v1"
REQUIRED_CYCLES = 2
MAX_DURATION = 60.0


def cycle_plan(manifest, options):
    expected = manifest["expected"]
    drives = expected["drives"]
    if len(drives) != 1:
        raise AdapterError("Cycle-aware evaluation requires exactly one coupled motor.")
    chassis = manifest["chassis"]
    joints = {joint["id"]: joint for joint in expected["joints"]}
    bearing = joints[drives[0]["joint"]]
    if chassis not in (bearing["bodyA"], bearing["bodyB"]):
        raise AdapterError("Cycle-aware motor input must be relative to the chassis.")
    motor_axis = bearing.get("axis")
    if motor_axis is not None:
        for joint in joints.values():
            axis = joint.get("axis")
            if axis is None:
                continue
            norms = [math.sqrt(sum(value*value for value in vector)) for vector in (motor_axis, axis)]
            if any(not math.isfinite(norm) or norm < 1e-12 for norm in norms):
                raise AdapterError("Gait cycle planning requires finite nonzero hinge axes.")
            alignment = abs(sum(a*b for a, b in zip(motor_axis, axis)))/(norms[0]*norms[1])
            if alignment < 1-1e-7:
                raise AdapterError("Spatial gait cycle planning requires a verified moving-frame repeat period; "
                                   "the planar gear-rate planner cannot certify it.")
    root = bearing["bodyB"] if bearing["bodyA"] == chassis else bearing["bodyA"]
    gear_edges = {}
    for gear in expected["gears"]:
        ratio = gear["ratio"]
        if isinstance(ratio, bool) or not math.isfinite(ratio) or ratio <= 0:
            raise AdapterError("Gear ratios must be finite and positive.")
        rational = Fraction(str(ratio)).limit_denominator(4096)
        if not math.isclose(float(rational), ratio, rel_tol=1e-10, abs_tol=1e-12):
            raise AdapterError("Gear ratio has no supported exact rational representation.")
        a, b = gear["bodyA"], gear["bodyB"]
        gear_edges.setdefault(a, []).append((b, -rational))
        gear_edges.setdefault(b, []).append((a, -1/rational))
    rates = {root: Fraction(1)}
    pending = [root]
    while pending:
        a = pending.pop()
        for b, ratio in gear_edges.get(a, []):
            rate = rates[a]*ratio
            if b in rates:
                if rates[b] != rate:
                    raise AdapterError("Inconsistent coupled gear ratios.")
            else:
                rates[b] = rate
                pending.append(b)
    if set(gear_edges) - rates.keys():
        raise AdapterError("Every gear train must be coupled to the motor.")
    adjacency = {}
    for joint in expected["joints"]:
        a, b = joint["bodyA"], joint["bodyB"]
        if chassis not in (a, b):
            adjacency.setdefault(a, set()).add(b)
            adjacency.setdefault(b, set()).add(a)
    feet = {}
    relevant = set()
    for foot in manifest["feet"]:
        reached, todo, influences = set(), [foot["body"]], set()
        while todo:
            body = todo.pop()
            if body in reached:
                continue
            reached.add(body)
            if body in rates:
                influences.add(body)
            else:
                todo.extend(adjacency.get(body, set()) - reached)
        if not influences:
            raise AdapterError(f"Foot {foot['id']} has no supported motor-coupled input.")
        relevant.update(influences)
        feet[foot["id"]] = {body: [rates[body].numerator, rates[body].denominator] for body in sorted(influences)}
    turns = math.lcm(*(rates[body].denominator for body in relevant))
    seconds = 2*math.pi*turns/abs(options.speed)
    requested = options.settle + REQUIRED_CYCLES*seconds
    if requested > MAX_DURATION + 1e-9:
        raise AdapterError(f"Gait cycle exceeds the evaluation envelope: {turns} motor turns per cycle; "
                           f"{REQUIRED_CYCLES} cycles require {requested:.3f} s, cap {MAX_DURATION:.0f} s.")
    duration = max(options.duration, math.ceil((requested-1e-10)/options.timestep)*options.timestep)
    if duration > MAX_DURATION:
        raise AdapterError("Cycle-aware duration exceeds the fixed evaluation cap.")
    return {"version": VERSION, "motorTurnsPerCycle": turns, "nominalCycleSeconds": seconds,
            "requiredCycles": REQUIRED_CYCLES, "durationSeconds": duration,
            "maximumDurationSeconds": MAX_DURATION, "feet": feet}


def planned_options(manifest, options):
    if not options.cycle_aware:
        return options, None
    plan = cycle_plan(manifest, options)
    return replace(options, duration=plan["durationSeconds"]), plan


class CycleProgress:
    def __init__(self, plan, speed):
        self.turns = plan["motorTurnsPerCycle"]
        self.direction = math.copysign(1, speed)
        self.previous = None
        self.next_boundary = self.turns
        self.boundary = None
        self.intervals = []
        self.maximum = 0.0

    def observe(self, time, motor_turns, forward):
        progress = self.direction*motor_turns
        current = (time, progress, forward)
        if self.previous is None:
            self.previous = current
            self.boundary = (time, forward)
        old_time, old_progress, old_forward = self.previous
        while progress >= self.next_boundary and progress > old_progress:
            fraction = (self.next_boundary-old_progress)/(progress-old_progress)
            boundary = (old_time+fraction*(time-old_time), old_forward+fraction*(forward-old_forward))
            elapsed = boundary[0]-self.boundary[0]
            distance = boundary[1]-self.boundary[1]
            self.intervals.append({"seconds": elapsed, "forwardMeters": distance,
                                   "speedMetersPerSecond": distance/elapsed})
            self.boundary = boundary
            self.next_boundary += self.turns
        self.maximum = max(self.maximum, progress)
        self.previous = current

    def report(self):
        return {"completeCycles": len(self.intervals), "cycleIntervals": self.intervals,
                "cycleCoverage": self.maximum/self.turns,
                "repeatableForwardMotion": len(self.intervals) >= REQUIRED_CYCLES
                and all(interval["forwardMeters"] > 0 for interval in self.intervals),
                "minimumCycleSpeedMetersPerSecond": min(
                    (interval["speedMetersPerSecond"] for interval in self.intervals), default=0.0)}