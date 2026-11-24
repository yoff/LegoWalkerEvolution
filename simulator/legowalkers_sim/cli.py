from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from .bridge import AdapterError, check, write_json
from .evaluation import EvaluationOptions, evaluate
from .simulation_gate import SimulationGateRejected
from . import motor


def parser():
    root = argparse.ArgumentParser(description="Validate and evaluate F# LegoWalkers exports.")
    commands = root.add_subparsers(dest="command", required=True)
    screen = commands.add_parser("screen", help="Screen a bounded realization batch without running physics episodes")
    screen.add_argument("input", type=Path, help="F# realize output containing candidates.json")
    screen.add_argument("--output", type=Path, required=True)
    screen.add_argument("--ldraw-library", type=Path, default=os.environ.get("LDRAW_LIBRARY"))
    screen.add_argument("--max-checks", type=int, default=32)
    screen.add_argument("--payload-profile", type=str)
    screen.add_argument("--no-model-load", action="store_true", help="Stop at reconstruction; do not report ready-for-dynamics")
    for command in ("check", "evaluate", "batch"):
        item = commands.add_parser(command)
        item.add_argument("input", type=Path, help="manifest.json, or a sweep directory for batch")
        item.add_argument("--ldraw-library", type=Path, default=os.environ.get("LDRAW_LIBRARY"))
        item.add_argument("--output", type=Path, required=command != "check")
        if command != "check":
            item.add_argument("--duration", type=float, default=5.0)
            item.add_argument("--settle", type=float, default=1.0)
            item.add_argument("--short-gate", type=float, default=0.0,
                              help="Early safety gate for this many driven seconds after settling; 0 disables (default)")
            item.add_argument("--timestep", type=float, default=0.001)
            item.add_argument("--floor-contact-time-constant", type=float,
                              help="Part-floor contact time constant in seconds; critical damping, at least two timesteps. "
                                   "Omit to preserve MuJoCo defaults. Does not change self-contact or loop constraints.")
            item.add_argument("--friction", type=float, default=1.0)
            item.add_argument("--speed", type=float, default=motor.NO_LOAD_SPEED,
                              help="Requested unloaded motor speed; calibrated default is full-power 9 V.")
            item.add_argument("--torque", type=float, default=0.4)
            item.add_argument("--motor-model", choices=(motor.MODEL, "ideal-servo"), default=motor.MODEL)
            item.add_argument("--payload-profile", type=str)
            item.add_argument("--capture-poses", action="store_true")
            item.add_argument("--cycle-aware", action="store_true", help="Require two repeatable gait cycles within a fixed 60-second cap")
            collision = item.add_mutually_exclusive_group()
            collision.add_argument("--self-collision", dest="self_collision", action="store_true",
                                   help="Part hulls and thin axle rods (default)")
            collision.add_argument("--ground-only", dest="self_collision", action="store_false",
                                   help="Preview only: disable self-contact; never rank this run")
            item.set_defaults(self_collision=True)
        if command == "evaluate":
            item.add_argument("--anchor", action="store_true", help="Fix chassis; not a walking-fitness test")
    return root


def run(args):
    if args.ldraw_library is None:
        raise AdapterError("Supply --ldraw-library or set LDRAW_LIBRARY.")
    if args.command == "screen":
        from .screening import screen_batch
        return screen_batch(args.input, args.output, args.ldraw_library,
                            max_checks=args.max_checks, load_model=not args.no_model_load,
                            payload_profile=args.payload_profile)
    if args.command == "check":
        report = check(args.input, args.ldraw_library).report
        if args.output:
            write_json(args.output, report)
        return report
    options = EvaluationOptions(
        duration=args.duration, settle=args.settle, timestep=args.timestep,
        friction=args.friction, speed=args.speed, torque=args.torque,
        anchor=getattr(args, "anchor", False), self_collision=args.self_collision,
        payload_profile=args.payload_profile, capture_poses=args.capture_poses,
        cycle_aware=args.cycle_aware,
        short_gate=args.short_gate,
        motor_model=args.motor_model,
        floor_contact_time_constant=args.floor_contact_time_constant,
    )
    options.validate()
    if args.command == "evaluate":
        return evaluate(check(args.input, args.ldraw_library), args.output, options)
    if args.output.exists() and any(args.output.iterdir()):
        raise AdapterError("Batch output directory must be empty.")
    summary = json.loads((args.input / "sweep.json").read_text(encoding="utf-8"))
    if not isinstance(summary, dict) or summary.get("schemaVersion") != 1 or not isinstance(summary.get("generated"), list):
        raise AdapterError("Invalid sweep summary.")
    names = []
    for entry in summary["generated"]:
        if not isinstance(entry, dict) or not isinstance(entry.get("directory"), str) or not entry["directory"]:
            raise AdapterError("Every sweep entry must contain a nonempty directory name.")
        names.append(entry["directory"])
    if not names or len(names) != len(set(names)):
        raise AdapterError("A sweep must contain unique candidate directories.")
    outcomes = []
    for entry in summary["generated"]:
        candidate = (args.input / entry["directory"]).resolve()
        if candidate.parent != args.input.resolve():
            raise AdapterError("Sweep candidates must be immediate subdirectories.")
        destination = args.output / candidate.name
        try:
            result = evaluate(check(candidate / "manifest.json", args.ldraw_library), destination, options)
            if result["status"] != "evaluated":
                print(f"ERROR [{candidate.name}]: {result['status']}; see result.json", file=sys.stderr)
            outcomes.append({"candidate": candidate.name, "status": result["status"], "metrics": result["metrics"]})
        except SimulationGateRejected as error:
            print(f"REJECTED [{candidate.name}]: {error}", file=sys.stderr)
            rejected = {"candidate": candidate.name, "status": "rejected", "error": str(error)}
            write_json(destination / "rejection.json", rejected)
            outcomes.append(rejected)
        except (AdapterError, OSError) as error:
            print(f"ERROR [{candidate.name}]: {error}", file=sys.stderr)
            failed = {"candidate": candidate.name, "status": "failed", "error": str(error)}
            write_json(destination / "error.json", failed)
            outcomes.append(failed)
    ranked = sorted(
        (outcome for outcome in outcomes if outcome["status"] == "evaluated" and outcome["metrics"]["eligibleForRanking"]),
        key=lambda outcome: outcome["metrics"]["speedMetersPerSecond"], reverse=True,
    )
    result = {"schemaVersion": 1, "status": "completed", "outcomes": outcomes,
              "rankedCandidates": [outcome["candidate"] for outcome in ranked],
              "failures": sum(outcome["status"] not in ("evaluated", "rejected") for outcome in outcomes),
              "rejections": sum(outcome["status"] == "rejected" for outcome in outcomes)}
    write_json(args.output / "batch.json", result)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        result = run(args)
    except SimulationGateRejected as error:
        print(f"REJECTED: {error}", file=sys.stderr)
        write_json(args.output / "rejection.json", {"status": "rejected", "error": str(error)})
        return 1
    except (AdapterError, OSError, json.JSONDecodeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    if result.get("status") == "interference":
        print("ERROR: Self-interference detected; see the collision diagnostics in result.json.", file=sys.stderr)
    screening_rejected = args.command == "screen" and any(
        result["counts"][key] for key in ("rejected", "motionRejections", "screeningRejections", "cachedRejections"))
    if screening_rejected:
        print("ERROR: Candidates were rejected; see the persisted screening gate diagnostics.", file=sys.stderr)
    return 1 if result.get("failures", 0) or result.get("rejections", 0) or result.get("status") == "interference" or screening_rejected else 0
