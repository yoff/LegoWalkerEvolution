"""Conservative steady-state PF XL calibration, measured at regulated 9 V."""

import math

from .bridge import AdapterError

MODEL = "pf-xl-9v"
VERSION = "pf-xl-torque-speed-v2"
NO_LOAD_SPEED = 220.0 * 2.0 * math.pi / 60.0
LOAD_SPEED = 146.0 * 2.0 * math.pi / 60.0
LOAD_TORQUE = 0.145
STALL_LIMIT = 0.4
SLOPE = LOAD_TORQUE / (NO_LOAD_SPEED - LOAD_SPEED)
SOURCE = "https://www.philohome.com/motors/motorcomp.htm"


def validate(model, speed, torque):
    if model not in (MODEL, "ideal-servo"):
        raise AdapterError(f"Unknown motor model: {model}.")
    if model == MODEL:
        if abs(speed) > NO_LOAD_SPEED + 1e-10:
            raise AdapterError("PF XL speed requests cannot exceed the measured 9 V no-load speed.")
        if torque > STALL_LIMIT:
            raise AdapterError("PF XL torque cannot exceed its conservative 0.4 Nm stall limit; use gearing.")


def torque_at(speed, requested_speed, limit=STALL_LIMIT):
    if not all(math.isfinite(x) for x in (speed, requested_speed, limit)):
        raise AdapterError("Motor speed and torque limit must be finite.")
    if requested_speed == 0 or limit <= 0:
        raise AdapterError("Driven motor requests require nonzero speed and a positive torque limit.")
    validate(MODEL, requested_speed, limit)
    duty = abs(requested_speed) / NO_LOAD_SPEED
    cap = min(limit, STALL_LIMIT) * duty
    return max(-cap, min(cap, SLOPE * (requested_speed - speed)))


def definition(model, speed, torque):
    validate(model, speed, torque)
    if model == "ideal-servo":
        return {"model": model, "authority": "preview-only", "torqueLimitNm": torque}
    return {
        "model": model, "version": VERSION, "source": SOURCE,
        "supportedParts": ["58121.dat", "58121c01.dat"],
        "supplyVolts": 9.0, "supply": "regulated; battery sag and receiver losses not modelled",
        "noLoadRpm": 220.0,
        "loadedPoint": {"rpm": 146.0, "torqueNm": LOAD_TORQUE, "currentAmps": 0.55},
        "noLoadCurrentAmps": 0.08, "measuredStallCurrentAmps": 1.8,
        "measuredStallTorqueNm": STALL_LIMIT, "stallTorqueMeasurement": "very imprecise",
        "torqueSlopeNmPerRadPerSecond": SLOPE,
        "fittedStallTorqueNm": SLOPE * NO_LOAD_SPEED,
        "torqueLimitNm": torque, "pwmDuty": abs(speed) / NO_LOAD_SPEED,
        "controller": "open-loop voltage/PWM; no velocity-holding servo",
        "lowDutyApproximation": "linear averaged voltage; stall cap scaled by duty",
        "settling": "unpowered/float",
        "motorJointDamping": "zero; generic exporter damping would double-count calibrated motor drag",
        "limitations": ["steady-state approximation", "no thermal protection",
                       "no PWM switching transients", "no calibrated external gear losses"],
    }


def install(root, manifest, options):
    validate(options.motor_model, options.speed, options.torque)
    actuators = root.findall("actuator/velocity")
    if len(actuators) != len(manifest["expected"]["drives"]):
        raise AdapterError("Exported motor actuators do not match declared drives.")
    if options.motor_model == MODEL:
        parts = {part["id"]: part["part"] for part in manifest["parts"]}
        if any(parts.get(drive["motorPart"]) not in ("58121.dat", "58121c01.dat")
               for drive in manifest["expected"]["drives"]):
            raise AdapterError("The calibrated 9 V motor model requires a Power Functions XL motor.")
    for actuator in actuators:
        if options.motor_model == MODEL:
            hinge = root.find(f".//joint[@name='{actuator.attrib['joint']}']")
            if hinge is None:
                raise AdapterError("Calibrated motor actuator has no declared physical hinge.")
            hinge.set("damping", "0")
            actuator.tag = "motor"
            actuator.attrib.pop("kv", None)
        actuator.set("forcelimited", "true")
        actuator.set("forcerange", f"-{options.torque} {options.torque}")
        actuator.set("ctrllimited", "true")
        limit = options.torque if options.motor_model == MODEL else abs(options.speed)
        actuator.set("ctrlrange", f"-{limit} {limit}")
