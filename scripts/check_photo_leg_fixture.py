"""Check one isolated photo-leg fixture; never perform dynamics or write campaign data."""

import argparse
import json
from pathlib import Path

import numpy as np
from lego_technic_sim.physics.connection_ports import PortType
from lego_technic_sim.physics.unit_builder import _connector_shaft, _find_port_connections, _port_on_shaft

from legowalkers_sim.bridge import AdapterError, check, write_json
from legowalkers_sim.screening import _load_only


def check_engagement(checked):
    parts = checked.manifest["parts"]
    structural = [(index, geometry) for index, (part, geometry)
                  in enumerate(zip(parts, checked.build.parts, strict=True)) if part["body"] is not None]
    groups = {f"pivot-{group['id']}": set(group["bodies"])
              for group in checked.manifest["motion"]["sharedPivots"]}
    contacts = []
    for part, geometry in zip(parts, checked.build.parts, strict=True):
        if part["body"] is not None:
            continue
        start, end, axis = _connector_shaft(geometry)
        length = float(np.linalg.norm(end - start))
        matches = []
        for index, structural_part in structural:
            ports = [port for port in structural_part.ports if port.port_type != PortType.STUD
                     and _port_on_shaft(port, start, end, axis)]
            if ports:
                # All fixture sockets are standard thick 20-LDU sockets.
                if not any(10 - 1e-6 <= float(np.dot(port.position - start, axis))
                           <= length - 10 + 1e-6 for port in ports):
                    raise AdapterError(f"Connector does not cover the full socket wall: {part['id']}, {parts[index]['id']}")
                matches.append(index)
        retained = {index for index, _ in _find_port_connections(geometry, structural)}
        if set(matches) != retained:
            raise AdapterError(f"Upstream discarded over-capacity contacts at {part['id']}")
        if {parts[index]["body"] for index in matches} != groups.get(part["id"], {"chassis"}):
            raise AdapterError(f"Unexpected detected socket engagement at {part['id']}")
        contacts.append({"connector": part["id"], "retainedParts": [parts[i]["id"] for i in matches],
                         "shaftLengthLdu": length, "shaftStartLdu": start.tolist(), "shaftEndLdu": end.tolist(),
                         "fullSocketWalls": True, "droppedContacts": 0})
    return contacts


def verify(source, output, library):
    source, output = source.resolve(), output.resolve()
    if output.exists() and any(output.iterdir()):
        raise AdapterError(f"Check output must be empty: {output}")
    if output == source or source in output.parents:
        raise AdapterError("Keep check evidence outside the immutable fixture.")
    fixture = json.loads((source / "fixture.json").read_text())
    if (fixture["recipe"] not in {"photo-double-parallelogram-leg-fixture-v1",
                                "photo-double-parallelogram-leg-fixture-v2",
                                "photo-double-parallelogram-leg-fixture-v3",
                                "photo-double-parallelogram-leg-fixture-v4"}
            or fixture["nativeCandidate"] or fixture["dynamicTrials"] != 0):
        raise AdapterError("Expected the isolated, non-candidate, zero-dynamics fixture.")
    report = {"schemaVersion": 1, "classification": fixture["classification"],
              "assemblyId": fixture["assemblyId"], "dynamicTrials": 0,
              "simulationSeconds": 0, "status": "checking"}
    try:
        checked = check(source / "manifest.json", library)
        counts = {name: checked.report[name] for name in ("bodies", "joints", "motors", "gearMeshes")}
        if counts != {"bodies": 8, "joints": 10, "motors": 1, "gearMeshes": 0}:
            raise AdapterError(f"Unexpected fixture graph: {counts}")
        if checked.manifest["motion"]["assemblyId"] != fixture["assemblyId"]:
            raise AdapterError("Fixture identity does not match the checked manifest.")
        report["reconstruction"] = checked.report
        report["connectorEngagement"] = check_engagement(checked)
        report["loadOnly"] = _load_only(checked, output / "load-only")
        loaded = report["loadOnly"]
        if loaded["simulationSeconds"] != 0 or loaded["dynamicTrials"] != 0:
            raise AdapterError("Fixture checking must never advance time.")
        if loaded["maxClosureErrorMeters"] > 1e-6:
            raise AdapterError("Initial joint closure exceeds 1 micrometer.")
        report["status"] = "initial-interference" if loaded["initialSelfInterference"] else "pass"
        report["limits"] = [
            "Exact static reconstruction and initial pose only; not a swept-clearance proof.",
            "No dynamics, walking claim, native recipe compilation, search injection or campaign change.",
            "Direct motor is a fixture substitute for the unresolved photographed reduction train.",
        ]
    except AdapterError as error:
        report.update(status="rejected", error=str(error))
        write_json(output / "check.json", report)
        raise
    write_json(output / "check.json", report)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ldraw-library", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.input, args.output, args.ldraw_library), indent=2))
