"""Explicit payload mass replacement and approximate compound-body inertia."""

import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
from lego_technic_sim.ldraw.model import Triangle
from lego_technic_sim.physics.mesh_properties import ABS_DENSITY_KG_PER_M3, mesh_volume_and_com

from .bridge import AdapterError, LDU_TO_METERS
from .collision import to_mujoco, vector_text


def load_profile(path=None):
    profile = json.loads(Path(path or Path(__file__).with_name("payloads.json")).read_text(encoding="utf-8"))
    if (not isinstance(profile, dict) or profile.get("schemaVersion") != 1
            or not isinstance(profile.get("components"), dict)
            or set(profile["components"]) != {"8881", "8884"}):
        raise AdapterError("Payload profile must define schemaVersion 1 and components 8881/8884.")
    for product, part in (("8881", "58119.dat"), ("8884", "58123.dat")):
        item = profile["components"][product]
        if not isinstance(item, dict):
            raise AdapterError(f"Invalid payload component record: {product}.")
        mass, center = item.get("massKg"), item.get("centerLdu")
        if (item.get("part") != part or type(mass) not in (int, float) or not math.isfinite(mass)
                or mass <= 0 or not isinstance(center, list) or len(center) != 3
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in center)):
            raise AdapterError(f"Invalid payload mass/center/part for {product}.")
    return profile


def part_mass_center(part, payload=None):
    vertices = np.array([[tri.v0, tri.v1, tri.v2] for tri in part.geometry.triangles])
    if (vertices.ndim != 3 or vertices.shape[1:] != (3, 3)
            or not np.isfinite(vertices).all()):
        raise AdapterError(f"Invalid mass geometry: {part.instance}")
    lower = vertices.min(axis=(0, 1))*LDU_TO_METERS
    upper = vertices.max(axis=(0, 1))*LDU_TO_METERS
    if payload is None:
        # LDraw meshes are not always watertight; world-origin integration can change mass after translation.
        origin = part.geometry.position
        local = vertices-origin
        signed = np.einsum("ij,ij->i", local[:, 0], np.cross(local[:, 1], local[:, 2]))/6
        magnitude = float(np.abs(signed).sum())
        if magnitude == 0 or abs(float(signed.sum())) <= 64*np.finfo(float).eps*magnitude:
            raise AdapterError(f"Cancelled signed mesh volume: {part.instance}; check BFC/reflection winding.")
        triangles = [Triangle(tri.v0-origin, tri.v1-origin, tri.v2-origin) for tri in part.geometry.triangles]
        volume, local_center = mesh_volume_and_com(triangles)
        center = local_center+origin*LDU_TO_METERS
        mass = volume * ABS_DENSITY_KG_PER_M3
    else:
        mass = payload["massKg"]
        center = (part.geometry.transform[:3, :3] @ np.array(payload["centerLdu"])
                  + part.geometry.position) * LDU_TO_METERS
    if not math.isfinite(mass) or mass <= 0 or not np.isfinite(center).all():
        raise AdapterError(f"Invalid mass properties: {part.instance}")
    if np.any(center < lower-1e-6) or np.any(center > upper+1e-6):
        raise AdapterError(f"Mass-property component center outside physical geometry: {part.instance}; "
                           "check mesh winding or payload profile.")
    if payload is None and volume > float(np.prod(upper-lower))*(1+1e-6):
        raise AdapterError(f"Mesh volume exceeds physical bounds: {part.instance}; check mesh winding.")
    return mass, center


def apply_payloads(scene, parts, manifest, profile):
    payloads = {payload["partInstance"]: profile["components"][payload["product"]]
                for payload in manifest.get("payloads", [])}
    if not payloads:
        return scene, {}
    affected = {part.unit for part in parts if part.instance in payloads}
    units, tensors = list(scene.units), {}
    for index in affected:
        properties = []
        for part in parts:
            if part.unit != index:
                continue
            vertices = np.array([v for triangle in part.geometry.triangles
                                 for v in (triangle.v0, triangle.v1, triangle.v2)]) * LDU_TO_METERS
            mass, center = part_mass_center(part, payloads.get(part.instance))
            size = np.ptp(vertices, axis=0)
            own_inertia = np.diag(mass / 12 * (np.sum(size * size) - size * size))
            properties.append((mass, center, own_inertia))
        mass = sum(value[0] for value in properties)
        center = sum(m * c for m, c, _ in properties) / mass
        tensor = np.zeros((3, 3))
        for m, c, own in properties:
            offset = c - center
            tensor += own + m * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
        transform = to_mujoco(np.eye(3)).T
        tensors[index] = transform @ tensor @ transform.T
        units[index] = replace(units[index], mass=mass, center_of_mass=center)
    return replace(scene, units=units), tensors


def install_inertia(bodies, tensors):
    for index, tensor in tensors.items():
        if tensor.shape != (3, 3) or not np.isfinite(tensor).all():
            raise AdapterError(f"Invalid inertia tensor for unit_{index}.")
        if not np.allclose(tensor, tensor.T, rtol=0, atol=1e-12):
            raise AdapterError(f"Asymmetric inertia tensor for unit_{index}.")
        moments = np.linalg.eigvalsh(tensor)
        if moments[0] <= 0 or moments[2] > moments[0]+moments[1]+1e-12:
            raise AdapterError(f"Nonphysical inertia tensor for unit_{index}.")
        inertial = bodies[f"unit_{index}"].find("inertial")
        inertial.attrib.pop("diaginertia", None)
        inertial.attrib.pop("quat", None)
        inertial.set("fullinertia", vector_text(
            [tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]]))
