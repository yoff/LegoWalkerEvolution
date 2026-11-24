"""Carrier-relative torque coupling for physically detected right-angle gears."""

import math
from xml.etree import ElementTree as ET

import numpy as np
from lego_technic_sim.physics.gears import MODULE_LDU, _gear_info
from lego_technic_sim.physics.model import JointType, PhysicsScene

from .bridge import AdapterError, LDU_TO_METERS
from .collision import to_mujoco


def _pitch_witness(scene, gear, axes, position):
    candidates = []
    for a in scene.units[gear.unit_a_index].bricks:
        info_a = _gear_info(a)
        if info_a is None:
            continue
        teeth_a, kind_a, center_a, axis_a = info_a
        for b in scene.units[gear.unit_b_index].bricks:
            info_b = _gear_info(b)
            if info_b is None:
                continue
            teeth_b, kind_b, center_b, axis_b = info_b
            if (kind_a not in ("bevel", "double_bevel") or kind_b not in ("bevel", "double_bevel")
                    or not math.isclose(teeth_a/teeth_b, gear.ratio, rel_tol=1e-7)
                    or np.linalg.norm(axis_a-axes[0]) > 1e-7
                    or np.linalg.norm(axis_b-axes[1]) > 1e-7):
                continue
            offsets = [center_a-position, center_b-position]
            distances = [float(np.dot(delta, axis)) for delta, axis in zip(offsets, axes)]
            # At 90 degrees, each cone's axial distance is the OTHER gear's
            # pitch radius. Upstream's loose detector alone does not prove this.
            if (any(np.linalg.norm(np.cross(delta, axis)) > 1e-5
                    for delta, axis in zip(offsets, axes))
                    or not math.isclose(abs(distances[0]), teeth_b*MODULE_LDU, abs_tol=1e-5)
                    or not math.isclose(abs(distances[1]), teeth_a*MODULE_LDU, abs_tol=1e-5)):
                continue
            candidates.append(math.copysign(1., distances[0]*distances[1]))
    if len(candidates) != 1:
        raise AdapterError("Bevel coupling requires one stock gear pair with witnessed pitch geometry.")
    return candidates[0]


def install_bevel_couplings(root: ET.Element, scene: PhysicsScene) -> dict:
    """Replace two-joint approximations; never add a gear or an actuator.

    On a yaw carrier, the mesh equation is
    mesh_sign * ratio * (input_angle - carrier_angle) + output_relative_angle = 0.
    A fixed tendon gives reaction torque to all three independent hinges.
    """
    report = {"schemaVersion": 1, "version": "carrier-bevel-v1", "couplings": []}
    perpendicular = []
    for index, gear in enumerate(scene.gears):
        axes = [np.asarray(axis, dtype=float) for axis in (gear.axis_a, gear.axis_b)]
        if any(axis.shape != (3,) or not np.isfinite(axis).all()
               or np.linalg.norm(axis) < 1e-12 for axis in axes):
            raise AdapterError(f"Gear {index} has an invalid detected axis.")
        axes = [axis / np.linalg.norm(axis) for axis in axes]
        dot = abs(float(np.dot(*axes)))
        if dot < .9:
            if dot > 1e-7:
                raise AdapterError(f"Gear {index} is neither parallel nor an exact right-angle mesh.")
            if not math.isfinite(gear.ratio) or gear.ratio <= 0:
                raise AdapterError(f"Gear {index} has an invalid detected ratio.")
            perpendicular.append((index, gear, axes))
    if not perpendicular:
        return report

    elements = {int(body.attrib["name"].removeprefix("unit_")): body
                for body in root.findall(".//body") if body.get("name", "").startswith("unit_")}
    parents = {}
    for parent, element in elements.items():
        if any(key in element.attrib for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
            raise AdapterError("Bevel export requires the upstream translation-only initial body frames.")
        for child in element.findall("body"):
            if child.get("name", "").startswith("unit_"):
                parents[int(child.attrib["name"].removeprefix("unit_"))] = parent

    def hinge(unit):
        element = elements.get(unit)
        joints = [] if element is None else element.findall("joint")
        if len(joints) != 1 or joints[0].get("type", "hinge") != "hinge" or unit not in parents:
            raise AdapterError(f"Bevel unit {unit} needs one independent tree hinge.")
        xml_joint = joints[0]
        name = xml_joint.get("name", "")
        try:
            index = int(name.removeprefix("joint_"))
        except ValueError as error:
            raise AdapterError(f"Unrecognized bevel hinge {name}.") from error
        if not 0 <= index < len(scene.joints):
            raise AdapterError(f"Unknown detected bevel hinge {name}.")
        joint = scene.joints[index]
        if (joint.joint_type != JointType.REVOLUTE
                or {joint.unit_a_index, joint.unit_b_index} != {unit, parents[unit]}):
            raise AdapterError(f"Bevel hinge {name} differs from its detected tree edge.")
        axis = np.fromstring(xml_joint.get("axis", "0 0 1"), sep=" ")
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
            raise AdapterError(f"Invalid bevel tree axis for {name}.")
        if float(xml_joint.get("ref", "0")) != 0:
            raise AdapterError("Bevel export requires zero-reference upstream joint coordinates.")
        return name, axis / np.linalg.norm(axis), joint.position

    def coefficient(unit, gear_axis, mesh_position):
        name, axis, pivot = hinge(unit)
        world_axis = to_mujoco(gear_axis)
        alignment = float(np.dot(axis, world_axis))
        if abs(abs(alignment)-1) > 1e-7:
            raise AdapterError(f"Bevel gear axis does not align with {name}.")
        offset = np.cross(to_mujoco(mesh_position-pivot), axis)
        if np.linalg.norm(offset) > 1e-6:
            raise AdapterError(f"Bevel mesh does not lie on the hinge line of {name}.")
        return name, math.copysign(1., alignment)

    equality = root.find("equality")
    plans = []
    for index, gear, axes in perpendicular:
        a, b = gear.unit_a_index, gear.unit_b_index
        if a == b or a not in elements or b not in elements:
            raise AdapterError(f"Bevel gear {index} references invalid units.")
        if a not in parents or b not in parents:
            raise AdapterError(f"Bevel gear {index} needs supported moving shafts.")
        position_ldu = np.asarray(gear.position, dtype=float)
        if position_ldu.shape != (3,) or not np.isfinite(position_ldu).all():
            raise AdapterError(f"Bevel gear {index} has an invalid mesh position.")
        mesh_sign = _pitch_witness(scene, gear, axes, position_ldu)
        ratio = gear.ratio
        carrier = None
        if parents[a] == parents[b]:
            pass
        elif parents.get(parents[b]) == parents[a]:
            carrier = parents[b]
        elif parents.get(parents[a]) == parents[b]:
            a, b = b, a
            axes = axes[::-1]
            ratio = 1 / ratio
            carrier = parents[b]
        else:
            raise AdapterError(f"Bevel gear {index} has an unsupported carrier hierarchy.")
        # This pinned detector stores mesh positions in LDU, unlike joint pivots.
        position = position_ldu * LDU_TO_METERS
        input_name, input_sign = coefficient(a, axes[0], position)
        output_name, output_sign = coefficient(b, axes[1], position)
        terms = [(input_name, mesh_sign*ratio*input_sign), (output_name, output_sign)]
        if carrier is not None:
            carrier_name, carrier_sign = coefficient(carrier, axes[0], position)
            terms.append((carrier_name, -mesh_sign*ratio*carrier_sign))
        constraint = None if equality is None else equality.find(f"joint[@name='gear_{index}']")
        if constraint is None:
            raise AdapterError(f"Detected bevel gear {index} lacks its upstream equality.")
        tendon_name = f"bevel_{index}"
        if root.find(f"./tendon/*[@name='{tendon_name}']") is not None:
            raise AdapterError(f"Duplicate bevel tendon {tendon_name}.")
        plans.append((index, carrier, mesh_sign, terms, constraint, tendon_name))

    tendon = root.find("tendon")
    if tendon is None:
        tendon = ET.SubElement(root, "tendon")
    for index, carrier, mesh_sign, terms, constraint, name in plans:
        fixed = ET.SubElement(tendon, "fixed", name=name)
        for joint, factor in terms:
            ET.SubElement(fixed, "joint", joint=joint, coef=f"{factor:.17g}")
        equality.remove(constraint)
        ET.SubElement(equality, "tendon", name=f"gear_{index}", tendon1=name,
                      polycoef="0 0 0 0 0")
        report["couplings"].append({
            "gear": index, "carrierUnit": carrier, "meshSign": mesh_sign,
            "terms": [{"joint": joint, "coefficient": factor} for joint, factor in terms],
        })
    return report
