"""Choose exact tree hinges for verified multi-body shared pivots."""

from collections import deque
from copy import deepcopy
from xml.etree import ElementTree as ET

import numpy as np
from lego_technic_sim.physics.model import JointType, PhysicsScene

from .bridge import AdapterError, _shared_pivot_matches
from .collision import to_mujoco, vector_text


def prioritize_shared_pivots(root: ET.Element, scene: PhysicsScene, manifest: dict,
                            body_indices: dict[str, int]) -> dict:
    groups = sorted(
        (group for group in manifest.get("motion", {}).get("sharedPivots", [])
         if len(group["bodies"]) > 2), key=lambda group: group["id"])
    report = {"schemaVersion": 1, "version": "shared-axis-tree-v1",
              "hardSharedPivots": [], "softSharedPivots": []}
    if not groups:
        return report

    bodies = {index: root.find(f".//body[@name='unit_{index}']")
              for index in range(len(scene.units))}
    if any(body is None for body in bodies.values()):
        raise AdapterError("Shared-axis export requires every scene body in the MJCF tree.")
    equality = root.find("equality")
    loops = {}
    if equality is not None:
        for constraint in equality:
            name = constraint.get("name", "")
            if name.startswith("loop_"):
                index = int(name.removeprefix("loop_weld_").removeprefix("loop_"))
                loops[index] = constraint
    old_tree = set(range(len(scene.joints))) - loops.keys()
    old_parents = {}
    for parent, element in bodies.items():
        for child in element.findall("body"):
            index = int(child.attrib["name"].removeprefix("unit_"))
            matches = [ji for ji in old_tree if
                       {scene.joints[ji].unit_a_index, scene.joints[ji].unit_b_index} == {parent, index}]
            if len(matches) != 1:
                raise AdapterError("Shared-axis export cannot identify an upstream tree edge.")
            old_parents[index] = (parent, matches[0])

    required = {drive.joint_index for drive in scene.motors}
    for element in root.iter():
        for attribute in ("joint", "joint1", "joint2"):
            name = element.get(attribute)
            if name is not None:
                required.add(int(name.removeprefix("joint_")))
    protected = set()
    for index in required:
        children = [child for child, (_, edge) in old_parents.items() if edge == index]
        if len(children) != 1:
            raise AdapterError("A motor or gear reference is not an upstream tree joint.")
        child = children[0]
        while child in old_parents:
            child, edge = old_parents[child]
            protected.add(edge)

    components = list(range(len(scene.units)))
    selected = set()

    def find(index):
        while components[index] != index:
            index = components[index]
        return index

    def add(index):
        joint = scene.joints[index]
        a, b = find(joint.unit_a_index), find(joint.unit_b_index)
        if a == b:
            return False
        components[a] = b
        selected.add(index)
        return True

    for index in sorted(protected):
        if not add(index):
            raise AdapterError("Protected motor/gear paths contain a cycle.")
    for group in groups:
        declared = [joint for joint in manifest["expected"]["joints"]
                    if joint["id"] in group["abstractHinges"]
                    or joint.get("connector") == f"pivot-{group['id']}"]
        members = {body_indices[name] for name in group["bodies"]}
        if not declared:
            raise AdapterError(f"Shared pivot {group['id']} has no declared hinges.")
        candidates, matched = _shared_pivot_matches(declared, members, scene)
        if not matched:
            raise AdapterError(f"Shared pivot {group['id']} differs from its verified physical joint tree.")
        previous_components, previous_selected = components.copy(), selected.copy()
        if all(index in selected or add(index) for index in sorted(candidates)):
            report["hardSharedPivots"].append({"id": group["id"], "joints": sorted(candidates)})
        else:
            components[:], selected = previous_components, previous_selected
            report["softSharedPivots"].append({
                "id": group["id"], "joints": sorted(candidates),
                "reason": "A complete hard shared axis would close a cycle with protected or earlier shared-pivot edges.",
            })
    for index in [*sorted(old_tree), *sorted(loops)]:
        if index not in selected:
            add(index)
    if len(selected) != len(scene.units) - 1:
        raise AdapterError("Shared-axis export requires a connected physical joint graph.")
    report["treeJoints"] = sorted(selected)
    report["loopJoints"] = sorted(set(range(len(scene.joints))) - selected)
    report["protectedJoints"] = sorted(protected)
    if selected == old_tree:
        return report

    adjacency = [[] for _ in scene.units]
    for index in sorted(selected):
        joint = scene.joints[index]
        adjacency[joint.unit_a_index].append((joint.unit_b_index, index))
        adjacency[joint.unit_b_index].append((joint.unit_a_index, index))
    parents = {0: None}
    queue = deque([0])
    while queue:
        parent = queue.popleft()
        for child, index in adjacency[parent]:
            if child not in parents:
                parents[child] = (parent, index)
                queue.append(child)

    templates = {int(joint.attrib["name"].removeprefix("joint_")): deepcopy(joint)
                 for body in bodies.values() for joint in body.findall("joint")}
    for child, (parent, _) in old_parents.items():
        bodies[parent].remove(bodies[child])
    for body in bodies.values():
        for joint in list(body.findall("joint")):
            body.remove(joint)
    for child, edge in parents.items():
        if edge is None:
            continue
        parent, index = edge
        joint = scene.joints[index]
        bodies[child].set("pos", vector_text(to_mujoco(
            scene.units[child].center_of_mass - scene.units[parent].center_of_mass)))
        if joint.joint_type != JointType.FIXED:
            if joint.joint_type not in (JointType.REVOLUTE, JointType.SLIDER):
                raise AdapterError("Unsupported shared-axis tree joint type.")
            element = deepcopy(templates[index]) if index in templates else ET.Element("joint")
            element.attrib.update({
                "name": f"joint_{index}",
                "type": "hinge" if joint.joint_type == JointType.REVOLUTE else "slide",
                "pos": vector_text(to_mujoco(joint.position - scene.units[child].center_of_mass)),
                "axis": vector_text(to_mujoco(joint.axis / np.linalg.norm(joint.axis))),
            })
            bodies[child].insert(0, element)
        bodies[parent].append(bodies[child])

    if equality is None:
        equality = ET.SubElement(root, "equality")
    for constraint in loops.values():
        equality.remove(constraint)
    for index in report["loopJoints"]:
        if index in loops:
            equality.append(loops[index])
            continue
        joint = scene.joints[index]
        attributes = {"body1": f"unit_{joint.unit_a_index}", "body2": f"unit_{joint.unit_b_index}"}
        if joint.joint_type == JointType.FIXED:
            ET.SubElement(equality, "weld", name=f"loop_weld_{index}", **attributes)
        elif joint.joint_type == JointType.REVOLUTE:
            ET.SubElement(equality, "connect", name=f"loop_{index}",
                          anchor=vector_text(to_mujoco(
                              joint.position - scene.units[joint.unit_a_index].center_of_mass)),
                          **attributes)
        else:
            raise AdapterError("Shared-axis export cannot move a slider into loop closure.")
    return report
