import os
from pathlib import Path

import numpy as np
import pytest

from legowalkers_sim.bridge import AdapterError, check, load_manifest
from legowalkers_sim.evaluation import EvaluationOptions, prepare_xml


@pytest.mark.integration
def test_real_hitch_has_two_independent_free_pitch_dofs_without_steering(tmp_path):
    source = os.environ.get("LEGOWALKERS_PITCH_HITCH")
    library = os.environ.get("LDRAW_LIBRARY")
    if not source or not library:
        pytest.skip("Set LEGOWALKERS_PITCH_HITCH and LDRAW_LIBRARY for articulated-wagon validation.")
    mujoco = pytest.importorskip("mujoco")
    manifest_path = Path(source) / "candidate/manifest.json"
    checked = check(manifest_path, Path(library))
    assert checked.manifest["grammarVersion"] == "technic-fourbar-v5"
    assert checked.manifest["genome"]["towWagon"]
    assert len({checked.body_indices[name] for name in ("chassis", "drawbar", "wagon-chassis")}) == 3
    xml, *_ = prepare_xml(checked, tmp_path / "export", EvaluationOptions(duration=20, timestep=.002))
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    joints = []
    for name in ("drawbar-walker-pitch", "drawbar-wagon-pitch"):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{checked.joint_indices[name]}")
        assert joint >= 0 and model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE
        assert not model.jnt_limited[joint] and model.jnt_stiffness[joint] == 0
        dof = model.jnt_dofadr[joint]
        assert model.dof_damping[dof] == model.dof_frictionloss[dof] == 0
        assert joint not in model.actuator_trnid[:, 0]
        joints.append(joint)
    def body_id(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"unit_{checked.body_indices[name]}")
    chassis, bar, wagon = map(body_id, ("chassis", "drawbar", "wagon-chassis"))
    initial = data.xquat.copy()
    data.qpos[model.jnt_qposadr[joints[0]]] = .25
    mujoco.mj_forward(model, data)
    assert np.allclose(data.xquat[chassis], initial[chassis])
    assert not np.allclose(data.xquat[bar], initial[bar])
    assert not np.allclose(data.xquat[wagon], initial[wagon])
    data.qpos[model.jnt_qposadr[joints[1]]] = -.25
    mujoco.mj_forward(model, data)
    assert np.allclose(data.xquat[wagon], initial[wagon])
    assert not np.allclose(data.xquat[bar], initial[bar])
    assert data.time == 0  # Kinematic checks only, not an extra simulation trial.

    import json
    broken = json.loads(manifest_path.read_text())
    broken["expected"]["joints"] = [joint for joint in broken["expected"]["joints"]
                                      if joint["id"] != "drawbar-wagon-pitch"]
    bad = tmp_path / "invalid"
    bad.mkdir()
    (bad / "model.ldr").write_bytes(manifest_path.with_name("model.ldr").read_bytes())
    (bad / "manifest.json").write_text(json.dumps(broken))
    with pytest.raises(AdapterError, match="two passive"):
        load_manifest(bad / "manifest.json")
