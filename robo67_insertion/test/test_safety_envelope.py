"""Tests for robo67_insertion.lib.safety_envelope (Candidate 4).

The safety envelope composes the low-level :mod:`robo67_insertion.lib.safety`
primitives (workspace clamp, step clamp, force abort) behind ONE module with
two command-path safety profiles that differ only in their ANCHOR policy:

* :class:`MMCSafetyProfile`        -- step anchored on the MEASURED EE.
* :class:`ImpedanceSafetyProfile`  -- step anchored on the PREVIOUS COMMAND,
  with the socket-top z-floor folded into the workspace AABB z-min.

These tests drive everything through the module interface (``apply``) and
assert the standardized workspace-then-step ordering, the per-profile anchor
policy, the folded impedance z-floor, and the force-abort behavior.
"""

import numpy as np
import pytest

from robo67_insertion.lib.safety_envelope import (
    ImpedanceSafetyProfile,
    MMCSafetyProfile,
    SafetyEnvelopeModule,
    SafetyInput,
    SafetyOutput,
)


AABB = [[0.20, 0.65], [-0.40, 0.40], [0.02, 0.60]]
SOCKET_TOP_Z = 0.10
MAX_PRESS_DEPTH = 0.05  # folded z-min = max(0.02, 0.10 - 0.05) = 0.05
NO_FORCE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _mmc(max_lead_m=0.05, fz_abort_n=15.0, aabb=AABB):
    return SafetyEnvelopeModule(
        MMCSafetyProfile(workspace_aabb=aabb, max_lead_m=max_lead_m,
                         fz_abort_n=fz_abort_n)
    )


def _impedance(max_step_m=0.0003, f_abort_n=20.0, aabb=AABB,
               socket_top_z=SOCKET_TOP_Z, max_press_depth_m=MAX_PRESS_DEPTH):
    return SafetyEnvelopeModule(
        ImpedanceSafetyProfile(workspace_aabb=aabb, max_step_m=max_step_m,
                               f_abort_n=f_abort_n, socket_top_z=socket_top_z,
                               max_press_depth_m=max_press_depth_m)
    )


def _within_aabb(xyz, aabb=AABB):
    a = np.asarray(aabb, float).reshape(3, 2)
    x = np.asarray(xyz, float)
    return bool(np.all((x >= a[:, 0] - 1e-9) & (x <= a[:, 1] + 1e-9)))


# ---------------------------------------------------------------------------
# Output type / shape
# ---------------------------------------------------------------------------
def test_apply_returns_safety_output():
    mod = _mmc()
    out = mod.apply(SafetyInput(desired_xyz=(0.40, 0.0, 0.30),
                                ee_xyz=(0.40, 0.0, 0.30),
                                prev_cmd_xyz=(0.40, 0.0, 0.30),
                                wrench6=NO_FORCE))
    assert isinstance(out, SafetyOutput)
    assert len(out.safe_xyz) == 3
    assert type(out.abort) is bool


# ---------------------------------------------------------------------------
# Workspace clamp: a desired point outside the AABB is clipped onto the box.
# (anchor is placed adjacent to the clamped face so the step clamp is a no-op)
# ---------------------------------------------------------------------------
def test_mmc_workspace_clamps_onto_box_face():
    mod = _mmc(max_lead_m=0.05)
    # x below xmin -> clamps to 0.20; anchor (ee) sits just inside that face.
    out = mod.apply(SafetyInput(desired_xyz=(0.10, 0.0, 0.30),
                                ee_xyz=(0.21, 0.0, 0.30),
                                prev_cmd_xyz=(0.50, 0.0, 0.30),
                                wrench6=NO_FORCE))
    assert out.safe_xyz[0] == pytest.approx(0.20)  # clipped onto xmin face
    assert _within_aabb(out.safe_xyz)


def test_impedance_workspace_clamps_onto_box_face():
    mod = _impedance(max_step_m=0.05)
    # z above zmax -> clamps to 0.60; anchor (prev_cmd) sits just inside.
    out = mod.apply(SafetyInput(desired_xyz=(0.40, 0.0, 0.90),
                                ee_xyz=(0.10, 0.0, 0.30),
                                prev_cmd_xyz=(0.40, 0.0, 0.59),
                                wrench6=NO_FORCE))
    assert out.safe_xyz[2] == pytest.approx(0.60)  # clipped onto zmax face
    assert _within_aabb(out.safe_xyz, aabb=_impedance().profile.aabb)


# ---------------------------------------------------------------------------
# Step clamp + ANCHOR policy: result moves exactly max_step from the PROFILE's
# anchor (ee for MMC; prev_cmd for Impedance).
# ---------------------------------------------------------------------------
def test_mmc_step_anchored_on_measured_ee():
    mod = _mmc(max_lead_m=0.05)
    ee = np.array([0.40, 0.0, 0.30])
    out = mod.apply(SafetyInput(desired_xyz=(0.62, 0.0, 0.30),  # in box, far +x
                                ee_xyz=tuple(ee),
                                prev_cmd_xyz=(0.45, 0.0, 0.30),
                                wrench6=NO_FORCE))
    # moved exactly max_lead_m from the MEASURED EE.
    assert np.linalg.norm(np.asarray(out.safe_xyz) - ee) == pytest.approx(0.05)


def test_impedance_step_anchored_on_prev_cmd():
    mod = _impedance(max_step_m=0.05)
    prev = np.array([0.45, 0.0, 0.30])
    out = mod.apply(SafetyInput(desired_xyz=(0.62, 0.0, 0.30),
                                ee_xyz=(0.40, 0.0, 0.30),
                                prev_cmd_xyz=tuple(prev),
                                wrench6=NO_FORCE))
    # moved exactly max_step from the PREVIOUS COMMAND.
    assert np.linalg.norm(np.asarray(out.safe_xyz) - prev) == pytest.approx(0.05)


def test_anchor_policies_differ_when_ee_neq_prev_cmd():
    # Same AABB and same max_step for both profiles, so any difference in the
    # result is due to the anchor policy ALONE.
    ee = (0.40, 0.0, 0.30)
    prev = (0.45, 0.0, 0.30)
    desired = (0.62, 0.0, 0.30)
    mmc = _mmc(max_lead_m=0.05)
    imp = _impedance(max_step_m=0.05)
    data = SafetyInput(desired_xyz=desired, ee_xyz=ee,
                       prev_cmd_xyz=prev, wrench6=NO_FORCE)
    mmc_out = mmc.apply(data)
    imp_out = imp.apply(data)
    # MMC steps from ee=0.40 -> 0.45; Impedance steps from prev=0.45 -> 0.50.
    assert not np.allclose(mmc_out.safe_xyz, imp_out.safe_xyz)
    assert mmc_out.safe_xyz[0] == pytest.approx(0.45)
    assert imp_out.safe_xyz[0] == pytest.approx(0.50)


# ---------------------------------------------------------------------------
# Ordering: workspace-then-step always keeps the result inside the AABB.
# ---------------------------------------------------------------------------
def test_mmc_result_always_within_aabb():
    mod = _mmc(max_lead_m=0.05)
    for desired in [(2.0, 2.0, 2.0), (-1.0, -1.0, -1.0), (0.4, 0.0, -5.0)]:
        out = mod.apply(SafetyInput(desired_xyz=desired,
                                    ee_xyz=(0.40, 0.0, 0.30),
                                    prev_cmd_xyz=(0.40, 0.0, 0.30),
                                    wrench6=NO_FORCE))
        assert _within_aabb(out.safe_xyz)


def test_impedance_result_always_within_folded_aabb():
    mod = _impedance(max_step_m=0.05)
    folded = mod.profile.aabb
    for desired in [(2.0, 2.0, 2.0), (-1.0, -1.0, -1.0), (0.4, 0.0, -5.0)]:
        out = mod.apply(SafetyInput(desired_xyz=desired,
                                    ee_xyz=(0.40, 0.0, 0.30),
                                    prev_cmd_xyz=(0.40, 0.0, 0.30),
                                    wrench6=NO_FORCE))
        assert _within_aabb(out.safe_xyz, aabb=folded)


# ---------------------------------------------------------------------------
# Impedance z-floor: a desired far below the socket top is clamped to
# socket_top_z - max_press_depth_m, NOT the raw workspace zmin.
# ---------------------------------------------------------------------------
def test_impedance_z_floor_folded_below_socket():
    mod = _impedance(max_step_m=0.05)
    # folded z-min must be socket_top_z - max_press_depth_m, not raw zmin (0.02).
    assert mod.profile.aabb[2, 0] == pytest.approx(SOCKET_TOP_Z - MAX_PRESS_DEPTH)
    assert mod.profile.aabb[2, 0] > AABB[2][0]
    # prev_cmd at the floor so the step clamp is a no-op and we read the floor.
    out = mod.apply(SafetyInput(desired_xyz=(0.45, 0.0, -1.0),
                                ee_xyz=(0.45, 0.0, 0.30),
                                prev_cmd_xyz=(0.45, 0.0, 0.05),
                                wrench6=NO_FORCE))
    assert out.safe_xyz[2] == pytest.approx(SOCKET_TOP_Z - MAX_PRESS_DEPTH)
    assert out.safe_xyz[2] > AABB[2][0]  # NOT the raw workspace zmin


def test_impedance_z_floor_respects_higher_raw_zmin():
    # If the raw workspace zmin is ABOVE socket_top_z - max_press_depth, the
    # higher raw zmin wins (max of the two).
    mod = _impedance(max_step_m=0.05, aabb=[[0.20, 0.65], [-0.40, 0.40], [0.08, 0.60]],
                     socket_top_z=0.10, max_press_depth_m=0.05)  # fold -> 0.05 < 0.08
    assert mod.profile.aabb[2, 0] == pytest.approx(0.08)


# ---------------------------------------------------------------------------
# Force abort: wrench within caps -> False; a component over its cap -> True.
# ---------------------------------------------------------------------------
def test_mmc_force_abort_false_within_caps():
    mod = _mmc(fz_abort_n=15.0)
    out = mod.apply(SafetyInput(desired_xyz=(0.40, 0.0, 0.30),
                                ee_xyz=(0.40, 0.0, 0.30),
                                prev_cmd_xyz=(0.40, 0.0, 0.30),
                                wrench6=(0.0, 0.0, 10.0, 0.0, 0.0, 0.0)))
    assert out.abort is False


def test_mmc_force_abort_true_when_fz_over_fz_abort():
    mod = _mmc(fz_abort_n=15.0)
    out = mod.apply(SafetyInput(desired_xyz=(0.40, 0.0, 0.30),
                                ee_xyz=(0.40, 0.0, 0.30),
                                prev_cmd_xyz=(0.40, 0.0, 0.30),
                                wrench6=(0.0, 0.0, 16.0, 0.0, 0.0, 0.0)))
    assert out.abort is True


def test_impedance_force_abort_false_within_caps():
    mod = _impedance(f_abort_n=20.0)
    out = mod.apply(SafetyInput(desired_xyz=(0.45, 0.0, 0.10),
                                ee_xyz=(0.45, 0.0, 0.10),
                                prev_cmd_xyz=(0.45, 0.0, 0.10),
                                wrench6=(0.0, 0.0, 10.0, 0.0, 0.0, 0.0)))
    assert out.abort is False


def test_impedance_force_abort_true_when_fz_over_f_abort():
    mod = _impedance(f_abort_n=20.0)
    out = mod.apply(SafetyInput(desired_xyz=(0.45, 0.0, 0.10),
                                ee_xyz=(0.45, 0.0, 0.10),
                                prev_cmd_xyz=(0.45, 0.0, 0.10),
                                wrench6=(0.0, 0.0, 25.0, 0.0, 0.0, 0.0)))
    assert out.abort is True
