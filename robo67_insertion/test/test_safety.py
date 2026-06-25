"""Tests for robo67_insertion.lib.safety (pure-Python safety clamps).

These are the non-negotiable safety clamps applied to every commanded
Cartesian setpoint before it is published to the robot.
"""

import numpy as np

from robo67_insertion.lib.safety import (
    clamp_step,
    clamp_to_workspace,
    force_exceeded,
)


AABB = [[0.25, 0.65], [-0.30, 0.30], [0.02, 0.60]]


# ---------------------------------------------------------------------------
# clamp_to_workspace
# ---------------------------------------------------------------------------
def test_clamp_to_workspace_clips_out_of_bounds():
    out = clamp_to_workspace([0.1, 0.0, 0.7], AABB)
    assert isinstance(out, np.ndarray)
    assert out.shape == (3,)
    # x below xmin -> 0.25, z above zmax -> 0.60, y already inside.
    assert np.allclose(out, [0.25, 0.0, 0.60])


def test_clamp_to_workspace_inside_unchanged():
    pt = [0.40, 0.10, 0.30]
    out = clamp_to_workspace(pt, AABB)
    assert isinstance(out, np.ndarray)
    assert np.allclose(out, pt)


def test_clamp_to_workspace_clips_all_axes():
    out = clamp_to_workspace([0.0, -1.0, 1.0], AABB)
    assert np.allclose(out, [0.25, -0.30, 0.60])


def test_clamp_to_workspace_accepts_ndarray_input():
    out = clamp_to_workspace(np.array([0.9, 0.5, -0.5]), AABB)
    assert np.allclose(out, [0.65, 0.30, 0.02])


# ---------------------------------------------------------------------------
# clamp_step
# ---------------------------------------------------------------------------
def test_clamp_step_long_move_is_capped():
    out = clamp_step([0, 0, 0], [0.1, 0, 0], 0.002)
    assert isinstance(out, np.ndarray)
    assert np.allclose(out, [0.002, 0, 0])


def test_clamp_step_within_step_returns_target():
    out = clamp_step([0, 0, 0], [0.001, 0, 0], 0.002)
    assert isinstance(out, np.ndarray)
    assert np.allclose(out, [0.001, 0, 0])


def test_clamp_step_distance_never_exceeds_cap():
    prev = np.array([0.3, -0.1, 0.2])
    target = np.array([0.5, 0.25, 0.55])
    max_step = 0.01
    out = clamp_step(prev, target, max_step)
    dist = np.linalg.norm(out - prev)
    assert dist <= max_step + 1e-9


def test_clamp_step_equal_distance_returns_target():
    # Distance exactly equals max_step -> within step, returns target.
    out = clamp_step([0, 0, 0], [0.002, 0, 0], 0.002)
    assert np.allclose(out, [0.002, 0, 0])


def test_clamp_step_zero_move():
    out = clamp_step([0.1, 0.2, 0.3], [0.1, 0.2, 0.3], 0.005)
    assert np.allclose(out, [0.1, 0.2, 0.3])


# ---------------------------------------------------------------------------
# force_exceeded
# ---------------------------------------------------------------------------
def test_force_exceeded_true_when_over_cap():
    assert force_exceeded([0, 0, 30, 0, 0, 0], [20, 20, 25, 5, 5, 5]) is True


def test_force_exceeded_false_when_under_cap():
    assert force_exceeded([0, 0, 10, 0, 0, 0], [20, 20, 25, 5, 5, 5]) is False


def test_force_exceeded_uses_absolute_value():
    assert force_exceeded([0, 0, -30, 0, 0, 0], [20, 20, 25, 5, 5, 5]) is True


def test_force_exceeded_moment_axis():
    assert force_exceeded([0, 0, 0, 0, 0, 6], [20, 20, 25, 5, 5, 5]) is True


def test_force_exceeded_at_cap_is_false():
    # Strictly greater-than: equal to cap is NOT exceeded.
    assert force_exceeded([0, 0, 25, 0, 0, 0], [20, 20, 25, 5, 5, 5]) is False


def test_force_exceeded_returns_python_bool():
    out = force_exceeded([0, 0, 30, 0, 0, 0], [20, 20, 25, 5, 5, 5])
    assert type(out) is bool
