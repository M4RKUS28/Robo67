"""Tests for the pure insertion-telemetry seam (lib.telemetry)."""
import math

from robo67_insertion.lib.telemetry import (
    InsertionTelemetry,
    SpeedTracker,
    diagnostic_pairs,
)


def test_speed_tracker_first_sample_is_zero():
    st = SpeedTracker()
    assert st.update((0.0, 0.0, 0.0), 0.0) == 0.0


def test_speed_tracker_finite_difference():
    st = SpeedTracker()
    st.update((0.0, 0.0, 0.0), 0.0)
    # move 0.1 m in 0.1 s -> 1.0 m/s
    assert math.isclose(st.update((0.1, 0.0, 0.0), 0.1), 1.0, rel_tol=1e-6)
    # move 0.0 -> speed drops to 0
    assert math.isclose(st.update((0.1, 0.0, 0.0), 0.2), 0.0, abs_tol=1e-9)


def test_speed_tracker_nonadvancing_time_keeps_previous():
    st = SpeedTracker()
    st.update((0.0, 0.0, 0.0), 0.0)
    st.update((0.1, 0.0, 0.0), 0.1)  # speed = 1.0
    # same timestamp -> no division, keep last speed
    assert math.isclose(st.update((0.5, 0.0, 0.0), 0.1), 1.0, rel_tol=1e-6)


def test_speed_tracker_reset():
    st = SpeedTracker()
    st.update((0.0, 0.0, 0.0), 0.0)
    st.update((1.0, 0.0, 0.0), 0.1)
    st.reset()
    assert st.speed == 0.0
    assert st.update((5.0, 0.0, 0.0), 1.0) == 0.0  # first sample after reset


def _tel(**kw):
    base = dict(
        t=1.5, phase="SEARCH_SPIRAL", ee_xyz=(0.45, 0.0, 0.10),
        cmd_xyz=(0.45, 0.0, 0.09), speed=0.012,
        wrench6=(1.0, 2.0, 2.0, 0.0, 0.0, 0.0), fz=2.0, fz_baseline=0.5,
        contact=True, retries=1,
    )
    base.update(kw)
    return InsertionTelemetry(**base)


def test_force_mag():
    tel = _tel(wrench6=(3.0, 0.0, 4.0, 0.0, 0.0, 0.0))
    assert math.isclose(tel.force_mag, 5.0, rel_tol=1e-9)


def test_diagnostic_pairs_keys_and_strings():
    pairs = diagnostic_pairs(_tel(socket_xyz=(0.45, 0.0, 0.10), contact_z=0.10))
    keys = [k for k, _ in pairs]
    for required in ("phase", "ee_xyz_m", "cmd_xyz_m", "socket_xyz_m",
                     "speed_mps", "fz_n", "fz_baseline_n", "force_mag_n",
                     "contact", "retries", "contact_z_m", "abort", "done",
                     "error"):
        assert required in keys
    # every value must be a string (DiagnosticStatus.values are KeyValue strings)
    assert all(isinstance(v, str) for _, v in pairs)
    d = dict(pairs)
    assert d["phase"] == "SEARCH_SPIRAL"
    assert d["contact"] == "true"
    assert d["retries"] == "1"


def test_diagnostic_pairs_handles_missing_optionals():
    d = dict(diagnostic_pairs(_tel(socket_xyz=None, contact_z=None, error=None)))
    assert d["socket_xyz_m"] == "n/a"
    assert d["contact_z_m"] == "n/a"
    assert d["error"] == ""
