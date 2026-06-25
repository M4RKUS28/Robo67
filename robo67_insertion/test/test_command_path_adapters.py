"""Adapter conformance tests (TDD, canonical seam).

Both command-path adapters wrap the SAME canonical
:class:`~robo67_insertion.lib.insertion_intent.InsertionIntentModule`. They
differ ONLY in how a canonical absolute target becomes a controller command:

* :class:`MMCCommandPathAdapter` (sim): held orientation quaternion + a light
  geometric push bias; the node lead-clamps from the EE.
* :class:`ImpedanceCommandPathAdapter` (real arm): below-surface equilibrium
  gaps (force F needs gap F/pos_stiff) + a held ROW-MAJOR 3x3 R, with px and
  R22 kept non-zero.

The headline guarantee: fed the SAME sensor sequence, BOTH adapters traverse
the SAME phase sequence (proving one transition model).
"""
import numpy as np
import pytest

from robo67_insertion.lib.insertion_intent import IntentParams, IntentSensors
from robo67_insertion.lib.command_path_adapters import (
    ImpedanceCommandPathAdapter,
    MMCCommandPathAdapter,
)

SOCKET = (0.5, 0.0, 0.2)
DOWN_QUAT = (0.0, 1.0, 0.0, 0.0)
POS_STIFF = 200.0
PRESS_FORCE_N = 3.0
INSERT_PRESS_N = 6.0


def isensors(ee, fz=0.0, baseline=0.0, t=0.0):
    return IntentSensors(
        ee_xyz=tuple(float(v) for v in ee), fz=fz, fz_baseline=baseline, t=t
    )


# An OPEN-LOOP sensor sequence that drives a full insertion: arrive above,
# descend, contact at z=0.2, spiral, drop into hole, push to seat, confirm,
# retract, done. Fed identically to both adapters.
def sensor_sequence():
    return [
        isensors((0.5, 0.0, 0.25), fz=0.0, t=0.00),    # MOVE_ABOVE arrive
        isensors((0.5, 0.0, 0.22), fz=0.0, t=0.02),    # descending, no contact
        isensors((0.5, 0.0, 0.20), fz=10.0, t=0.04),   # contact -> contact_z=0.2
        isensors((0.5, 0.0, 0.20), fz=10.0, t=0.06),   # spiral
        isensors((0.5, 0.0, 0.20), fz=10.0, t=0.08),   # spiral
        isensors((0.505, 0.0, 0.192), fz=10.0, t=0.10),  # drop into hole
        isensors((0.505, 0.0, 0.17), fz=10.0, t=0.12),   # pushing
        isensors((0.505, 0.0, 0.159), fz=10.0, t=0.14),  # seated -> confirm
        isensors((0.505, 0.0, 0.159), fz=10.0, t=0.16),  # confirm depth ok
        isensors((0.505, 0.0, 0.18), fz=0.0, t=0.18),    # retracting
        isensors((0.5, 0.0, 0.25), fz=0.0, t=0.20),      # above contact -> done
        isensors((0.5, 0.0, 0.25), fz=0.0, t=0.22),      # done holds
    ]


def run_mmc():
    a = MMCCommandPathAdapter(
        SOCKET, IntentParams(), down_quat=DOWN_QUAT, push_step_m=0.0015
    )
    phase = "MOVE_ABOVE"
    rows = []
    for s in sensor_sequence():
        cmd = a.step(phase, s)
        rows.append((phase, cmd))
        phase = cmd.next_phase
    return a, rows


def run_impedance():
    a = ImpedanceCommandPathAdapter(
        SOCKET,
        IntentParams(),
        pos_stiff=POS_STIFF,
        press_force_n=PRESS_FORCE_N,
        insert_press_n=INSERT_PRESS_N,
        max_press_depth_m=0.05,
    )
    phase = "MOVE_ABOVE"
    rows = []
    for s in sensor_sequence():
        cmd = a.step(phase, s)
        rows.append((phase, cmd))
        phase = cmd.next_phase
    return a, rows


class TestSingleTransitionModel:
    def test_identical_phase_sequence(self):
        _, mmc_rows = run_mmc()
        _, imp_rows = run_impedance()
        mmc_phases = [(ph, cmd.next_phase) for ph, cmd in mmc_rows]
        imp_phases = [(ph, cmd.next_phase) for ph, cmd in imp_rows]
        assert mmc_phases == imp_phases

    def test_sequence_reaches_done(self):
        _, mmc_rows = run_mmc()
        last = mmc_rows[-1][1]
        assert last.next_phase == "DONE"
        assert last.done is True
        visited = {ph for ph, _ in mmc_rows}
        for expected in ("DESCEND_TO_CONTACT", "SEARCH_SPIRAL", "PUSH_INSERT",
                         "CONFIRM", "RETRACT"):
            assert expected in visited


class TestMMCAdapter:
    def test_holds_orientation_quaternion(self):
        _, rows = run_mmc()
        for _, cmd in rows:
            assert tuple(cmd.desired_quat) == pytest.approx(DOWN_QUAT)
            assert len(cmd.desired_quat) == 4

    def test_targets_finite(self):
        _, rows = run_mmc()
        for _, cmd in rows:
            assert len(cmd.desired_xyz) == 3
            assert np.all(np.isfinite(cmd.desired_xyz))

    def test_drives_down_during_contact_phases(self):
        _, rows = run_mmc()
        for ph, cmd in rows:
            if ph in ("DESCEND_TO_CONTACT", "SEARCH_SPIRAL", "PUSH_INSERT"):
                # carrot is at or below the socket-top plane so the arm descends
                assert cmd.desired_xyz[2] <= SOCKET[2] + 1e-9


class TestImpedanceAdapter:
    def test_search_spiral_uses_press_gap(self):
        a, rows = run_impedance()
        press_gap = PRESS_FORCE_N / POS_STIFF
        contact_z = a.module.contact_z
        assert contact_z is not None
        seen = 0
        for ph, cmd in rows:
            if ph == "SEARCH_SPIRAL":
                seen += 1
                assert cmd.goal_xyz[2] == pytest.approx(contact_z - press_gap)
        assert seen >= 1

    def test_push_insert_uses_insert_gap(self):
        a, rows = run_impedance()
        insert_gap = INSERT_PRESS_N / POS_STIFF
        contact_z = a.module.contact_z
        seen = 0
        for ph, cmd in rows:
            if ph == "PUSH_INSERT":
                seen += 1
                assert cmd.goal_xyz[2] == pytest.approx(contact_z - insert_gap)
        assert seen >= 1

    def test_contact_phases_command_below_surface(self):
        _, rows = run_impedance()
        for ph, cmd in rows:
            if ph in ("DESCEND_TO_CONTACT", "SEARCH_SPIRAL", "PUSH_INSERT"):
                assert cmd.goal_xyz[2] < SOCKET[2]

    def test_gap_helpers(self):
        a = ImpedanceCommandPathAdapter(
            SOCKET, IntentParams(), pos_stiff=POS_STIFF,
            press_force_n=PRESS_FORCE_N, insert_press_n=INSERT_PRESS_N,
        )
        assert a.press_gap_m == pytest.approx(PRESS_FORCE_N / POS_STIFF)
        assert a.insert_gap_m == pytest.approx(INSERT_PRESS_N / POS_STIFF)

    def test_pose_desired_never_zero_px_or_r22(self):
        # socket at x=0 and a degenerate R with R22=0 -> the adapter MUST still
        # emit a non-zero px and non-zero R22 (controller quirk).
        a = ImpedanceCommandPathAdapter(
            (0.0, 0.0, 0.2),
            IntentParams(),
            pos_stiff=POS_STIFF,
            R=np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]]),
        )
        cmd = a.step("MOVE_ABOVE", isensors((0.0, 0.0, 0.25)))
        data = cmd.pose_desired()
        assert len(data) == 12
        assert abs(data[0]) > 0.0   # px non-zero
        assert abs(data[11]) > 0.0  # R22 non-zero
        assert np.all(np.isfinite(data))
