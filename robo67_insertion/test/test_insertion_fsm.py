"""Tests for robo67_insertion.lib.insertion_fsm (TDD).

The insertion FSM is PURE: given (current_state, Sensors) it returns a
Decision (desired setpoint + next_state). Z is UP in the robot base frame,
so descending means decreasing z. "Contact" is when the peg bottom touches
the socket top surface at height ``contact_z``; the peg drops INTO the hole
when ``ee_z`` falls below ``contact_z``.

These tests drive the implementation table-style: each transition is
exercised in isolation, plus one full integration run through a synthetic
sensor timeline.
"""
import numpy as np
import pytest

from robo67_insertion.lib.insertion_fsm import (
    Decision,
    FsmParams,
    InsertionFSM,
    Sensors,
)

SOCKET = np.array([0.5, 0.0, 0.2])
BASELINE = 0.0


def make_fsm(socket=SOCKET, params=None):
    return InsertionFSM(socket, params if params is not None else FsmParams())


def sensors(ee, fz=BASELINE, baseline=BASELINE, t=0.0):
    return Sensors(ee_xyz=np.asarray(ee, float), fz=fz, fz_baseline=baseline, t=t)


def assert_decision_shape(d, p):
    assert isinstance(d, Decision)
    assert isinstance(d.desired_xyz, np.ndarray)
    assert d.desired_xyz.shape == (3,)
    assert np.all(np.isfinite(d.desired_xyz))
    # desired_quat is ALWAYS the fixed tool-down orientation.
    assert tuple(d.desired_quat) == tuple(p.down_quat)


class TestIdle:
    def test_idle_goes_to_move_above_at_standoff(self):
        fsm = make_fsm()
        p = fsm.p
        d = fsm.step("IDLE", sensors(SOCKET + [0, 0, 0.1]))
        assert_decision_shape(d, p)
        assert d.next_state == "MOVE_ABOVE"
        assert d.desired_xyz[2] == pytest.approx(SOCKET[2] + p.standoff_m)
        assert d.desired_xyz[0] == pytest.approx(SOCKET[0])
        assert d.desired_xyz[1] == pytest.approx(SOCKET[1])
        assert d.done is False


class TestMoveAbove:
    def test_within_tol_descends(self):
        fsm = make_fsm()
        p = fsm.p
        target = SOCKET + np.array([0, 0, p.standoff_m])
        # ee just inside the tolerance ball
        ee = target + np.array([p.approach_tol_m * 0.5, 0, 0])
        d = fsm.step("MOVE_ABOVE", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "DESCEND_TO_CONTACT"
        assert np.allclose(d.desired_xyz, target)

    def test_far_stays(self):
        fsm = make_fsm()
        p = fsm.p
        target = SOCKET + np.array([0, 0, p.standoff_m])
        ee = target + np.array([0.05, 0, 0])
        d = fsm.step("MOVE_ABOVE", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "MOVE_ABOVE"
        assert np.allclose(d.desired_xyz, target)


class TestDescendToContact:
    def test_no_contact_steps_down(self):
        fsm = make_fsm()
        p = fsm.p
        ee = np.array([0.5, 0.0, 0.24])
        d = fsm.step("DESCEND_TO_CONTACT", sensors(ee, fz=BASELINE, baseline=BASELINE))
        assert_decision_shape(d, p)
        assert d.next_state == "DESCEND_TO_CONTACT"
        # xy snaps to socket xy, z decremented by descend_step
        assert d.desired_xyz[0] == pytest.approx(SOCKET[0])
        assert d.desired_xyz[1] == pytest.approx(SOCKET[1])
        assert d.desired_xyz[2] == pytest.approx(ee[2] - p.descend_step_m)
        assert fsm.contact_z is None

    def test_contact_records_contact_z_and_searches(self):
        fsm = make_fsm()
        p = fsm.p
        ee = np.array([0.5, 0.0, 0.2])
        fz = BASELINE + p.contact_fz_threshold_n + 1.0
        d = fsm.step("DESCEND_TO_CONTACT", sensors(ee, fz=fz, baseline=BASELINE, t=3.0))
        assert_decision_shape(d, p)
        assert d.next_state == "SEARCH_SPIRAL"
        assert fsm.contact_z == pytest.approx(ee[2])
        assert fsm.spiral_t0 == pytest.approx(3.0)
        assert fsm.retries == 0
        # holds position on the contact step
        assert np.allclose(d.desired_xyz, ee)


class TestSearchSpiral:
    def test_offset_grows_with_time(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        fsm.spiral_t0 = 0.0
        ee = np.array([0.5, 0.0, 0.2])  # ee_z stays ~ contact_z
        prev_norm = -1.0
        for t in (0.5, 1.0, 2.0, 4.0):
            d = fsm.step("SEARCH_SPIRAL", sensors(ee, t=t))
            assert_decision_shape(d, p)
            assert d.next_state == "SEARCH_SPIRAL"
            offset = d.desired_xyz[:2] - SOCKET[:2]
            norm = float(np.linalg.norm(offset))
            assert norm > prev_norm
            prev_norm = norm
            # pushes gently below contact while searching
            assert d.desired_xyz[2] == pytest.approx(fsm.contact_z - p.push_step_m)

    def test_drop_transitions_to_push_insert(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        fsm.spiral_t0 = 0.0
        ee = np.array([0.5, 0.0, fsm.contact_z - 2 * p.z_drop_threshold_m])
        d = fsm.step("SEARCH_SPIRAL", sensors(ee, t=1.0))
        assert_decision_shape(d, p)
        assert d.next_state == "PUSH_INSERT"


class TestPushInsert:
    def test_continues_until_depth(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        # well above target depth -> keep pushing
        ee = np.array([0.5, 0.0, fsm.contact_z - 0.005])
        d = fsm.step("PUSH_INSERT", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "PUSH_INSERT"
        target_z = fsm.contact_z - p.insert_depth_m
        assert d.desired_xyz[2] == pytest.approx(max(target_z, ee[2] - p.push_step_m))
        assert d.desired_xyz[0] == pytest.approx(SOCKET[0])
        assert d.desired_xyz[1] == pytest.approx(SOCKET[1])

    def test_reaches_target_goes_to_confirm(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        target_z = fsm.contact_z - p.insert_depth_m
        ee = np.array([0.5, 0.0, target_z + 0.4 * p.z_drop_threshold_m])
        d = fsm.step("PUSH_INSERT", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "CONFIRM"


class TestConfirm:
    def test_depth_ok_retracts(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        ee = np.array([0.5, 0.0, fsm.contact_z - p.insert_depth_m])  # deep enough
        d = fsm.step("CONFIRM", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "RETRACT"
        assert np.allclose(d.desired_xyz, ee)

    def test_not_deep_retries_back_to_search(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        fsm.retries = 0
        ee = np.array([0.5, 0.0, fsm.contact_z - 0.001])  # not deep
        d = fsm.step("CONFIRM", sensors(ee, t=9.0))
        assert_decision_shape(d, p)
        assert d.next_state == "SEARCH_SPIRAL"
        assert fsm.retries == 1
        assert fsm.spiral_t0 == pytest.approx(9.0)
        assert d.error is None

    def test_not_deep_exhausted_errors(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        fsm.retries = p.retry_limit
        ee = np.array([0.5, 0.0, fsm.contact_z - 0.001])  # not deep
        d = fsm.step("CONFIRM", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "ERROR"
        assert d.done is True
        assert d.error is not None


class TestRetract:
    def test_below_contact_keeps_retracting(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        target = SOCKET + np.array([0, 0, p.standoff_m])
        ee = np.array([0.5, 0.0, fsm.contact_z - 0.01])
        d = fsm.step("RETRACT", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "RETRACT"
        assert np.allclose(d.desired_xyz, target)

    def test_above_contact_is_done(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        ee = np.array([0.5, 0.0, fsm.contact_z + 0.05])
        d = fsm.step("RETRACT", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "DONE"


class TestTerminal:
    def test_done_is_done_and_holds(self):
        fsm = make_fsm()
        p = fsm.p
        ee = np.array([0.5, 0.0, 0.25])
        d = fsm.step("DONE", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "DONE"
        assert d.done is True
        assert np.allclose(d.desired_xyz, ee)

    def test_error_holds_and_preserves(self):
        fsm = make_fsm()
        p = fsm.p
        fsm.error = "boom"
        ee = np.array([0.5, 0.0, 0.25])
        d = fsm.step("ERROR", sensors(ee))
        assert_decision_shape(d, p)
        assert d.next_state == "ERROR"
        assert d.done is True
        assert d.error == "boom"
        assert np.allclose(d.desired_xyz, ee)


class TestSpiralExhaustion:
    def test_radius_exceeded_eventually_errors(self):
        # Drive SEARCH_SPIRAL with a large, ever-growing elapsed time so the
        # spiral radius exceeds the max on every call. Each restart resets
        # spiral_t0 to s.t, so t must keep increasing for the next elapsed to
        # again blow past the radius bound. After retry_limit restarts the
        # next one -> ERROR.
        fsm = make_fsm()
        p = fsm.p
        fsm.contact_z = 0.2
        fsm.spiral_t0 = 0.0
        fsm.retries = 0
        ee = np.array([0.5, 0.0, 0.2])  # never drops
        dt_big = 1000.0  # >> time needed for r to exceed spiral_max_radius_m
        last = None
        # retry_limit restarts allowed; on the (retry_limit+1)-th -> ERROR
        for i in range(p.retry_limit + 1):
            last = fsm.step("SEARCH_SPIRAL", sensors(ee, t=dt_big * (i + 1)))
            assert np.all(np.isfinite(last.desired_xyz))
        assert last.next_state == "ERROR"
        assert last.done is True
        assert last.error is not None


class TestIntegration:
    def test_full_run_reaches_done(self):
        socket = np.array([0.5, 0.0, 0.2])
        fsm = make_fsm(socket)
        p = fsm.p
        contact_height = socket[2]  # peg bottom touches socket top here
        baseline = 0.0

        ee = socket + np.array([0.0, 0.0, p.standoff_m])  # start at standoff target
        state = "IDLE"
        t = 0.0
        dt = 0.02  # 50 Hz
        dropped = False
        spiral_steps = 0
        reached_done = False
        max_steps = 2000

        for _ in range(max_steps):
            # synthesize contact force based on current ee height
            if ee[2] <= contact_height + 1e-9:
                fz = baseline + p.contact_fz_threshold_n + 1.0
            else:
                fz = baseline
            s = sensors(ee.copy(), fz=fz, baseline=baseline, t=t)
            d = fsm.step(state, s)

            assert np.all(np.isfinite(d.desired_xyz))
            assert d.next_state != "ERROR"
            assert d.error is None

            # simple dynamics: ee jumps fully to the commanded setpoint
            ee = np.asarray(d.desired_xyz, float).copy()
            state = d.next_state

            # once searching, after a few steps force the peg to drop in
            if state == "SEARCH_SPIRAL":
                spiral_steps += 1
                if spiral_steps >= 3 and not dropped:
                    ee[2] = fsm.contact_z - 2.0 * p.z_drop_threshold_m
                    dropped = True

            if state == "DONE":
                reached_done = True
                break
            t += dt

        assert reached_done is True
        assert dropped is True
