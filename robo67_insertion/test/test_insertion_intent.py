"""Tests for robo67_insertion.lib.insertion_intent (TDD, canonical seam).

The canonical insertion intent module is PURE and controller-AGNOSTIC: given
``(phase, IntentSensors)`` :meth:`InsertionIntentModule.step` returns an
:class:`InsertionIntent` (the NEXT phase plus an absolute, base-frame
``target_xyz``). Controller quirks (lead-clamp carrot, below-surface
equilibrium gaps, held orientation) belong to the command-path ADAPTERS, never
here.

Conventions match the legacy FSM: Z is UP, so descending means decreasing z.
"Contact" records ``contact_z`` (the socket-top height); the peg drops INTO the
hole when ``ee_z`` falls below ``contact_z``. These tests mirror the rigor of
``test_insertion_fsm.py`` but target the canonical absolute targets.
"""
import numpy as np
import pytest

from robo67_insertion.lib.insertion_intent import (
    PHASES,
    InsertionIntent,
    InsertionIntentModule,
    IntentParams,
    IntentSensors,
)

SOCKET = (0.5, 0.0, 0.2)
BASELINE = 0.0


def make_module(socket=SOCKET, params=None):
    return InsertionIntentModule(socket, params if params is not None else IntentParams())


def isensors(ee, fz=BASELINE, baseline=BASELINE, t=0.0):
    return IntentSensors(
        ee_xyz=tuple(float(v) for v in ee), fz=fz, fz_baseline=baseline, t=t
    )


def assert_intent_shape(out):
    assert isinstance(out, InsertionIntent)
    assert out.phase in PHASES
    assert len(out.target_xyz) == 3
    assert np.all(np.isfinite(out.target_xyz))


class TestIdle:
    def test_idle_emits_move_above_at_standoff(self):
        m = make_module()
        p = m.params
        out = m.step("IDLE", isensors([0.5, 0.0, 0.31]))
        assert_intent_shape(out)
        assert out.phase == "MOVE_ABOVE"
        assert out.target_xyz == pytest.approx(
            (SOCKET[0], SOCKET[1], SOCKET[2] + p.standoff_m)
        )
        assert out.done is False
        assert out.error is None


class TestMoveAbove:
    def test_within_tol_descends(self):
        m = make_module()
        p = m.params
        target = (SOCKET[0], SOCKET[1], SOCKET[2] + p.standoff_m)
        ee = (target[0] + 0.5 * p.approach_tol_m, target[1], target[2])
        out = m.step("MOVE_ABOVE", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "DESCEND_TO_CONTACT"
        assert out.target_xyz == pytest.approx(target)

    def test_far_stays(self):
        m = make_module()
        p = m.params
        target = (SOCKET[0], SOCKET[1], SOCKET[2] + p.standoff_m)
        ee = (target[0] + 0.05, target[1], target[2])
        out = m.step("MOVE_ABOVE", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "MOVE_ABOVE"
        assert out.target_xyz == pytest.approx(target)


class TestDescendToContact:
    def test_no_contact_aims_below_surface(self):
        m = make_module()
        p = m.params
        ee = (0.5, 0.0, 0.24)
        out = m.step("DESCEND_TO_CONTACT", isensors(ee, fz=BASELINE, baseline=BASELINE))
        assert_intent_shape(out)
        assert out.phase == "DESCEND_TO_CONTACT"
        # canonical target aims DOWN past the surface so both controllers descend
        assert out.target_xyz[0] == pytest.approx(SOCKET[0])
        assert out.target_xyz[1] == pytest.approx(SOCKET[1])
        assert out.target_xyz[2] == pytest.approx(SOCKET[2] - p.insert_depth_m)
        assert out.target_xyz[2] < SOCKET[2]
        assert m.contact_z is None
        assert out.contact_z is None

    def test_contact_records_contact_z_and_searches(self):
        m = make_module()
        p = m.params
        ee = (0.5, 0.0, 0.2)
        fz = BASELINE + p.contact_fz_threshold_n + 1.0
        out = m.step("DESCEND_TO_CONTACT", isensors(ee, fz=fz, baseline=BASELINE, t=3.0))
        assert_intent_shape(out)
        assert out.phase == "SEARCH_SPIRAL"
        assert m.contact_z == pytest.approx(ee[2])
        assert out.contact_z == pytest.approx(ee[2])
        assert m.spiral_t0 == pytest.approx(3.0)
        assert m.retries == 0


class TestSearchSpiral:
    def test_offset_grows_with_time_at_contact_plane(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        m.spiral_t0 = 0.0
        ee = (0.5, 0.0, 0.2)
        prev_norm = -1.0
        for t in (0.5, 1.0, 2.0, 4.0):
            out = m.step("SEARCH_SPIRAL", isensors(ee, t=t))
            assert_intent_shape(out)
            assert out.phase == "SEARCH_SPIRAL"
            offset = np.array(out.target_xyz[:2]) - np.array(SOCKET[:2])
            norm = float(np.linalg.norm(offset))
            assert norm > prev_norm
            prev_norm = norm
            # canonical Z is the CONTACT PLANE; downward press bias is an adapter job
            assert out.target_xyz[2] == pytest.approx(m.contact_z)

    def test_drop_records_hole_xy_and_pushes(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        m.spiral_t0 = 0.0
        ee = (0.51, 0.01, m.contact_z - 2 * p.z_drop_threshold_m)
        out = m.step("SEARCH_SPIRAL", isensors(ee, t=1.0))
        assert_intent_shape(out)
        assert out.phase == "PUSH_INSERT"
        assert m.hole_xy is not None
        assert tuple(m.hole_xy) == pytest.approx((ee[0], ee[1]))


class TestPushInsert:
    def test_continues_until_seated(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        m.hole_xy = (0.51, 0.01)
        ee = (0.51, 0.01, m.contact_z - 0.005)
        out = m.step("PUSH_INSERT", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "PUSH_INSERT"
        # canonical seated target: hole xy + (contact_z - insert_depth)
        assert out.target_xyz[0] == pytest.approx(0.51)
        assert out.target_xyz[1] == pytest.approx(0.01)
        assert out.target_xyz[2] == pytest.approx(m.contact_z - p.insert_depth_m)

    def test_reaches_seated_goes_to_confirm(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        target_z = m.contact_z - p.insert_depth_m
        ee = (0.5, 0.0, target_z + 0.4 * p.z_drop_threshold_m)
        out = m.step("PUSH_INSERT", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "CONFIRM"


class TestConfirm:
    def test_depth_ok_retracts(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        ee = (0.5, 0.0, m.contact_z - p.insert_depth_m)
        out = m.step("CONFIRM", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "RETRACT"

    def test_not_deep_retries_back_to_search(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        m.retries = 0
        ee = (0.5, 0.0, m.contact_z - 0.001)
        out = m.step("CONFIRM", isensors(ee, t=9.0))
        assert_intent_shape(out)
        assert out.phase == "SEARCH_SPIRAL"
        assert m.retries == 1
        assert m.spiral_t0 == pytest.approx(9.0)
        assert out.error is None

    def test_not_deep_exhausted_errors(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        m.retries = p.retry_limit
        ee = (0.5, 0.0, m.contact_z - 0.001)
        out = m.step("CONFIRM", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "ERROR"
        assert out.done is True
        assert out.error is not None


class TestRetract:
    def test_below_contact_keeps_retracting(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        ee = (0.5, 0.0, m.contact_z - 0.01)
        out = m.step("RETRACT", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "RETRACT"
        assert out.target_xyz == pytest.approx(
            (SOCKET[0], SOCKET[1], SOCKET[2] + p.standoff_m)
        )

    def test_above_contact_is_done(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        ee = (0.5, 0.0, m.contact_z + 0.05)
        out = m.step("RETRACT", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "DONE"


class TestTerminal:
    def test_done_holds_and_is_done(self):
        m = make_module()
        ee = (0.5, 0.0, 0.25)
        out = m.step("DONE", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "DONE"
        assert out.done is True
        assert out.target_xyz == pytest.approx(ee)

    def test_error_holds_and_preserves(self):
        m = make_module()
        m.error = "boom"
        ee = (0.5, 0.0, 0.25)
        out = m.step("ERROR", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "ERROR"
        assert out.done is True
        assert out.error == "boom"
        assert out.target_xyz == pytest.approx(ee)

    def test_unknown_phase_fails_safe(self):
        m = make_module()
        ee = (0.5, 0.0, 0.25)
        out = m.step("NONSENSE", isensors(ee))
        assert_intent_shape(out)
        assert out.phase == "ERROR"
        assert out.done is True
        assert out.error is not None
        assert out.target_xyz == pytest.approx(ee)


class TestSpiralExhaustion:
    def test_radius_exceeded_eventually_errors(self):
        m = make_module()
        p = m.params
        m.contact_z = 0.2
        m.spiral_t0 = 0.0
        m.retries = 0
        ee = (0.5, 0.0, 0.2)  # never drops
        dt_big = 1000.0
        last = None
        for i in range(p.retry_limit + 1):
            last = m.step("SEARCH_SPIRAL", isensors(ee, t=dt_big * (i + 1)))
            assert np.all(np.isfinite(last.target_xyz))
        assert last.phase == "ERROR"
        assert last.done is True
        assert last.error is not None


class TestIntegration:
    def test_full_run_reaches_done(self):
        socket = (0.5, 0.0, 0.2)
        m = make_module(socket)
        p = m.params
        contact_height = socket[2]
        baseline = 0.0

        ee = np.array([socket[0], socket[1], socket[2] + p.standoff_m])
        phase = "IDLE"
        t = 0.0
        dt = 0.02
        dropped = False
        spiral_steps = 0
        reached_done = False

        for _ in range(2000):
            if ee[2] <= contact_height + 1e-9:
                fz = baseline + p.contact_fz_threshold_n + 1.0
            else:
                fz = baseline
            out = m.step(phase, isensors(ee.copy(), fz=fz, baseline=baseline, t=t))
            assert np.all(np.isfinite(out.target_xyz))
            assert out.error is None

            # simple dynamics: ee jumps fully to the commanded canonical target
            ee = np.asarray(out.target_xyz, float).copy()
            phase = out.phase

            if phase == "SEARCH_SPIRAL":
                spiral_steps += 1
                if spiral_steps >= 3 and not dropped:
                    ee[2] = m.contact_z - 2.0 * p.z_drop_threshold_m
                    dropped = True

            if phase == "DONE":
                reached_done = True
                break
            t += dt

        assert reached_done is True
        assert dropped is True
        assert m.contact_z is not None
        assert m.hole_xy is not None
