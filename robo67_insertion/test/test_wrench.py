"""Tests for robo67_insertion.lib.wrench (TDD).

Fz is the Z component of the Franka external wrench estimate
o_f_ext_hat_k (index 2). Contact is detected when the deviation of Fz
from a running baseline exceeds a threshold.
"""
import pytest

from robo67_insertion.lib.wrench import BaselineEstimator, contact_detected


class TestContactDetected:
    def test_negative_deviation_above_threshold_is_contact(self):
        assert contact_detected(-6.0, 0.0, 5.0) is True

    def test_negative_deviation_below_threshold_is_not_contact(self):
        assert contact_detected(-3.0, 0.0, 5.0) is False

    def test_positive_deviation_above_threshold_is_contact(self):
        assert contact_detected(7.0, 0.0, 5.0) is True

    def test_deviation_relative_to_nonzero_baseline(self):
        assert contact_detected(2.0, 1.0, 5.0) is False

    def test_deviation_exactly_at_threshold_is_contact(self):
        assert contact_detected(5.0, 0.0, 5.0) is True


class TestBaselineEstimator:
    def test_constant_input_converges_to_value(self):
        est = BaselineEstimator(alpha=0.1, initial=0.0)
        for _ in range(200):
            est.update(4.0)
        assert est.value == pytest.approx(4.0, abs=1e-2)

    def test_reset_sets_value(self):
        est = BaselineEstimator(alpha=0.1, initial=0.0)
        for _ in range(50):
            est.update(4.0)
        est.reset(0.0)
        assert est.value == 0.0

    def test_reset_default_is_zero(self):
        est = BaselineEstimator(alpha=0.2, initial=3.0)
        est.reset()
        assert est.value == 0.0

    def test_value_moves_monotonically_toward_constant_input(self):
        est = BaselineEstimator(alpha=0.1, initial=0.0)
        prev = est.value
        for _ in range(50):
            cur = est.update(4.0)
            assert cur >= prev
            prev = cur
        assert prev <= 4.0

    def test_update_returns_new_value(self):
        est = BaselineEstimator(alpha=0.5, initial=0.0)
        ret = est.update(2.0)
        assert ret == est.value
        assert ret == pytest.approx(1.0)

    def test_initial_seeds_ema_before_formula(self):
        # First update starts ema at `initial`, then applies the EMA formula.
        est = BaselineEstimator(alpha=0.1, initial=10.0)
        ret = est.update(0.0)
        # ema = (1-0.1)*10.0 + 0.1*0.0 = 9.0
        assert ret == pytest.approx(9.0)

    def test_default_alpha(self):
        est = BaselineEstimator()
        ret = est.update(1.0)
        # default alpha=0.1, default initial=0.0 -> (0.9*0.0)+(0.1*1.0)=0.1
        assert ret == pytest.approx(0.1)
