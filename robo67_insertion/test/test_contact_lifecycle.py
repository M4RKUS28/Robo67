"""Tests for robo67_insertion.lib.contact_lifecycle (TDD).

The contact lifecycle module owns the baseline update/freeze policy that
used to live inline in the sim orchestrator: the free-space Fz baseline
(EMA) is tracked ONLY in free space and FROZEN in every contact phase,
where it is used as the reference for symmetric contact detection.
"""
import pytest

from robo67_insertion.lib.contact_lifecycle import (
    ContactLifecycleModule,
    ContactOutcome,
)

FROZEN_MODES = ("contact_search", "insert", "confirm")


class TestFreeSpaceTracking:
    def test_baseline_converges_to_constant_fz(self):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        out = None
        for _ in range(200):
            out = mod.observe("free_space", 4.0)
        assert isinstance(out, ContactOutcome)
        assert out.baseline_fz == pytest.approx(4.0, abs=1e-2)

    def test_baseline_starts_at_initial_then_tracks(self):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=10.0)
        out = mod.observe("free_space", 0.0)
        # ema = (1 - 0.1) * 10.0 + 0.1 * 0.0 = 9.0
        assert out.baseline_fz == pytest.approx(9.0)

    def test_free_space_never_reports_contact(self):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        # Even a huge fz deviation must not flag contact in free space.
        for fz in (0.0, 100.0, -100.0, 4.0):
            out = mod.observe("free_space", fz)
            assert out.contact_detected is False


class TestFreeze:
    @pytest.mark.parametrize("mode", FROZEN_MODES)
    def test_baseline_frozen_across_varying_fz(self, mode):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        # Establish a baseline in free space.
        for _ in range(200):
            mod.observe("free_space", 4.0)
        established = mod.observe("free_space", 4.0).baseline_fz
        # Now in a contact phase the baseline must not move, even as fz varies.
        for fz in (0.0, 50.0, -50.0, 4.0, 9.0):
            out = mod.observe(mode, fz)
            assert out.baseline_fz == pytest.approx(established)


class TestContactDetection:
    @pytest.mark.parametrize("mode", FROZEN_MODES)
    def test_at_threshold_positive_is_contact(self, mode):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        # baseline stays at initial 0.0 in frozen modes
        out = mod.observe(mode, 5.0)
        assert out.contact_detected is True

    @pytest.mark.parametrize("mode", FROZEN_MODES)
    def test_at_threshold_negative_is_contact(self, mode):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        out = mod.observe(mode, -5.0)
        assert out.contact_detected is True

    @pytest.mark.parametrize("mode", FROZEN_MODES)
    def test_just_below_threshold_positive_is_not_contact(self, mode):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        out = mod.observe(mode, 4.999)
        assert out.contact_detected is False

    @pytest.mark.parametrize("mode", FROZEN_MODES)
    def test_just_below_threshold_negative_is_not_contact(self, mode):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        out = mod.observe(mode, -4.999)
        assert out.contact_detected is False

    def test_detection_relative_to_frozen_nonzero_baseline(self):
        mod = ContactLifecycleModule(threshold_n=5.0, alpha=0.1, initial=0.0)
        for _ in range(500):
            mod.observe("free_space", 10.0)
        baseline = mod.observe("free_space", 10.0).baseline_fz
        assert baseline == pytest.approx(10.0, abs=1e-3)
        # deviation measured from the frozen baseline, not from zero
        assert mod.observe("insert", 12.0).contact_detected is False
        assert mod.observe("insert", 15.0).contact_detected is True
