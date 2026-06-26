from robo67_insertion.lib.insertion_event import (
    InsertionEventParams, InsertionEventDetector)


def _det(**kw):
    base = dict(fz_filter_alpha=1.0, slacken_frac=0.4, confirm_drop_m=0.003,
                confirm_window_s=1.0, min_press_n=2.0)
    base.update(kw)
    return InsertionEventDetector(InsertionEventParams(**base))


def test_constant_press_no_events():
    d = _det()
    t = 0.0
    ev = None
    for _ in range(20):
        ev = d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t)
        t += 0.05
    assert not ev.slacken and not ev.inserted


def test_slacken_when_press_drops():
    d = _det()
    t = 0.0
    for _ in range(10):                      # build & hold a 5 N press
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t)
        t += 0.05
    ev = d.observe(press_n=1.0, z_ee=0.10, descending=False, t=t)  # -80%
    assert ev.slacken and not ev.inserted    # slack, but no descent yet


def test_inserted_requires_confirmed_descent():
    d = _det()
    t = 0.0
    for _ in range(10):
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t)
        t += 0.05
    d.observe(press_n=1.0, z_ee=0.10, descending=True, t=t)        # slacken
    t += 0.05
    ev = d.observe(press_n=1.0, z_ee=0.10 - 0.004, descending=True, t=t)  # sank 4 mm
    assert ev.inserted


def test_slacken_without_descent_resets_after_window():
    d = _det()
    t = 0.0
    for _ in range(10):
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t)
        t += 0.05
    d.observe(press_n=1.0, z_ee=0.10, descending=False, t=t)             # slacken
    ev = d.observe(press_n=1.0, z_ee=0.10, descending=False, t=t + 2.0)  # window gone
    assert not ev.inserted                   # latch reset, never confirmed


def test_filter_rejects_single_noise_dip():
    d = _det(fz_filter_alpha=0.2)            # heavy smoothing
    t = 0.0
    for _ in range(20):
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t)
        t += 0.05
    ev = d.observe(press_n=0.0, z_ee=0.10, descending=False, t=t)        # 1-sample dip
    assert not ev.slacken                    # EMA barely moves -> no false slacken


def test_invalid_alpha_rejected():
    import pytest
    with pytest.raises(ValueError):
        InsertionEventDetector(InsertionEventParams(fz_filter_alpha=0.0))
