import math

from robo67_insertion.lib.force_regulator import AxialForceParams, AxialForceRegulator

SOCKET_TOP = 0.10


def _reg(**kw):
    p = AxialForceParams(pos_stiff=2000.0, k_adm=0.001, v_cap_mps=0.01,
                         max_press_depth_m=0.05, max_force_n=12.0, **kw)
    return AxialForceRegulator(p, socket_top_z=SOCKET_TOP)


def test_seed_reproduces_press_without_jump():
    r = _reg()
    z_ee = 0.10
    z0 = r.seed(z_ee, press_n=4.0)          # 4 N at 2000 N/m -> gap 2 mm
    assert math.isclose(z0, z_ee - 4.0 / 2000.0, abs_tol=1e-9)


def test_holds_when_at_target():
    r = _reg()
    z = r.step(z_cmd_prev=0.098, z_ee=0.10, press_n=5.0, f_target_n=5.0, dt=0.01)
    assert math.isclose(z, 0.098, abs_tol=1e-9)   # err 0 -> no move


def test_descends_when_slack():
    r = _reg()
    z = r.step(z_cmd_prev=0.098, z_ee=0.10, press_n=1.0, f_target_n=5.0, dt=0.01)
    # err=+4 N -> v=k_adm*4=0.004 m/s -> dz=4e-5 m downward
    assert z < 0.098 and math.isclose(0.098 - z, 0.001 * 4.0 * 0.01, abs_tol=1e-9)


def test_eases_when_overpressed_but_not_above_ee():
    r = _reg()
    z = r.step(z_cmd_prev=0.099, z_ee=0.10, press_n=12.0, f_target_n=5.0, dt=0.01)
    assert z > 0.099 and z <= 0.10            # rises (reduces force), capped at z_ee


def test_reduces_force_when_too_high():
    # User requirement: if the press force exceeds the target, the regulator must
    # back the equilibrium UP so the commanded force comes back down.
    r = _reg()
    z_prev = 0.090                            # 10 mm below the EE -> deep press
    z = r.step(z_cmd_prev=z_prev, z_ee=0.10, press_n=20.0, f_target_n=5.0, dt=0.01)
    assert z > z_prev                         # equilibrium rose -> gap (and force) shrank
    # over-pressed by 15 N, gain*err = -0.015 m/s, capped to -v_cap -> +0.01 m/s up
    assert math.isclose(z - z_prev, 0.01 * 0.01, abs_tol=1e-9)


def test_velocity_cap():
    r = _reg()
    z = r.step(z_cmd_prev=0.099, z_ee=0.10, press_n=0.0, f_target_n=12.0, dt=1.0)
    assert math.isclose(0.099 - z, 0.01 * 1.0, abs_tol=1e-9)   # |dz| == v_cap*dt


def test_z_floor():
    r = _reg()
    floor = SOCKET_TOP - 0.05
    z = r.step(z_cmd_prev=floor + 1e-4, z_ee=0.20, press_n=0.0, f_target_n=12.0, dt=1.0)
    assert z >= floor - 1e-12 and math.isclose(z, floor, abs_tol=1e-9)


def test_f_target_clamped_to_max_force():
    r = _reg()
    z_hi = r.step(z_cmd_prev=0.099, z_ee=0.10, press_n=0.0, f_target_n=100.0, dt=0.001)
    z_at_cap = r.step(z_cmd_prev=0.099, z_ee=0.10, press_n=0.0, f_target_n=12.0, dt=0.001)
    assert math.isclose(z_hi, z_at_cap, abs_tol=1e-12)
