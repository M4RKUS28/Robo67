"""Config loader tests."""

import os

from robo67_insertion.config_schema import RoboConfig, load_config

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "robo67.yaml")


def test_defaults_construct():
    cfg = RoboConfig()
    assert cfg.stiffness.free_translational == 400.0
    assert cfg.insertion.control_rate_hz == 50.0
    assert len(cfg.safety.workspace_aabb) == 3


def test_load_yaml():
    cfg = load_config(CONFIG)
    assert cfg.topics.controller_name == "panda_cartesian_impedance_controller"
    assert cfg.spiral.max_radius_m == 0.012
    assert cfg.safety.fz_abort_n == 25.0
    assert cfg.socket_cube_height_m == 0.06


def test_force_search_defaults():
    from robo67_insertion.config_schema import ForceSearchCfg
    cfg = RoboConfig()
    assert isinstance(cfg.force_search, ForceSearchCfg)
    assert cfg.force_search.enabled is False
    assert cfg.force_search.search_press_n == 5.0
    assert cfg.force_search.slacken_frac == 0.4


def test_force_search_loaded_from_package_yaml():
    cfg = load_config(CONFIG)
    assert cfg.force_search.enabled is False
    assert cfg.force_search.k_adm == 0.0008


def test_force_search_override(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("force_search:\n  enabled: true\n  search_press_n: 7.0\n")
    cfg = load_config(str(p))
    assert cfg.force_search.enabled is True
    assert cfg.force_search.search_press_n == 7.0
    assert cfg.force_search.slacken_frac == 0.4   # untouched default kept
