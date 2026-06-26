# Force-guided Search & Insertion — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Why** lives in [ADR-0002](../../adr/0002-force-guided-search-and-insertion.md) and the [design doc](../../architecture/force-guided-insertion-2026-06-26.md). This file is the *how*.

**Goal:** Replace the fixed below-surface equilibrium in the real-arm search/seat
phases with a regulated **constant gentle axial press (admittance)** that chases
the peg down as resistance slackens, and detect insertion from the **force-slacken
+ confirmed descent** instead of a fragile absolute z-drop.

**Architecture:** Two new PURE, host-tested seams — `AxialForceRegulator`
(force→equilibrium-z) and `InsertionEventDetector` (slacken/inserted from
filtered Fz) — composed by `hardware_insertion_node`. The canonical
`insertion_intent.py` stays controller-agnostic and position-only (ADR-0001);
the impedance adapter gains an opt-in `force_mode` that yields its XY while the
node supplies the regulated z. All gated behind `force_search.enabled` (default
OFF = current verified behavior).

**Tech Stack:** Python 3, numpy + stdlib (pure seams), pytest, ROS 2 Humble
(`rclpy`) only in the node, MuJoCo for sim validation.

## Global Constraints

- Branch: all commits to the working branch off `jearningers` (per `CLAUDE.md`); worktrees branch from and merge back to `jearningers`.
- Paths below are **relative to the repo root**. The Python package root is `robo67_insertion/` and the importable package is `robo67_insertion/robo67_insertion/`.
- Run tests with the package root on the path: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test -q`.
- Pure seams (`lib/*.py`) import **no rclpy/ROS/cv2/scipy** — numpy + stdlib only — so they stay host-unit-testable. The `--selftest` path must stay ROS-free.
- `insertion_intent.py` MUST stay controller-agnostic and position-based (ADR-0001). Force/equilibrium math is an adapter/node concern.
- Every published setpoint still passes `SafetyEnvelopeModule(ImpedanceSafetyProfile)`: AABB clamp + z-floor (`socket_top − max_press_depth`) + per-tick step cap (`v_max/rate`, anchor = previous command) + force/moment abort.
- `--pos-stiff` MUST match the running controller (the regulator uses it).
- `press_n` is the **press magnitude** `|fz_meas − fz_baseline|` (N); the search only ever presses down, so magnitude is sign-robust against the unknown sign of `o_f_ext_hat_k[2]`.
- New behavior is opt-in (`force_search.enabled: false` default); the adapter conformance test must still prove an identical canonical phase sequence.

---

## File Structure

```
robo67_insertion/robo67_insertion/
  lib/
    force_regulator.py        # NEW (pure)  AxialForceRegulator
    insertion_event.py        # NEW (pure)  InsertionEventDetector
    command_path_adapters.py  # EDIT  ImpedanceCommandPathAdapter(force_mode=...)
  config_schema.py            # EDIT  ForceSearchCfg + RoboConfig field + merge
  nodes/
    hardware_insertion_node.py# EDIT  own regulator+detector, CLI, selftest, run_ros
  config/robo67.yaml          # EDIT  force_search: {...} defaults
robo67_insertion/test/
  test_force_regulator.py     # NEW
  test_insertion_event.py     # NEW
  test_command_path_adapters.py # EDIT (force_mode cases; conformance unchanged)
  test_config.py              # EDIT (force_search defaults + override)
docs/runbooks/automated-insertion.md  # EDIT  --force-mode section
```

---

## Task 1: `AxialForceRegulator` pure seam

**Files:**
- Create: `robo67_insertion/robo67_insertion/lib/force_regulator.py`
- Test: `robo67_insertion/test/test_force_regulator.py`

**Interfaces:**
- Produces:
  - `AxialForceParams(pos_stiff, k_adm=0.0008, v_cap_mps=0.01, max_press_depth_m=0.05, max_force_n=12.0)`
  - `AxialForceRegulator(params, socket_top_z)` with
    `seed(z_ee, press_n) -> float` and
    `step(z_cmd_prev, z_ee, press_n, f_target_n, dt) -> float`.
  - Convention: returned z never below `socket_top_z − max_press_depth_m` and never above `z_ee` (always pressing down or zero). `v_cap` bounds the per-tick equilibrium speed.

- [ ] **Step 1: Write the failing test**

```python
# robo67_insertion/test/test_force_regulator.py
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
    assert math.isclose(z0, z_ee - 4.0/2000.0, abs_tol=1e-9)

def test_holds_when_at_target():
    r = _reg()
    z = r.step(z_cmd_prev=0.098, z_ee=0.10, press_n=5.0, f_target_n=5.0, dt=0.01)
    assert math.isclose(z, 0.098, abs_tol=1e-9)   # err 0 -> no move

def test_descends_when_slack():
    r = _reg()
    z = r.step(z_cmd_prev=0.098, z_ee=0.10, press_n=1.0, f_target_n=5.0, dt=0.01)
    # err=+4 N -> v=k_adm*4=0.004 m/s -> dz=4e-5 m downward
    assert z < 0.098 and math.isclose(0.098 - z, 0.001*4.0*0.01, abs_tol=1e-9)

def test_eases_when_overpressed_but_not_above_ee():
    r = _reg()
    z = r.step(z_cmd_prev=0.099, z_ee=0.10, press_n=12.0, f_target_n=5.0, dt=0.01)
    assert z > 0.099 and z <= 0.10            # rises, capped at z_ee

def test_velocity_cap():
    r = _reg()
    z = r.step(z_cmd_prev=0.099, z_ee=0.10, press_n=0.0, f_target_n=12.0, dt=1.0)
    assert math.isclose(0.099 - z, 0.01*1.0, abs_tol=1e-9)   # |dz| == v_cap*dt

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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_force_regulator.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'robo67_insertion.lib.force_regulator'`.

- [ ] **Step 3: Write minimal implementation**

```python
# robo67_insertion/robo67_insertion/lib/force_regulator.py
"""Axial force regulator (admittance) for force-guided peg-in-hole (pure).

Converts a target press FORCE into a commanded equilibrium HEIGHT for the soft
Cartesian impedance controller (Fz ~ pos_stiff * (z_ee - z_cmd)). The equilibrium
ratchets from the previous command at an admittance rate so the arm holds a
constant gentle press AND chases the peg down when resistance slackens. See
docs/architecture/force-guided-insertion-2026-06-26.md and ADR-0002.

press_n is the press MAGNITUDE |fz_meas - fz_baseline| (N); search only presses
down, so magnitude is sign-robust against the unknown sign of o_f_ext_hat_k[2].

numpy + stdlib only. Must NOT import rclpy/ROS/cv2/scipy.
"""
from __future__ import annotations

from dataclasses import dataclass

__all__ = ["AxialForceParams", "AxialForceRegulator"]


@dataclass(frozen=True)
class AxialForceParams:
    pos_stiff: float                 # N/m, MUST match the running controller
    k_adm: float = 0.0008            # m/s per N (admittance gain)
    v_cap_mps: float = 0.01          # axial equilibrium speed cap (<= v_max)
    max_press_depth_m: float = 0.05  # z-floor = socket_top - this
    max_force_n: float = 12.0        # clamp on the force target


class AxialForceRegulator:
    """Force-target -> equilibrium-z, ratcheting from the previous command."""

    def __init__(self, params: AxialForceParams, socket_top_z: float) -> None:
        self.params = params
        self.z_floor = float(socket_top_z) - float(params.max_press_depth_m)

    def _clamp_z(self, z: float, z_ee: float | None = None) -> float:
        z = max(float(z), self.z_floor)            # never deeper than the floor
        if z_ee is not None:
            z = min(z, float(z_ee))                # never above the EE (press only)
        return z

    def seed(self, z_ee: float, press_n: float) -> float:
        """Initial equilibrium reproducing the measured press with NO jump."""
        return self._clamp_z(float(z_ee) - float(press_n) / self.params.pos_stiff, z_ee)

    def step(self, z_cmd_prev: float, z_ee: float, press_n: float,
             f_target_n: float, dt: float) -> float:
        """One admittance tick -> next commanded equilibrium height."""
        p = self.params
        f_target = min(float(f_target_n), p.max_force_n)
        err = f_target - float(press_n)            # >0 = under-pressed -> descend
        v = max(-p.v_cap_mps, min(p.v_cap_mps, p.k_adm * err))  # +v = descend
        z_next = float(z_cmd_prev) - v * float(dt)
        return self._clamp_z(z_next, z_ee)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_force_regulator.py -q`
Expected: PASS (7 passed).

- [ ] **Step 5: Commit**

```bash
git add robo67_insertion/robo67_insertion/lib/force_regulator.py robo67_insertion/test/test_force_regulator.py
git commit -m "feat(robo67): AxialForceRegulator (admittance) pure seam for force-guided search"
```

---

## Task 2: `InsertionEventDetector` pure seam

**Files:**
- Create: `robo67_insertion/robo67_insertion/lib/insertion_event.py`
- Test: `robo67_insertion/test/test_insertion_event.py`

**Interfaces:**
- Consumes: `press_n` = `|fz_meas − fz_baseline|` magnitude (same convention as Task 1).
- Produces:
  - `InsertionEventParams(fz_filter_alpha=0.2, slacken_frac=0.4, confirm_drop_m=0.003, confirm_window_s=1.0, min_press_n=2.0)`
  - `InsertionEvent(press_filt_n, slacken, inserted)`
  - `InsertionEventDetector(params)` with `observe(press_n, z_ee, descending, t) -> InsertionEvent`. (`contact_z` is not needed once detection is force-based; dropped from the design's signature.)

- [ ] **Step 1: Write the failing test**

```python
# robo67_insertion/test/test_insertion_event.py
from robo67_insertion.lib.insertion_event import (
    InsertionEventParams, InsertionEventDetector)

def _det(**kw):
    return InsertionEventDetector(InsertionEventParams(
        fz_filter_alpha=1.0, slacken_frac=0.4, confirm_drop_m=0.003,
        confirm_window_s=1.0, min_press_n=2.0, **kw))

def test_constant_press_no_events():
    d = _det()
    z, t = 0.10, 0.0
    ev = None
    for _ in range(20):
        ev = d.observe(press_n=5.0, z_ee=z, descending=False, t=t); t += 0.05
    assert not ev.slacken and not ev.inserted

def test_slacken_when_press_drops():
    d = _det()
    t = 0.0
    for _ in range(10):                      # build & hold a 5 N press
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t); t += 0.05
    ev = d.observe(press_n=1.0, z_ee=0.10, descending=False, t=t)  # -80%
    assert ev.slacken and not ev.inserted    # slack, but no descent yet

def test_inserted_requires_confirmed_descent():
    d = _det()
    t = 0.0
    for _ in range(10):
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t); t += 0.05
    d.observe(press_n=1.0, z_ee=0.10, descending=True, t=t); t += 0.05   # slacken
    ev = d.observe(press_n=1.0, z_ee=0.10 - 0.004, descending=True, t=t)  # sank 4 mm
    assert ev.inserted

def test_slacken_without_descent_resets_after_window():
    d = _det()
    t = 0.0
    for _ in range(10):
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t); t += 0.05
    d.observe(press_n=1.0, z_ee=0.10, descending=False, t=t)             # slacken
    ev = d.observe(press_n=1.0, z_ee=0.10, descending=False, t=t + 2.0)  # window gone
    assert not ev.inserted                   # latch reset, never confirmed

def test_filter_rejects_single_noise_dip():
    d = _det(fz_filter_alpha=0.2)            # heavy smoothing
    t = 0.0
    for _ in range(20):
        d.observe(press_n=5.0, z_ee=0.10, descending=False, t=t); t += 0.05
    ev = d.observe(press_n=0.0, z_ee=0.10, descending=False, t=t)        # 1-sample dip
    assert not ev.slacken                    # EMA barely moves -> no false slacken
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_insertion_event.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'robo67_insertion.lib.insertion_event'`.

- [ ] **Step 3: Write minimal implementation**

```python
# robo67_insertion/robo67_insertion/lib/insertion_event.py
"""Force-slacken + confirmed-descent insertion-event detector (pure).

Replaces the fragile absolute z-drop trigger: insertion is detected when the
filtered press MAGNITUDE drops by >= slacken_frac of the recently-held press
(the bore opened) AND the EE then descends >= confirm_drop_m within
confirm_window_s while still being driven down. See ADR-0002 and
docs/architecture/force-guided-insertion-2026-06-26.md.

numpy + stdlib only. Must NOT import rclpy/ROS/cv2/scipy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

__all__ = ["InsertionEventParams", "InsertionEvent", "InsertionEventDetector"]


@dataclass(frozen=True)
class InsertionEventParams:
    fz_filter_alpha: float = 0.2     # EMA smoothing of the press estimate (0,1]
    slacken_frac: float = 0.4        # fraction of held press lost = slacken
    confirm_drop_m: float = 0.003    # EE descent to confirm entry
    confirm_window_s: float = 1.0    # confirm must occur within this of slacken
    min_press_n: float = 2.0         # ignore slacken until a real press was held


@dataclass(frozen=True)
class InsertionEvent:
    press_filt_n: float
    slacken: bool
    inserted: bool


class InsertionEventDetector:
    def __init__(self, params: InsertionEventParams) -> None:
        if not (0.0 < params.fz_filter_alpha <= 1.0):
            raise ValueError("fz_filter_alpha must be in (0, 1]")
        self.p = params
        self._press_filt: Optional[float] = None
        self._press_hold = 0.0
        self._slack_latched = False
        self._t_slack = 0.0
        self._z_slack = 0.0

    def observe(self, press_n: float, z_ee: float, descending: bool,
                t: float) -> InsertionEvent:
        a = self.p.fz_filter_alpha
        pf = float(press_n) if self._press_filt is None else (
            (1.0 - a) * self._press_filt + a * float(press_n))
        self._press_filt = pf

        slacken = (self._press_hold >= self.p.min_press_n
                   and pf < (1.0 - self.p.slacken_frac) * self._press_hold)

        if slacken and not self._slack_latched:
            self._slack_latched = True
            self._t_slack = float(t)
            self._z_slack = float(z_ee)

        inserted = False
        if self._slack_latched:
            within = (float(t) - self._t_slack) <= self.p.confirm_window_s
            sank = (self._z_slack - float(z_ee)) >= self.p.confirm_drop_m
            if within and descending and sank:
                inserted = True
            elif not within:
                self._slack_latched = False        # expired -> resume tracking

        if not slacken:                            # don't let the drop raise the ref
            self._press_hold = max(self._press_hold, pf)

        return InsertionEvent(press_filt_n=pf, slacken=slacken, inserted=inserted)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_insertion_event.py -q`
Expected: PASS (5 passed).

- [ ] **Step 5: Commit**

```bash
git add robo67_insertion/robo67_insertion/lib/insertion_event.py robo67_insertion/test/test_insertion_event.py
git commit -m "feat(robo67): InsertionEventDetector (force-slacken + confirmed descent) pure seam"
```

---

## Task 3: `ForceSearchCfg` config + yaml + loader

**Files:**
- Modify: `robo67_insertion/robo67_insertion/config_schema.py`
- Modify: `robo67_insertion/config/robo67.yaml`
- Test: `robo67_insertion/test/test_config.py`

**Interfaces:**
- Produces: `ForceSearchCfg` dataclass + `RoboConfig.force_search` field; `load_config` merges the `force_search:` yaml block.

- [ ] **Step 1: Write the failing test** (append to `test_config.py`)

```python
def test_force_search_defaults():
    from robo67_insertion.config_schema import RoboConfig, ForceSearchCfg
    cfg = RoboConfig()
    assert isinstance(cfg.force_search, ForceSearchCfg)
    assert cfg.force_search.enabled is False
    assert cfg.force_search.search_press_n == 5.0
    assert cfg.force_search.slacken_frac == 0.4

def test_force_search_override(tmp_path):
    from robo67_insertion.config_schema import load_config
    p = tmp_path / "c.yaml"
    p.write_text("force_search:\n  enabled: true\n  search_press_n: 7.0\n")
    cfg = load_config(str(p))
    assert cfg.force_search.enabled is True
    assert cfg.force_search.search_press_n == 7.0
    assert cfg.force_search.slacken_frac == 0.4   # untouched default kept
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_config.py -q`
Expected: FAIL — `ImportError: cannot import name 'ForceSearchCfg'`.

- [ ] **Step 3: Write minimal implementation**

Add the dataclass near `InsertionCfg` in `config_schema.py`:

```python
@dataclass
class ForceSearchCfg:
    enabled: bool = False            # opt-in; False = current verified behavior
    search_press_n: float = 5.0      # F* target press during SEARCH_SPIRAL
    insert_press_n: float = 6.0      # F* during PUSH_INSERT (if not releasing)
    k_adm: float = 0.0008            # m/s per N
    v_cap_mps: float = 0.01
    ramp_s: float = 0.5              # contact-force -> search_press ramp
    fz_filter_alpha: float = 0.2
    slacken_frac: float = 0.4
    confirm_drop_m: float = 0.003
    confirm_window_s: float = 1.0
    spiral_freeze_on_slacken: bool = True
    settle_s: float = 0.4
    max_force_n: float = 12.0
```

Add the field to `RoboConfig` (alongside `spiral`) and merge it in `load_config`
(mirror the existing `cfg.spiral = _merge(cfg.spiral, raw.get("spiral", {}))`
line):

```python
    force_search: ForceSearchCfg = field(default_factory=ForceSearchCfg)
```
```python
    cfg.force_search = _merge(cfg.force_search, raw.get("force_search", {}))
```

Append to `robo67_insertion/config/robo67.yaml`:

```yaml
force_search:
  enabled: false
  search_press_n: 5.0
  insert_press_n: 6.0
  k_adm: 0.0008
  v_cap_mps: 0.01
  ramp_s: 0.5
  fz_filter_alpha: 0.2
  slacken_frac: 0.4
  confirm_drop_m: 0.003
  confirm_window_s: 1.0
  spiral_freeze_on_slacken: true
  settle_s: 0.4
  max_force_n: 12.0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_config.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add robo67_insertion/robo67_insertion/config_schema.py robo67_insertion/config/robo67.yaml robo67_insertion/test/test_config.py
git commit -m "feat(robo67): ForceSearchCfg config + yaml defaults (opt-in force-guided search)"
```

---

## Task 4: Adapter `force_mode` (XY from intent, z regulated downstream)

**Files:**
- Modify: `robo67_insertion/robo67_insertion/lib/command_path_adapters.py`
- Test: `robo67_insertion/test/test_command_path_adapters.py`

**Interfaces:**
- Consumes: existing `ImpedanceCommandPathAdapter(socket_xyz, params, pos_stiff, press_force_n, insert_press_n, max_press_depth_m, R)`.
- Produces: new kwarg `force_mode: bool = False`. When `True`, for `SEARCH_SPIRAL`/`PUSH_INSERT` the adapter emits `goal_xyz = (tx, ty, cz)` (the bare contact plane — z is regulated by the node's `AxialForceRegulator`); all other phases and the returned `next_phase` are byte-for-byte unchanged. The existing `press_gap_m`/`insert_gap_m` properties remain for the non-force path.

- [ ] **Step 1: Write the failing test** (append to `test_command_path_adapters.py`)

```python
def test_force_mode_search_emits_contact_plane_z():
    import numpy as np
    from robo67_insertion.lib.command_path_adapters import ImpedanceCommandPathAdapter
    from robo67_insertion.lib.insertion_intent import IntentSensors
    socket = (0.45, 0.0, 0.10)
    a = ImpedanceCommandPathAdapter(socket, pos_stiff=2000.0, force_mode=True)
    # drive to contact so contact_z is set
    a.step("DESCEND_TO_CONTACT", IntentSensors(ee_xyz=(0.45, 0.0, 0.101),
            fz=10.0, fz_baseline=0.0, t=0.0))            # |Fz|>=contact_fz -> contact
    cz = a.module.contact_z
    out = a.step("SEARCH_SPIRAL", IntentSensors(ee_xyz=(0.45, 0.0, cz),
            fz=0.0, fz_baseline=0.0, t=0.1))
    assert abs(out.goal_xyz[2] - cz) < 1e-9              # z = contact plane (no gap)

def test_force_mode_preserves_phase_sequence():
    # force_mode must NOT change canonical transitions (ADR-0001 / conformance).
    import numpy as np
    from robo67_insertion.lib.command_path_adapters import ImpedanceCommandPathAdapter
    from robo67_insertion.lib.insertion_intent import IntentSensors
    socket = (0.45, 0.0, 0.10)
    base = ImpedanceCommandPathAdapter(socket, pos_stiff=2000.0, force_mode=False)
    forced = ImpedanceCommandPathAdapter(socket, pos_stiff=2000.0, force_mode=True)
    phase_b, phase_f = "MOVE_ABOVE", "MOVE_ABOVE"
    ee = np.array([0.45, 0.0, 0.20]); t = 0.0
    for _ in range(50):
        s = IntentSensors(ee_xyz=tuple(ee), fz=0.0, fz_baseline=0.0, t=t)
        ob, of = base.step(phase_b, s), forced.step(phase_f, s)
        assert ob.next_phase == of.next_phase
        phase_b, phase_f = ob.next_phase, of.next_phase
        ee[2] = max(0.099, ee[2] - 0.003); t += 0.02
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_command_path_adapters.py -q`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'force_mode'`.

- [ ] **Step 3: Write minimal implementation**

In `ImpedanceCommandPathAdapter.__init__`, add `force_mode: bool = False` and store `self.force_mode = bool(force_mode)`. In `step`, change the `SEARCH_SPIRAL`/`PUSH_INSERT` branches so that in force mode the z is the contact plane (downstream regulation):

```python
        elif phase == "SEARCH_SPIRAL" and cz is not None:
            goal = (tx, ty, cz if self.force_mode else cz - self.press_gap_m)
        elif phase == "PUSH_INSERT" and cz is not None:
            goal = (tx, ty, cz if self.force_mode else cz - self.insert_gap_m)
```

(Leave `DESCEND_TO_CONTACT` and all other branches untouched — descend math and
the phase machine are unchanged.)

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test/test_command_path_adapters.py -q`
Expected: PASS (existing conformance + 2 new tests).

- [ ] **Step 5: Commit**

```bash
git add robo67_insertion/robo67_insertion/lib/command_path_adapters.py robo67_insertion/test/test_command_path_adapters.py
git commit -m "feat(robo67): ImpedanceCommandPathAdapter force_mode (XY from intent, z regulated)"
```

---

## Task 5: Wire regulator + detector into the node, extend the offline self-test

**Files:**
- Modify: `robo67_insertion/robo67_insertion/nodes/hardware_insertion_node.py`

**Interfaces:**
- Consumes: Tasks 1–4 (`AxialForceRegulator`, `InsertionEventDetector`, adapter `force_mode`, `ForceSearchCfg`).
- Produces: `--force-mode` (+ `--search-press`, `--k-adm`, `--adm-v-cap`, `--slacken-frac`, `--confirm-drop`, `--no-spiral-freeze`) CLI; the control loop in `run_ros` and the offline `selftest` use the regulator for the search/seat z and the detector for the insert trigger when force mode is on.

This task is mostly **ROS wiring** (validated on hardware) plus a **host-runnable
self-test** that is the gate. Do the self-test gate first (TDD-style), then the
`run_ros` wiring.

- [ ] **Step 1: Add imports + CLI flags**

Add near the other lib imports:
```python
from robo67_insertion.lib.force_regulator import AxialForceParams, AxialForceRegulator
from robo67_insertion.lib.insertion_event import InsertionEventParams, InsertionEventDetector
```
Add to `build_parser()`:
```python
    ap.add_argument("--force-mode", action="store_true",
                    help="regulate a constant gentle axial press (admittance) during "
                         "SEARCH_SPIRAL/PUSH_INSERT and detect insertion from the force-slacken")
    ap.add_argument("--search-press", type=float, default=5.0, help="F* press target (N) in force mode")
    ap.add_argument("--k-adm", type=float, default=0.0008, help="admittance gain (m/s per N)")
    ap.add_argument("--adm-v-cap", type=float, default=0.01, help="axial equilibrium speed cap (m/s)")
    ap.add_argument("--slacken-frac", type=float, default=0.4)
    ap.add_argument("--confirm-drop", type=float, default=0.003)
    ap.add_argument("--no-spiral-freeze", action="store_true")
```

- [ ] **Step 2: Extend the offline self-test to cover force mode**

In `selftest(args)`, when `args.force_mode`, build the adapter with `force_mode=True`
and a regulator/detector, and replace the fixed search/seat z with the regulated z.
Add explicit assertions (the new gate): force stays bounded, the EE actively
descends into the hole after alignment, and the detector's `inserted` fires.
Concretely, after constructing `adapter`, add:

```python
    reg = det = None
    if getattr(args, "force_mode", False):
        adapter = build_intent_adapter(socket, pos_stiff=200.0)  # rebuilt with force_mode
        adapter.force_mode = True
        reg = AxialForceRegulator(
            AxialForceParams(pos_stiff=pos_stiff, k_adm=args.k_adm,
                             v_cap_mps=args.adm_v_cap, max_press_depth_m=adapter.max_press_depth_m,
                             max_force_n=12.0),
            socket_top_z=float(socket[2]))
        det = InsertionEventDetector(InsertionEventParams(
            slacken_frac=args.slacken_frac, confirm_drop_m=args.confirm_drop))
        cmd[2] = reg.seed(float(ee[2]), 0.0)
    inserted_fired = False
```

and inside the loop, after `out = adapter.step(...)` and computing `goal`, when
`reg is not None and phase in ("SEARCH_SPIRAL", "PUSH_INSERT")` override the goal z
and run the detector:

```python
        if reg is not None and phase in ("SEARCH_SPIRAL", "PUSH_INSERT"):
            press = abs(fz - outcome.baseline_fz)
            zc = reg.step(float(cmd[2]), float(ee[2]), press, args.search_press, dt)
            goal = np.array([goal[0], goal[1], zc])
            ev = det.observe(press, float(ee[2]),
                             descending=(zc < cmd[2]), t=t)
            inserted_fired = inserted_fired or ev.inserted
```

Extend the final `ok` to require, in force mode, `inserted_fired` and that the
plant's force never exceeded the abort cap (already asserted via `max_speed`/`min_z`
invariants; add `and (not args.force_mode or inserted_fired)`).

- [ ] **Step 3: Run the self-tests (both modes) to verify**

Run:
```bash
PYTHONPATH=$(pwd)/robo67_insertion python3 -m robo67_insertion.nodes.hardware_insertion_node --selftest
PYTHONPATH=$(pwd)/robo67_insertion python3 -m robo67_insertion.nodes.hardware_insertion_node --selftest --force-mode
```
Expected: both print `RESULT : PASS`. The force-mode run must visit DESCEND→SPIRAL,
actively descend into the hole, and report the detector fired.

- [ ] **Step 4: Wire `run_ros` (hardware path)**

In `run_ros`, mirror the self-test composition: build the adapter with
`force_mode=args.force_mode`, construct `reg`/`det` from the CLI args + `--pos-stiff`.
In the main loop, when `args.force_mode and phase in ("SEARCH_SPIRAL","PUSH_INSERT")`:
compute `press = abs(fz - outcome.baseline_fz)`, set `goal[2] = reg.step(cmd[2], ee[2], press, f_target, dt)` (where `f_target` ramps from the contact force to `args.search_press` over `ramp_s`), run `ev = det.observe(press, ee[2], descending=(goal[2] < cmd[2]), t=t)`, and on `ev.inserted` take the existing `--release-on-insert` branch (open gripper + retract). When `args.no_spiral_freeze` is false and `ev.slacken`, hold the previous XY goal for `settle_s`. Keep the safety envelope, telemetry (publish `press_filt`/`slacken` via the diagnostics KeyValues), and the legacy z-drop trigger as a fallback when `--force-mode` is off. Seed the regulator at the DESCEND→SPIRAL handoff: `cmd[2] = reg.seed(ee[2], press_at_detection)`.

- [ ] **Step 5: Verify whole suite + dry-run sanity, then commit**

```bash
PYTHONPATH=$(pwd)/robo67_insertion python3 -m pytest robo67_insertion/test -q          # all green
PYTHONPATH=$(pwd)/robo67_insertion python3 -m robo67_insertion.nodes.hardware_insertion_node --selftest --force-mode
git add robo67_insertion/robo67_insertion/nodes/hardware_insertion_node.py
git commit -m "feat(robo67): force-mode loop (admittance press + slacken detect) + self-test gate"
```

> Real-arm bring-up validation (dry-run then guarded live with `--force-mode`, human on the e-stop) follows the runbook in Task 6 — it is NOT a code step and is done on the robot, not in CI.

---

## Task 6: Runbook + diagram note

**Files:**
- Modify: `docs/runbooks/automated-insertion.md`
- Modify: `docs/architecture/diagrams/insertion_z_force_timeline.puml` (add a force-mode note)

- [ ] **Step 1:** Add a "Force-guided mode (`--force-mode`)" section to the runbook: what it changes (regulated press vs fixed gap; slacken detection), the new flags + verified values, the bring-up order (dry-run → guarded live), and the recovery notes (still `--release-on-insert`; firmware reflex still the hard guardrail).
- [ ] **Step 2:** Add a legend line to the timeline `.puml` pointing to ADR-0002 for the force-regulated variant, then re-render: `~/.claude/skills/plantuml-diagrams/scripts/render.sh "docs/architecture/diagrams/insertion_z_force_timeline.puml" svg && plantuml -tpng docs/architecture/diagrams/insertion_z_force_timeline.puml`.
- [ ] **Step 3: Commit**

```bash
git add docs/runbooks/automated-insertion.md docs/architecture/diagrams/insertion_z_force_timeline.*
git commit -m "docs(robo67): runbook + diagram note for force-guided insertion (--force-mode)"
```

---

## Self-Review

- **Spec coverage:** admittance press → Tasks 1,5; force-slacken detection → Tasks 2,5; opt-in config → Task 3; XY-from-intent/z-regulated adapter → Task 4; contact-handoff seed (no bounce) → Tasks 1 (`seed`) + 5 (handoff wiring); spiral-freeze-on-slacken → Tasks 3 (cfg) + 5 (loop); safety unchanged → Global Constraints + Task 5 keeps the envelope; sim-first/host gate → Task 5 self-test + design §8. Covered.
- **Backward compatibility:** `force_search.enabled`/`--force-mode` default OFF; conformance test (Task 4) proves identical phase sequence.
- **Type consistency:** `press_n` is the magnitude `|fz − fz_baseline|` everywhere (Global Constraints, Tasks 1, 2, 5). `AxialForceRegulator.step` / `seed` and `InsertionEventDetector.observe` signatures match between the seam definitions (Tasks 1–2) and their call sites (Task 5).
- **No placeholders:** every code step ships complete code; ROS-only wiring (Task 5 Step 4) is described with exact call sites and is gated by the host self-test in Step 3.
