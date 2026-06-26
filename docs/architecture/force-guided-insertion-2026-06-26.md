# Force-guided search & insertion — design (2026-06-26)

Companion to [ADR-0002](../adr/0002-force-guided-search-and-insertion.md). This
document is the *why/what*; the [implementation plan](../superpowers/plans/2026-06-26-force-guided-insertion.md)
is the *how*. Diagram of the current z/force behavior:
[`diagrams/insertion_z_force_timeline.puml`](diagrams/insertion_z_force_timeline.svg).

Convention (unchanged): **z is UP** in the base frame, so "descend" = decreasing
z. `z_cmd` = the commanded *equilibrium* height fed to the soft Cartesian
impedance controller; `z_ee` = measured EE height.

---

## 1. Problem statement (observed on the real arm, 2026-06-26)

1. **Bounce at contact** — the EE rebounds slightly the moment contact is
   detected.
2. **No deeper motion at the correct XY** — once the peg is over the bore it
   does not sink/seat.
3. **Insertion not detected** — the z-drop trigger does not fire reliably.

## 2. Why the current scheme causes this

The real controller (`franka_controllers` Cartesian impedance, subscriber path
`/cartesian_impedance/pose_desired`) is a pure stiffness law:

```
Fz_applied  ≈  pos_stiff · (z_ee − z_cmd)        (downward press when z_cmd < z_ee)
```

The `ImpedanceCommandPathAdapter` produces the equilibrium z per phase
(`robo67_insertion/lib/command_path_adapters.py`):

| Phase | commanded `z_cmd` | resulting axial behavior |
|-------|-------------------|--------------------------|
| `DESCEND_TO_CONTACT` | `socket_top − max_press_depth` (deep, ≈ −5 cm) | arm sinks until `|Fz−base| ≥ contact_fz`, records `contact_z = z_ee` |
| `SEARCH_SPIRAL` | **fixed** `contact_z − press_gap` (`press_gap = press_force/pos_stiff`) | constant press `press_force` **while blocked**; force **decays to 0** as soon as the peg sinks toward the fixed equilibrium |
| `PUSH_INSERT` | **fixed** `contact_z − insert_gap` | same: fixed equilibrium, force decays during entry |

Two structural consequences:

- **The press is constant only while the surface blocks the peg.** During entry
  the *fixed* equilibrium means the arm relaxes (gap shrinks → force → 0) rather
  than continuing to push. So it does **not chase** the peg into the bore → "no
  deeper motion" + stiction stalls.
- **Detection is positional** (`insertion_intent.py`: `z_ee < contact_z −
  z_drop`; node direct trigger: `z_ee < contact_z − insert_drop_trigger`). It
  needs an accurate `contact_z` and real millimetres of sink — both corrupted by
  the contact bounce and stiction.
- **The handoff jumps the equilibrium** from `−5 cm` (deep descend) up to
  `contact_z − press_gap`, a force discontinuity that (with the soft controller's
  contact transient) shows as the rebound.

> Insight: "hold a constant light downward force and let it push in when the
> force slackens" is the right model — and the code is *almost* there. The fix
> is to **regulate** the press force against the *moving* EE (admittance) instead
> of commanding a *fixed* equilibrium, and to **detect on the slacken** instead
> of on a fragile absolute position.

## 3. Proposed control: axial force regulation (admittance)

Replace the fixed search/insert equilibrium with a per-tick **admittance**
update that holds a target press force `F*` against the measured force, anchored
on the previous command (the same ratcheting anchor the impedance safety profile
already uses).

```
# inputs each tick: z_cmd_prev, z_ee, fz_meas, fz_baseline, F*, dt
press   = |fz_meas − fz_baseline|          # press MAGNITUDE (N); search only presses down
err     = F* − press                       # N ; >0 = under-pressed, want to go deeper
v_cmd   = clamp(k_adm · err, −v_cap, +v_cap)   # m/s ; >0 = descend (decrease z)
z_cmd   = z_cmd_prev − v_cmd · dt          # ratchet the equilibrium
# bounds:
z_cmd   = max(z_cmd, socket_top − max_press_depth)        # z-floor (never deeper than the press-depth limit)
z_cmd   = min(z_cmd, z_ee + 0.0)                          # never command the equilibrium ABOVE the EE during press
# anti-windup: if z_cmd hit a bound, do not keep integrating v in that direction
```

Behavior this produces (the whole point):

- **On the surface, aligned wrong:** `press ≈ F*` → `err ≈ 0` → `z_cmd` holds →
  constant gentle force `F*`. (Same steady state as today, but *regulated*.)
- **Bore opens (correct XY):** the peg starts to give → `press` drops below `F*`
  → `err > 0` → `v_cmd > 0` → `z_cmd` ratchets **down**, the arm **actively
  follows the peg into the bore** and restores the press toward `F*`. This is
  the "force slackens → go down" behavior, as a stable loop.
- **Jam / bottom:** `press` rises above `F*` → `err < 0` → `z_cmd` eases up →
  force self-limits around `F*` (well under the firmware reflex if `F*` is
  gentle). This is *also* why it is safer than a deeper fixed gap.

`k_adm` is a soft admittance (m/s per N); `v_cap ≤ v_max`. The model term
`z_cmd ≈ z_ee − F*/pos_stiff` is the feed-forward equivalent (degenerate, high
`k_adm`); the integral form above tolerates `pos_stiff` mismatch and Fz bias.

### Contact handoff (kills the bounce)

At `DESCEND_TO_CONTACT → SEARCH_SPIRAL`, **seed** the regulator instead of
snapping the equilibrium:

```
F*_0   = press_at_detection            # = contact_fz at the trigger instant
z_cmd  = z_ee − press_at_detection / pos_stiff   # == current equilibrium → no jump
# then ramp F* from F*_0 to search_press_n over t_ramp seconds
```

No equilibrium discontinuity ⇒ no force step ⇒ no rebound. (If a *physical*
bounce remains, it is controller damping — raise `damping_ratio`/`contact_z`
firmness in `StiffnessCfg`; that is a separate, controller-level knob noted here
for completeness.)

## 4. Proposed detection: force-slacken + confirmed descent

New event detector (pure), fed `(fz_meas, fz_baseline, z_ee, contact_z, t)`:

1. **Filter** the press estimate (EMA, `fz_filter_alpha`, or short median) — the
   external wrench is noisy.
2. **Slacken event:** filtered press drops by ≥ `slacken_frac · F*` below the
   recently-held press (track a slow max/hold of the press as the reference).
3. **Inserted event (confirm):** after a slacken, require `z_ee` to descend by
   ≥ `confirm_drop_m` (default 3 mm) within `confirm_window_s` *while the
   regulator is still commanding downward* (err > 0). Slacken + confirmed
   descent together reject momentary noise dips.
4. The legacy positional drop (`z_ee < contact_z − z_drop`) remains as a
   **fallback** OR — when `force_mode` is on — `inserted` is the primary signal.

On `inserted` → existing `--release-on-insert` flow (open gripper, leave peg,
retract) — unchanged.

### Optional: freeze the spiral on slacken

When a slacken fires, hold the XY spiral target for `settle_s` so the axial
admittance can pull the peg in before the spiral wanders off the bore. Cheap,
addresses "the spiral moved on before it could sink". Gated by
`spiral_freeze_on_slacken`.

## 5. Architecture & seams (where each piece lives)

Honor ADR-0001: phase semantics stay in `insertion_intent.py`; controller-
specific force/equilibrium math stays in the adapter/node.

```
lib/insertion_intent.py        UNCHANGED  — canonical phases + spiral XY (position-only)
lib/force_regulator.py         NEW (pure) — AxialForceRegulator.step(...) -> z_cmd_next
lib/insertion_event.py         NEW (pure) — InsertionEventDetector.observe(...) -> events
lib/command_path_adapters.py   EDIT       — ImpedanceCommandPathAdapter(force_mode=True):
                                            XY from intent, z from the regulator
nodes/hardware_insertion_node.py EDIT      — own the regulator+detector, wire telemetry,
                                            CLI flags; keep safety envelope + release-on-insert
config_schema.py               EDIT       — ForceSearchCfg (new) + InsertionCfg
config/robo67.yaml             EDIT       — defaults for the above
```

### New pure interfaces (frozen for the plan)

```python
# lib/force_regulator.py
@dataclass(frozen=True)
class AxialForceParams:
    pos_stiff: float                 # N/m, MUST match controller
    k_adm: float = 0.0008            # m/s per N (admittance gain)
    v_cap_mps: float = 0.01          # axial equilibrium speed cap (<= v_max)
    max_press_depth_m: float = 0.05  # z-floor = socket_top - this
    max_force_n: float = 12.0        # clamp F* and the implied gap

class AxialForceRegulator:
    def __init__(self, params: AxialForceParams, socket_top_z: float): ...
    def seed(self, z_cmd: float, press_n: float) -> None: ...
    def step(self, z_cmd_prev: float, z_ee: float, press_n: float,
             f_target_n: float, dt: float) -> float: ...   # returns z_cmd_next
```

```python
# lib/insertion_event.py
@dataclass(frozen=True)
class InsertionEventParams:
    fz_filter_alpha: float = 0.2
    slacken_frac: float = 0.4
    confirm_drop_m: float = 0.003
    confirm_window_s: float = 1.0
    min_press_n: float = 2.0         # ignore slacken until a real press was held

@dataclass(frozen=True)
class InsertionEvent:
    press_filt_n: float
    slacken: bool
    inserted: bool

class InsertionEventDetector:
    def __init__(self, params: InsertionEventParams): ...
    def observe(self, press_n: float, z_ee: float,
                descending: bool, t: float) -> InsertionEvent: ...
    # NB: detection is force-based, so contact_z is not needed here.
```

`press_n` everywhere is the press **magnitude** `|fz_meas − fz_baseline|` (the
search only ever presses down, so magnitude is sign-robust against the unknown
sign of `o_f_ext_hat_k[2]`), keeping the convention in one place.

## 6. Configuration (new `ForceSearchCfg`)

```yaml
force_search:
  enabled: false            # force_mode; default OFF = current verified behavior
  search_press_n: 5.0       # F* during SEARCH_SPIRAL
  insert_press_n: 6.0       # F* during PUSH_INSERT (if not releasing)
  k_adm: 0.0008             # m/s per N
  v_cap_mps: 0.01
  ramp_s: 0.5               # contact-force -> search_press ramp
  fz_filter_alpha: 0.2
  slacken_frac: 0.4
  confirm_drop_m: 0.003
  confirm_window_s: 1.0
  spiral_freeze_on_slacken: true
  settle_s: 0.4
```

CLI mirrors (on `hardware_insertion_node`): `--force-mode`, `--search-press`,
`--k-adm`, `--adm-v-cap`, `--slacken-frac`, `--confirm-drop`,
`--no-spiral-freeze`. Keep `--pos-stiff` authoritative (regulator uses it).

## 7. Safety analysis

- **Firmware reflex:** the regulator self-limits force at `F*`; with `F*` gentle
  (≈ contact_fz … few N) and `max_force_n` clamp, a jam stays well under the
  reflex. `--release-on-insert` still fires on `inserted` so we never enter the
  sustained seating push that historically crashed the bringup.
- **Envelope unchanged:** every `z_cmd` still passes
  `SafetyEnvelopeModule(ImpedanceSafetyProfile)`: AABB clamp with z-floor
  `socket_top − max_press_depth`, per-tick step cap `v_max/rate` (anchor =
  previous command), force/moment abort. The regulator's `v_cap_mps ≤ v_max` and
  its own z-floor are redundant belts to the envelope's.
- **Sensor noise:** detector filters + hysteresis; `inserted` requires confirmed
  descent, not a single sample.
- **Backward compatible:** `enabled: false` ⇒ byte-for-byte current behavior;
  the adapter conformance test asserts the canonical phase sequence is unchanged.

## 8. Validation path (mandatory order, per CLAUDE.md)

1. **Host pytest** for the two pure seams (TDD) + adapter force-mode +
   conformance — `python3 -m pytest robo67_insertion/test -q`
   (`PYTHONPATH=<repo>/robo67_insertion`).
2. **Offline plant self-test** (`hardware_insertion_node --selftest`, extended):
   the existing virtual spring+table+hole plant must show constant force while
   blocked, active descent + restored force when the hole opens, `inserted`
   firing, no equilibrium discontinuity at contact, and force ≤ abort caps.
3. **MuJoCo sim** run.
4. **Real arm**: `--dry-run` (log z_cmd/press), then a guarded live run with a
   human on the e-stop, `--force-mode`.

## 9. Rejected alternatives

See ADR-0002 §"Alternatives considered" (deeper fixed gap; detection-only;
direct re-reference without integral; force logic inside `insertion_intent`).
