# Automated peg-in-hole insertion — runbook

End-to-end, one-command (or one-button) peg-in-hole insertion on the **real
Franka**, verified live 2026-06-25. This documents how to start it, how to
monitor it, the exact parameter set that works, and every problem we hit getting
there (with the fix), plus the recovery runbook.

```
detect cube (overhead C920, via camera topic)
   → move above (gentle, tool-down, software overshoot)
   → descend to contact (force-probe the hole-top Z)
   → spiral search under firm press
   → peg dips below the hole-top  → OPEN GRIPPER (leave peg in hole) → retract
```

The runner is `robo67_insertion/scripts/hw_peg_in_hole_vision.py` (vision +
CLI), which hands off to `robo67_insertion/nodes/hardware_insertion_node.py`
(the insertion FSM + safety). The gripper is the real `franka_gripper`.

---

## 1. Quick start

### From the dashboard (preferred)
1. Bring up the logging/camera graph + dashboard (so the overhead feed is
   published and the UI is up) — see §3.
2. Open the dashboard (`http://127.0.0.1:8088`). Top-right: **Start insertion**
   → **Confirm** (it commands the real arm). **Stop** cancels at any time.
   - The button is **live-mode only**; in mock mode it shows `insertion · live-only`.
   - Status chip shows `inserting · Ns` and the latest log line while running.
   - If a started run **fails** (force/torque abort, setup error, bringup crash —
     anything that ends without seating the peg and isn't a user Stop), a recovery
     dialog pops up with one **Relaunch & restart insertion** action: it relaunches
     the arm and, once it verifies Move, restarts the insertion automatically. See §7.4.
3. If the bringup ever crashes / goes Idle / trips a reflex (see §4/§5), hit the
   **Relaunch arm** button (header, left of Start insertion) → **Confirm**. It
   stops + relaunches `franka.launch.py` and the gripper node, clears a reflex,
   and verifies Move (2) + `/panda_gripper/move` — the §5 clean-restart, one click.
   See §7.1.
4. **Home** button (header, next to Relaunch arm) → **Confirm**: **moves** the arm
   to the defined HOME pose (a fixed taught start position, tool-down vertical).
   Use it to restore the start position after working / jogging the arm. See §7.2.
5. The **Logs** tab shows the live stdout of all three managed runs (insertion,
   arm relaunch, home) — the same ring-buffered output the status chips summarise.

### From the CLI (inside `multipanda-container`, ROS sourced, domain 1)
```bash
PYTHONPATH=/host/Code/Robo67/robo67_insertion \
python3 robo67_insertion/scripts/hw_peg_in_hole_vision.py \
  --socket-top-z 0.1465 \
  --pos-stiff 2000 --approach-tol 0.015 \
  --v-max 0.02 --standoff 0.05 \
  --contact-fz 5 --press-force 18 \
  --max-press-depth 0.05 --spiral-max-radius 0.02 \
  --f-abort 30 --torque-abort 12 \
  --release-on-insert --insert-drop-trigger 0.003 \
  --gripper-open-width 0.08 --retract-after 0.06
```
Always `--dry-run` first (perceives the socket, prints the plan, publishes
nothing). Add `--source device` only if no `camera_publisher` owns the C920.

These are exactly the dashboard's `DEFAULT_ARGS`
(`dashboard/server/insertion_control.py`). Keep the two in sync.

---

## 2. Prerequisites & one-time setup

Run everything **inside `multipanda-container`** on `ROS_DOMAIN_ID=1`,
`ROS_LOCALHOST_ONLY=1`, with `LD_LIBRARY_PATH` including libfranka. See the main
`CLAUDE.md` runbook for the exact source/export lines.

1. **Desk / FCI**: FCI must actually have control. Blue LED + a *usable* Desk UI
   means **Desk holds control and FCI is locked out** → release Desk control,
   then (re)launch so the FCI client acquires it (physical SPoC button tap if
   prompted). See §4 problem #1.
2. **Arm bringup** (auto-activates `cartesian_impedance_controller`):
   ```bash
   ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67 \
       use_fake_hardware:=false arm_id:=panda
   ```
   Health: `robot_mode: 2` (Move) and `control_command_success_rate: 1.0` on
   `/franka_robot_state_broadcaster/robot_state`. `1`/`0.0` = Idle, not engaged
   → relaunch (§5).
3. **Controller stiffness** = **`pos_stiff: 2000`, `rot_stiff: 50`** in
   `franka_bringup/.../config/real/single_controllers.yaml` (we bumped from
   500/30). The controller reads it **at activation only** — a runtime
   `ros2 param set` does NOT change the live control law; you must edit the
   config and relaunch. (`--pos-stiff 2000` MUST match, or the press math is
   10× off.)
4. **Gripper node** (separate launch — do NOT use `load_gripper:=true`, which
   would re-load the hand and shift the EE frame, invalidating the homography):
   ```bash
   ros2 launch franka_gripper gripper.launch.py robot_ip:=192.168.1.67 \
       arm_id:=panda use_fake_hardware:=false
   ```
   Confirm `/panda_gripper/move` exists and `/panda_gripper/joint_states` reads
   ~`0.014`/finger (peg clamped) at ~15 Hz. A frozen ~1 Hz reading at ~`0.037`
   = dead gripper connection → relaunch the gripper node (§5).
5. **Logging/camera graph + dashboard** (so detection works off the topic and
   the UI/Start button are available):
   ```bash
   ros2 launch robo67_insertion logging.launch.py socket_top_z:=0.1465
   ./run_live.sh            # dashboard in live mode (subscribes only)
   ```
6. **Physical**: peg clamped in the gripper; **hole empty**; the white socket
   cube **in the overhead C920's view** (centre-ish for best homography
   accuracy); arm clear of the camera at start; a human at the e-stop.
7. **Calibration**: `config/c920_homography.npz` present (overhead C920 → base
   XY). Missing → run the calibration first; the script refuses to move.

---

## 3. What the parameters mean (and why these values)

| Flag | Value | Why |
|------|-------|-----|
| `--pos-stiff` | 2000 | MUST equal the controller's `pos_stiff`. Sets the gap→force math (`gap=F/pos_stiff`) **and** the free-space deadband (~3.7 cm @500 → ~1.9 cm @2000). |
| `--approach-tol` | 0.015 | MOVE_ABOVE "arrived" tolerance; must be ≥ the ~8 mm stiction deadband or the FSM stalls before descending. |
| `--press-force` | 18 | Search press. Must beat the Z stiction deadband (~16 N) so the peg sits **on** the surface, not hovering ~1 mm above it. |
| `--spiral-max-radius` | 0.02 | Cover detection (~5 mm) + deadband offset. |
| `--torque-abort` | 12 | Moment abort (was hardcoded 5 in `ImpedanceSafetyProfile`). A constant peg-weight torque offset (~2.8 Nm) + lateral scrubbing ate the 5 Nm cap; 12 gives headroom. |
| `--release-on-insert` | on | Open the gripper the instant the peg drops in — **avoids the sustained seating push that crashes the bringup**. |
| `--insert-drop-trigger` | 0.003 | Release when EE z dips this far below the DESCEND `contact_z` hole-top. Direct + robust vs the FSM's rate-based z-drop. |
| `--v-max` | 0.02 | Gentle command velocity. |
| `--source` | topic (default) | Subscribe to `camera_publisher` (one owner, BEST_EFFORT QoS). No V4L2 contention. |

---

## 4. Problems encountered & fixes (chronological)

1. **Robot Idle / `success_rate=0`, no motion despite commands.** Desk held
   SPoC control (blue LED + usable Desk) → FCI locked out. **Fix:** release Desk
   control + clean relaunch so the FCI client acquires control.
2. **Move "twitch" — arm jumped to target.** `hw_move_to` stalled (EE-anchored
   1 cm carrot too weak for the soft controller) then the hold-after published
   the raw target in one tick. **Fix:** time-parameterised straight-line ramp
   (constant slow command velocity), hold the *reached* pose (never the raw target).
3. **~3.7 cm free-space positioning deadband + ~12° tool tilt.** Pure impedance,
   no integral / no friction comp; joint stiction. **Fixes:** raised stiffness
   (config + relaunch; runtime param is ignored), **software overshoot/integral**
   in `hw_move_to` (push the equilibrium past target until the EE arrives), and
   `--tool-down` (command vertical, yaw-preserving). → ~6 mm + vertical.
4. **Insertion stalled in MOVE_ABOVE.** 6 mm hardcoded approach tol < 8 mm
   deadband. **Fix:** exposed `--approach-tol`.
5. **Spiral aborted on torque (~5 Nm).** Lateral scrubbing under press + a
   constant peg-weight torque offset. The cap was hardcoded `_MOMENT_CAPS=5` in
   `safety_envelope.py` (the node's `caps` var is nudge-only). **Fix:** made
   `ImpedanceSafetyProfile.moment_cap_n` configurable, wired `--torque-abort`.
6. **Bringup crashed during the sustained seating push** (`libfranka
   NetworkException: UDP receive: Timeout`) — the firmware force reflex (the
   un-catchable guardrail) tripping. **Fix:** `--release-on-insert` — open the
   gripper on the drop and retract; never do the sustained push.
7. **Gripper reads open (~7.5 cm) though the peg is clamped.** The gripper
   node's TCP link to the hardware reset (`Connection reset by peer`), so its
   joint state froze at ~1 Hz. **Fix:** relaunch the gripper node (the hardware
   keeps its grip; relaunch reads the true width again).
8. **Vision detection failed: "no frames from C920".** The `camera_publisher`
   owns the V4L2 device; the script tried to open it directly. **Fix:** the
   script now **subscribes** to `/robo67/camera/overhead/image_raw/compressed`
   (`--source topic`, `qos_profile_sensor_data` to match BEST_EFFORT). A RELIABLE
   subscriber gets nothing (QoS incompatible) — must be BEST_EFFORT.
9. **Cube not detected — empty carpet in view.** The socket cube wasn't in the
   overhead FOV. **Fix:** place it in view; exposure is auto-locked to 100 for
   the overhead cam (config `c920_exposure`, `-1`→100).
10. **Peg hovered ~1 mm above the hole during the spiral.** Search press too
    light (8 N) vs the Z deadband → peg lifted off the surface. **Fix:** bump
    `--press-force` to 18.
11. **Peg clearly inserted but the gripper didn't open.** The FSM's rate-based
    z-drop threshold was too strict for a gradual entry. **Fix:** `--insert-drop-trigger`
    — recompute the hole-top from DESCEND `contact_z` and release as soon as the
    EE dips below it.

> **Recurring**: hand-guiding the arm and any control/SPoC change tend to crash
> the bringup (NetworkException) or leave it Idle/Reflex — a **relaunch** is the
> standard recovery. Keep moves gentle; the firmware joint-limit/force reflexes
> are the real, undisable-able guardrail.

---

## 5. Recovery runbook

| Symptom | Cause | Action |
|---|---|---|
| `robot_mode=1` (Idle), `success_rate=0`, no motion | control not engaged (after stop/reflex/Desk) | **relaunch** `franka.launch.py` (error_recovery alone is not enough) |
| `robot_mode=4` (Reflex) | firmware reflex tripped | relaunch (or `hw_recover.py` then relaunch if still Idle) |
| `robot_mode=3` (Guiding) | guiding buttons engaged / hand-guided | release guiding → it usually crashes the bringup → relaunch |
| `robot_mode=5` (UserStopped) | e-stop pressed | release the e-stop → relaunch |
| bringup process gone / `NetworkException` in launch log | firmware reflex / SPoC change crashed `franka_control2` | kill stragglers + relaunch |
| gripper open ~7.5 cm but peg is clamped, joint_states ~1 Hz frozen | gripper TCP link reset | relaunch the **gripper node** only |
| detection "no frames from C920" | `camera_publisher` owns the device | use `--source topic` (default); ensure the camera feed is publishing |
| detection "no socket detected" | cube out of FOV / wrong exposure | place cube in overhead view; exposure auto-locks to 100 |
| `Stop` from the dashboard | — | sends SIGINT → the node holds its last pose; SIGKILL if it doesn't exit |

Clean relaunch (the dashboard **Relaunch arm** button does exactly this, plus
the gripper relaunch + error_recovery + verification — see §7.1):
```bash
pkill -f "franka.launch.py|franka_control2_node|ros2_control_node|controller_manager" || true
ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67 \
    use_fake_hardware:=false arm_id:=panda
```

---

## 6. Monitoring

- **Dashboard** (`run_live.sh`, live mode): phase, EE/cmd pose, speed, wrench,
  Fz vs baseline, contact, retries — all from `/robo67/insertion/*`. The Start/Stop
  control + run status (elapsed + latest log line) live in the header.
- **Telemetry topics** (`hardware_insertion_node`, ~20 Hz): `/robo67/insertion/{phase,
  ee_pose,ee_speed,command_pose,wrench,fz,fz_baseline,contact,retries,diagnostics}`.
- **Console / process log**: the CLI prints phase transitions and the release
  (`INSERTION DETECTED (ee_z=… < hole-top … - …) -> releasing peg` → `gripper
  opened` → `retracted`). The dashboard captures the same in its status `log`.
- **Healthy run** ends with `release-on-insert complete: peg left in hole, arm
  retracted.` and the arm back in `Move`, gripper open above the socket.

## 7. Dashboard control internals

- `dashboard/server/insertion_control.py` — `InsertionController` spawns the
  runner as a subprocess (own process group), ring-buffers stdout, one run at a
  time. `start()` is **live-mode only**. `stop()` sends **SIGINT** to the group
  (the node's interrupt handler holds the last pose), escalating to SIGKILL.
- `dashboard/server/serve.py` — `POST /api/insertion/start`, `POST
  /api/insertion/stop`, `GET /api/insertion/status`.
- `dashboard/web/src/components/InsertionControl.tsx` — the header Start/Stop
  buttons + status, polling `/api/insertion/status` at 1 Hz.

### 7.1 Relaunch-arm control internals

The **Relaunch arm** button (header, left of Start insertion) is the §5 clean
restart as a single confirm-gated action. **Live-mode only** (the dashboard must
be running inside the container with ROS sourced, so it can spawn `ros2 launch`).

- `dashboard/server/bringup_control.py` — `BringupController.relaunch()` runs a
  background sequence (one at a time): **kill** any `franka.launch.py` /
  `franka_control2_node` / `ros2_control_node` / `controller_manager` /
  `gripper.launch.py` / `franka_gripper` (broad `pkill`, SIGTERM→SIGKILL; the
  pattern is fed over **stdin** so it can't match its own shell in the shared
  host PID namespace) → **launch** `franka_bringup franka.launch.py
  robot_ip:=192.168.1.67 use_fake_hardware:=false arm_id:=panda` (tracked, own
  session) → **wait** for `FrankaState` to come back live → if `robot_mode != 2`
  call `/panda_error_recovery_service_server/error_recovery` → **launch**
  `franka_gripper gripper.launch.py robot_ip:=192.168.1.67 arm_id:=panda
  use_fake_hardware:=false` (separate, NOT `load_gripper:=true`) → **verify**
  `robot_mode == 2` (Move) and `/panda_gripper/move` action present.
  Scope = **bringup + gripper only** (it never touches the logging/camera graph
  or the dashboard). The launches use `start_new_session`, so they survive a
  dashboard restart like the manual terminal launch. `robot_ip`/`arm_id`/gripper
  namespace are env-overridable (`ROBO67_ROBOT_IP`, `ROBO67_ARM_ID`,
  `ROBO67_GRIPPER_NS`). `robot_mode` for the verify step is read off the live
  provider's `FrankaState` subscription.
- `dashboard/server/serve.py` — `POST /api/bringup/relaunch`, `GET
  /api/bringup/status` (`{busy, phase, phase_label, bringup_running,
  gripper_running, robot_mode, mode_ok, gripper_ok, ok, error, elapsed_s, log}`).
- `dashboard/web/src/components/BringupControl.tsx` — confirm-gated button +
  progress chip (phase + elapsed + last log line) + an outcome chip
  (`arm ready` / mode + gripper warnings), polling `/api/bringup/status` at 1 Hz.
- **Caveat**: relaunching kills a running bringup and therefore any in-flight
  insertion talking to it — hence the confirm step. Stop an insertion first.

### 7.2 Home-control internals

The **Home** button **moves** the arm to a defined HOME pose — a FIXED taught
start position (hand-guide the arm to a good default, read its EE, hard-code it),
NOT the current pose. Use it to restore the start position after working/jogging.
Confirm-gated (real motion), **live-mode only**.

- `dashboard/server/home_control.py` — `HomeController.run()` spawns
  `scripts/hw_move_to.py --xyz <HOME> --tool-down --speed 0.02 --cmd-mode auto`
  (own process group, ring-buffered stdout, one at a time). `hw_move_to` does a
  gentle time-parameterised ramp + overshoot settle to the target XYZ with a
  vertical (z-down, yaw-preserving) orientation, with the workspace AABB clamp +
  force/reflex aborts. The HOME pose is hard-coded (`_DEFAULT_HOME_XYZ`, captured
  live from the operator's taught default `≈ (0.2145, -0.0278, 0.4451)` m base
  frame) and **env-overridable** with `ROBO67_HOME_XYZ="x y z"`. `status()`
  reports `home_xyz` so the UI can show the target.
- `dashboard/server/serve.py` — `POST /api/home/run`, `POST /api/home/stop`
  (SIGINT), `GET /api/home/status`.
- `dashboard/web/src/components/HomeControl.tsx` — confirm-gated **Home** button
  (the confirm shows the target XYZ) + `homing · Ns` chip, polling
  `/api/home/status` at 1 Hz.

### 7.3 Logs page

`dashboard/web/src/routes/Logs.tsx` (`/logs`) renders three `LogPanel`s
(`components/LogPanel.tsx`) — **Insertion**, **Arm relaunch**, **Home** — each
showing the ring-buffered stdout of the corresponding managed subprocess (from
`/api/{insertion,bringup,home}/status`), polled at 1 Hz, newest line stuck to the
bottom, with a running/elapsed/outcome chip. Populates in **live mode** only.

### 7.4 Insertion-failure recovery dialog

`dashboard/web/src/components/InsertionFailureModal.tsx` (mounted once in the
AppShell) watches `/api/insertion/status`. When a run it observed **start** then
**ends without succeeding** and wasn't a user Stop, it pops a modal with one
combined recovery action: **Relaunch & restart insertion**.

- **Why log-classified, not exit-code-classified**: `run_ros` returns **0 even on
  a force/torque abort** (it just holds + tries error recovery), so `last_exit`
  alone misses the most common failure. The modal instead checks the captured
  stdout: **success** = `release-on-insert complete` / `sequence finished`;
  **user Stop** = `STOP requested`; anything else after a run we saw running =
  **failure**. The shown reason prefers the cause line (`FORCE ABORT` / `[ERROR]`
  / `refusing…`) over the generic `exited (rc=N)` footer.
- **One action, two steps**: clicking **Relaunch & restart insertion** does
  `POST /api/bringup/relaunch`, then polls `/api/bringup/status` until the
  sequence finishes (showing the live phase). **Only if it verifies OK**
  (Move + `/panda_gripper/move`) does it then `POST /api/insertion/start` and
  close the dialog (the new run shows in the header chip). If the relaunch
  fails/times out it stops there, surfaces the error, and offers **Retry** (it
  does NOT blindly restart insertion onto a still-broken arm). **Dismiss** / ✕
  closes it. Live mode only (no run ⇒ no popup).

---

## 8. Force-guided mode (`--force-mode`, experimental — ADR-0002)

**What it changes.** Instead of a *fixed* below-surface equilibrium during
SEARCH_SPIRAL/PUSH_INSERT (constant force only while the peg is blocked, then
the force *decays* as it sinks), `--force-mode` **regulates** a constant gentle
axial press with an admittance loop (`lib/force_regulator.py`):

- under-pressed → the equilibrium ratchets **down**, actively pushing the peg in
  as resistance slackens (fixes "doesn't go deeper at the right spot");
- **over-pressed → the equilibrium ratchets back up so the force is reduced
  again** (self-limits around the target, well under the firmware reflex);
- insertion is detected from the **force-slacken + a confirmed descent**
  (`lib/insertion_event.py`), not the fragile absolute z-drop, then
  `--release-on-insert` fires as usual;
- the contact handoff is **seeded** (no equilibrium jump) to kill the contact bounce.

Off by default ⇒ the verified fixed-equilibrium behavior in §1 is unchanged.

**Design + rationale:** [`docs/adr/0002-force-guided-search-and-insertion.md`](../adr/0002-force-guided-search-and-insertion.md),
[`docs/architecture/force-guided-insertion-2026-06-26.md`](../architecture/force-guided-insertion-2026-06-26.md).

**Flags (in addition to §1):**

| Flag | Default | Why |
|------|---------|-----|
| `--force-mode` | off | enable the regulated press + force-slacken detection |
| `--search-press` | 5 | F* press target (N) during the spiral. Keep gentle. With `pos_stiff 2000` this is what the peg feels, NOT the fixed-gap `--press-force`. |
| `--insert-press` | 6 | F* during PUSH (usually not reached — we release on slacken) |
| `--k-adm` | 0.0008 | admittance gain (m/s per N). Higher = chases/backs-off faster. |
| `--adm-v-cap` | 0.01 | axial equilibrium speed cap (m/s); keep ≤ `--v-max` |
| `--adm-max-force` | 12 | soft clamp on the regulated force target |
| `--slacken-frac` | 0.4 | fraction of the held press lost that counts as a slacken |
| `--confirm-drop` | 0.003 | EE descent (m) after a slacken needed to confirm insertion |
| `--no-spiral-freeze` / `--settle-s` | freeze on, 0.4 s | hold the XY spiral briefly after a slacken so the axial pull-in can seat the peg |

**Bring-up order (mandatory).**

1. Host gate (no robot): `python3 -m robo67_insertion.nodes.hardware_insertion_node --selftest --force-mode` → must print `RESULT: PASS`, `insertion det: True`, and a bounded `max press`.
2. Real arm `--dry-run` first (reads state, logs the regulated `cmd_z`/press, publishes NOTHING): add `--force-mode --dry-run` to the §1 command.
3. Guarded live run, human on the e-stop: add `--force-mode` to the §1 command (keep `--release-on-insert`).

**Recovery notes are unchanged (§5).** `--release-on-insert` still fires on the
insertion event, so the arm never enters the sustained seating push; the
firmware force/torque reflex remains the hard, undisable-able guardrail. If a
jam still trips it, recover via `/panda_error_recovery_service_server/error_recovery`
and relaunch as in §5.
