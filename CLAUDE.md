# Robo67 — CLAUDE.md

EE26 Hackathon. Franka Emika Panda arm. **Challenge 1 (peg-in-hole insertion) only.**
**We will NOT use 5. Intel acceleration — bonus points** its buggy and we rather prefer other methods.
Full challenge doc, hardware credentials, and stack reference: [`docs/CHALLENGE.md`](docs/CHALLENGE.md)

**Branch:** `jearningers` — all commits go here, never `main`. If you use a worktree or branch branch from `jearningers` and back onto `jearningers`

## Rules

- Classical vision + force first. Only pivot to VLA/imitation if it's working.
- Prototype in MuJoCo sim before touching real hardware.
- Cartesian impedance controller is the starting point — gives compliant contact.
- Never use the joint-position controller — known bad motor behavior.
- Eigen 3.3.9 only — 3.4.0 breaks compilation.
- After a `ControlException`, call `~/service_server/error_recovery` — no need to reload the controller.

## Insertion stack architecture (Phase 8 seams — 2026-06-25)

The `robo67_insertion` insertion logic is organized as deep, host-testable
(`pytest`, no rclpy) seams under `robo67_insertion/lib/`, each composed by thin
node wrappers. See [`docs/architecture/deepening-roadmap-2026-06-25.md`](docs/architecture/deepening-roadmap-2026-06-25.md)
and [`docs/adr/0001-canonical-insertion-intent-module.md`](docs/adr/0001-canonical-insertion-intent-module.md).
Visual overview (PlantUML, `.puml` + rendered `.svg`/`.png`) in
[`docs/architecture/diagrams/`](docs/architecture/diagrams/): `peg_in_hole_workflow`
(the insertion process / phase flow) and `peg_in_hole_architecture` (nodes → seams →
primitives). Re-render with `plantuml -tsvg docs/architecture/diagrams/*.puml`.

- **`insertion_intent.py`** — the ONE canonical phase model (IDLE→…→DONE/ERROR),
  controller-agnostic. All transition logic lives here.
- **`command_path_adapters.py`** — `MMCCommandPathAdapter` (sim;
  `CartesianImpedanceGoal`, held quaternion, carrot lead) and
  `ImpedanceCommandPathAdapter` (real arm; `Float64MultiArray`, held row-major R,
  below-surface equilibrium gaps `force/pos_stiff`, px/R22 non-zero). Both delegate
  to the intent module — controller quirks live ONLY in adapters.
- **`contact_lifecycle.py`** — owns force-baseline update/freeze keyed on an
  explicit `ContactMode` (was orchestrator glue).
- **`safety_envelope.py`** — composes workspace+step clamps and force abort;
  `MMCSafetyProfile` (anchor = measured EE) vs `ImpedanceSafetyProfile` (anchor =
  previous command, socket-top z-floor folded into its AABB).
- **`pixel_mapping.py`** — one `PixelToBaseMappingModule`; `HomographyMappingAdapter`
  (C920 overhead) and `PinholeMappingAdapter` (D405 eye-in-hand, gain stays in the node).
- `insertion_fsm.py` is now a thin shim over `insertion_intent` (kept as the MMC
  parity harness). Primitives `wrench/safety/geometry/servoing` remain as building
  blocks the seams compose. Run `python3 -m pytest robo67_insertion/test -q` (144 green).

## Logging & observability (rostopics — 2026-06-25)

Everything the insertion stack does is published to rostopics (single source of
truth for the dashboard + `ros2 bag`). Full reference:
[`docs/architecture/logging-topics.md`](docs/architecture/logging-topics.md).

- **Cameras** (`sensor_msgs/CompressedImage`, jpeg): a dedicated
  `camera_publisher` node OWNS each `/dev/videoN` (only one process may open a
  V4L2 device) and streams the raw feed; the detector nodes SUBSCRIBE (no device
  contention) and republish an **overlay** feed (detection burned in):
  `/robo67/camera/{overhead,gripper}/{image_raw,overlay}/compressed`.
- **Insertion telemetry** (`hardware_insertion_node`, default on, throttled to
  `--telemetry-rate` 20 Hz, also in `--dry-run`): `/robo67/insertion/{phase,
  ee_pose,ee_speed,command_pose,wrench,fz,fz_baseline,contact,retries,diagnostics}`.
  Observational only — never commands the arm.
- New host-tested seams: `lib/image_overlay.py` (overlay drawing) and
  `lib/telemetry.py` (speed tracker + diagnostic rollup). Topic names live in
  `config_schema.TopicsCfg` / `config/robo67.yaml`.
- Bring up the logging graph: `ros2 launch robo67_insertion logging.launch.py
  socket_top_z:=<z> [gripper:=true]`. New entry points: `camera_publisher`,
  `hardware_insertion`.
- The **dashboard** (`dashboard/`) live mode now SUBSCRIBES to all of these
  (cameras + telemetry); it no longer opens any camera device. The C920/D405
  panels have a Raw/Processed toggle (processed = the ROS overlay feed).

## Hardware runs (real arm — verified 2026-06-25)

- Real bringup runs **inside `multipanda-container`** on **`ROS_DOMAIN_ID=1`** (`ROS_LOCALHOST_ONLY=0`), `robot_ip:=192.168.1.67`. A leftover sim sits on domain 7 — keep real work on domain 1.
- Launch: `ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67 use_fake_hardware:=false arm_id:=panda` (add `/home/developer/Libraries/libfranka/lib` to `LD_LIBRARY_PATH`). It **auto-activates** `cartesian_impedance_controller` (`franka_controllers/CartesianImpedanceController`), which subscribes to `/cartesian_impedance/pose_desired` (`std_msgs/Float64MultiArray` = `[px,py,pz, R00..R22]` row-major; px and R22 must stay non-zero). State: `/franka_robot_state_broadcaster/robot_state` (~29 Hz); `robot_mode` 1=Idle 2=Move 4=Reflex 5=UserStopped.
- This subscriber controller is the **real-hardware** Cartesian-impedance path; the "MMC controller only" guidance is about **sim parity**, not hardware.
- **FCI is activated only via Desk** (`https://192.168.1.67/desk/`, franka/frankaRSI). Taking control from another tab needs a physical button tap at the robot (Single Point of Control); FCI active ⇒ Desk UI is locked out.
- A force-threshold reflex (`robot_mode` 4, "Configured force thresholds reached") crashes the whole bringup — recover via `/panda_error_recovery_service_server/error_recovery`, then relaunch.
- `fun/robot_dance.py` is a safe demo: it streams compliant eased offsets around the _current_ EE pose with hard speed/box/force clamps and an offline `--selftest`. It's an ~80 s routine ending in a 20 s "everything spins at once" finale that lands back at the anchor. It also has a **joint-limit guard** (reads measured `q` from `FrankaState`): it _refuses to start_ if any joint is within `--joint-preflight-margin` (default 0.15 rad) of its limit, and at runtime _backs the equilibrium off to the anchor_ whenever any joint comes within `--joint-guard-margin` (default 0.10 rad) of a limit. Lesson learned the hard way: cranking amplitude/speed past the clamps walked joint 2 onto its mechanical stop (`joint_position_limits_violation`), which the impedance controller cannot recover from (its startup transient re-trips the limit) — it needed a physical hand-guide. Keep hardware runs gentle (e.g. `--amp-scale 0.35 --v-max 0.15 --w-max 0.8`); the firmware joint-limit/force reflexes are the real, undisable-able guardrail.
- **Domain-1 contamination (seen 2026-06-25):** a MuJoCo sim stack (`/mujoco_server`, `/mujoco_ros2_control`, `/panda_gripper_sim_node`) had leaked onto domain 1, so `FrankaState` was not getting through and `/controller_manager` services timed out even though `franka_control2_node` was alive and `/joint_states` flowed. Fix = the runbook clean restart with **`ROS_LOCALHOST_ONLY=1`** (isolates the graph; does NOT affect the libfranka TCP link to `192.168.1.67`). After that the real bringup connected cleanly and `FrankaState` was live. Prefer `ROS_LOCALHOST_ONLY=1` for real runs to avoid this. The `hardware_insertion_node` dry-run + `--nudge` + `scripts/hw_probe_contact.py` were all re-validated on the real arm via the subscriber path.

## Automated insertion (real arm — verified end-to-end 2026-06-25)

Full doc + problems-encountered + recovery table: [`docs/runbooks/automated-insertion.md`](docs/runbooks/automated-insertion.md).

- **One command**: `hw_peg_in_hole_vision.py` perceives the socket (overhead C920)
  then hands off to `hardware_insertion_node`: detect → MOVE_ABOVE → DESCEND_TO_CONTACT
  → SEARCH_SPIRAL → **release-on-insert** (open gripper, leave peg in hole) → retract.
- **Dashboard buttons**: live dashboard now has **Start insertion / Stop** in the
  header (`dashboard/server/insertion_control.py` spawns/SIGINTs the runner;
  `POST /api/insertion/{start,stop}`, `GET /api/insertion/status`). Start is
  live-mode only and confirm-gated. Stop = SIGINT → node holds last pose.
- **Relaunch arm button**: live dashboard header also has **Relaunch arm**
  (`dashboard/server/bringup_control.py`; `POST /api/bringup/relaunch`, `GET
  /api/bringup/status`; UI `dashboard/web/src/components/BringupControl.tsx`).
  Confirm-gated, live-mode only. It is the §5 clean restart in one click: kill
  any `franka.launch.py`/`franka_control2_node`/`controller_manager`/
  `gripper.launch.py`/`franka_gripper` → relaunch `franka.launch.py` → wait for
  `FrankaState` → `error_recovery` if `robot_mode != 2` → relaunch
  `franka_gripper gripper.launch.py` (separate, NOT `load_gripper:=true`) →
  verify mode 2 + `/panda_gripper/move`. Scope = bringup + gripper only (leaves
  the logging/camera graph + dashboard alone); launches use `start_new_session`
  so they survive a dashboard restart. Relaunching kills any in-flight insertion
  → stop the insertion first. `robot_ip`/`arm_id`/gripper-ns are env-overridable
  (`ROBO67_ROBOT_IP`/`ROBO67_ARM_ID`/`ROBO67_GRIPPER_NS`). Full doc:
  [`docs/runbooks/automated-insertion.md`](docs/runbooks/automated-insertion.md) §7.1.
- **Home button**: live dashboard header **Home** (`dashboard/server/home_control.py`;
  `POST /api/home/run`+`/api/home/stop`, `GET /api/home/status`; UI
  `dashboard/web/src/components/HomeControl.tsx`). Confirm-gated, live-mode only.
  "Home" = hold the pose the arm is in RIGHT NOW (not a fixed config): spawns
  `scripts/hw_cartesian_hold.py --secs 4.0 --cmd-mode auto`, which captures the
  current EE and streams it as the equilibrium so the arm settles where it is
  (≈ no net motion; the controller retains the last equilibrium after exit). Use
  it to settle the soft controller after relaunch/nudge/drift. Hold time =
  `ROBO67_HOME_HOLD_S`. Doc §7.2.
- **Logs page**: dashboard has a **Logs** tab (`/logs`,
  `dashboard/web/src/routes/Logs.tsx` + `components/LogPanel.tsx`) showing the
  ring-buffered stdout of all three managed runs (insertion / arm relaunch /
  home) from `/api/{insertion,bringup,home}/status`, polled 1 Hz, newest at the
  bottom. Live mode only. Doc §7.3.
- **Verified param set** (keep in sync with the dashboard `DEFAULT_ARGS`):
  `--pos-stiff 2000 --approach-tol 0.015 --press-force 18 --spiral-max-radius 0.02
  --torque-abort 12 --release-on-insert --insert-drop-trigger 0.003`.
- **Controller stiffness is now `pos_stiff 2000 / rot_stiff 50`** in
  `single_controllers.yaml` (was 500/30). It is read **at activation only** — a
  runtime `ros2 param set` does NOT change the live law; edit config + relaunch.
  `--pos-stiff` MUST match (gap→force is `F/pos_stiff`; mismatch = big over-press).
- **Vision feed = subscribe, don't open the device**: the script subscribes to
  `/robo67/camera/overhead/image_raw/compressed` (`--source topic`, BEST_EFFORT
  QoS) so it never fights `camera_publisher` for the V4L2 device.
- **Gripper**: launch `franka_gripper gripper.launch.py` SEPARATELY (NOT
  `load_gripper:=true`, which shifts the EE frame). If joint_states freeze ~1 Hz
  at ~0.037 (open) while a peg is clamped → the gripper TCP link reset; relaunch
  the gripper node (hardware keeps its grip).
- **Soft-controller realities**: pure impedance, no integral/friction comp →
  ~cm free-space deadband (overshoot/integral + `--tool-down` in `hw_move_to`
  fixes positioning); the **sustained seating push trips the firmware reflex and
  crashes the bringup** → that's why we release on the z-drop instead of pushing
  home. Hand-guiding / SPoC changes crash the bringup; relaunch is the standard recovery.

## Runbook (How A Run Works)

Use this exact sequence for a predictable real-arm run.

1. Preconditions on Desk

- FCI active on Desk (`https://192.168.1.67/desk/`).
- No active robot motion/program in Desk "Move" mode.
- If SPoC says control is held elsewhere, perform handoff (physical confirmation button).

2. Container shell + environment

- Run inside `multipanda-container`.
- Source:
  - `source /opt/ros/humble/setup.bash`
  - `source /home/developer/multipanda_ws/install/setup.bash`
- Export:
  - `export ROS_DOMAIN_ID=1`
  - `export ROS_LOCALHOST_ONLY=1` (isolates graph from external noisy publishers)
  - `export LD_LIBRARY_PATH=/home/developer/Libraries/libfranka/lib:$LD_LIBRARY_PATH`
  - `export PYTHONPATH=/host/Code/Robo67/robo67_insertion:$PYTHONPATH`

3. Clean restart of bringup

- Kill stale bringups/controllers first, then relaunch:
  - `pkill -f "franka.launch.py|franka_control2_node|mujoco|sim.launch.py|joint_state_publisher|robot_state_publisher|ros2_control_node|controller_manager" || true`
  - `ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67 use_fake_hardware:=false arm_id:=panda`

4. Health checks (must pass before motion)

- `/joint_states` must be valid (`name/position/velocity/effort` lengths match).
- `FrankaState` must be live at `/franka_robot_state_broadcaster/robot_state`.
- Active command path is detected by subscriber presence:
  - MMC path: `/panda/panda_cartesian_impedance_controller/desired_pose` (`CartesianImpedanceGoal`)
  - Subscriber path: `/cartesian_impedance/pose_desired` (`Float64MultiArray`)

5. Script behavior (important)

- Hardware helper scripts auto-detect command path now (`--cmd-mode auto`):
  - `scripts/hw_move_to.py`
  - `scripts/hw_probe_contact.py`
  - `scripts/hw_handguide.py`
  - `scripts/hw_recover.py`
  - `scripts/hw_cartesian_hold.py`
- You can force a path with `--cmd-mode mmc` or `--cmd-mode subscriber`.
- In subscriber mode, stiffness parameter service is unavailable; scripts continue safely with warnings.

6. Minimal sanity sequence

- Motion sanity:
  - `python3 scripts/hw_move_to.py --xyz <x> <y> <z> --speed 0.015 --cmd-mode auto`
- Recovery/hold:
  - `python3 scripts/hw_recover.py --cmd-mode auto`
- Guarded contact probe:
  - `python3 scripts/hw_probe_contact.py --max-drop 0.20 --contact-n 6 --cmd-mode auto`

7. Quick safe demo (fun)

- `python3 /host/Code/Robo67/fun/robot_dance.py --selftest`
- `python3 /host/Code/Robo67/fun/robot_dance.py --topic /cartesian_impedance/pose_desired --message-type float_array --state-topic /franka_robot_state_broadcaster/robot_state --time-scale 1.8 --amp-scale 0.45 --v-max 0.18 --w-max 1.0 --max-runtime 20`

8. Common failure signatures

- `NO MOTION` while state is live: command path mismatch or no subscriber on selected topic.
- `Robot is in command error state` / mode `Move`: stop Desk motion and run recovery, then relaunch.
- `Robot state publisher ignored an invalid JointState message`: mixed graph contamination; use `ROS_LOCALHOST_ONLY=1` and relaunch clean.
