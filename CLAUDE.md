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

## Hardware runs (real arm — verified 2026-06-25)

- Real bringup runs **inside `multipanda-container`** on **`ROS_DOMAIN_ID=1`** (`ROS_LOCALHOST_ONLY=0`), `robot_ip:=192.168.1.67`. A leftover sim sits on domain 7 — keep real work on domain 1.
- Launch: `ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67 use_fake_hardware:=false arm_id:=panda` (add `/home/developer/Libraries/libfranka/lib` to `LD_LIBRARY_PATH`). It **auto-activates** `cartesian_impedance_controller` (`franka_controllers/CartesianImpedanceController`), which subscribes to `/cartesian_impedance/pose_desired` (`std_msgs/Float64MultiArray` = `[px,py,pz, R00..R22]` row-major; px and R22 must stay non-zero). State: `/franka_robot_state_broadcaster/robot_state` (~29 Hz); `robot_mode` 1=Idle 2=Move 4=Reflex 5=UserStopped.
- This subscriber controller is the **real-hardware** Cartesian-impedance path; the "MMC controller only" guidance is about **sim parity**, not hardware.
- **FCI is activated only via Desk** (`https://192.168.1.67/desk/`, franka/frankaRSI). Taking control from another tab needs a physical button tap at the robot (Single Point of Control); FCI active ⇒ Desk UI is locked out.
- A force-threshold reflex (`robot_mode` 4, "Configured force thresholds reached") crashes the whole bringup — recover via `/panda_error_recovery_service_server/error_recovery`, then relaunch.
- `fun/robot_dance.py` is a safe demo: it streams compliant eased offsets around the _current_ EE pose with hard speed/box/force clamps and an offline `--selftest`.
- **Domain-1 contamination (seen 2026-06-25):** a MuJoCo sim stack (`/mujoco_server`, `/mujoco_ros2_control`, `/panda_gripper_sim_node`) had leaked onto domain 1, so `FrankaState` was not getting through and `/controller_manager` services timed out even though `franka_control2_node` was alive and `/joint_states` flowed. Fix = the runbook clean restart with **`ROS_LOCALHOST_ONLY=1`** (isolates the graph; does NOT affect the libfranka TCP link to `192.168.1.67`). After that the real bringup connected cleanly and `FrankaState` was live. Prefer `ROS_LOCALHOST_ONLY=1` for real runs to avoid this. The `hardware_insertion_node` dry-run + `--nudge` + `scripts/hw_probe_contact.py` were all re-validated on the real arm via the subscriber path.

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
