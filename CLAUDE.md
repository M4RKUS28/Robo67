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

## Hardware runs (real arm — verified 2026-06-25)

- Real bringup runs **inside `multipanda-container`** on **`ROS_DOMAIN_ID=1`** (`ROS_LOCALHOST_ONLY=0`), `robot_ip:=192.168.1.67`. A leftover sim sits on domain 7 — keep real work on domain 1.
- Launch: `ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67 use_fake_hardware:=false arm_id:=panda` (add `/home/developer/Libraries/libfranka/lib` to `LD_LIBRARY_PATH`). It **auto-activates** `cartesian_impedance_controller` (`franka_controllers/CartesianImpedanceController`), which subscribes to `/cartesian_impedance/pose_desired` (`std_msgs/Float64MultiArray` = `[px,py,pz, R00..R22]` row-major; px and R22 must stay non-zero). State: `/franka_robot_state_broadcaster/robot_state` (~29 Hz); `robot_mode` 1=Idle 2=Move 4=Reflex 5=UserStopped.
- This subscriber controller is the **real-hardware** Cartesian-impedance path; the "MMC controller only" guidance is about **sim parity**, not hardware.
- **FCI is activated only via Desk** (`https://192.168.1.67/desk/`, franka/frankaRSI). Taking control from another tab needs a physical button tap at the robot (Single Point of Control); FCI active ⇒ Desk UI is locked out.
- A force-threshold reflex (`robot_mode` 4, "Configured force thresholds reached") crashes the whole bringup — recover via `/panda_error_recovery_service_server/error_recovery`, then relaunch.
- `fun/robot_dance.py` is a safe demo: it streams compliant eased offsets around the *current* EE pose with hard speed/box/force clamps and an offline `--selftest`. `--routine spin` is a bigger "breakdance" (orbit the base = joint-1 spin, stand up vertical, spin the flange/"hand" yaw) via absolute base-frame poses + a `SpinLimiter` (reach/floor/speed clamps); note a real Panda can't do a literal 360 (joints 1 & 7 are ±166°) so each "spin" is a full-range sweep. Sim first; clear a WIDE area.
