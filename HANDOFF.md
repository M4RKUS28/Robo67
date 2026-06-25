# HANDOFF — Challenge 1 Peg-in-Hole — Full Session Export & Execution Brief

> **Date:** 2026-06-25 (Thursday) · **Branch:** `jearningers` · **Team:** Robo67
> **Status:** Grilling/design session COMPLETE. All architecture decisions locked.
> **Your job (next agent):** (1) Turn the "Roadmap" below into a detailed TDD implementation
> plan using the `writing-plans` skill, saved to `docs/superpowers/plans/2026-06-25-peg-in-hole-insertion.md`.
> (2) Orchestrate the swarm to build it (`subagent-driven-development` / `dispatching-parallel-agents`
> + `using-git-worktrees`). All architecture below is DECIDED — do not re-litigate; expand into tasks.

---

## 0. TL;DR — what we are building

A **classical vision + force** peg-in-hole insertion system for the **Franka Emika Panda**, driven
through the `multipanda_ros2` MMC **Cartesian impedance controller** (works in sim **and** on hardware).

- **Perceive:** overhead **Logitech C920** detects the socket's dark round hole → maps the hole pixel
  to robot-base XY via a pre-fitted **homography**.
- **Insert:** stream compliant Cartesian setpoints → **descend until force contact** (Fz) →
  **Archimedean spiral search** while pushing gently down → the chamfer + compliance seat the peg →
  confirm by EE **Z-drop** + Fz relaxing.
- **Refine (later milestones):** eye-in-hand **D405** visual-servos the hole to image center and
  reads depth for Z.
- **No ML on the critical path.** RTX 5090 is an escape hatch only.

**Staged delivery: A → B → C.**
- **A (floor, guaranteed):** human pre-clamps a peg in the gripper; robot finds the socket with C920
  and does one clean compliant insertion.
- **B (the real target):** human hands pegs one at a time; robot inserts each into its detected
  socket — **3 consecutive successes** with the socket re-placed randomly, with **auto-retry** on miss.
- **C (stretch):** robot autonomously detects a peg on its stand, grasps it (color + D405 depth),
  then inserts. ≥1 fully autonomous pick+insert.

**Hard deadline:** hacking ends **Friday 2026-06-26 10:00**. Everything is scoped around this.

---

## 1. How this session went (conversation export)

The user is a strong software engineer (deep learning, computer vision, RL) but **new to robotics**.
They asked for a relentless, doc-grounded grilling session, then a multi-phase roadmap that a swarm
of AI agents implements end-to-end with TDD on git worktrees, interacting with the real arm.

I (the design agent) did the following before grilling:
1. Read all repo docs: `docs/cameras.md`, `docs/franka/{bringup_api,specs,fci_overview}.md`,
   `docs/hackathon/{hacker_handbook,intel_challenge}.md`.
2. Inspected all setup photos in `docs/assets/` (workspace, peg, socket, wrist/D405 mount).
3. Dispatched a sub-agent to exhaustively map the `multipanda_ros2` control interfaces
   (cited verbatim in §4).
4. Verified operational reality directly: container up, robot reachable, camera device nodes,
   sim controller config.

Then we ran a 12-question grilling session (the user overruled my recommendation several times —
those overrides are authoritative and captured in §3). The earlier handoff's "ArUco ID 5 + 4 cans
with a ruler" plan is **SUPERSEDED** by the decisions below.

The detailed inline plan write-up kept getting cut off by connection drops, so per the user's
instruction this document is the **single source of truth**: it fully exports the chat + decisions
and tells the next AI how to build it. The next AI writes the granular TDD plan.

---

## 2. Physical setup (from the photos in `docs/assets/`)

- **Socket** = white 3D-printed **cube** with a **chamfered round hole** on top. The hole is a dark
  circle on a bright white cube against the gray grooved table → an ideal classical-CV target.
  **Organizer-fixed** (we do NOT control the print). Clearance ~2 mm (prior note — re-measure).
- **Peg** = **round** cylinder (white/red/tan variants) standing on a square base stand. **Round ⇒
  insertion orientation about Z does not matter** — we only need XY + a vertical approach.
- **Table** = aluminum extrusion with strong **parallel grooves** (gray). White cube segments cleanly
  against it; grooves are lines, not circles, so they won't fool a circle detector.
- **Cameras:**
  - **Logitech C920** — overhead, on a clamp. **This is our primary perception camera.** Overexposed
    by default → lock exposure. **MUST NOT be moved after calibration.**
  - **Microdia** — second overhead webcam on an articulated arm (unused in the primary plan).
  - **Intel RealSense D405** — **eye-in-hand**, on a 3D-printed bracket on the wrist (two stereo
    lenses). Short-range RGB-D, ideal 7–50 cm. Used for fine refinement in milestone B+ and grasping
    in C.
- **E-stop** present (yellow box, red button). Custom red ridged 3D-printed gripper fingertips.

---

## 3. DECISION LOG (authoritative — do not re-open)

| # | Question | DECISION | Rationale / notes |
|---|----------|----------|-------------------|
| 1 | Demo scope vs. prior "one pre-grasped insertion" | **Staged A → B → C** | Guaranteed floor first, then repeatable, then autonomous. (User picked over my "B".) |
| 2 | Socket 3D localization / keep D405? | **C920 XY + D405 fine XY/Z**, force-probe always the Z safety net | User kept D405 in the critical path (I'd recommended C920-only). **Constraint I imposed & user accepted:** force-probe + spiral stays regardless; D405 is *layered refinement*, never a blocker for the floor demo. |
| 3 | C920 → base calibration | **Robot-as-groundtruth homography** | Move EE/gripper-tag to known base XY points, detect in C920, fit pixel→base homography. No rulers. **Calibrate at the SOCKET-TOP plane height, not the table**, to remove parallax on the hole. |
| 4 | Socket detection method | **Direct dark-hole CV** (center px + radius); ArUco = documented fallback | Marker-free; radius doubles as a socket-size cue for milestone-B matching. |
| 5 | Insertion algorithm | **Compliant impedance + descend-to-contact (Fz) + Archimedean spiral + force/Z-drop success** | Deterministic, no training, plays to the RT impedance controller. |
| 6 | Sim vs hardware split | **Sim = plumbing + motion logic + descend-to-box; Hardware = real insertion tuning** | Don't build a high-fidelity contact sim. Sim lets parallel agents work without the one arm. |
| 7 | D405 method | **Visual servoing (no hand-eye calibration)**; depth → Z | Tool is always vertical ⇒ D405 image axes map to base XY via depth+intrinsics scale. Self-correcting. Full hand-eye = documented fallback. |
| 8 | RTX 5090 / learning role | **Escape hatch only — fully classical** | NVIDIA/AWS box is unrelated to the Intel OpenVINO bonus (which we skip anyway). GPU only if classical grasping in C fails. |
| 9 | Execution model | **Plan → handoff → a NEW session orchestrates the swarm** | This document + the TDD plan are the contract. The design agent does NOT execute. |
| 10 | Success bar | **1 guaranteed, then repeatable** | A = 1 clean insertion. B = 3 consecutive, socket re-placed randomly each time, auto-retry on miss. C = ≥1 autonomous pick+insert. These define the acceptance tests. |
| 11 | Hardware safety/supervision | **Fully autonomous; single-arm mutex (one agent at a time)** | User trusts the agents (overruled my human-gating rec). I still require non-negotiable **software** safety caps (§7) and recommend a human near the E-stop. |
| 12 | Do we control the prints? | **No — sockets are organizer-fixed** | Work with given chamfer + ~2 mm clearance; spiral + compliance carry robustness. |

---

## 4. VERIFIED software interfaces (cite these; do not guess)

All paths under `/home/minga-08/Code/multipanda_ros2`. **Use the MMC Cartesian impedance controller**
— it runs in **both sim and real**. Do **NOT** use the subscriber `franka_controllers/CartesianImpedanceController`
(`/cartesian_impedance/pose_desired`, Float64MultiArray) — it is **real-robot-only** and breaks sim parity.
Never use the joint-position controller (bad motors).

### 4.1 Activate the controller
- **Service:** `/set_controllers` (global, no namespace)
- **Type:** `multi_mode_control_msgs/srv/SetControllers` → request `Controller[] controllers`
- **`Controller.msg`:** `string name`, `string[] resources`
- Single arm: `name="panda_cartesian_impedance_controller"`, `resources=["panda"]`
- Confirmed registered in single-arm **sim**: `franka_bringup/config/sim/single_sim_controllers.yaml:65-77`
  (controllers list includes `panda_cartesian_impedance_controller`, resource `["panda"]`;
  `start_controllers` defaults to joint impedance → switch to cartesian via this service).

### 4.2 Command a pose (stream ~50 Hz)
- **Topic:** `/panda/panda_cartesian_impedance_controller/desired_pose`
- **Type:** `multi_mode_control_msgs/msg/CartesianImpedanceGoal`
  ```
  geometry_msgs/Pose pose      # equilibrium pose the arm springs toward
  float64[7] q_n               # nullspace joint config
  ```
  (`.../multi_mode_control_msgs/msg/CartesianImpedanceGoal.msg`)

### 4.3 Set stiffness/damping at runtime
- **Service:** `/panda/panda_cartesian_impedance_controller/parameters`
- **Type:** `multi_mode_control_msgs/srv/SetCartesianImpedance`
  ```
  float64[36] stiffness        # 6x6, COLUMN-MAJOR
  float64[6]  damping_ratio
  float64     nullspace_stiffness
  ```
  (`.../multi_mode_control_msgs/srv/SetCartesianImpedance.srv`)
- **Reference values** from `docs/example_scripts/mmc_demo_script.py`: translational stiffness
  **400 N/m**, rotational **20**, `damping_ratio` **0.8** all axes, `nullspace_stiffness` **10**.
  For insertion: keep Z moderate to push, **soften XY near contact** so the chamfer guides the peg.

### 4.4 Read robot state (force + EE pose)
- **Broadcaster:** `franka_robot_state_broadcaster/FrankaRobotStateBroadcaster`
- **Topic:** `~/<arm_id>/robot_state` → likely **`/franka_robot_state_broadcaster/panda/robot_state`**
  (an earlier note said `/franka_robot_state_broadcaster/robot_state` — **CONFIRM exact name at
  runtime with `ros2 topic list`** in Phase 0).
- **Type:** `franka_msgs/msg/FrankaState`. Key fields:
  - `float64[6] o_f_ext_hat_k` — external wrench `[Fx,Fy,Fz,Mx,My,Mz]`; **Fz = index 2** (contact signal)
  - `float64[16] o_t_ee` — EE pose, **column-major 4×4** (translation = indices 12,13,14)
  - `float64[7] q` — joint positions (use for `q_n` nullspace)
  - `robot_mode`, `current_errors` — detect REFLEX/error states
- Sim publishes at 60 Hz (`single_sim_controllers.yaml`).

### 4.5 Gripper (identical in sim)
- Node `franka_gripper_node`; actions `~/homing` (`franka_msgs/action/Homing`),
  `~/move` (`Move`: `width`,`speed`), `~/grasp` (`Grasp`: `width`,`epsilon`,`speed`,`force`),
  `~/gripper_action` (`control_msgs/action/GripperCommand`).
- **CONFIRM the namespace** at runtime (e.g. `/panda_gripper/...` or `/franka_gripper/...`).

### 4.6 Error recovery (rehearse this!)
- **Service:** `/panda_error_recovery_service_server/error_recovery`, type `franka_msgs/srv/ErrorRecovery`
  (empty request → `bool success, string error`). On recovery it re-runs the previous control loop —
  no controller reload needed.

### 4.7 Launch
- **Sim (single):** `ros2 launch franka_bringup franka_sim.launch.py`
  (args: `arm_id:=panda`, `initial_positions`, `use_rviz`). Opens MuJoCo with one Panda.
- **Real (single):** `ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>` OR
  `multimode_franka.launch.py robot_ip:=<fci-ip>`. **CONFIRM which real launch loads the
  `multi_mode_controller`** (Phase 0). Sim has no cameras.

---

## 5. Target architecture & FROZEN interface contracts

> These contracts are what let parallel worktree agents NOT diverge. The orchestrator freezes them
> in Phase 0; downstream agents implement against them exactly.

### 5.1 Data flow
```
C920 frame ──▶ hole_detect (u,v,radius) ──▶ pixel_to_base(H) ──▶ socket XY (base) ──┐
                                                                                     ▼
                                                            /robo67/socket_pose (PoseStamped, panda_link0)
                                                                                     │
[milestone B+] EE above socket ─▶ D405 IBVS centers hole + depth→Z ─▶ refine pose ──┤
                                                                                     ▼
                          Insertion FSM ──streams CartesianImpedanceGoal @~50Hz──▶ MMC controller
                                  ▲ reads FrankaState (o_f_ext_hat_k Fz, o_t_ee) ─────┘
                          force = Z sensor + success signal; safety clamps every setpoint
```

### 5.2 Package
- **`robo67_insertion`** — ROS 2 **ament_python** package, source at
  `/home/minga-08/Code/Robo67/robo67_insertion/` (in this repo → version-controlled on `jearningers`).
  Built as an **overlay** in the container from the host mount `/host/Code/Robo67/robo67_insertion`.
- **NO custom messages** (keeps the build a pure ament_python overlay — no rosidl/CMake). Use
  `geometry_msgs`, `std_msgs`, and the existing `multi_mode_control_msgs` / `franka_msgs`.
- Language: **Python `rclpy`** (decided).

### 5.3 ROS topics we own
- `/robo67/socket_pose` — `geometry_msgs/PoseStamped`, `frame_id="panda_link0"`. Position = socket
  **top-center** in base meters; orientation = **identity** (tool vertical).
- `/robo67/socket_detection` — `std_msgs/Float64MultiArray` = `[u, v, radius_px, score]` (diagnostics/debug).

### 5.4 Pure-logic library (NO `rclpy`; fully unit-testable with pytest + numpy + opencv)
Each is a **separate file = separate worktree owner** → clean parallel merges.

- `robo67_insertion/lib/geometry.py`
  - `fit_homography(pixels: np.ndarray[N,2], base_xy: np.ndarray[N,2]) -> np.ndarray[3,3]`
  - `pixel_to_base(H: np.ndarray[3,3], uv: np.ndarray[...,2]) -> np.ndarray[...,2]`
  - `mat4_colmajor_to_xyz_quat(o_t_ee: Sequence[float]) -> tuple[np.ndarray[3], np.ndarray[4]]`  (quat xyzw)
  - `xyz_quat_to_pose(xyz, quat_xyzw) -> geometry_msgs.msg.Pose`  (only ROS msg import allowed here)
- `robo67_insertion/lib/hole_detect.py`
  - `@dataclass Hole: u: float; v: float; radius_px: float; score: float`
  - `detect_holes(bgr: np.ndarray, params: HoleParams) -> list[Hole]`  (white-cube mask → dark circle via contour/`minEnclosingCircle`/Hough; sorted by score)
- `robo67_insertion/lib/spiral.py`
  - `archimedean_offset(t: float, pitch_m: float, lin_speed_mps: float) -> tuple[float,float]`
  - `spiral_waypoints(max_radius_m: float, pitch_m: float, pts_per_rev: int) -> np.ndarray[N,2]`
- `robo67_insertion/lib/insertion_fsm.py`
  - States: `IDLE, MOVE_ABOVE, DESCEND_TO_CONTACT, SEARCH_SPIRAL, PUSH_INSERT, CONFIRM, RETRACT, DONE, ERROR`
  - `@dataclass Sensors: ee_xyz; ee_quat; fz; fz_baseline; t`
  - `@dataclass Decision: desired_xyz; desired_quat; gripper_cmd; next_state; done; error`
  - `class InsertionFSM: step(self, state, sensors: Sensors, params) -> Decision`  (**pure** — the crown jewel; test exhaustively with synthetic sensor sequences)
- `robo67_insertion/lib/servoing.py`
  - `ibvs_correction(hole_uv, image_center, depth_m, fx, fy, gain) -> tuple[float,float]`  (Δbase XY)
- `robo67_insertion/lib/wrench.py`
  - `contact_detected(fz: float, baseline: float, threshold_n: float) -> bool`
  - `BaselineEstimator` (running estimate of free-space Fz to subtract gravity/peg weight)
- `robo67_insertion/lib/safety.py`  (**non-negotiable** — see §7)
  - `clamp_to_workspace(xyz, aabb) -> xyz`
  - `clamp_step(prev_xyz, target_xyz, max_step_m) -> xyz`  (velocity cap via setpoint step limit)
  - `force_exceeded(wrench6, caps6) -> bool`

### 5.5 ROS nodes (thin wrappers around lib)
- `nodes/socket_detector_node.py` — grabs C920 frames, `detect_holes` + `pixel_to_base` → publishes `/robo67/socket_pose` + `/robo67/socket_detection`.
- `nodes/calibration_node.py` — robot-as-groundtruth homography capture; saves `config/c920_homography.npz`.
- `nodes/insertion_orchestrator_node.py` — runs `InsertionFSM`; subscribes FrankaState; publishes `CartesianImpedanceGoal`; calls `/set_controllers`, `/parameters`, gripper actions, error_recovery; applies `safety.py` to every setpoint.
- `nodes/d405_servo_node.py` — D405 frames + `ibvs_correction`; refinement for milestone B+.

### 5.6 Config & launch
- `config/robo67.yaml` — topic/service/action names, camera devices, stiffness matrices (free/contact),
  damping, nullspace, **Fz contact threshold**, **Fz abort cap**, spiral params (max_r, pitch, speed),
  **workspace AABB**, **max setpoint step**, standoff/approach Z, socket cube height, insert depth,
  watchdog timeout, retry limits.
- `config/c920_homography.npz`, `config/c920_intrinsics.npz` (committed for reproducibility).
- `launch/sim.launch.py`, `launch/hardware.launch.py`.

---

## 6. ROADMAP (next AI expands each into bite-sized TDD tasks)

### Phase 0 — Foundation (SEQUENTIAL, orchestrator only)
Scaffold `robo67_insertion` (ament_python: `setup.py`, `package.xml`, `lib/`, `nodes/`, `tests/`,
`config/`, `launch/`). Freeze the §5 contracts. In the container:
`pip3 install opencv-python pyrealsense2` and install `v4l-utils`. Build the overlay; smoke-launch
sim, **switch to `panda_cartesian_impedance_controller` via `/set_controllers`**, `ros2 topic echo`
the robot_state topic → **record the exact robot_state topic name and gripper namespace**. Capture
sample **C920 + D405** frames (GStreamer, §8) for offline vision dev. **Measure** cube height, hole
& peg diameters, clearance. Deliverable: working skeleton + a "verified interfaces" appendix.

### Phase 1 — Pure-logic libs (PARALLEL worktrees, TDD, NO hardware)
One agent per file: `geometry.py`, `hole_detect.py` (test on captured C920 frames), `spiral.py`,
`insertion_fsm.py`, `servoing.py`, `wrench.py`, `safety.py`. Disjoint files → clean merges back to
`jearningers`. Acceptance = pytest green per module.

### Phase 2 — ROS nodes (PARALLEL after Phase 1 merged)
`socket_detector_node`, `insertion_orchestrator_node`, `calibration_node`, `d405_servo_node` + launch
+ config wiring. Node-level tests where feasible; otherwise integration procedure in Phase 3.

### Phase 3 — Sim integration (sim; serialize on the sim instance)
Orchestrator drives MMC Cartesian impedance against a **hardcoded socket pose**. Validate: controller
activation, ~50 Hz setpoint streaming, FrankaState reads, **descend-to-contact vs the floor/a box**,
spiral geometry, FSM transitions, **safety clamps**, error-recovery path. This proves the whole
software stack with zero arm risk.

### Phase 4 — Calibration (HARDWARE · arm mutex)
C920 exposure lock; run `calibration_node` → robot-as-groundtruth homography **at socket-top height**;
save `c920_homography.npz`; verify reprojection error (target: small enough that spiral max-radius
covers residual). **C920 must not move afterward.**

### Phase 5 — Milestone A (HARDWARE · arm mutex)
Desk: unlock joints + activate FCI (Playwright at `https://192.168.1.67/desk/` OK, or manual). Real
bringup with MMC. Human pre-clamps a peg. Run full pipeline: detect socket (C920) → MOVE_ABOVE →
DESCEND_TO_CONTACT → SEARCH_SPIRAL → PUSH_INSERT → CONFIRM → RETRACT. Tune Fz threshold + stiffness
schedule → **one clean insertion**.

### Phase 6 — Milestone B (HARDWARE)
Multi-socket detection; peg→socket via radius (if sizes differ); **auto-retry** on miss; integrate
D405 IBVS refinement (optional, if it helps). Achieve **3 consecutive** successes with the socket
re-placed randomly each time.

### Phase 7 — Milestone C (STRETCH)
Autonomous peg grasp: detect colored cylinder on its stand (color + D405 depth) → grasp (gripper
`grasp` action) → insert. GPU escape hatch only if classical grasp detection is unreliable.

---

## 7. Non-negotiable software safety (bake into `safety.py` + config)

Even though hardware runs are autonomous (user's call), every commanded setpoint MUST pass:
1. **Workspace AABB clamp** — reject/clip any desired XYZ outside a configured safe box.
2. **Setpoint step cap** — limit per-cycle position delta (a velocity ceiling); no teleports.
3. **Force abort** — if `|Fz|` (or any axis) exceeds a configured cap, stop commanding & hold.
4. **Watchdog** — if FrankaState is stale (> N ms), hold the last safe pose; do not stream blind.
5. **Auto error-recovery** — on REFLEX/`ControlException`, call error_recovery (bounded attempts),
   then resume; abort if it keeps tripping.
6. **Conservative defaults** — low speed, soft stiffness, **loose collision thresholds** so reflexes
   don't fire mid-insertion.
7. **Gripper force cap** for grasp.
Recommended (not gated): a human near the **E-stop** during hardware runs.

---

## 8. Operational reference (verified this session)

- **Container:** `multipanda-container` (up). Image `build-env:multipanda_ros2-amd64`. User `developer`.
  Host `/home/minga-08` → container `/host`. Workspace `/home/developer/multipanda_ws/` (prebuilt).
  - Source chain: `source /opt/ros/humble/setup.bash && source ~/multipanda_ws/install/setup.bash`
  - Extra shell: `docker exec -it --user developer multipanda-container bash`
  - Missing pip pkgs: `docker exec multipanda-container pip3 install opencv-python pyrealsense2`
- **Robot/Desk:** `192.168.1.67` (reachable, ~0.27 ms direct link). Desk `https://192.168.1.67/desk/`,
  login `franka` / `frankaRSI`. **Confirm FCI IP** (likely same). Desk **OR** FCI, never both.
- **Cameras (stable `by-id`):** C920 = `/dev/video8`; D405 color = `/dev/video6`; D405 depth (Z16) =
  `/dev/video2`. Bare numbers are unstable — prefer `/dev/v4l/by-id/...`.
  - **C920 exposure lock:** `v4l2-ctl -d /dev/video8 --set-ctrl=auto_exposure=1,exposure_time_absolute=150`
    (`v4l-utils` not installed → install).
  - **Grab a frame (host has GStreamer):**
    `gst-launch-1.0 v4l2src device=/dev/video8 num-buffers=1 ! jpegenc ! filesink location=cam_c920.jpg`
  - D405 depth needs `pyrealsense2` / `realsense-viewer` (not JPEG-encodable).
- **Version pins:** Eigen **3.3.9** (NOT 3.4.0), libfranka 0.9.2, firmware 4.2.1/4.2.2, MuJoCo 3.2.0,
  ROS 2 Humble / Ubuntu 22.04.
- **Never** use the joint-position controller (bad motor behavior).
- **Deadline:** Friday 2026-06-26 **10:00**.

---

## 9. Swarm execution model (for the orchestrator)

- **Plan-as-contract:** write the granular TDD plan first (`writing-plans` →
  `docs/superpowers/plans/2026-06-25-peg-in-hole-insertion.md`), then execute with
  `subagent-driven-development` (fresh subagent per task + review) and `dispatching-parallel-agents`
  for Phase 1/2 fan-out.
- **Worktrees:** each parallel agent on its own git worktree **branched from `jearningers`**, merging
  **back to `jearningers`** (per `CLAUDE.md`). Assign **disjoint file ownership** (one lib file per
  agent) so merges are conflict-free.
- **Sequencing:** Phase 0 (solo) → Phase 1 (parallel) → merge → Phase 2 (parallel) → merge →
  Phase 3 (sim) → Phases 4–7 (hardware, serialized).
- **HARDWARE SINGLE-ARM MUTEX:** only **one** agent may drive the arm at a time. Protocol:
  acquire a lockfile (e.g. `/tmp/robo67_arm.lock` via `flock`) before any hardware launch/motion;
  release when done. FCI is exclusive anyway (Desk OR FCI). Sim has no such limit — parallelize freely.
- **TDD throughout:** pure-logic libs have real pytest suites; ROS/hardware tasks have explicit
  acceptance procedures (the §6 milestone bars are the acceptance tests).

---

## 10. Open items to resolve in Phase 0 (don't block design — measure/confirm)

- [ ] Exact `robot_state` topic name (sim vs real) and gripper action namespace (`ros2 topic/action list`).
- [ ] Which **real** launch file loads `multi_mode_controller` (`franka` vs `multimode_franka`).
- [ ] Confirm FCI IP == `192.168.1.67`.
- [ ] Measure: socket cube height, hole diameter, peg diameter, clearance, chamfer depth.
- [ ] Do multiple sockets differ in diameter? (decides whether peg→socket matching is needed in B).
- [ ] C920 intrinsics (for D405 IBVS metric scale; get D405 intrinsics from RealSense SDK).
- [ ] Homography residual after calibration → set spiral `max_radius` to comfortably exceed it.

---

## 11. Files & docs to read first (next agent)

- `CLAUDE.md` (repo rules: branch `jearningers`, Eigen 3.3.9, error_recovery, no joint-position ctrl,
  skip Intel acceleration).
- `docs/cameras.md`, `docs/franka/*.md`, `docs/hackathon/intel_challenge.md` (challenge text).
- `docs/assets/*.jpeg` + `docs/assets/depthCam/*` (setup photos; `IMG_5803.jpeg` = peg+socket close-up;
  `depthCam/depthCam.jpg` = D405 mount).
- `multipanda_ros2` paths cited in §4 (control interfaces).
- `multipanda_ros2/docs/example_scripts/mmc_demo_script.py` (reference: SetControllers /
  SetCartesianImpedance usage pattern).

**This document supersedes the earlier "ArUco ID 5 + 4 cans" handoff. The decisions in §3 are final.**
