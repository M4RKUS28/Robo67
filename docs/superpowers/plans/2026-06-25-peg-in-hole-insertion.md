# Peg-in-Hole Insertion — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Full context, decisions, and verified interfaces live in [`/HANDOFF.md`](../../../HANDOFF.md).** This plan is the task breakdown; HANDOFF.md is the source of truth for *why*.

**Goal:** A classical vision + force peg-in-hole insertion system for the Franka Panda, driven through the `multipanda_ros2` MMC Cartesian impedance controller, delivered staged A→B→C.

**Architecture:** Overhead C920 detects the socket's dark round hole → homography maps the pixel to robot-base XY → an insertion FSM streams compliant Cartesian setpoints, descends until force contact, runs an Archimedean spiral search, and confirms by Z-drop. D405 eye-in-hand visual-servoing refines XY/Z in later milestones. No ML on the critical path.

**Tech Stack:** ROS 2 Humble, `rclpy`, `multipanda_ros2` (MMC Cartesian impedance), MuJoCo sim, OpenCV, NumPy, pytest.

## Global Constraints

- Branch: all commits to `jearningers` (per `CLAUDE.md`); worktrees branch from and merge back to `jearningers`.
- Package `robo67_insertion`: ROS 2 **ament_python**, source at `/home/minga-08/Code/Robo67/robo67_insertion/`. **No custom messages** — use `geometry_msgs`, `std_msgs`, `multi_mode_control_msgs`, `franka_msgs`.
- Controller: **MMC `panda_cartesian_impedance_controller`** only (works in sim + real). Never the `/cartesian_impedance/pose_desired` subscriber controller (real-only). Never the joint-position controller.
- Eigen 3.3.9, libfranka 0.9.2, MuJoCo 3.2.0, ROS 2 Humble.
- Pure-logic library files import **no `rclpy`** → unit-testable on the host/container with pytest+numpy+opencv.
- Every commanded setpoint passes `safety.py` (workspace AABB clamp, step/velocity cap, force abort, watchdog).
- Hardware: single-arm mutex (`flock /tmp/robo67_arm.lock`); only one agent drives the arm at a time. Sim has no such limit.
- Deadline: Friday 2026-06-26 10:00.

---

## File Structure

```
robo67_insertion/
  package.xml                # ament_python, deps: rclpy, geometry_msgs, std_msgs,
                             #   multi_mode_control_msgs, franka_msgs, sensor_msgs
  setup.py                   # entry_points for the 4 nodes
  setup.cfg
  resource/robo67_insertion
  robo67_insertion/
    __init__.py
    lib/                     # PURE LOGIC — no rclpy — parallel-owned, TDD
      __init__.py
      geometry.py            # homography + frame conversions
      hole_detect.py         # C920 dark-hole detection
      spiral.py              # Archimedean search pattern
      insertion_fsm.py       # the insertion state machine (pure)
      servoing.py            # D405 IBVS control law
      wrench.py              # contact detection + force baseline
      safety.py              # workspace/step/force clamps
    nodes/                   # thin rclpy wrappers around lib/
      __init__.py
      socket_detector_node.py
      calibration_node.py
      insertion_orchestrator_node.py
      d405_servo_node.py
    config_schema.py         # dataclasses mirroring config/robo67.yaml + loader
  config/
    robo67.yaml              # all params (see HANDOFF §5.6)
    c920_homography.npz      # produced by calibration_node (committed)
    c920_intrinsics.npz      # optional
  launch/
    sim.launch.py
    hardware.launch.py
  test/
    test_geometry.py test_hole_detect.py test_spiral.py
    test_insertion_fsm.py test_servoing.py test_wrench.py test_safety.py
    fixtures/                # captured C920/D405 frames for vision tests
```

**Frozen ROS interface (HANDOFF §4–§5):**
- Publish `multi_mode_control_msgs/CartesianImpedanceGoal` → `/panda/panda_cartesian_impedance_controller/desired_pose`
- Activate via `/set_controllers` (`SetControllers`); stiffness via `…/parameters` (`SetCartesianImpedance`, 36 col-major)
- Read `franka_msgs/FrankaState` (`o_f_ext_hat_k`[2]=Fz, `o_t_ee`[12:15]=EE xyz)
- Output `/robo67/socket_pose` (`geometry_msgs/PoseStamped`, frame `panda_link0`), `/robo67/socket_detection` (`std_msgs/Float64MultiArray` = [u,v,radius_px,score])
- Error recovery: `/panda_error_recovery_service_server/error_recovery`

---

## Phase 0 — Foundation (SEQUENTIAL — orchestrator only)

### Task 0.1: Scaffold the package
**Files:** Create the full tree above (empty modules with docstrings + signatures, `__init__.py`s, `package.xml`, `setup.py`, `setup.cfg`, `resource/`).
- [ ] Create `package.xml` (ament_python, deps listed above).
- [ ] Create `setup.py` with console_scripts entry points: `socket_detector`, `calibration`, `insertion_orchestrator`, `d405_servo`.
- [ ] Create `test/` with a trivial `test_smoke.py` (`def test_import(): import robo67_insertion`).
- [ ] Run `python3 -m pytest test/ -q` → PASS.
- [ ] Commit: `feat(robo67): scaffold ament_python package`.

### Task 0.2: Config schema + defaults
**Files:** `robo67_insertion/config_schema.py`, `config/robo67.yaml`.
- [ ] Define dataclasses: `StiffnessCfg`, `SpiralCfg`, `SafetyCfg`, `TopicsCfg`, `InsertionCfg`, `RoboConfig`, plus `load_config(path)->RoboConfig` (yaml).
- [ ] Populate `config/robo67.yaml` with defaults: free-space stiffness 400/20, contact stiffness translational-XY 150 / Z 400 / rot 20, damping 0.8, nullspace 10, Fz contact threshold 5 N, Fz abort 25 N, spiral max_radius 0.012 m / pitch 0.002 m / speed 0.005 m/s, workspace AABB (fill after Phase 0 sim), max_step 0.002 m/cycle, standoff 0.05 m, insert_depth 0.04 m, watchdog 0.2 s, retry_limit 3.
- [ ] Test `test/test_config.py`: load yaml → assert fields.
- [ ] Commit.

### Task 0.3: Verify interfaces in container + capture frames
**This freezes the contracts the swarm relies on.** No code change beyond an appendix file.
- [ ] In container: `pip3 install opencv-python pyrealsense2`; `apt-get install -y v4l-utils` (or note if rootless).
- [ ] Launch sim: `ros2 launch franka_bringup franka_sim.launch.py`; in a second shell switch to cartesian: `ros2 service call /set_controllers multi_mode_control_msgs/srv/SetControllers '{controllers: [{name: panda_cartesian_impedance_controller, resources: [panda]}]}'`.
- [ ] Record exact names: `ros2 topic list | grep -i robot_state`, `ros2 action list | grep -i grip`, `ros2 topic info /panda/panda_cartesian_impedance_controller/desired_pose`. Write findings to `docs/superpowers/plans/PHASE0_VERIFIED.md`.
- [ ] Capture host frames: C920 `gst-launch-1.0 v4l2src device=/dev/video8 num-buffers=1 ! jpegenc ! filesink location=robo67_insertion/test/fixtures/c920_socket.jpg` (after exposure lock) and D405 color `/dev/video6`. Commit fixtures.
- [ ] Commit: `chore(robo67): phase0 verified interfaces + capture fixtures`.

---

## Phase 1 — Pure-logic libraries (PARALLEL worktrees, TDD)

> Dispatch one subagent per file via `dispatching-parallel-agents`, each in a worktree off `jearningers`. Files are disjoint → conflict-free merges. Every task: write failing test → run (fail) → implement → run (pass) → commit.

### Task 1.1: `lib/geometry.py`
**Interfaces — Produces:**
- `fit_homography(pixels: np.ndarray, base_xy: np.ndarray) -> np.ndarray` (3×3)
- `pixel_to_base(H: np.ndarray, uv: np.ndarray) -> np.ndarray`
- `mat4_colmajor_to_xyz_quat(o_t_ee: Sequence[float]) -> tuple[np.ndarray, np.ndarray]` (xyz, quat xyzw)

- [ ] **Test** (`test/test_geometry.py`): build a known 3×3 affine H, map 6 base points to pixels, `fit_homography` recovers H s.t. `pixel_to_base` round-trips to <1e-6. For `mat4_colmajor_to_xyz_quat`, feed a column-major identity-rotation+translation `[1,0,0,0, 0,1,0,0, 0,0,1,0, 0.3,0.2,0.5,1]` → assert xyz≈[0.3,0.2,0.5], quat≈[0,0,0,1].
- [ ] Run → FAIL (not implemented).
- [ ] Implement with `cv2.findHomography` (or least-squares DLT) and numpy quaternion-from-matrix.
- [ ] Run → PASS. Commit.

### Task 1.2: `lib/hole_detect.py`
**Produces:** `@dataclass Hole(u,v,radius_px,score)`; `detect_holes(bgr: np.ndarray, params: HoleParams) -> list[Hole]` (white-cube mask via HSV/brightness → dark blob inside → `cv2.minEnclosingCircle`/`HoughCircles`; sort by score desc).
- [ ] **Test:** synthesize a 480×640 gray image, draw a white square with a black filled circle at (cx,cy,r); assert top `Hole` within 2 px of (cx,cy) and radius within 2 px. Add a negative test (no white region → empty list). If `test/fixtures/c920_socket.jpg` exists, assert ≥1 detection (regression).
- [ ] Run → FAIL. Implement. Run → PASS. Commit.

### Task 1.3: `lib/spiral.py`
**Produces:** `archimedean_offset(t, pitch_m, lin_speed_mps) -> (dx,dy)`; `spiral_waypoints(max_radius_m, pitch_m, pts_per_rev) -> np.ndarray[N,2]`.
- [ ] **Test:** waypoints start at ~(0,0); radius monotonically increases; max radius ≤ `max_radius_m`; consecutive spacing ≈ arc consistent with `pts_per_rev`. `archimedean_offset(0,…)==(0,0)`.
- [ ] Run → FAIL. Implement (r=pitch*θ/2π, x=r·cosθ, y=r·sinθ). Run → PASS. Commit.

### Task 1.4: `lib/wrench.py`
**Produces:** `contact_detected(fz, baseline, threshold_n) -> bool`; `class BaselineEstimator` (EMA of free-space Fz; `update(fz)`, `value`).
- [ ] **Test:** baseline 0, fz -6, thr 5 → True; fz -3 → False. EMA converges to a constant input. (Fz sign: contact pushes up → negative Fz on EE in base; test both polarities via `abs(fz-baseline)>thr`.)
- [ ] Run → FAIL. Implement. Run → PASS. Commit.

### Task 1.5: `lib/safety.py`
**Produces:** `clamp_to_workspace(xyz, aabb) -> xyz`; `clamp_step(prev_xyz, target_xyz, max_step_m) -> xyz`; `force_exceeded(wrench6, caps6) -> bool`.
- [ ] **Test:** point outside AABB clipped to face; `clamp_step` with target 0.1 m away and max 0.002 → moves exactly 0.002 toward target; wrench [0,0,30,…] with cap 25 on Fz → True.
- [ ] Run → FAIL. Implement. Run → PASS. Commit.

### Task 1.6: `lib/servoing.py`
**Produces:** `ibvs_correction(hole_uv, image_center, depth_m, fx, fy, gain) -> (dx_base, dy_base)`.
- [ ] **Test:** hole at image center → (0,0). Hole offset +10 px in u with fx=600, depth=0.1, gain=1 → dx = gain·(du·depth/fx) within tol; sign matches the tool→base convention documented in the file.
- [ ] Run → FAIL. Implement. Run → PASS. Commit.

### Task 1.7: `lib/insertion_fsm.py` (the crown jewel)
**Produces:**
- `@dataclass Sensors(ee_xyz, ee_quat, fz, fz_baseline, t)`
- `@dataclass Decision(desired_xyz, desired_quat, gripper_cmd, next_state, done, error)`
- `class InsertionFSM` with `__init__(self, socket_xyz, params)` and `step(self, state: str, s: Sensors) -> Decision`. States: `IDLE→MOVE_ABOVE→DESCEND_TO_CONTACT→SEARCH_SPIRAL→PUSH_INSERT→CONFIRM→RETRACT→DONE`, plus `ERROR`.
- [ ] **Test (table-driven):**
  - `IDLE` → `MOVE_ABOVE`, desired = socket_xyz + [0,0,standoff].
  - `MOVE_ABOVE` once EE within tol of target → `DESCEND_TO_CONTACT`.
  - `DESCEND_TO_CONTACT`: desired steps downward; when `contact_detected(fz,baseline,thr)` → record contact_z, → `SEARCH_SPIRAL`.
  - `SEARCH_SPIRAL`: desired XY follows spiral about socket while pushing down; when EE_z drops > drop_thresh below contact_z (peg entered) → `PUSH_INSERT`.
  - `PUSH_INSERT`: push to insert_depth; reaching depth → `CONFIRM`.
  - `CONFIRM`: if depth held + Fz relaxed → `RETRACT`; else → `SEARCH_SPIRAL` (retry, bounded by retry_limit → `ERROR`).
  - `RETRACT` → `DONE`.
  - Feed synthetic `Sensors` sequences; assert state transitions + that every `desired_xyz` is finite.
- [ ] Run → FAIL. Implement (pure; uses `spiral`, `wrench`; no rclpy). Run → PASS. Commit.

### Task 1.8: Merge + full suite
- [ ] Merge all worktree branches back to `jearningers`. Run `python3 -m pytest robo67_insertion/test -q` → all green. Commit merge.

---

## Phase 2 — ROS nodes (PARALLEL after Phase 1 merged)

### Task 2.1: `nodes/socket_detector_node.py`
- [ ] Subscribe/grab C920 (`cv2.VideoCapture(8)` or sensor_msgs Image if a camera node exists). On each frame: `detect_holes` → best → load `c920_homography.npz` → `pixel_to_base` → publish `/robo67/socket_pose` (PoseStamped, identity quat, frame `panda_link0`) + `/robo67/socket_detection`.
- [ ] Test: launch node with a recorded image source; `ros2 topic echo /robo67/socket_pose` shows a stable pose. Commit.

### Task 2.2: `nodes/calibration_node.py`
- [ ] Robot-as-groundtruth: command MMC to N known base XY at **socket-top Z**, capture C920, detect a gripper tag / peg tip pixel, collect (pixel, base_xy) pairs, `fit_homography`, save `config/c920_homography.npz`; print reprojection error.
- [ ] Test (HARDWARE, Phase 4): residual < spiral max_radius. Commit.

### Task 2.3: `nodes/insertion_orchestrator_node.py`
- [ ] On start: call `/set_controllers` → cartesian; set stiffness via `…/parameters`. Subscribe FrankaState (extract `o_t_ee`→xyz/quat via geometry, `o_f_ext_hat_k`[2]→fz) and `/robo67/socket_pose`. Run `InsertionFSM.step` at ~50 Hz; each `Decision.desired_xyz` → `safety` clamps → publish `CartesianImpedanceGoal`. On REFLEX/error → call error_recovery (bounded). Watchdog on stale state.
- [ ] Test: see Phase 3. Commit.

### Task 2.4: `nodes/d405_servo_node.py` + launch + config wiring
- [ ] D405 frames (pyrealsense2 or `/dev/video6`); detect hole; `ibvs_correction`; publish a refinement offset or a refined `/robo67/socket_pose`. Used in milestone B+.
- [ ] `launch/sim.launch.py` (orchestrator + detector, sim params), `launch/hardware.launch.py` (+ calibration, hardware params). Commit.

---

## Phase 3 — Sim integration (sim)

### Task 3.1: End-to-end in MuJoCo against a hardcoded socket
- [ ] Launch sim; switch to cartesian impedance; publish a **fixed** `/robo67/socket_pose` (table center).
- [ ] Run orchestrator. Validate: controller activates; ~50 Hz streaming; FrankaState parsed; **descend-to-contact** against the floor/a box registers Fz; spiral motion visible; FSM walks the states; safety clamps prevent out-of-AABB / large steps; error-recovery path callable.
- [ ] Set the real `workspace AABB` in `config/robo67.yaml` from observed reachable sim poses. Commit.
- [ ] Acceptance: orchestrator completes IDLE→…→DONE in sim without violating any safety clamp.

---

## Phase 4 — Calibration (HARDWARE · arm mutex)

### Task 4.1
- [ ] `flock /tmp/robo67_arm.lock`. Lock C920 exposure. Desk: unlock joints + FCI (Playwright `https://192.168.1.67/desk/` or manual). Bring up real arm with MMC.
- [ ] Run `calibration_node` → homography at socket-top height → save + verify reprojection error. **Do not move C920 after.** Commit `config/c920_homography.npz`.

---

## Phase 5 — Milestone A (HARDWARE · arm mutex)

### Task 5.1
- [ ] Human pre-clamps a peg. Place a socket in view. Run `hardware.launch.py`.
- [ ] Tune Fz contact threshold + contact-phase stiffness so descend is gentle and the spiral seats the peg. Achieve **one clean insertion** (peg seated, FSM→DONE). Record video. Commit any param tuning.
- [ ] Acceptance: 1 successful insertion, no reflex abort, retract clean.

---

## Phase 6 — Milestone B (HARDWARE)

### Task 6.1
- [ ] `detect_holes` returns multiple sockets; pick target (nearest / by radius if sizes differ). Add **auto-retry** loop (re-detect + re-spiral on miss, bounded). Optionally integrate D405 IBVS refinement before descend.
- [ ] Acceptance: **3 consecutive** insertions with the socket re-placed randomly each time.

---

## Phase 7 — Milestone C (STRETCH)

### Task 7.1
- [ ] Detect a colored cylinder peg on its stand (HSV color + D405 depth) → compute grasp pose → `grasp` action → lift → run insertion FSM. GPU escape hatch only if classical grasp detection is unreliable.
- [ ] Acceptance: ≥1 fully autonomous pick + insert.

---

## Self-Review notes
- Spec coverage: A (Ph0–5), B (Ph6), C (Ph7); calibration (Ph4); sim parity (Ph3); safety (1.5 + 2.3); D405 (1.6 + 2.4 + 6/7). ✓
- Types consistent: `Hole`, `Sensors`, `Decision`, `fit_homography`/`pixel_to_base` used identically across tasks. ✓
- No placeholders: each lib task has concrete test criteria + algorithm. ROS/hardware tasks use the milestone bars as acceptance tests (can't unit-test without the arm). ✓
