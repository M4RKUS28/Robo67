# Cable Insertion — Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> Companion to the peg-in-hole plan ([2026-06-25-peg-in-hole-insertion.md](2026-06-25-peg-in-hole-insertion.md)).
> Domain glossary: [CONTEXT.md](../../../CONTEXT.md). Stack rules: [CLAUDE.md](../../../CLAUDE.md).

**Goal:** Add a **cable-into-port insertion** task to the existing `robo67_insertion`
package, reusing the peg-in-hole insertion core unchanged. A gray multi-port box is
located with the overhead C920, the arm moves ~10 cm above it, the **wrist D405**
locates the exact target port, and the proven force-compliant insertion loop seats
the cable connector into that port.

**Branch / worktree:** `robo67_cable_insertion` (branched from `jearningers`, merges
back onto `jearningers`).

## Key insight: the insertion core is already target-agnostic

`InsertionIntentModule(socket_xyz, params)` runs the whole phase model
(`MOVE_ABOVE → DESCEND_TO_CONTACT → SEARCH_SPIRAL → PUSH_INSERT → CONFIRM → RETRACT`)
for *any* base-frame XYZ target, and `hardware_insertion_node.run_ros(args)` executes
it on the real arm with the full safety envelope, contact lifecycle, telemetry, and
gripper handling. For cable insertion the "socket" is simply the **port location**.
So the descend / spiral-search / contact / seat machinery is reused **unchanged**; the
new work is perception (box + port), wrist-camera hand-eye calibration, a two-stage
orchestration runner, and dashboard surfacing.

## Decisions locked with the user (2026-06-26)

1. **Seat while gripped (no release).** Unlike the peg task (which opens the gripper on
   the z-drop), the cable connector stays **gripped and is pushed to seat/click home**.
   This needs careful force handling: the sustained seating push is exactly what trips
   the firmware reflex on the soft controller, so the seat must be force/displacement
   bounded (small `insert_depth`, modest `insert_press`, hard moment cap) — see Phase 5.
2. **Hand-eye board = ChArUco displayed on a tablet.** Caveat to handle in Phase 4:
   the on-screen square size must be **measured with a ruler** and passed as
   `square_length` (screen DPI/zoom changes scale); mind screen glare (the D405 view is
   heavily window-backlit) — matte angle / dim the room.
3. **Vision first:** grab a **live overhead frame** to tune the gray-box detector
   against the real current scene before writing detection code.

## Global constraints (unchanged from peg plan)

- Package `robo67_insertion`: ROS 2 **ament_python**. **No custom messages** — reuse
  `geometry_msgs`, `std_msgs`, `franka_msgs`, `sensor_msgs`.
- Real-arm command path: subscriber controller `/cartesian_impedance/pose_desired`
  (`std_msgs/Float64MultiArray`), `pos_stiff` MUST match the running controller.
- Pure-logic `lib/` files import **no rclpy** → host-testable with pytest+numpy+opencv.
- Every commanded setpoint passes the safety envelope (workspace AABB, step/velocity
  cap, force abort, watchdog).
- Single-arm mutex (`flock /tmp/robo67_arm.lock`) for any real-arm run.
- Run real bringup inside `multipanda-container`, `ROS_DOMAIN_ID=1`,
  `ROS_LOCALHOST_ONLY=1`.

---

## Reuse map (what changes vs what's reused verbatim)

**Reused with ZERO changes:**
`lib/insertion_intent.py`, `lib/command_path_adapters.py`, `lib/contact_lifecycle.py`,
`lib/safety_envelope.py`, `lib/safety.py`, `lib/spiral.py`, `lib/wrench.py`,
`lib/geometry.py`, `lib/telemetry.py`, `lib/image_overlay.py`,
`nodes/camera_publisher_node.py`, `hardware_insertion_node.run_ros` (the whole real-arm
loop), the C920→base homography + its calibration tool.

**New / extended:**

| Area | File | Change |
|------|------|--------|
| Box detector (overhead) | `lib/box_detect.py` (new) | `detect_gray_box(bgr) -> list[Box]`: largest gray quad via contour + `approxPolyDP`, HSV-saturation + aspect/extent gates. Returns centroid + quad (Hole-compatible). |
| Port detector (wrist) | `lib/port_detect.py` (new) | `detect_ports(bgr) -> list[Port]`: dark rectangular/round port openings on the box face; returns `Hole`-like `(u,v,score)`. |
| Hand-eye math (pure) | `lib/handeye.py` (new) | `cv2.calibrateHandEye` wrapper (eye-in-hand): N×`(T_base_ee, T_board_cam)` → `T_cam_ee`. ChArUco/checkerboard board pose helper. |
| Absolute wrist mapping | `lib/pixel_mapping.py` (extend) | `EyeInHandMappingAdapter`: port pixel + depth + intrinsics + `T_cam_ee` + EE pose → **absolute base XYZ** (gives Z, which overhead can't). Keep `PinholeMappingAdapter` for optional servo refine. |
| Box detector node | `nodes/box_detector_node.py` (new, or extend `socket_detector_node` `kind`) | overhead overlay + base-XY pose publish, reuses homography + `camera_publisher`. |
| Port detector node | `nodes/port_detector_node.py` (new, or extend `d405_servo_node`) | wrist overlay + port pose publish. |
| Hand-eye calibration tool | `scripts/hw_calibrate_handeye.py` (new) | guided capture mirroring `hw_calibrate_socket_proxy.py`; saves `config/d405_handeye.npz`. |
| Orchestration runner | `scripts/hw_cable_insertion_vision.py` (new) | 4-stage flow; reuses `hardware_insertion_node.build_parser()/run_ros()`. |
| Config | `config/robo67.yaml` + `config_schema.py` (extend) | additive `cable`/`box`/`port` section + `d405_handeye.npz` path + box approach height. |
| Launch | `launch/logging.launch.py` (extend) | optional box/port detector nodes (additive, behind flags). |
| Dashboard server | `dashboard/server/cable_control.py` (new) + `serve.py` | generalize the insertion spawner; `POST /api/cable/{start,stop}`, `GET /api/cable/status`. |
| Dashboard web | `components/CableInsertionControl.tsx` (new) + `api/queries.ts` | clone of `InsertionControl.tsx`; D405 panel already exists + subscribed. |

---

## Workflow (the runner)

```
1. PERCEIVE BOX   overhead C920 + homography      → box center base XY (+ taught top Z)
2. MOVE_ABOVE     ~10 cm above box, tool-down      → reuse run_ros MOVE_ABOVE / hw_move_to
3. PERCEIVE PORT  wrist D405 + hand-eye + depth    → exact port base XYZ (incl. Z)
   (optional)     D405 IBVS servo refine           → reuse PinholeMappingAdapter
4. INSERT (SEAT)  run_ros(socket_xyz=port,         → reuse entire insertion loop,
                  keep gripped, bounded seat)        NO gripper release
```

---

## Phases & tasks

### Phase 0 — Live scene capture (vision groundwork) ✅
- [x] Confirm overhead `camera_publisher` is up (or grab the C920 device directly); save a
      current overhead frame to `robo67_insertion/captures/` and confirm WITH THE USER
      which object is the target gray box. (`captures/overhead_live_cable.jpg`; target =
      bottom-center dark industrial I/O box, target port = a LAN/RJ45 jack.)
- [x] Save representative frame as a detector fixture (`test/fixtures/c920_io_box.jpg`).

### Phase 1 — Gray-box detector (overhead, pure, TDD) ✅
- [x] `lib/box_detect.py`: `Box` + `BoxParams` + `detect_gray_box(bgr)` (local-texture-energy
      detector — box body ≈ carpet brightness, so texture not intensity discriminates).
- [x] **Upgraded to object-SPECIFIC ORB template matching** (`OrbBoxMatcher` /
      `detect_box_orb` + `BoxOrbParams`, reference `config/box_template.jpg`): the texture
      heuristic only finds the "busiest blob", which the cluttered scene hijacked (white
      retail box / teal package / knob box → wrong box, arm moved to wrong place). ORB+RANSAC
      locks onto THIS box regardless of position/rotation and rejects all distractors
      (~110+ inliers cross-pose); texture kept as fallback.
- [x] `test/test_box_detect.py`: texture (synthetic + real fixture) AND ORB (reference frame,
      moved frame, absent→empty, matcher reuse). All green.
- [x] Overlay support: `draw_box_overlay` (oriented quad + centroid + base-XY label).
- [x] Map box centroid → base XY through the existing C920 homography (`HomographyMappingAdapter`).

### Phase 2 — Box detector node + overhead pose ✅
- [x] `nodes/box_detector_node.py`: subscribe to `cam_overhead_raw` (BEST_EFFORT
      `camera_qos`), publish `box_pose` + `box_detection` + overlay feed.
- [x] Wire into `launch/logging.launch.py` behind `detector:=socket|box`.
- [x] Live smoke test in `multipanda-container`: locks onto the I/O box, publishes
      `box_pose ≈ (0.466, -0.231, taught_z)`.

### Phase 3 — MOVE_ABOVE the box ✅
- [x] `scripts/hw_cable_insertion_vision.py`: perceive box (overhead, ORB by default) →
      compute tool-down target ~10 cm above box center → move via `hw_move_to.Mover` (ROS
      imports lazy so `--selftest` runs offline). Offline `--selftest` PASS.
- [x] LIVE move-above on the real arm (verified): ORB perceived the correct box (112
      inliers, base (0.477, -0.275)); gentle ramp+settle, Fz 4-5 N, `mode 2` throughout,
      `reached=True`, final EE (0.470, -0.267, 0.253) ≈ 10 cm above the box.
      (First live attempt used the texture detector and moved to the WRONG box — that
      motivated the ORB upgrade in Phase 1.)

### Phase 4 — D405 hand-eye calibration (NEW)
- [ ] `lib/handeye.py`: pure `cv2.calibrateHandEye` wrapper + ChArUco/checkerboard board
      pose (`cv2.aruco` / `findChessboardCorners` + `solvePnP`). Host-tested with synthetic poses.
- [ ] `scripts/hw_calibrate_handeye.py`: guided capture (hand-guide arm to N board views),
      record `(T_base_ee from FrankaState o_t_ee, T_board_cam)`, solve `T_cam_ee`, save
      `config/d405_handeye.npz`. Mirror `hw_calibrate_socket_proxy.py` UX + `start_calibration`.
- [ ] Determine depth source: `pyrealsense2` in-container? else taught standoff.

### Phase 5 — Port detector + absolute wrist mapping + SEAT
- [ ] `lib/port_detect.py` + `test/test_port_detect.py`.
- [ ] `lib/pixel_mapping.py`: `EyeInHandMappingAdapter` (port pixel + depth + `T_cam_ee` +
      EE pose → absolute base XYZ); `test/test_pixel_mapping.py` additions.
- [ ] `nodes/port_detector_node.py` (or extend `d405_servo_node`) + wrist overlay.
- [ ] Tune insertion params for **seat-while-gripped**: small `insert_depth`, bounded
      `insert_press`, hard moment cap, NO `--release-on-insert`. Add a seat-success
      criterion (z-drop + bounded force plateau) without tripping the reflex.

### Phase 6 — Orchestration runner
- [ ] `scripts/hw_cable_insertion_vision.py`: chain Phases 1→5; reuse
      `hardware_insertion_node.build_parser()/run_ros()` for MOVE_ABOVE + INSERT.
      Offline `--selftest` (synthetic box+port+plant), `--dry-run` on real arm, then live.

### Phase 7 — Dashboard
- [ ] Server: generalize the insertion spawner; `dashboard/server/cable_control.py` +
      `POST /api/cable/{start,stop}` + `GET /api/cable/status` in `serve.py`.
- [ ] Web: `components/CableInsertionControl.tsx` (clone of `InsertionControl.tsx`) in the
      header + `api/queries.ts` hooks. D405 panel already exists + subscribed (shows the
      port overlay once `port_detector` publishes).

### Phase 8 — Docs & verification
- [ ] `python3 -m pytest robo67_insertion/test -q` green (incl. new detector/handeye tests).
- [ ] Runbook `docs/runbooks/cable-insertion.md` (mirror automated-insertion.md).
- [ ] Update `CLAUDE.md` (new scripts/nodes/endpoints), diagrams, and check the boxes here.

---

## Open items to resolve as we go
- Port search pattern: Archimedean spiral likely fine; revisit for rectangular ports.
- Whether box top Z is taught (constant) or measured by the wrist depth at MOVE_ABOVE.
- Multi-port disambiguation: which port is the target (left-most? a marked one? user-chosen?).
