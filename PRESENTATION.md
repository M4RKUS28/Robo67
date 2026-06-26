# Robo67 — Peg-in-Hole: Vision + State Machine

A two-part pipeline: **classical OpenCV** finds the socket in the overhead camera,
maps its pixel to a robot base-frame point, then a **controller-agnostic state
machine** drives a compliant (Cartesian impedance) insertion. No learning, no
depth model — intensity thresholds, contours, and force.

---

## 1. OpenCV socket detection (`lib/hole_detect.py`)

An overhead Logitech C920 looks straight down at the workspace. A single
detector — `detect_white_cubes` (`lib/hole_detect.py`, pure numpy + OpenCV,
host-testable, no ROS) — returns a list of `Hole(u, v, radius_px, score)` in
pixel coordinates, best first.

**`detect_white_cubes`** — the socket is a small bright **white cube** on a dark
mat; we detect that square and return its **centroid** (the bore sits ~centred on
the cube, so the centroid is a stable proxy for it, and it stays robust when the
white-on-white bore overexposes to flat white).

Pipeline:

1. Grayscale + median blur.
2. **Adaptive brightness threshold** `max(bright_floor, pctl(gray, 99.5) − drop)`
   → binary white mask (adapts to exposure, stays above the carpet).
3. Morphological **close then open** to fuse the cube and drop speckle.
4. External contours.
5. Keep contours by **area band** (the cube has a bounded apparent size under the
   fixed camera — the upper cap rejects larger white clutter like boxes/devices),
   then by **aspect + fill (extent)** measured against the **rotated** min-area
   rectangle (`cv2.minAreaRect`) so a socket at *any rotation* still reads as a
   filled ~square (a 45° square fills only ~50% of its axis-aligned bbox).
6. Centroid via image **moments**; results sorted by contour **area** descending
   (largest cube first).

**Why classical works here:** the white cube is the only large, filled, roughly
square bright blob on a dark mat; the rotated-rect shape test rejects the
elongated arm / cables / edges; and the area cap rejects bigger white clutter.
It keys on the cube *body*, not the bore, so it cannot tell the socket from an
identical blank cube — keep only the socket in view.

**Pixel → base frame.** The best detection's `(u, v)` is mapped to a metric
base-frame point (`lib/pixel_mapping.py`): a calibrated **homography** for the
fixed overhead C920, or a pinhole map for the eye-in-hand D405. That point is the
**socket top center** handed to the state machine as `socket_xyz`.

---

## 2. Insertion state machine (`lib/insertion_intent.py`)

One canonical, controller-agnostic model (ADR-0001). Each tick,
`step(phase, sensors)` consumes a sensor snapshot
(`ee_xyz`, `fz`, `fz_baseline`, `t`) and returns the **next phase** plus an
**absolute base-frame target**. Controller quirks (held orientation,
below-surface equilibrium, carrot lead) live in the command-path *adapters*, not
here. Convention: **Z is up**, so *descending* = decreasing z.

```
IDLE → MOVE_ABOVE → DESCEND_TO_CONTACT → SEARCH_SPIRAL → PUSH_INSERT → CONFIRM → RETRACT → DONE
                                              │                  │
                                              └──────────────────┴──→ ERROR (retries exhausted)
```

### What happens in each state

- **IDLE** — Entry. Immediately targets a standoff point directly above the
  detected socket (`socket_z + standoff`) and hands off to `MOVE_ABOVE`.

- **MOVE_ABOVE** — Drive to the standoff pose above the hole. When the EE is
  within `approach_tol` of the target, advance to `DESCEND_TO_CONTACT`.

- **DESCEND_TO_CONTACT** — Aim *below* the surface so the compliant controller
  keeps pressing downward. Watch the contact force: when `fz` rises past
  `contact_fz_threshold` above its baseline, **record `contact_z`** (the
  socket-top surface height), reset the spiral clock/retries, and go to
  `SEARCH_SPIRAL`.

- **SEARCH_SPIRAL** — While maintaining contact, sweep an **Archimedean spiral**
  in XY around the contact point to hunt for the hole mouth. If the EE suddenly
  **drops** below `contact_z` by `z_drop_threshold`, the peg has caught the hole:
  record `hole_xy` and go to `PUSH_INSERT`. If the spiral grows past
  `spiral_max_radius`, restart from center and increment retries; exceeding
  `retry_limit` → `ERROR`.

- **PUSH_INSERT** — Press straight down at `hole_xy` toward
  `contact_z − insert_depth`. When the EE reaches (within half a drop-threshold
  of) that depth, the peg is seated → `CONFIRM`.

- **CONFIRM** — Verify depth: if the EE is at least `0.8 × insert_depth` below
  contact, insertion is confirmed → `RETRACT`. Otherwise retry the search
  (back to `SEARCH_SPIRAL`); exceeding `retry_limit` → `ERROR`.

- **RETRACT** — Climb back to the standoff above the socket. Once the EE is at or
  above `contact_z`, finish → `DONE`.

- **DONE** — Terminal success; holds position, `done = True`.

- **ERROR** — Terminal failure (search exhausted, not confirmed, or unknown
  phase). Holds the current pose with an error reason — fail-safe, never commands
  motion.

### Real-arm variant (ADR-0002)

On hardware the soft impedance controller can't safely *push home* — a sustained
seating push trips the firmware force reflex and crashes the bringup. So the real
runner uses a **force-guided (admittance) search** and **releases on the z-drop**:
the moment the peg drops into the hole it **opens the gripper and leaves the peg
seated**, then retracts — no `PUSH_INSERT` press against the bottom. The phase
flow up to the drop is identical; only the "commit" step differs.
