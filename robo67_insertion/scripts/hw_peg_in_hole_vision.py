#!/usr/bin/env python3
"""hw_peg_in_hole_vision.py -- end-to-end peg-in-hole with C920 vision feedback.

A single self-contained runner (in the spirit of ``fun/robot_dance.py``) that
closes the WHOLE peg-in-hole loop on the REAL arm using overhead-camera
feedback. It owns ONLY the vision front-end + CLI; ALL insertion and safety
logic is reused from ``hardware_insertion_node`` (no duplicated control logic).

WORKFLOW (matches docs/architecture/diagrams/peg_in_hole_workflow)
------------------------------------------------------------------
1. PERCEIVE (vision) -- grab overhead C920 frame(s), detect the dark socket
   hole (:func:`~robo67_insertion.lib.hole_detect.detect_holes`), and map the
   best detection pixel -> robot-base XY through the calibrated homography
   (:class:`~robo67_insertion.lib.pixel_mapping.HomographyMappingAdapter`).
   Several frames are taken and the per-axis MEDIAN base XY is used for
   robustness. The socket-top Z cannot come from a single overhead camera, so
   it is taught via ``--socket-top-z``; the force-probe (DESCEND_TO_CONTACT)
   finds the true contact Z regardless.
2. INSERT -- hand the perceived socket ``(x, y, socket_top_z)`` to the proven
   real-arm insertion loop
   (:func:`~robo67_insertion.nodes.hardware_insertion_node.run_ros`):
   MOVE_ABOVE -> DESCEND_TO_CONTACT -> SEARCH_SPIRAL -> PUSH_INSERT -> CONFIRM
   -> RETRACT, with the full safety envelope (workspace AABB + per-tick step
   clamp + force abort + state watchdog) on every published setpoint.

Command path: the real subscriber controller ``/cartesian_impedance/pose_desired``
(``std_msgs/Float64MultiArray`` = ``[px,py,pz, R00..R22]`` row-major), the same
path used by ``hardware_insertion_node`` and ``robot_dance``.

USAGE (run INSIDE multipanda-container; see CLAUDE.md runbook)
-------------------------------------------------------------
Verify offline with NO robot, NO camera, NO ROS (synthetic vision + plant):
    python3 scripts/hw_peg_in_hole_vision.py --selftest

ALWAYS dry-run on the real arm first (reads state + perceives the socket,
publishes NOTHING):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_peg_in_hole_vision.py --socket-top-z 0.10 --dry-run

Perceive from a saved still instead of the live camera (offline check of the
homography + detector without touching the arm):
    python3 scripts/hw_peg_in_hole_vision.py \
        --image robo67_insertion/captures/c920_socket_newfloor.jpg \
        --socket-top-z 0.10 --dry-run

Live insertion (gentle, peg clamped, human at the e-stop):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 scripts/hw_peg_in_hole_vision.py --socket-top-z 0.10 --confirm

PREREQUISITE: a calibrated C920->base homography ``c920_homography.npz`` (from
``calibration_node``). If it is missing this script refuses to run and never
moves the arm -- run the calibration first.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

# Make ``import robo67_insertion`` work even without PYTHONPATH set: the package
# root is the parent of this scripts/ directory.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import numpy as np  # noqa: E402

# All of these are PURE (numpy/cv2/stdlib only -- NO rclpy at import time), so
# the script (incl. --selftest) imports fine on a host without ROS. rclpy is
# only pulled in lazily inside hardware_insertion_node.run_ros / the live grab.
from robo67_insertion.lib.hole_detect import (  # noqa: E402
    HoleParams,
    WhiteCubeParams,
    WhiteSocketParams,
    detect_holes,
    detect_sockets,
    detect_white_cubes,
)
from robo67_insertion.lib.pixel_mapping import (  # noqa: E402
    HomographyMappingAdapter,
    MappingContext,
    PixelObservation,
)
from robo67_insertion.config_schema import real_arm_workspace_aabb_flat  # noqa: E402
from robo67_insertion.nodes import hardware_insertion_node as hin  # noqa: E402


DEFAULT_HOMOGRAPHY = os.path.join(_PKG_ROOT, "config", "c920_homography.npz")
DEFAULT_CONFIG = os.path.join(_PKG_ROOT, "config", "robo67.yaml")


# ---------------------------------------------------------------------------
# Vision front-end (pure: no ROS). Detect the socket hole in one or more frames
# and map the best detection to robot-base XY through the calibrated homography.
# ---------------------------------------------------------------------------

def build_detector(args):
    """Return a callable ``img -> list[Hole]`` for the chosen detector.

    ``cube`` (default) detects the white cube CENTROID
    (:func:`detect_white_cubes`) -- robust to overexposure and the feature the
    socket-proxy homography is calibrated against (keep only the socket in view).
    ``white`` detects the bore (:func:`detect_sockets`); ``dark`` is the legacy
    dark-hole detector (:func:`detect_holes`, tunable via --dark-max etc).
    """
    if args.detector == "cube":
        return lambda img: detect_white_cubes(img, WhiteCubeParams())
    if args.detector == "white":
        return lambda img: detect_sockets(img, WhiteSocketParams())
    params = HoleParams(
        min_radius_px=args.min_radius,
        max_radius_px=args.max_radius,
        dark_max_value=args.dark_max,
        min_circularity=args.min_circularity,
    )
    return lambda img: detect_holes(img, params)


def _grab_frames_topic(args):
    """Grab N frames by SUBSCRIBING to the camera_publisher's compressed feed.

    This is the preferred path: the logging ``camera_publisher`` node OWNS the
    /dev/videoN device (only one process may open a V4L2 device) and streams it;
    every consumer -- detectors, dashboard, and this insertion runner -- just
    subscribes. No device contention. Inits/​shuts down its own rclpy context so
    the later ``hardware_insertion_node.run_ros`` can re-init cleanly.
    """
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    own_ctx = not rclpy.ok()
    if own_ctx:
        rclpy.init()
    node = Node("hw_peg_vision_grab")
    got = []

    def _cb(msg):
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            got.append(img)

    # BEST_EFFORT to match the camera_publisher's sensor-data QoS (a RELIABLE
    # subscriber would be QoS-incompatible and receive nothing).
    node.create_subscription(CompressedImage, args.camera_topic, _cb,
                             qos_profile_sensor_data)
    want = max(1, int(args.frames))
    t0 = time.time()
    while len(got) < want and time.time() - t0 < args.camera_timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if own_ctx:
        rclpy.shutdown()
    if not got:
        return [], (f"no frames on camera topic {args.camera_topic} within "
                    f"{args.camera_timeout:.0f}s -- is camera_publisher running?")
    return got[:want], None


def _grab_frames(args):
    """Return a list of BGR frames to run detection on.

    ``--image`` -> a saved still; ``--source topic`` (default) -> subscribe to the
    camera_publisher feed; ``--source device`` -> open the C920 directly (only
    works when no camera_publisher owns the device).
    """
    if args.image:
        import cv2

        img = cv2.imread(args.image)
        if img is None:
            return [], f"could not read --image {args.image!r}"
        return [img], None

    if args.source == "topic":
        return _grab_frames_topic(args)

    # Direct-device path: import the GStreamer grabber lazily (its module imports
    # rclpy at module load, which we only want inside the container/live run).
    from robo67_insertion.nodes.socket_detector_node import grab_frame_gst

    frames = []
    for _ in range(max(1, int(args.frames))):
        f = grab_frame_gst(args.c920_device, exposure=args.exposure)
        if f is not None:
            frames.append(f)
        time.sleep(0.05)
    if not frames:
        return [], f"no frames grabbed from C920 device {args.c920_device}"
    return frames, None


def perceive_socket_xy(frames, mapper, detect_fn):
    """Detect the socket per frame, map to base XY, return median + detail.

    ``detect_fn`` maps a BGR image to a list of :class:`Hole` (see
    :func:`build_detector`). Returns ``(base_xy (2,), detections)`` where
    ``detections`` lists ``(u, v, radius_px, score, base_x, base_y)`` for every
    frame that yielded a detection, or ``(None, [])`` if none did.
    """
    detections = []
    for img in frames:
        holes = detect_fn(img)
        if not holes:
            continue
        h = holes[0]  # already sorted best (roundest) first
        bx, by = mapper.map_xy(PixelObservation(h.u, h.v), MappingContext())
        detections.append((h.u, h.v, h.radius_px, h.score, float(bx), float(by)))
    if not detections:
        return None, []
    base_xy = np.array([
        statistics.median(d[4] for d in detections),
        statistics.median(d[5] for d in detections),
    ], float)
    return base_xy, detections


def load_mapper(homography_path):
    """Load the calibrated homography and return a HomographyMappingAdapter."""
    data = np.load(homography_path)
    return HomographyMappingAdapter(data["H"])


# ---------------------------------------------------------------------------
# Hand off the perceived socket to the proven real-arm insertion loop. We build
# the hardware node's own argparse Namespace (so every field it reads exists and
# defaults to the node's value) and override only what this script exposes.
# ---------------------------------------------------------------------------

def build_insertion_args(args, socket_xyz):
    ns = hin.build_parser().parse_args([])
    ns.selftest = False
    ns.nudge = None
    ns.socket_from_current = False
    ns.socket_xyz = [float(socket_xyz[0]), float(socket_xyz[1]), float(socket_xyz[2])]

    # behaviour / safety knobs this script forwards
    ns.dry_run = args.dry_run
    ns.dry_run_seconds = args.dry_run_seconds
    ns.confirm = args.confirm
    ns.countdown = args.countdown

    ns.topic = args.topic
    ns.state_topic = args.state_topic
    ns.recovery_srv = args.recovery_srv

    ns.rate = args.rate
    ns.v_max = args.v_max
    ns.standoff = args.standoff
    ns.pos_stiff = args.pos_stiff
    ns.approach_tol = args.approach_tol
    ns.contact_fz = args.contact_fz
    ns.press_force = args.press_force
    ns.insert_press = args.insert_press
    ns.max_press_depth = args.max_press_depth
    ns.insert_depth = args.insert_depth
    ns.spiral_max_radius = args.spiral_max_radius

    ns.f_abort = args.f_abort
    ns.torque_abort = args.torque_abort
    ns.watchdog_s = args.watchdog_s
    ns.workspace_aabb = list(args.workspace_aabb)

    # release-on-insert: open the gripper to leave the peg in the hole on the
    # z-drop (avoids the sustained seating push that crashes the bringup).
    ns.release_on_insert = args.release_on_insert
    ns.insert_drop_trigger = args.insert_drop_trigger
    ns.gripper_ns = args.gripper_ns
    ns.gripper_open_width = args.gripper_open_width
    ns.gripper_speed = args.gripper_speed
    ns.retract_after = args.retract_after
    return ns


def run_live(args):
    if not os.path.exists(args.homography):
        print(f"ERROR: homography not found: {args.homography}", file=sys.stderr)
        print("       Run the C920->base calibration first, e.g.:", file=sys.stderr)
        print("         python3 -m robo67_insertion.nodes.calibration_node --ros-args \\",
              file=sys.stderr)
        print(f"             -p mode:=fit -p points_file:=<corr.csv> -p out:={args.homography}",
              file=sys.stderr)
        print("       Refusing to run (would have no base-frame socket XY).", file=sys.stderr)
        return 2

    if args.c920_device is None:
        from robo67_insertion.config_schema import load_config

        # device may be a stable by-id symlink path or a bare int index; pass
        # through unchanged (grab_frame_gst resolves either form).
        args.c920_device = load_config(args.config).camera.c920_device

    mapper = load_mapper(args.homography)
    aabb = np.array(args.workspace_aabb, float).reshape(3, 2)

    frames, err = _grab_frames(args)
    if err:
        print(f"ERROR: vision: {err} -- refusing to move.", file=sys.stderr)
        return 1
    base_xy, dets = perceive_socket_xy(frames, mapper, build_detector(args))
    if base_xy is None:
        print(f"ERROR: no socket detected ({args.detector}) in {len(frames)} frame(s) "
              "-- refusing to move.", file=sys.stderr)
        return 1

    socket = np.array([base_xy[0], base_xy[1], args.socket_top_z], float)

    print("=== perceived socket ===")
    print(f"frames used        : {len(dets)}/{len(frames)} with a detection")
    best = max(dets, key=lambda d: d[3])
    print(f"best pixel (u,v)    : ({best[0]:.1f}, {best[1]:.1f})  "
          f"r={best[2]:.1f}px score={best[3]:.3f}")
    print(f"socket XY (median)  : x={socket[0]:.4f} y={socket[1]:.4f} m")
    print(f"socket top Z (taught): z={socket[2]:.4f} m")

    if not np.all((socket >= aabb[:, 0]) & (socket <= aabb[:, 1])):
        print(f"ERROR: perceived socket {socket.tolist()} outside workspace AABB "
              f"{aabb.tolist()} -- refusing to move.", file=sys.stderr)
        return 1

    ins_args = build_insertion_args(args, socket)
    print("handing socket to the insertion loop "
          f"({'DRY-RUN' if args.dry_run else 'LIVE'}) ...")
    return hin.run_ros(ins_args)


# ---------------------------------------------------------------------------
# Offline self-test: NO ROS, NO camera, NO robot. Exercises the vision path on a
# synthetic socket image + a demo homography, then runs the insertion plant
# self-test (hardware_insertion_node.selftest). Verifies the perceived socket
# also wires cleanly into the insertion arg namespace.
# ---------------------------------------------------------------------------

def _synthetic_socket_image():
    """White cube on a DARK background with a bright recessed bore at (320, 240),
    matching the real white-on-dark socket the default detector targets."""
    import cv2

    img = np.full((480, 640, 3), 35, np.uint8)           # dark table/carpet
    cv2.rectangle(img, (290, 210), (350, 270), (252, 252, 252), -1)  # white cube top
    cv2.circle(img, (320, 240), 20, (150, 150, 150), 2)  # bore rim shadow (edge)
    cv2.circle(img, (320, 240), 18, (205, 205, 205), -1)  # bore bottom (bright < cube)
    return img


def _demo_homography():
    """A simple scale+offset homography: pixel (320, 240) -> base (0.45, 0.0)."""
    sx = sy = 5.0e-4  # m per pixel
    H = np.array([[sx, 0.0, 0.45 - 320 * sx],
                  [0.0, sy, 0.00 - 240 * sy],
                  [0.0, 0.0, 1.0]], float)
    return H


def selftest(args):
    print("=== peg_in_hole_vision self-test ===")
    mapper = HomographyMappingAdapter(_demo_homography())
    img = _synthetic_socket_image()
    base_xy, dets = perceive_socket_xy([img] * 3, mapper, build_detector(args))

    vision_ok = base_xy is not None and len(dets) == 3
    if vision_ok:
        u, v = dets[0][0], dets[0][1]
        vision_ok = abs(u - 320) < 6 and abs(v - 240) < 6
    if base_xy is not None:
        socket = np.array([base_xy[0], base_xy[1], args.socket_top_z], float)
        aabb = np.array(args.workspace_aabb, float).reshape(3, 2)
        inside = bool(np.all((socket >= aabb[:, 0]) & (socket <= aabb[:, 1])))
        # the perceived socket must wire cleanly into the insertion namespace
        ins_args = build_insertion_args(args, socket)
        wired_ok = (ins_args.socket_xyz[:2] == [float(socket[0]), float(socket[1])]
                    and not ins_args.socket_from_current and ins_args.nudge is None)
        print(f"socket detected    : pixel=({dets[0][0]:.1f},{dets[0][1]:.1f}) "
              f"-> base XY=({socket[0]:.4f},{socket[1]:.4f}) m  inside_aabb={inside}")
    else:
        print("socket detected    : NONE")
        inside = wired_ok = False

    vision_ok = bool(vision_ok and inside and wired_ok)
    print(f"vision check       : {'PASS' if vision_ok else 'FAIL'}")
    print("--- delegating to insertion plant self-test ---")
    insertion_rc = hin.selftest(hin.build_parser().parse_args([]))

    ok = vision_ok and insertion_rc == 0
    print("RESULT             :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser():
    ap = argparse.ArgumentParser(
        description="End-to-end peg-in-hole on the real arm with C920 vision feedback.")
    ap.add_argument("--selftest", action="store_true",
                    help="offline vision + insertion plant test (no ROS, no camera)")

    # vision
    ap.add_argument("--homography", default=DEFAULT_HOMOGRAPHY,
                    help="path to c920_homography.npz from calibration")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help="robo67 config (used for the C920 device number)")
    ap.add_argument("--image", default="",
                    help="detect on this still image instead of the live C920")
    ap.add_argument("--source", choices=["topic", "device"], default="topic",
                    help="topic (default) = subscribe to the camera_publisher feed (no device "
                         "contention); device = open the C920 directly (needs the device free)")
    ap.add_argument("--camera-topic",
                    default="/robo67/camera/overhead/image_raw/compressed",
                    help="overhead camera CompressedImage topic to subscribe to (--source topic)")
    ap.add_argument("--camera-timeout", type=float, default=6.0,
                    help="seconds to wait for frames on --camera-topic before giving up")
    ap.add_argument("--c920-device", type=str, default=None,
                    help="overhead C920 device: a by-id symlink/path or a bare "
                         "/dev/video index (default: from config; used only by --source device)")
    ap.add_argument("--detector", choices=["cube", "white", "dark"], default="cube",
                    help="cube = white cube centroid (default; matches the socket-proxy "
                         "calibration); white = bore detector; dark = legacy dark-hole")
    ap.add_argument("--exposure", type=int, default=100,
                    help="lock C920 manual exposure (~40-120) so the white socket "
                         "doesn't overexpose; the live grab passes this through")
    ap.add_argument("--frames", type=int, default=5,
                    help="number of live frames to fuse (per-axis median)")
    ap.add_argument("--socket-top-z", type=float, default=None,
                    help="taught socket-top Z in base frame (m); REQUIRED for a live/dry run")

    # dark-hole detection tuning (used only when --detector dark; match HoleParams)
    ap.add_argument("--dark-max", type=int, default=HoleParams.dark_max_value,
                    help="pixels darker than this are hole candidates")
    ap.add_argument("--min-circularity", type=float, default=HoleParams.min_circularity)
    ap.add_argument("--min-radius", type=float, default=HoleParams.min_radius_px)
    ap.add_argument("--max-radius", type=float, default=HoleParams.max_radius_px)

    # insertion / safety (forwarded to hardware_insertion_node; defaults mirror it)
    ap.add_argument("--dry-run", action="store_true",
                    help="perceive socket + compute setpoints but publish NOTHING")
    ap.add_argument("--dry-run-seconds", type=float, default=20.0)
    ap.add_argument("--confirm", action="store_true", help="prompt YES before motion")
    ap.add_argument("--countdown", type=int, default=3)

    ap.add_argument("--topic", default="/cartesian_impedance/pose_desired")
    ap.add_argument("--state-topic", default="/franka_robot_state_broadcaster/robot_state")
    ap.add_argument("--recovery-srv",
                    default="/panda_error_recovery_service_server/error_recovery")

    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--v-max", type=float, default=0.03, help="command speed cap (m/s)")
    ap.add_argument("--standoff", type=float, default=0.05)
    ap.add_argument("--pos-stiff", type=float, default=200.0,
                    help="MUST match the controller's pos_stiff (real controller is 2000)")
    ap.add_argument("--approach-tol", type=float, default=0.006,
                    help="MOVE_ABOVE 'arrived' tol (m); set >= the controller stiction deadband")
    ap.add_argument("--contact-fz", type=float, default=4.0)
    ap.add_argument("--press-force", type=float, default=3.0)
    ap.add_argument("--insert-press", type=float, default=6.0)
    ap.add_argument("--max-press-depth", type=float, default=0.05)
    ap.add_argument("--insert-depth", type=float, default=0.03)
    ap.add_argument("--spiral-max-radius", type=float, default=0.012)

    ap.add_argument("--f-abort", type=float, default=20.0)
    ap.add_argument("--torque-abort", type=float, default=5.0,
                    help="abort cap on each moment axis (Nm); raise for a lateral spiral search")
    ap.add_argument("--watchdog-s", type=float, default=0.25)
    ap.add_argument("--workspace-aabb", type=float, nargs=6,
                    default=real_arm_workspace_aabb_flat(),
                    help="xmin xmax ymin ymax zmin zmax (m); default = "
                         "config_schema.REAL_ARM_WORKSPACE_AABB")

    # release-on-insert (open gripper on the bore z-drop, then retract empty)
    ap.add_argument("--release-on-insert", action="store_true",
                    help="open the gripper to leave the peg in the hole on insertion")
    ap.add_argument("--insert-drop-trigger", type=float, default=0.004,
                    help="release when EE z drops this far (m) below the DESCEND contact_z hole-top")
    ap.add_argument("--gripper-ns", default="/panda_gripper")
    ap.add_argument("--gripper-open-width", type=float, default=0.08)
    ap.add_argument("--gripper-speed", type=float, default=0.1)
    ap.add_argument("--retract-after", type=float, default=0.06)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        if args.socket_top_z is None:
            args.socket_top_z = 0.10  # representative value for the offline plant
        return selftest(args)
    if args.socket_top_z is None:
        print("ERROR: --socket-top-z is required for a live/dry run (an overhead "
              "camera cannot measure socket-top Z).", file=sys.stderr)
        return 2
    return run_live(args)


if __name__ == "__main__":
    raise SystemExit(main())
