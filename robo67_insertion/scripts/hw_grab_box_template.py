#!/usr/bin/env python3
"""Grab a fresh overhead frame and crop the detected box as a NEW ORB template.

Quick field utility (no robot motion): subscribe to the overhead camera, detect
the box with the CURRENT template, crop a padded bbox of the matched box, back up
the old template, and write the new one.

IMPORTANT: the box-frame port offset (``config/port_offset.npz``) is anchored to
the template's projected corners, so changing the template would otherwise
invalidate it. To avoid a re-jog, this tool RE-ANCHORS the offset onto the new
template IN THE SAME FRAME: it recovers the current physical port point from the
OLD detection + old offset, then expresses it in the NEW template's box frame and
overwrites the offset. (Skip with ``--no-migrate-offset``.)

Run inside multipanda-container (sourced, ROS_DOMAIN_ID=1, ROS_LOCALHOST_ONLY=1):
    python3 scripts/hw_grab_box_template.py
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_SCRIPTS_DIR)
for _p in (_SCRIPTS_DIR, _PKG_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402

from robo67_insertion.lib.box_detect import BoxOrbParams, OrbBoxMatcher  # noqa: E402
from robo67_insertion.lib.port_offset import (  # noqa: E402
    box_frame_base,
    map_corners_to_base,
    port_base_from_box,
    teach_port_offset,
)
from robo67_insertion.lib.pixel_mapping import MappingContext, PixelObservation  # noqa: E402
from hw_cable_insertion_vision import (  # noqa: E402
    DEFAULT_HOMOGRAPHY,
    DEFAULT_PORT_OFFSET,
    DEFAULT_TEMPLATE,
    load_port_offset,
)
from hw_cable_insertion_vision import build_parser as runner_parser  # noqa: E402
from hw_peg_in_hole_vision import _grab_frames, load_mapper  # noqa: E402


def _detect(frame, template_bgr, min_inliers):
    m = OrbBoxMatcher(template_bgr, BoxOrbParams(min_inliers=min_inliers))
    boxes = m.detect(frame)
    return boxes[0] if boxes else None


def main(argv=None):
    import cv2

    ap = argparse.ArgumentParser(description="Crop a fresh ORB box template + re-anchor the offset.")
    ap.add_argument("--template", default=DEFAULT_TEMPLATE, help="current template (also default --out)")
    ap.add_argument("--out", default=None, help="output template path (default: overwrite --template, backed up)")
    ap.add_argument("--homography", default=DEFAULT_HOMOGRAPHY)
    ap.add_argument("--port-offset", default=DEFAULT_PORT_OFFSET)
    ap.add_argument("--pad", type=float, default=0.12, help="bbox padding fraction around the box")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--camera-topic", default="/robo67/camera/overhead/image_raw/compressed")
    ap.add_argument("--camera-timeout", type=float, default=6.0)
    ap.add_argument("--orb-min-inliers", type=int, default=BoxOrbParams.min_inliers)
    ap.add_argument("--snapshot", default=os.path.join(_PKG_ROOT, "captures", "overhead_template_src.jpg"))
    ap.add_argument("--migrate-offset", action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    # minimal namespace for the shared topic frame-grab
    g = runner_parser().parse_args([])
    g.source, g.image = "topic", ""
    g.camera_topic, g.camera_timeout, g.frames = args.camera_topic, args.camera_timeout, args.frames
    frames, err = _grab_frames(g)
    if err:
        print(f"ERROR: vision: {err}", file=sys.stderr); return 1
    frame = frames[-1]
    os.makedirs(os.path.dirname(os.path.abspath(args.snapshot)), exist_ok=True)
    cv2.imwrite(args.snapshot, frame)
    print(f"source frame snapshot -> {args.snapshot}")

    old_tmpl = cv2.imread(args.template)
    if old_tmpl is None:
        print(f"ERROR: current template not found: {args.template}", file=sys.stderr); return 1
    b_old = _detect(frame, old_tmpl, args.orb_min_inliers)
    if b_old is None:
        print("ERROR: current template did not detect the box in this frame -- "
              "re-center / clear the view and retry.", file=sys.stderr); return 1
    print(f"OLD template detect : inliers={b_old.score:.0f}")

    # crop a padded axis-aligned bbox of the matched (tight) box
    corners = np.asarray(b_old.corners, float)
    (x0, y0), (x1, y1) = corners.min(0), corners.max(0)
    w, h = x1 - x0, y1 - y0
    H, W = frame.shape[:2]
    cx0, cy0 = int(max(0, x0 - args.pad * w)), int(max(0, y0 - args.pad * h))
    cx1, cy1 = int(min(W, x1 + args.pad * w)), int(min(H, y1 + args.pad * h))
    crop = frame[cy0:cy1, cx0:cx1].copy()
    if crop.shape[0] < 20 or crop.shape[1] < 20:
        print("ERROR: crop too small -- detection bbox degenerate.", file=sys.stderr); return 1

    out = args.out or args.template
    if os.path.exists(out):
        shutil.copy(out, out + ".bak")
        print(f"backed up old template -> {out}.bak")
    cv2.imwrite(out, crop)
    print(f"NEW template saved   -> {out}  ({crop.shape[1]}x{crop.shape[0]}px)")

    # sanity: the new template must still detect the box in this frame
    b_new = _detect(frame, cv2.imread(out), args.orb_min_inliers)
    if b_new is None:
        print("WARNING: new template did NOT re-detect in this frame; keeping it but "
              "verify with a dry-run. (old template still at *.bak)", file=sys.stderr)
        return 1
    print(f"NEW template detect : inliers={b_new.score:.0f}")

    # re-anchor the taught offset onto the new template frame (no re-jog)
    if args.migrate_offset:
        offset = load_port_offset(args.port_offset)
        if offset is None:
            print("note: no port_offset.npz to migrate (teach not done yet).")
        elif not os.path.exists(args.homography):
            print(f"WARNING: homography {args.homography} missing -> cannot migrate offset; "
                  "re-teach required.", file=sys.stderr)
        else:
            mapper = load_mapper(args.homography)

            def _map(u, v):
                return mapper.map_xy(PixelObservation(u, v), MappingContext())

            old_corners_base = map_corners_to_base(b_old.template_corners, _map)
            new_corners_base = map_corners_to_base(b_new.template_corners, _map)
            port_base = port_base_from_box(old_corners_base, offset)  # current physical port
            new_offset = teach_port_offset(new_corners_base, port_base)
            chk = port_base_from_box(new_corners_base, new_offset)
            data = dict(np.load(args.port_offset))
            shutil.copy(args.port_offset, args.port_offset + ".bak")
            data["offset_box"] = np.asarray(new_offset, float)
            np.savez(args.port_offset, **data)
            print(f"offset re-anchored   -> {args.port_offset} (backed up *.bak)")
            print(f"  port (base)        : ({port_base[0]:.4f}, {port_base[1]:.4f}) m  "
                  f"[preserved across template swap]")
            print(f"  new offset (box fr): dx={new_offset[0]*100:+.2f} dy={new_offset[1]*100:+.2f} cm  "
                  f"(re-anchor residual {np.linalg.norm(chk - port_base)*1000:.3f} mm)")

    print("\nNext (no motion): python3 scripts/hw_cable_insertion_vision.py "
          "--box-top-z 0.211 --dry-run --save-overlay captures/port_overlay.jpg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
