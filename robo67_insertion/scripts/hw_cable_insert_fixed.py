#!/usr/bin/env python3
"""Hardcoded cable seat for the demo video -- no vision, fixed taught pose.

The operator manually positioned the gripper with the cable SEATED in the port;
that EE flange pose was logged once from ``FrankaState`` and hardcoded here as
:data:`SEAT_XYZ`. From ANYWHERE in the frame this script hands the seat pose to
the proven real-arm insertion loop
(:func:`robo67_insertion.nodes.hardware_insertion_node.run_ros`):

    MOVE_ABOVE (``--standoff`` above the seat point, tool-down)
      -> DESCEND_TO_CONTACT  (force-probe down)
      -> SEARCH_SPIRAL       (find the port mouth)
      -> PUSH_INSERT         (bounded seat push)   [kept GRIPPED, NO release]

No box detection / no overhead camera. The whole safety envelope (workspace
AABB, per-tick step + velocity cap, force/torque abort, reflex recovery,
watchdog) and telemetry come from ``run_ros`` unchanged.

Captured 2026-06-26 on the real arm: cable seated, tool-down dot 0.998,
quat(xyzw) ~ (0.976, -0.214, 0.033, -0.001).

USAGE (inside multipanda-container; see CLAUDE.md runbook)
----------------------------------------------------------
Dry run (NO publish to the arm; prints the plan, exercises the loop):
    PYTHONPATH=/host/Code/robo67_cable_insertion/robo67_insertion \
    python3 scripts/hw_cable_insert_fixed.py --dry-run

Live (gentle, human at the e-stop; arm may start ANYWHERE in the frame):
    PYTHONPATH=/host/Code/robo67_cable_insertion/robo67_insertion \
    python3 scripts/hw_cable_insert_fixed.py --confirm
"""
from __future__ import annotations

import argparse
import os
import sys

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_SCRIPTS_DIR)
for _p in (_SCRIPTS_DIR, _PKG_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# rclpy is imported LAZILY inside run_ros, so importing this stays ROS-free.
from robo67_insertion.nodes import hardware_insertion_node as hin  # noqa: E402

# EE flange pose (base frame, metres) with the cable SEATED in the port,
# logged once from FrankaState o_t_ee. This is the insertion target.
SEAT_XYZ = (0.45013, -0.09317, 0.2088)


def build_insertion_args(seat_xyz, a):
    """Fill the ``hardware_insertion_node`` namespace for a SEAT-while-gripped run
    at ``seat_xyz`` (defaults come from ``hin.build_parser``; we override the
    target + the seat-tuned, bounded force knobs and DISABLE gripper release)."""
    ns = hin.build_parser().parse_args([])
    ns.selftest = False
    ns.nudge = None
    ns.socket_from_current = False
    ns.socket_xyz = [float(seat_xyz[0]), float(seat_xyz[1]), float(seat_xyz[2])]

    ns.dry_run = a.dry_run
    ns.confirm = a.confirm
    ns.countdown = a.countdown

    ns.standoff = a.standoff          # "slightly above" the seat to start the descent
    ns.v_max = a.v_max
    ns.pos_stiff = a.pos_stiff         # MUST match the running controller
    ns.approach_tol = a.approach_tol
    ns.contact_fz = a.contact_fz
    ns.press_force = a.press_force
    ns.insert_press = a.insert_press   # bounded seat push
    ns.insert_depth = a.insert_depth   # small commanded seat depth
    ns.max_press_depth = a.max_press_depth
    ns.spiral_max_radius = a.spiral_max_radius
    ns.f_abort = a.f_abort
    ns.torque_abort = a.torque_abort   # hard moment cap (seat-while-gripped guard)
    ns.watchdog_s = a.watchdog_s       # tolerance for FrankaState delivery hiccups

    # FORCE MODE (ported from jearningers / ADR-0002): regulate a constant gentle
    # axial PRESS (admittance) during SEARCH_SPIRAL + PUSH_INSERT so the spiral
    # presses INTO the face instead of skating over it, and detect the seat from
    # the force-slacken + confirmed descent. ON by default for this demo.
    ns.force_mode = a.force_mode
    ns.search_press = a.search_press
    ns.k_adm = a.k_adm
    ns.adm_v_cap = a.adm_v_cap
    ns.adm_max_force = a.adm_max_force
    ns.ramp_s = a.ramp_s
    ns.fz_filter_alpha = a.fz_filter_alpha
    ns.slacken_frac = a.slacken_frac
    ns.confirm_drop = a.confirm_drop
    ns.confirm_window = a.confirm_window
    ns.no_spiral_freeze = a.no_spiral_freeze
    ns.settle_s = a.settle_s

    ns.release_on_insert = False       # KEEP GRIPPED -- cable stays clamped, pushed to seat
    return ns


def build_parser():
    ap = argparse.ArgumentParser(
        description="Hardcoded cable seat: move above a taught seated pose, then spiral + seat.")
    ap.add_argument("--seat-xyz", type=float, nargs=3, default=list(SEAT_XYZ),
                    help="taught seated EE flange pose (m); default = the logged SEAT_XYZ")
    ap.add_argument("--standoff", type=float, default=0.06,
                    help="height above the seat point to start the descent (m)")
    ap.add_argument("--pos-stiff", type=float, default=2000.0,
                    help="MUST match the running controller's translational stiffness")
    ap.add_argument("--approach-tol", type=float, default=0.015)
    ap.add_argument("--contact-fz", type=float, default=4.0)
    ap.add_argument("--press-force", type=float, default=18.0)
    ap.add_argument("--insert-press", type=float, default=10.0,
                    help="bounded seat push force (N); keep modest -- a sustained hard "
                         "push trips the firmware reflex on the soft controller")
    ap.add_argument("--insert-depth", type=float, default=0.008,
                    help="small commanded seat depth (m) for the gripped connector")
    ap.add_argument("--max-press-depth", type=float, default=0.02)
    ap.add_argument("--spiral-max-radius", type=float, default=0.02)
    ap.add_argument("--torque-abort", type=float, default=10.0,
                    help="hard external-moment cap (Nm) -- the seat-while-gripped guard")
    ap.add_argument("--f-abort", type=float, default=20.0)
    ap.add_argument("--watchdog-s", type=float, default=0.25,
                    help="hold if FrankaState is older than this (s); raise to tolerate "
                         "delivery hiccups on a busy domain")
    ap.add_argument("--v-max", type=float, default=0.03, help="command speed cap (m/s)")
    ap.add_argument("--countdown", type=int, default=3)
    ap.add_argument("--dry-run", action="store_true",
                    help="run the loop but publish NOTHING to the arm")
    ap.add_argument("--confirm", action="store_true", help="prompt YES before motion")

    # FORCE MODE (ADR-0002, ported from jearningers). ON by default here so the
    # spiral PRESSES into the box face (admittance) rather than skating over it.
    ap.add_argument("--force-mode", action=argparse.BooleanOptionalAction, default=True,
                    help="regulate a constant gentle axial press during SEARCH_SPIRAL/"
                         "PUSH_INSERT and detect the seat from the force-slacken "
                         "(--no-force-mode for the old fixed-equilibrium spiral)")
    ap.add_argument("--search-press", type=float, default=6.0,
                    help="F* press-force target (N) during the spiral")
    ap.add_argument("--k-adm", type=float, default=0.0008, help="admittance gain (m/s per N)")
    ap.add_argument("--adm-v-cap", type=float, default=0.01,
                    help="axial equilibrium speed cap (m/s); keep <= --v-max")
    ap.add_argument("--adm-max-force", type=float, default=12.0,
                    help="soft clamp on the regulated force target (N)")
    ap.add_argument("--ramp-s", type=float, default=0.5,
                    help="ramp F* from contact force up to --search-press over this many s")
    ap.add_argument("--fz-filter-alpha", type=float, default=0.2,
                    help="EMA smoothing of the press estimate for slacken detection")
    ap.add_argument("--slacken-frac", type=float, default=0.4,
                    help="fraction of held press lost that counts as a slacken (seat cue)")
    ap.add_argument("--confirm-drop", type=float, default=0.003,
                    help="EE descent (m) after a slacken needed to confirm the seat")
    ap.add_argument("--confirm-window", type=float, default=1.0,
                    help="seconds after a slacken within which the descent must confirm")
    ap.add_argument("--no-spiral-freeze", action="store_true",
                    help="do NOT freeze the XY spiral while confirming a slacken")
    ap.add_argument("--settle-s", type=float, default=0.4,
                    help="seconds to hold the XY spiral after a slacken")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    ns = build_insertion_args(a.seat_xyz, a)
    fm = (f"FORCE-MODE search_press={a.search_press}N insert_press={a.insert_press}N "
          f"k_adm={a.k_adm} adm_v_cap={a.adm_v_cap} max_force={a.adm_max_force}N"
          if a.force_mode else "fixed-equilibrium spiral (no force mode)")
    print(f"[fixed-seat] seat(socket)={[round(v,4) for v in ns.socket_xyz]} "
          f"standoff={a.standoff} pos_stiff={a.pos_stiff} "
          f"insert_depth={a.insert_depth} torque_abort={a.torque_abort} "
          f"release_on_insert={ns.release_on_insert} {'DRY-RUN' if a.dry_run else 'LIVE'}")
    print(f"[fixed-seat] {fm}")
    return hin.run_ros(ns)


if __name__ == "__main__":
    raise SystemExit(main())
