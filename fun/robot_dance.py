#!/usr/bin/env python3
"""
robot_dance.py -- a (safe) little dance + wave routine for the Franka Panda.

Just for fun. Targets the Cartesian impedance controller used by Robo67:

    topic : /cartesian_impedance/pose_desired   (std_msgs/Float64MultiArray)
    data  : [px, py, pz, R00, R01, R02, R10, R11, R12, R20, R21, R22]
            (position in metres, orientation as a ROW-MAJOR 3x3 rotation matrix)

It anchors the choreography to wherever the arm currently is by reading the
end-effector pose from:

    topic : /franka_robot_state_broadcaster/robot_state   (franka_msgs/FrankaState)
    field : o_t_ee  (4x4 homogeneous transform, COLUMN-MAJOR, Franka convention)

All motion is generated as small, eased offsets *around* that anchor, with hard
per-step velocity/position clamps, so the arm can only ever wiggle gently near
where it started -- it can never command a far/violent jump.

------------------------------------------------------------------------------
RUN IN SIM FIRST (per the Robo67 rules -- prototype in MuJoCo, hardware last):

  1. Bring up the MuJoCo sim + controllers (multipanda_ros2).
  2. Make sure the Cartesian impedance controller that subscribes to
     /cartesian_impedance/pose_desired is *active*.
  3. ros2 run / python3 fun/robot_dance.py        # dances around current pose

Verify the math with NO robot and NO ROS at all:

     python3 fun/robot_dance.py --selftest

Useful flags:
  --routine spin      the big "breakdance": spin around the base, stand up
                      vertically, then spin the flange/"hand" on top
                      (default is "dance" -- the gentle wiggle above)
  --amp-scale 0.5     make every motion smaller (gentler)
  --time-scale 1.5    make everything slower
  --v-max 0.25        lower the hard linear speed cap (m/s)
  --loop              repeat forever until Ctrl-C
  --confirm           require typing YES before any motion (recommended on hw)
  --assume-home       run without reading state (assumes arm is already at the
                      default anchor pose -- only use if you know it is!)
------------------------------------------------------------------------------
THE "spin" ROUTINE (--routine spin)

Expresses a spin-around / stand-tall / spin-the-hand breakdance purely through
the *Cartesian impedance* controller (we never touch the joint-position
controller -- it has known bad motor behaviour):

  * "spin around"  -> the end-effector orbits the base z-axis, which IS joint 1
                      rotating. NOTE: a real Panda CANNOT do a full 360 -- joint
                      1 is limited to +/-166 deg -- so the "spin" is the widest
                      safe back-and-forth sweep, not a continuous turn.
  * "stand up"     -> the arm rises to a tall, near-vertical pose.
  * "spin hands"   -> at the top, the flange ("the hand") yaw spins back and
                      forth (again capped by joint 7's +/-166 deg range).

Unlike the gentle dance it commands *absolute* base-frame poses (so it can swing
far from the start), but a dedicated SpinLimiter still enforces the reachable
sphere, a table floor, the px/R22 controller quirks, and hard speed caps. It
ramps smoothly from wherever the arm currently is. Prototype in MuJoCo first.
"""

import argparse
import math
import sys
import time

# ----------------------------------------------------------------------------
# Pure-python math (no numpy) so the choreography can be self-tested anywhere.
# A 3x3 matrix is a list of 3 rows, each a list of 3 floats.
# ----------------------------------------------------------------------------

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]

def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]

def matmul3(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]

def mat_rowmajor(R):
    return [R[i][j] for i in range(3) for j in range(3)]

# Default "pointing straight down" orientation (rotation of pi about base X).
# R22 = -1 (non-zero -> controller will accept the orientation update).
R_DOWN = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]

DEFAULT_ANCHOR_P = (0.40, 0.0, 0.45)

# ----------------------------------------------------------------------------
# Easing helpers -- all start and end with ~zero velocity for smooth motion.
# ----------------------------------------------------------------------------

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def smoothstep(x):
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)

def window(x):
    # 0 at x=0 and x=1, smooth peak of 1 at x=0.5
    return math.sin(math.pi * clamp(x, 0.0, 1.0))

# ----------------------------------------------------------------------------
# Choreography. Each segment returns an offset (dx, dy, dz) in metres relative
# to the anchor, and (roll, pitch, yaw) in radians applied in the base frame as
# R = Rz(yaw) @ Ry(pitch) @ Rx(roll) @ R_anchor.
# Segments are stitched so position/orientation are continuous at every seam.
# ----------------------------------------------------------------------------

HOVER_DZ = 0.12  # how high above the anchor the "ready to dance" hover sits

def _settle(t, dur):
    return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

def _rise(t, dur):
    s = smoothstep(t / dur)
    return (0.0, 0.0, HOVER_DZ * s), (0.0, 0.0, 0.0)

def _wave(t, dur):
    w = window(t / dur)
    f = 0.6
    roll = 0.50 * w * math.sin(2 * math.pi * f * t)
    dy = 0.05 * w * math.sin(2 * math.pi * f * t)
    return (0.0, dy, HOVER_DZ), (roll, 0.0, 0.0)

# ---------------------------------------------------------------------------
# Breakdance moves. Each is window-enveloped so it begins and ends at the hover
# pose -> every segment seam stays continuous. Amplitudes/frequencies are tuned
# to peak ~0.35 m/s and ~1.9 rad/s, just under the SafetyLimiter caps, so the
# motion is wild but still crisp (the limiter clamps anything that isn't).
# ---------------------------------------------------------------------------

def _groove(t, dur):
    w = window(t / dur)
    f1 = 0.35
    dx = 0.06 * w * math.sin(2 * math.pi * f1 * t + math.pi / 2)
    dy = 0.12 * w * math.sin(2 * math.pi * f1 * t)
    dz = 0.05 * w * math.sin(2 * math.pi * (2 * f1) * t)      # figure-8 in y-z
    yaw = 0.30 * w * math.sin(2 * math.pi * f1 * t)
    pitch = 0.18 * w * math.sin(2 * math.pi * (2 * f1) * t)
    return (dx, dy, HOVER_DZ + dz), (0.0, pitch, yaw)

def _windmill(t, dur):
    w = window(t / dur); f = 0.40
    a = 2 * math.pi * f * t
    return (0.15 * w * math.cos(a), 0.15 * w * math.sin(a), HOVER_DZ), \
           (0.55 * w * math.sin(a), 0.35 * w * math.cos(a), 0.0)

def _spin(t, dur):
    w = window(t / dur); f = 0.50
    yaw = 0.60 * w * math.sin(2 * math.pi * f * t)
    dz = 0.06 * w * math.sin(2 * math.pi * 2 * f * t)
    return (0.0, 0.0, HOVER_DZ + dz), (0.0, 0.0, yaw)

def _pop(t, dur):
    w = window(t / dur); f = 0.55
    dz = 0.10 * w * math.sin(2 * math.pi * f * t)
    pitch = 0.45 * w * math.sin(2 * math.pi * f * t)
    return (0.0, 0.0, HOVER_DZ + dz), (0.0, pitch, 0.0)

def _slide(t, dur):
    w = window(t / dur); f = 0.40
    dx = 0.15 * w * math.sin(2 * math.pi * f * t)
    roll = 0.45 * w * math.sin(2 * math.pi * f * t + math.pi / 2)
    return (dx, 0.0, HOVER_DZ), (roll, 0.0, 0.0)

def _headspin(t, dur):
    w = window(t / dur); f = 0.55
    a = 2 * math.pi * f * t
    return (0.07 * w * math.cos(a), 0.07 * w * math.sin(a),
            HOVER_DZ + 0.03 * w * math.sin(2 * a)), \
           (0.0, 0.0, 0.55 * w * math.sin(a))

def _worm(t, dur):
    w = window(t / dur); f = 0.45
    dy = 0.11 * w * math.sin(2 * math.pi * f * t)
    dz = 0.04 * w * math.sin(2 * math.pi * 2 * f * t)
    pitch = 0.50 * w * math.sin(2 * math.pi * f * t + math.pi / 2)
    return (0.0, dy, HOVER_DZ + dz), (0.0, pitch, 0.0)

def _freeze(t, dur):
    g = window(t / dur)                                       # tilt-and-hold
    return (0.10 * g, 0.0, HOVER_DZ), (0.60 * g, 0.30 * g, 0.0)

def _return(t, dur):
    s = smoothstep(t / dur)
    return (0.0, 0.0, HOVER_DZ * (1.0 - s)), (0.0, 0.0, 0.0)

# One breakdance cycle (~64 s), repeated to fill the routine.
_CYCLE = [
    ("windmill",   10.0, _windmill),
    ("spin",        8.0, _spin),
    ("pop",         6.0, _pop),
    ("groove",      9.0, _groove),
    ("slide",       8.0, _slide),
    ("headspin",   10.0, _headspin),
    ("worm",        8.0, _worm),
    ("freeze",      5.0, _freeze),
]

def build_segments(total=360.0):
    """Intro (settle/rise/wave) -> breakdance cycle on repeat -> outro (return),
    sized to about `total` seconds, all at the normal pace."""
    intro = [("settle", 2.0, _settle), ("rise", 3.0, _rise), ("wave hello", 5.0, _wave)]
    outro_dur = 3.0
    segs = list(intro)
    acc = sum(d for _, d, _ in intro)
    budget = total - outro_dur
    i = 0
    while acc < budget - 0.5:
        name, dur, fn = _CYCLE[i % len(_CYCLE)]
        if acc + dur > budget:
            dur = round(budget - acc, 3)
        segs.append((name, dur, fn))
        acc += dur
        i += 1
    segs.append(("return", outro_dur, _return))
    return segs

# ~360 s of breakdance, at the normal pace (override with --time-scale/--amp-scale).
SEGMENTS = build_segments(360.0)


def choreography(t, time_scale=1.0, amp_scale=1.0):
    """Return (name, (dx,dy,dz), (roll,pitch,yaw)) at elapsed time t, or None
    once the routine is finished."""
    elapsed = 0.0
    for name, dur, fn in SEGMENTS:
        d = dur * time_scale
        if t < elapsed + d:
            dp, rpy = fn(t - elapsed, d)
            dp = tuple(c * amp_scale for c in dp)
            rpy = tuple(a * amp_scale for a in rpy)
            return name, dp, rpy
        elapsed += d
    return None


def total_duration(time_scale=1.0):
    return sum(dur for _, dur, _ in SEGMENTS) * time_scale


# ----------------------------------------------------------------------------
# The "spin" breakdance (--routine spin).
#
# This one speaks in *absolute* base-frame poses (px, py, pz) + base-frame
# (roll, pitch, yaw), NOT small offsets around an anchor -- so it can swing the
# arm right around the base. Each segment is built to be position- and
# orientation-continuous at the seams, and the whole thing is fed through the
# SpinLimiter (below) which still enforces reach / floor / speed caps.
#
# Reality check: a Franka Panda CANNOT spin a joint a full 360 deg. Joint 1 is
# +/-166 deg and joint 7 (the flange / "hand") is +/-166 deg, so every "spin"
# here is the widest safe back-and-forth SWEEP, not a continuous turn. We stay
# comfortably inside those limits.
# ----------------------------------------------------------------------------

ORBIT_R = 0.38        # orbit radius about the base z-axis (m)
ORBIT_H = 0.35        # orbit height above the base (m)
TALL_PX = 0.12        # x of the tall "standing" pose (kept > 0 for px quirk)
TALL_Z = 0.62         # z of the tall "standing" pose (m)
AZIM_MAX = 2.5        # base-spin sweep amplitude (rad) -- < joint-1 limit 2.8973
HAND_YAW_MAX = 2.5    # flange-spin sweep amplitude (rad) -- < joint-7 limit 2.8973
SPIN_HOME_P = (0.40, 0.0, 0.45)   # safe pose to settle back to at the end


def _lerp(a, b, s):
    return a + (b - a) * s


def _sp_ready(t, dur):
    # Hold at the orbit start (tool pointing down). The SpinLimiter ramps the
    # arm here from wherever it actually is, so this segment absorbs the approach.
    return (ORBIT_R, 0.0, ORBIT_H), (math.pi, 0.0, 0.0)


def _sp_spin(t, dur):
    # Orbit the base z-axis: azimuth 0 -> +max -> 0 -> -max -> 0 over the segment
    # (one full period, so it ends exactly where it started -> continuous seam).
    f = 1.0 / dur
    az = AZIM_MAX * math.sin(2 * math.pi * f * t)
    return (ORBIT_R * math.cos(az), ORBIT_R * math.sin(az), ORBIT_H), \
           (math.pi, 0.0, az)


def _sp_standup(t, dur):
    # Rise from the orbit pose to the tall, near-vertical pose; roll pi -> 0
    # rolls the tool from pointing-down to pointing-up as it stands.
    s = smoothstep(t / dur)
    return (_lerp(ORBIT_R, TALL_PX, s), 0.0, _lerp(ORBIT_H, TALL_Z, s)), \
           (_lerp(math.pi, 0.0, s), 0.0, 0.0)


def _sp_hands(t, dur):
    # Stand tall and spin the flange ("the hand"): yaw 0 -> +max -> 0 -> -max -> 0.
    f = 1.0 / dur
    yaw = HAND_YAW_MAX * math.sin(2 * math.pi * f * t)
    return (TALL_PX, 0.0, TALL_Z), (0.0, 0.0, yaw)


def _sp_return(t, dur):
    # Ease from the tall pose back to a safe home, tool rolling back down.
    s = smoothstep(t / dur)
    return (_lerp(TALL_PX, SPIN_HOME_P[0], s), 0.0, _lerp(TALL_Z, SPIN_HOME_P[2], s)), \
           (_lerp(0.0, math.pi, s), 0.0, 0.0)


SPIN_SEGMENTS = [
    ("ready",        5.0, _sp_ready),
    ("spin around", 16.0, _sp_spin),
    ("stand up",     6.0, _sp_standup),
    ("spin hands",  12.0, _sp_hands),
    ("return home",  6.0, _sp_return),
]


def spin_choreography(t, time_scale=1.0, amp_scale=1.0):
    """Return (name, (px,py,pz), (roll,pitch,yaw)) -- ABSOLUTE base-frame pose --
    at elapsed time t, or None once the routine is finished. amp_scale is accepted
    for signature parity with choreography() but is intentionally ignored here:
    the spin's amplitudes are pinned to stay inside the joint limits."""
    elapsed = 0.0
    for name, dur, fn in SPIN_SEGMENTS:
        d = dur * time_scale
        if t < elapsed + d:
            p, rpy = fn(t - elapsed, d)
            return name, p, rpy
        elapsed += d
    return None


def spin_total_duration(time_scale=1.0):
    return sum(dur for _, dur, _ in SPIN_SEGMENTS) * time_scale


# ----------------------------------------------------------------------------
# Safety: turn an offset into a clamped absolute target. This is the layer that
# guarantees the arm only ever moves gently, regardless of the choreography.
# ----------------------------------------------------------------------------

class SafetyLimiter:
    # offset box around the anchor (metres)
    DX = 0.18
    DY = 0.18
    DZ_LO = -0.12
    DZ_HI = 0.22
    # absolute floor so we never dive into the table (metres, base frame z)
    Z_FLOOR = 0.10
    # reachable radius from base origin (metres)
    R_MIN = 0.25
    R_MAX = 0.80
    # per-axis orientation limit (radians)
    RPY_MAX = 0.8

    def __init__(self, anchor_p, anchor_R, v_max=0.4, w_max=2.0, rate=100.0):
        self.p0 = list(anchor_p)
        self.R0 = anchor_R
        self.lin_step = v_max / rate           # max metres per tick (Euclidean)
        self.ang_step = w_max / rate           # max radians per tick (per axis)
        self.prev_p = None                     # last published [x, y, z]
        self.prev_rpy = None                   # last published [roll, pitch, yaw]

    def _goal(self, dp, rpy):
        """Apply absolute clamps (box / floor / reachable sphere) -> goal pose."""
        px = self.p0[0] + clamp(dp[0], -self.DX, self.DX)
        py = self.p0[1] + clamp(dp[1], -self.DY, self.DY)
        pz = self.p0[2] + clamp(dp[2], self.DZ_LO, self.DZ_HI)
        pz = max(pz, self.Z_FLOOR)
        r = math.sqrt(px * px + py * py + pz * pz)
        if r > self.R_MAX:
            s = self.R_MAX / r
            px, py, pz = px * s, py * s, pz * s
        rpy = [clamp(rpy[0], -self.RPY_MAX, self.RPY_MAX),
               clamp(rpy[1], -self.RPY_MAX, self.RPY_MAX),
               clamp(rpy[2], -self.RPY_MAX, self.RPY_MAX)]
        return [px, py, pz], rpy

    def target(self, dp, rpy):
        goal_p, goal_rpy = self._goal(dp, rpy)

        # --- rate limit position by Euclidean step (true linear-speed cap) ---
        if self.prev_p is None:
            pub_p = list(goal_p)
        else:
            vec = [goal_p[i] - self.prev_p[i] for i in range(3)]
            dist = math.sqrt(sum(c * c for c in vec))
            if dist > self.lin_step:
                k = self.lin_step / dist
                pub_p = [self.prev_p[i] + vec[i] * k for i in range(3)]
            else:
                pub_p = list(goal_p)
        self.prev_p = pub_p

        # --- rate limit orientation per axis (angular-speed cap) ---
        if self.prev_rpy is None:
            pub_rpy = list(goal_rpy)
        else:
            pub_rpy = [self.prev_rpy[i]
                       + clamp(goal_rpy[i] - self.prev_rpy[i],
                               -self.ang_step, self.ang_step)
                       for i in range(3)]
        self.prev_rpy = pub_rpy

        px, py, pz = pub_p
        if abs(px) < 1e-6:                 # px must stay non-zero (controller quirk)
            px = 1e-6

        R = matmul3(rot_z(pub_rpy[2]), matmul3(rot_y(pub_rpy[1]), rot_x(pub_rpy[0])))
        R = matmul3(R, self.R0)
        if abs(R[2][2]) < 1e-6:            # R22 must stay non-zero (controller quirk)
            R[2][2] = -1e-6

        r = math.sqrt(px * px + py * py + pz * pz)
        data = [px, py, pz] + mat_rowmajor(R)
        return data, r


def rpy_from_R(R):
    """Recover (roll, pitch, yaw) from R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (ZYX).
    Used to seed the SpinLimiter from the arm's current orientation so the very
    first command doesn't snap the wrist."""
    pitch = math.atan2(-R[2][0], math.sqrt(R[0][0] ** 2 + R[1][0] ** 2))
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return [roll, pitch, yaw]


class SpinLimiter:
    """Absolute-pose safety clamp for the big 'spin' routine.

    SafetyLimiter keeps the EE inside a small box around an anchor; that is too
    tight for a spin-around-the-base move. This limiter instead accepts ABSOLUTE
    base-frame targets so the arm can swing far -- but it still guarantees the
    arm stays reachable, above the table, inside the px/R22 controller quirks,
    and under hard linear/angular speed caps. Seed prev_p/prev_rpy with the
    arm's current pose so motion ramps in from where it actually is."""

    Z_FLOOR = 0.10        # never dive below this base-frame z (m)
    R_MIN = 0.12          # keep at least this far from the base origin (m)
    R_MAX = 0.80          # reachable radius from the base origin (m)

    def __init__(self, v_max=0.4, w_max=2.0, rate=100.0,
                 start_p=None, start_rpy=None):
        self.lin_step = v_max / rate           # max metres per tick (Euclidean)
        self.ang_step = w_max / rate           # max radians per tick (per axis)
        self.prev_p = list(start_p) if start_p is not None else None
        self.prev_rpy = list(start_rpy) if start_rpy is not None else None

    def _clamp_abs(self, p):
        px, py, pz = p
        pz = max(pz, self.Z_FLOOR)
        r = math.sqrt(px * px + py * py + pz * pz)
        if r > self.R_MAX:
            s = self.R_MAX / r
            px, py, pz = px * s, py * s, pz * s
        elif 1e-9 < r < self.R_MIN:
            s = self.R_MIN / r
            px, py, pz = px * s, py * s, pz * s
        return [px, py, pz]

    def target(self, p, rpy):
        goal_p = self._clamp_abs(p)

        # --- rate limit position by Euclidean step (true linear-speed cap) ---
        if self.prev_p is None:
            pub_p = list(goal_p)
        else:
            vec = [goal_p[i] - self.prev_p[i] for i in range(3)]
            dist = math.sqrt(sum(c * c for c in vec))
            if dist > self.lin_step:
                k = self.lin_step / dist
                pub_p = [self.prev_p[i] + vec[i] * k for i in range(3)]
            else:
                pub_p = list(goal_p)
        self.prev_p = pub_p

        # --- rate limit orientation per axis (angular-speed cap) ---
        if self.prev_rpy is None:
            pub_rpy = list(rpy)
        else:
            pub_rpy = [self.prev_rpy[i]
                       + clamp(rpy[i] - self.prev_rpy[i],
                               -self.ang_step, self.ang_step)
                       for i in range(3)]
        self.prev_rpy = pub_rpy

        px, py, pz = pub_p
        if abs(px) < 1e-6:                 # px must stay non-zero (controller quirk)
            px = 1e-6

        R = matmul3(rot_z(pub_rpy[2]), matmul3(rot_y(pub_rpy[1]), rot_x(pub_rpy[0])))
        if abs(R[2][2]) < 1e-6:            # R22 must stay non-zero (controller quirk)
            R[2][2] = -1e-6

        r = math.sqrt(px * px + py * py + pz * pz)
        data = [px, py, pz] + mat_rowmajor(R)
        return data, r


# ----------------------------------------------------------------------------
# Offline self-test -- no ROS, no robot. Verifies the trajectory stays inside
# limits and never trips the controller's px / R22 quirks.
# ----------------------------------------------------------------------------

def selftest(args):
    rate = args.rate
    dt = 1.0 / rate
    if args.routine == "spin":
        # Seed the limiter at the routine's own start pose so the self-test
        # measures the choreography's intrinsic speeds (a real run seeds from
        # the arm's current pose and the limiter ramps the approach at v_max).
        start_p, start_rpy = _sp_ready(0.0, 1.0)
        lim = SpinLimiter(v_max=args.v_max, w_max=args.w_max, rate=rate,
                          start_p=start_p, start_rpy=list(start_rpy))
        get_step = lambda t: spin_choreography(t, args.time_scale, args.amp_scale)
        total = spin_total_duration(args.time_scale)
    else:
        lim = SafetyLimiter(DEFAULT_ANCHOR_P, R_DOWN, v_max=args.v_max,
                            w_max=args.w_max, rate=rate)
        get_step = lambda t: choreography(t, args.time_scale, args.amp_scale)
        total = total_duration(args.time_scale)

    prev_p = None
    prev_rpy = None
    max_speed = 0.0
    max_wspeed = 0.0
    min_z = float("inf")
    max_r = 0.0
    min_r = float("inf")
    min_absR22 = float("inf")
    min_abspx = float("inf")
    n = 0

    t = 0.0
    while t <= total + dt:
        step = get_step(t)
        if step is None:
            break
        _, A, B = step
        data, r = lim.target(A, B)
        px, py, pz = data[0], data[1], data[2]
        cur_rpy = lim.prev_rpy
        if prev_p is not None:
            max_speed = max(max_speed, math.dist((px, py, pz), prev_p) / dt)
            max_wspeed = max(max_wspeed,
                             max(abs(cur_rpy[i] - prev_rpy[i]) for i in range(3)) / dt)
        prev_p = (px, py, pz)
        prev_rpy = list(cur_rpy)
        min_z = min(min_z, pz)
        max_r = max(max_r, r)
        min_r = min(min_r, r)
        min_absR22 = min(min_absR22, abs(data[11]))
        min_abspx = min(min_abspx, abs(px))
        n += 1
        t += dt

    L = type(lim)
    print("=== robot_dance self-test ===")
    print(f"routine           : {args.routine}")
    print(f"samples           : {n}")
    print(f"routine duration  : {total:.1f} s")
    print(f"max linear speed  : {max_speed:.3f} m/s   (cap {args.v_max})")
    print(f"max angular speed : {max_wspeed:.3f} rad/s   (cap {args.w_max})")
    print(f"min z (floor)     : {min_z:.3f} m   (floor {L.Z_FLOOR})")
    print(f"radius range      : {min_r:.3f}..{max_r:.3f} m   (allowed {L.R_MIN}..{L.R_MAX})")
    print(f"min |R22|         : {min_absR22:.3e}   (must be > 0)")
    print(f"min |px|          : {min_abspx:.3e}   (must be > 0)")

    ok = (max_speed <= args.v_max + 1e-6
          and max_wspeed <= args.w_max + 1e-6
          and min_z >= L.Z_FLOOR - 1e-9
          and max_r <= L.R_MAX + 1e-6
          and min_r >= L.R_MIN - 1e-6
          and min_absR22 > 0.0
          and min_abspx > 0.0)
    print("RESULT            :", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ----------------------------------------------------------------------------
# ROS 2 node (rclpy imported lazily so --selftest works without ROS).
# ----------------------------------------------------------------------------

def run_ros(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    try:
        from franka_msgs.msg import FrankaState
    except Exception:
        FrankaState = None

    class DanceNode(Node):
        def __init__(self):
            super().__init__("robot_dance")
            self.pub = self.create_publisher(Float64MultiArray, args.topic, 10)
            self.anchor_p = None
            self.anchor_R = None
            self._got_state = False
            if FrankaState is not None and not args.assume_home:
                self.sub = self.create_subscription(
                    FrankaState, args.state_topic, self._on_state, 10)

        def _on_state(self, msg):
            if self._got_state:
                return
            m = list(msg.o_t_ee)          # 4x4 column-major
            self.anchor_p = (m[12], m[13], m[14])
            self.anchor_R = [[m[0 + 4 * 0], m[0 + 4 * 1], m[0 + 4 * 2]],
                             [m[1 + 4 * 0], m[1 + 4 * 1], m[1 + 4 * 2]],
                             [m[2 + 4 * 0], m[2 + 4 * 1], m[2 + 4 * 2]]]
            self._got_state = True

    rclpy.init()
    node = DanceNode()

    # Acquire the anchor pose (current EE pose) before moving.
    if not args.assume_home and FrankaState is not None:
        node.get_logger().info(
            f"Waiting up to {args.state_timeout:.0f}s for {args.state_topic} ...")
        deadline = time.time() + args.state_timeout
        while time.time() < deadline and not node._got_state:
            rclpy.spin_once(node, timeout_sec=0.1)

    if node.anchor_p is None:
        if not args.assume_home:
            node.get_logger().error(
                "Could not read current pose. Refusing to move (an absolute "
                "command could jump violently). Re-run with --assume-home "
                "ONLY if the arm is already at the default anchor pose.")
            node.destroy_node()
            rclpy.shutdown()
            return 1
        node.anchor_p = DEFAULT_ANCHOR_P
        node.anchor_R = R_DOWN
        node.get_logger().warn(
            f"--assume-home: using default anchor {DEFAULT_ANCHOR_P}.")

    p0 = node.anchor_p
    node.get_logger().info(
        f"Anchor pose: x={p0[0]:.3f} y={p0[1]:.3f} z={p0[2]:.3f}")

    if args.routine == "spin":
        # Absolute base-frame poses, seeded from the current pose so the arm
        # ramps in smoothly. The big spin swings far from the anchor by design.
        lim = SpinLimiter(v_max=args.v_max, w_max=args.w_max, rate=args.rate,
                          start_p=node.anchor_p,
                          start_rpy=rpy_from_R(node.anchor_R))
        get_step = lambda t: spin_choreography(t, args.time_scale, args.amp_scale)
        total = spin_total_duration(args.time_scale)
        node.get_logger().warn(
            "ROUTINE=spin: BIG motion -- arm orbits the base, stands tall and "
            "spins the flange. Stays reachable / above the table / under "
            f"{args.v_max} m/s, but clear a WIDE area. Publishing to {args.topic}.")
    else:
        lim = SafetyLimiter(node.anchor_p, node.anchor_R, v_max=args.v_max,
                            w_max=args.w_max, rate=args.rate)
        get_step = lambda t: choreography(t, args.time_scale, args.amp_scale)
        total = total_duration(args.time_scale)
        node.get_logger().info(
            f"Motion stays within +/-{SafetyLimiter.DX} m of anchor, "
            f"speed <= {args.v_max} m/s. Publishing to {args.topic}.")

    if args.confirm:
        try:
            if input("Area clear, e-stop in hand? Type YES to dance: ").strip() != "YES":
                node.get_logger().info("Not confirmed. Exiting without moving.")
                node.destroy_node(); rclpy.shutdown(); return 0
        except EOFError:
            node.get_logger().error("No stdin for --confirm. Exiting."); 
            node.destroy_node(); rclpy.shutdown(); return 1

    for c in range(args.countdown, 0, -1):
        node.get_logger().info(f"Dancing in {c} ...")
        time.sleep(1.0)

    msg = Float64MultiArray()
    dt = 1.0 / args.rate

    last_name = None
    try:
        while rclpy.ok():
            t0 = time.time()
            while True:
                t = time.time() - t0
                step = get_step(t)
                if step is None:
                    break
                name, A, B = step
                if name != last_name:
                    node.get_logger().info(f"-> {name}")
                    last_name = name
                data, _ = lim.target(A, B)
                msg.data = [float(x) for x in data]
                node.pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.0)
                sleep = dt - (time.time() - t0 - t)
                if sleep > 0:
                    time.sleep(sleep)
            if not args.loop:
                break
            last_name = None
        node.get_logger().info("Dance complete. Holding final pose. Bye!")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted -- freezing at current desired pose.")
        node.pub.publish(msg)

    node.destroy_node()
    rclpy.shutdown()
    return 0


def build_parser():
    p = argparse.ArgumentParser(description="Make the Franka Panda dance + wave.")
    p.add_argument("--routine", choices=["dance", "spin"], default="dance",
                   help="'dance' = gentle wiggle around the current pose; "
                        "'spin' = big breakdance (orbit base, stand tall, spin hand)")
    p.add_argument("--topic", default="/cartesian_impedance/pose_desired")
    p.add_argument("--state-topic",
                   default="/franka_robot_state_broadcaster/robot_state")
    p.add_argument("--rate", type=float, default=100.0, help="publish rate (Hz)")
    p.add_argument("--time-scale", type=float, default=1.0,
                   help=">1 slower, <1 faster")
    p.add_argument("--amp-scale", type=float, default=1.0,
                   help="<1 smaller/gentler motions")
    p.add_argument("--v-max", type=float, default=0.4,
                   help="hard linear speed cap (m/s)")
    p.add_argument("--w-max", type=float, default=2.0,
                   help="hard angular speed cap (rad/s)")
    p.add_argument("--countdown", type=int, default=3)
    p.add_argument("--state-timeout", type=float, default=5.0)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--confirm", action="store_true",
                   help="require typing YES before moving (use on hardware)")
    p.add_argument("--assume-home", action="store_true",
                   help="run without reading state (assumes arm at default anchor)")
    p.add_argument("--selftest", action="store_true",
                   help="verify the trajectory offline (no ROS, no robot)")
    return p


def main():
    args = build_parser().parse_args()
    if args.selftest:
        return selftest(args)
    return run_ros(args)


if __name__ == "__main__":
    sys.exit(main())
