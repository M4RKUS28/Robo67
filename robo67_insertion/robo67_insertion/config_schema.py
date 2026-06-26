"""Typed configuration for the Robo67 insertion stack.

Pure Python (no rclpy). Mirrors ``config/robo67.yaml``. Nodes load a
:class:`RoboConfig` once at startup; the pure-logic libs receive only the
sub-config dataclasses they need, which keeps them trivially unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Union

import yaml


@dataclass
class StiffnessCfg:
    free_translational: float = 400.0      # N/m, free-space moves
    free_rotational: float = 20.0          # Nm/rad
    contact_translational_xy: float = 150.0  # softened in XY near contact
    contact_z: float = 400.0               # keep Z firm to push down
    contact_rotational: float = 20.0
    damping_ratio: float = 0.8
    nullspace_stiffness: float = 10.0


@dataclass
class SpiralCfg:
    max_radius_m: float = 0.012
    pitch_m: float = 0.002
    speed_mps: float = 0.005
    pts_per_rev: int = 36


@dataclass
class SafetyCfg:
    # [[xmin,xmax],[ymin,ymax],[zmin,zmax]] in base frame; filled from sim in Phase 3.
    workspace_aabb: List[List[float]] = field(
        default_factory=lambda: [[0.25, 0.65], [-0.30, 0.30], [0.02, 0.60]]
    )
    # Max lead of the commanded equilibrium ahead of the ACTUAL EE pose. The MMC
    # cartesian impedance controller DISCARDS any desired pose > 0.1 m from the
    # current pose, so this MUST stay < 0.1. It is also the per-command velocity
    # bound (the carrot never gets farther than this from the arm).
    max_lead_m: float = 0.05
    fz_abort_n: float = 25.0         # abort if |external force| exceeds this
    watchdog_s: float = 0.2          # hold if robot state older than this


@dataclass
class InsertionCfg:
    standoff_m: float = 0.05         # height above socket top for MOVE_ABOVE
    approach_tol_m: float = 0.003    # "arrived" tolerance
    contact_fz_threshold_n: float = 5.0
    insert_depth_m: float = 0.04
    z_drop_threshold_m: float = 0.004  # EE drop that signals peg entered hole
    retry_limit: int = 3
    control_rate_hz: float = 50.0


@dataclass
class TopicsCfg:
    # VERIFIED against the running sim (Phase 0.3). See PHASE0_VERIFIED.md.
    controller_name: str = "panda_cartesian_impedance_controller"
    arm_resource: str = "panda"
    desired_pose: str = "/panda/panda_cartesian_impedance_controller/desired_pose"
    parameters: str = "/panda/panda_cartesian_impedance_controller/parameters"
    set_controllers: str = "/multi_mode_controller/set_controllers"   # namespaced!
    robot_state: str = "/franka_robot_state_broadcaster/robot_state"
    error_recovery: str = "/panda_error_recovery_service_server/error_recovery"  # hardware only
    gripper_ns: str = "/panda_gripper_sim_node"   # sim; hardware ns confirmed in Phase 4
    socket_pose: str = "/robo67/socket_pose"
    socket_detection: str = "/robo67/socket_detection"
    servo_correction: str = "/robo67/servo_correction"   # D405 eye-in-hand [dx, dy]
    # -- cable insertion: overhead I/O-box detection (box_detector_node) -----
    box_pose: str = "/robo67/box_pose"          # geometry_msgs/PoseStamped (base XY + taught Z)
    box_detection: str = "/robo67/box_detection"  # std_msgs/Float64MultiArray [u,v,w,h,angle,score]

    # -- logging: camera feeds (sensor_msgs/CompressedImage, format "jpeg") ---
    # Raw feeds are published by the dedicated camera_publisher node that OWNS
    # each /dev/videoN (only one process may open a V4L2 device); the overlay
    # feeds are the same frame annotated with the detection by the detector
    # nodes. See docs/architecture/logging-topics.md.
    cam_overhead_raw: str = "/robo67/camera/overhead/image_raw/compressed"
    cam_overhead_overlay: str = "/robo67/camera/overhead/overlay/compressed"
    cam_gripper_raw: str = "/robo67/camera/gripper/image_raw/compressed"
    cam_gripper_overlay: str = "/robo67/camera/gripper/overlay/compressed"

    # -- logging: insertion telemetry (published by hardware_insertion_node) --
    insertion_phase: str = "/robo67/insertion/phase"            # std_msgs/String
    insertion_ee_pose: str = "/robo67/insertion/ee_pose"        # geometry_msgs/PoseStamped
    insertion_ee_speed: str = "/robo67/insertion/ee_speed"      # std_msgs/Float64 (m/s)
    insertion_command_pose: str = "/robo67/insertion/command_pose"  # geometry_msgs/PoseStamped
    insertion_wrench: str = "/robo67/insertion/wrench"          # geometry_msgs/WrenchStamped
    insertion_fz: str = "/robo67/insertion/fz"                  # std_msgs/Float64 (N)
    insertion_fz_baseline: str = "/robo67/insertion/fz_baseline"  # std_msgs/Float64 (N)
    insertion_contact: str = "/robo67/insertion/contact"        # std_msgs/Bool
    insertion_retries: str = "/robo67/insertion/retries"        # std_msgs/Int32
    insertion_diagnostics: str = "/robo67/insertion/diagnostics"  # diagnostic_msgs/DiagnosticArray


@dataclass
class CameraCfg:
    # Bare /dev/videoN numbers renumber across replug/reboot, so the overhead
    # C920 is referenced by its STABLE by-id symlink (see docs/cameras.md). A
    # bare int index is still accepted; grab_frame_gst resolves either form.
    c920_device: Union[str, int] = (
        "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_C26B1F5F-video-index0"
    )
    d405_color_device: int = 6       # /dev/video6 (RealSense color; ALSO unstable)
    d405_depth_device: int = 2       # /dev/video2 (Z16)
    # Lock the C920 to MANUAL exposure (the auto setting blows the white socket
    # out to pure white, defeating the bright-bore detector). None -> leave auto.
    c920_exposure: int = 100
    c920_fx: float = 1000.0          # placeholder until intrinsics calibrated
    c920_fy: float = 1000.0
    d405_fx: float = 430.0
    d405_fy: float = 430.0
    # camera_publisher streaming defaults (logging path).
    publish_fps: float = 10.0        # raw CompressedImage publish rate per camera
    jpeg_quality: int = 80           # cv2.imencode JPEG quality (1-100)


@dataclass
class RoboConfig:
    stiffness: StiffnessCfg = field(default_factory=StiffnessCfg)
    spiral: SpiralCfg = field(default_factory=SpiralCfg)
    safety: SafetyCfg = field(default_factory=SafetyCfg)
    insertion: InsertionCfg = field(default_factory=InsertionCfg)
    topics: TopicsCfg = field(default_factory=TopicsCfg)
    camera: CameraCfg = field(default_factory=CameraCfg)
    socket_cube_height_m: float = 0.06   # measured in Phase 0


def _merge(dc, data: dict):
    """Return a dataclass of the same type as ``dc`` with ``data`` overlaid."""
    if not data:
        return dc
    kwargs = {}
    for f in dc.__dataclass_fields__:  # type: ignore[attr-defined]
        kwargs[f] = data.get(f, getattr(dc, f))
    return type(dc)(**kwargs)


def load_config(path: str) -> RoboConfig:
    """Load ``robo67.yaml`` into a :class:`RoboConfig` (missing keys -> defaults)."""
    with open(path, "r") as fh:
        raw = yaml.safe_load(fh) or {}
    cfg = RoboConfig()
    cfg.stiffness = _merge(cfg.stiffness, raw.get("stiffness", {}))
    cfg.spiral = _merge(cfg.spiral, raw.get("spiral", {}))
    cfg.safety = _merge(cfg.safety, raw.get("safety", {}))
    cfg.insertion = _merge(cfg.insertion, raw.get("insertion", {}))
    cfg.topics = _merge(cfg.topics, raw.get("topics", {}))
    cfg.camera = _merge(cfg.camera, raw.get("camera", {}))
    if "socket_cube_height_m" in raw:
        cfg.socket_cube_height_m = raw["socket_cube_height_m"]
    return cfg
