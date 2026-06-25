"""Launch the Robo67 nodes against the REAL arm (milestone A/B).

Prereqs (HARDWARE — single-arm mutex; see HANDOFF.md):
  1. Acquire the arm lock (flock /tmp/robo67_arm.lock).
  2. In Franka Desk (https://192.168.1.67/desk/): unlock joints + activate FCI.
  3. Bring up the real driver:  ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67
     (confirm it loads multi_mode_controller; if not, spawn it like activate_cartesian.sh).
  4. Activate the cartesian impedance controller (set_controllers) + set stiffness.
  5. Run a one-time C920->base calibration (calibration_node) and pass the homography.
Then:  ros2 launch robo67_insertion hardware.launch.py socket_top_z:=<measured>

Safety: orchestrator clamps every setpoint (workspace AABB, max_lead<0.1, force abort,
watchdog). Re-tighten workspace_aabb in config/robo67.yaml for the real reachable space.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    socket_top_z = LaunchConfiguration("socket_top_z")
    homography = LaunchConfiguration("homography_path")
    return LaunchDescription([
        DeclareLaunchArgument("socket_top_z", default_value="0.0",
                              description="Measured socket-top Z in base frame (m)."),
        DeclareLaunchArgument("homography_path", default_value="",
                              description="Path to c920_homography.npz from calibration."),
        Node(
            package="robo67_insertion", executable="socket_detector", name="socket_detector",
            output="screen",
            parameters=[{"socket_top_z": socket_top_z, "homography_path": homography, "rate_hz": 5.0}],
        ),
        Node(
            package="robo67_insertion", executable="insertion_orchestrator", name="insertion_orchestrator",
            output="screen",
            parameters=[{"set_stiffness": True, "activate_controller": False, "use_socket_topic": True}],
        ),
    ])
