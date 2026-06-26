"""Bring up the Robo67 *logging* graph: camera feeds + detection overlays.

This is the always-on observability layer. It starts ONE dedicated
``camera_publisher`` per camera (the device owner -- only one process may open a
``/dev/videoN``) and the detector nodes in ``source:=topic`` mode so they
SUBSCRIBE to those feeds instead of grabbing the device. Everything published
here is documented in ``docs/architecture/logging-topics.md``.

Published topics (sensor_msgs/CompressedImage "jpeg" unless noted):
  /robo67/camera/overhead/image_raw/compressed   (camera_publisher, C920)
  /robo67/camera/overhead/overlay/compressed      (socket_detector)
  /robo67/socket_pose, /robo67/socket_detection   (socket_detector)
  /robo67/camera/gripper/image_raw/compressed     (camera_publisher, D405)  [gripper:=true]
  /robo67/camera/gripper/overlay/compressed        (d405_servo)              [gripper:=true]
  /robo67/servo_correction                         (d405_servo)              [gripper:=true]

The insertion TELEMETRY (/robo67/insertion/*) comes from hardware_insertion_node,
which the operator launches separately (it prompts before motion):
  ros2 run robo67_insertion hardware_insertion --socket-from-current --dry-run

Usage:
  ros2 launch robo67_insertion logging.launch.py socket_top_z:=<measured>
  ros2 launch robo67_insertion logging.launch.py gripper:=true   # add D405 feeds
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    socket_top_z = LaunchConfiguration("socket_top_z")
    homography = LaunchConfiguration("homography_path")
    gripper = LaunchConfiguration("gripper")
    socket_kind = LaunchConfiguration("socket_kind")

    return LaunchDescription([
        DeclareLaunchArgument("socket_top_z", default_value="0.0",
                              description="Measured socket-top Z in base frame (m)."),
        DeclareLaunchArgument("homography_path", default_value="",
                              description="Path to c920_homography.npz from calibration."),
        DeclareLaunchArgument("gripper", default_value="false",
                              description="Also bring up the D405 gripper camera + servo feeds."),
        DeclareLaunchArgument("socket_kind", default_value="cube",
                              description="overhead detector: 'cube' -> detect_white_cubes "
                              "(white cube/socket centroid)."),

        # -- overhead C920: device owner + detector (overlay) ---------------
        Node(
            package="robo67_insertion", executable="camera_publisher",
            name="camera_publisher_overhead", output="screen",
            parameters=[{"camera": "overhead"}],
        ),
        Node(
            package="robo67_insertion", executable="socket_detector",
            name="socket_detector", output="screen",
            parameters=[{
                "source": "topic",
                "socket_top_z": socket_top_z,
                "homography_path": homography,
                "socket_kind": socket_kind,
                "rate_hz": 5.0,
            }],
        ),

        # -- gripper D405: device owner + servo (overlay) -------------------
        Node(
            package="robo67_insertion", executable="camera_publisher",
            name="camera_publisher_gripper", output="screen",
            condition=IfCondition(gripper),
            parameters=[{"camera": "gripper"}],
        ),
        Node(
            package="robo67_insertion", executable="d405_servo",
            name="d405_servo", output="screen",
            condition=IfCondition(gripper),
            parameters=[{"source": "topic", "rate_hz": 5.0}],
        ),
    ])
