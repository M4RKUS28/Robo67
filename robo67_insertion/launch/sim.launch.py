"""Launch the Robo67 nodes against the SIM.

Prereqs (separate, due to LD_LIBRARY_PATH + manual controller spawn — see
PHASE0_VERIFIED.md): run ``scripts/sim_bringup.sh`` then ``scripts/activate_cartesian.sh``.
Then:  ros2 launch robo67_insertion sim.launch.py

By default the detector runs in offline image mode (the container lacks camera
passthrough in sim); set ``detector_image:=''`` once a camera bridge exists.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detector_image = LaunchConfiguration("detector_image")
    use_socket_topic = LaunchConfiguration("use_socket_topic")
    return LaunchDescription([
        DeclareLaunchArgument("detector_image", default_value="",
                              description="If set, detector reads this still image instead of a camera."),
        DeclareLaunchArgument("use_socket_topic", default_value="false",
                              description="If true, orchestrator waits for /robo67/socket_pose; else auto-places a virtual socket."),
        Node(
            package="robo67_insertion", executable="socket_detector", name="socket_detector",
            output="screen",
            parameters=[{"image_path": detector_image, "rate_hz": 5.0}],
        ),
        Node(
            package="robo67_insertion", executable="insertion_orchestrator", name="insertion_orchestrator",
            output="screen",
            parameters=[{"set_stiffness": True, "activate_controller": False,
                         "use_socket_topic": use_socket_topic}],
        ),
    ])
