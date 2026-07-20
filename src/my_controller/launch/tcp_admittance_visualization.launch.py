"""Visualize the live libfranka TCP-admittance run with the FR3 URDF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    franka_description_share = get_package_share_directory("franka_description")
    my_controller_share = get_package_share_directory("my_controller")

    xacro_path = os.path.join(
        franka_description_share, "robots", "fr3", "fr3.urdf.xacro"
    )
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "hand": "false",
            "ros2_control": "false",
        },
    ).toprettyxml(indent="  ")

    rviz_path = os.path.join(
        my_controller_share, "rviz", "tcp_admittance_frames.rviz"
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="tcp_admittance_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="tcp_admittance_rviz",
                arguments=["-d", rviz_path],
                output="screen",
            ),
        ]
    )
