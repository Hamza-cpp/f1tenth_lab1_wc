#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    This launch file is used to launch the talker Python node based on the parameters defined in the 'params.yaml' file.
    """
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("lab1_pkg"), "config", "params.yaml"
    )

    talker_node = Node(
        package="lab1_pkg",
        executable="talker.py",
        # name="talker_node",
        output="screen",
        # parameters=[{"v": 1.0}, {"d": 0.5}],
        parameters=[config],
    )

    ld.add_action(talker_node)

    return ld
