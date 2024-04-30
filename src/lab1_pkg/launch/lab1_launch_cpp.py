#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    This launch file is used to launch the talker cpp node.
    """
    ld = LaunchDescription()

    talker_node = Node(
        package="lab1_pkg",
        executable="talker",
        output="screen",
        parameters=[{"v": 10.0}, {"d": 0.5}],
    )

    ld.add_action(talker_node)

    return ld
