#!/usr/bin/env python3
# coding : utf-8
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("pingpong_tester"),
        "config",
        "test.param.yaml"
    )

    gunner_node = Node(
        package="pingpong_gunner",
        executable="pingpong_gunner_exe",
        output="screen",
        parameters=[config]
    )

    tester_node = Node(
        package="pingpong_tester",
        executable="pingpong_tester_exe",
        output="screen",
        parameters=[config]
    )

    return LaunchDescription([
        gunner_node,
        tester_node,
    ])


