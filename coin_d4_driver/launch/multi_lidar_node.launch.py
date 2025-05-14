#!/usr/bin/env python3
# Copyright 2025 ROBOTIS CO., LTD.
# Authors: Hyeongjun Jeon

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    multi_lidar_node_param = LaunchConfiguration('multi_lidar_node_param')

    declare_multi_lidar_node_param = DeclareLaunchArgument(
        name='multi_lidar_node_param',
        default_value=os.path.join(
          get_package_share_directory('coin_d4_driver'),
          'params',
          'multi_lidar_node.yaml'))

    multi_coin_d4_node = Node(
        package='coin_d4_driver',
        executable='multi_coin_d4_node',
        parameters=[multi_lidar_node_param],
        output='screen',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=15),
        sigkill_timeout=LaunchConfiguration('sigkill_timeout', default=15))

    ld = LaunchDescription()

    ld.add_action(declare_multi_lidar_node_param)

    ld.add_action(multi_coin_d4_node)

    return ld
