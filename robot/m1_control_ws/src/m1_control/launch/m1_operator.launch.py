#!/usr/bin/env python3
"""Operator-PC stack: gaze HUD + SLAM + RViz (no hardware access needed).

Runs on the VMware Ubuntu 22.04 workstation. Subscribes to the robot's
BEST_EFFORT sensor streams and never publishes to /cmd_vel - gaze commands go
to /cmd_vel/gaze for the arbiter on the robot to accept.

    ros2 launch m1_control m1_operator.launch.py
    ros2 launch m1_control m1_operator.launch.py gaze_source:=mouse
    ros2 launch m1_control m1_operator.launch.py slam:=false rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    params_file = os.path.join(
        get_package_share_directory('m1_control'), 'config', 'm1_params.yaml')

    actions = []

    # ---------------- gaze HUD ----------------
    if arg('gaze').lower() == 'true':
        actions.append(Node(
            package='m1_control',
            executable='gaze_hud_node',
            name='gaze_hud_node',
            output='screen',
            parameters=[{
                'camera_topic': arg('camera_topic'),
                'gaze_source': arg('gaze_source'),
                'webcam_id': int(arg('webcam_id')),
                'dwell_time_sec': float(arg('dwell_time')),
                'deadzone_radius_ratio': float(arg('deadzone_ratio')),
            }],
        ))
    else:
        actions.append(LogInfo(msg='[operator] Gaze HUD OFF'))

    # ---------------- SLAM Toolbox ----------------
    if arg('slam').lower() == 'true':
        try:
            # Use our own parameter file rather than the stock online_async
            # profile: the defaults warn about the laser range and, more
            # importantly, drop scans over Wi-Fi because their transform
            # timeout is 0. See config/slam_toolbox_m1.yaml.
            slam_params = os.path.join(
                get_package_share_directory('m1_control'), 'config',
                'slam_toolbox_m1.yaml')
            if os.path.exists(slam_params):
                actions.append(Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_params]))
            else:
                actions.append(LogInfo(
                    msg=f'[operator] slam config missing at {slam_params} - '
                        f'falling back to the stock launch'))
                slam_share = get_package_share_directory('slam_toolbox')
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_share, 'launch',
                                     'online_async_launch.py')),
                    launch_arguments={'use_sim_time': 'false'}.items()))
        except Exception as exc:  # noqa: BLE001
            actions.append(LogInfo(
                msg=f'[operator] slam_toolbox unavailable ({exc}) - '
                    f'install ros-humble-slam-toolbox'))
    else:
        actions.append(LogInfo(msg='[operator] SLAM OFF'))

    # ---------------- RViz ----------------
    if arg('rviz').lower() == 'true':
        cfg = arg('rviz_config')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen',
            arguments=['-d', cfg] if cfg and os.path.exists(cfg) else [],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gaze', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('camera_topic',
                              default_value='/image_throttled/compressed'),
        DeclareLaunchArgument('gaze_source', default_value='mediapipe',
                              choices=['mediapipe', 'mouse', 'external', 'keyboard']),
        DeclareLaunchArgument('webcam_id', default_value='0'),
        DeclareLaunchArgument('dwell_time', default_value='0.40'),
        DeclareLaunchArgument('deadzone_ratio', default_value='0.25'),
        DeclareLaunchArgument('slam', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='Path to a .rviz config (optional)'),
        OpaqueFunction(function=launch_setup),
    ])
