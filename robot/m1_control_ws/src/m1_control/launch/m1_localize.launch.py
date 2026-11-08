#!/usr/bin/env python3
"""Localise against a saved map (AMCL), instead of mapping.

Run this INSTEAD of the mapping launch once you have a saved map.

    ros2 launch m1_control m1_localize.launch.py map:=/home/rescue/maps/office.yaml

Why a separate launch
---------------------
Mapping and localising are different jobs and must not run together:

  slam_toolbox  builds a map. It cannot recover from being moved.
  AMCL          uses a map. It converges from a rough pose estimate.

Both publish map -> odom. Running both at once gives two competing transforms
and the robot's pose jumps between them - the same failure as two mappers
publishing /map.

The pick-and-place workflow
---------------------------
1. Map once, save it (scripts/m1_map.sh save NAME).
2. Start this launch with that map.
3. Pick the robot up, put it down anywhere inside the map.
4. Give AMCL a rough starting pose:
     - RViz: "2D Pose Estimate" (the arrow button), click and drag, or
     - scripts/m1_map.sh relocalize X Y YAW
5. AMCL converges. The particle cloud collapses onto the true pose.

Step 4 is the part no algorithm can do reliably: a kidnapped robot has no idea
where it is, and one scan cannot always tell two similar corridors apart. The
human supplies the one fact the sensor lacks; AMCL does the rest.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    map_yaml = arg('map')
    actions = []

    if not map_yaml:
        actions.append(LogInfo(msg=(
            '[localize] no map:= given. Save one first:\n'
            '    scripts/m1_map.sh save office\n'
            '  then:\n'
            '    ros2 launch m1_control m1_localize.launch.py '
            'map:=$HOME/maps/office.yaml')))
        return actions

    if not os.path.exists(map_yaml):
        actions.append(LogInfo(msg=(
            f'[localize] map file not found: {map_yaml}\n'
            f'  Available maps:\n    '
            + '\n    '.join(
                sorted(
                    os.path.join(d, f)
                    for d in (os.path.expanduser('~/maps'),)
                    if os.path.isdir(d)
                    for f in os.listdir(d)
                    if f.endswith('.yaml'))
                or ['(none)']))))
        return actions

    amcl_params = os.path.join(
        get_package_share_directory('m1_control'), 'config', 'amcl_m1.yaml')

    # ---- map_server ------------------------------------------------------
    # lifecycle node: it must be told to configure and activate before it
    # publishes anything. A plain Node() would start it unconfigured and /map
    # would stay silent, which looks exactly like a broken map file.
    actions.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[amcl_params, {'yaml_filename': map_yaml}],
    ))

    # ---- AMCL ------------------------------------------------------------
    actions.append(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params],
    ))

    # ---- lifecycle manager ----------------------------------------------
    # Drives map_server and amcl through configure -> activate. Without it both
    # nodes sit in "unconfigured" forever and nothing is published.
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            # Generous: map_server may be loading a large PGM over a slow disk.
            'bond_timeout': 0.0,
        }],
    ))

    if arg('rviz').lower() == 'true':
        rviz_cfg = os.path.join(
            get_package_share_directory('m1_control'), 'rviz',
            'm1_operator.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        ))

    actions.append(LogInfo(msg=f'[localize] map: {map_yaml}'))
    actions.append(LogInfo(msg=(
        '[localize] if the pose is wrong, use RViz "2D Pose Estimate" '
        '(top toolbar) or: scripts/m1_map.sh relocalize X Y YAW')))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='Path to the map .yaml (from scripts/m1_map.sh save)'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz for the 2D Pose Estimate button'),
        OpaqueFunction(function=launch_setup),
    ])
