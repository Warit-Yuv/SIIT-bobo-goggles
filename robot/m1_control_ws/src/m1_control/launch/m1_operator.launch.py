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
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
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
        if not cfg:
            # Ship a config that already has LaserScan (Best Effort), Map,
            # RobotModel (/robot_description), TF, Odometry and the camera
            # image. Hand-adding those displays, and remembering that /scan
            # must be Best Effort, is the most common way to end up staring at
            # an empty RViz.
            cfg = os.path.join(
                get_package_share_directory('m1_control'), 'config',
                'm1_operator.rviz')
        cfg_path = cfg

        def rviz_actions(attempt=1):
            """One rviz2 node plus a handler that knows what to do if it dies.

            RViz has been observed to abort (SIGABRT, exit -6) about a second
            after starting, when the VM's software OpenGL stack is still coming
            up after a reboot. slam_toolbox does not care and keeps running, so
            the launch looks perfectly healthy - and because the launch holds
            the view lock, a second attempt is refused. The operator's symptom
            is "RViz is not shown again" with no error anywhere to explain it.

            So: retry once on an ABNORMAL exit, and say so loudly. A clean exit
            (code 0) is the operator closing the window, which must not make it
            spring back. The handler registers a handler for its own retry too,
            so a second death is reported rather than becoming silent again.
            """
            node = Node(
                package='rviz2', executable='rviz2', name='rviz2',
                output='screen',
                arguments=['-d', cfg_path]
                if cfg_path and os.path.exists(cfg_path) else [],
            )

            def on_exit(event, context, attempt=attempt):
                code = event.returncode
                # A shutdown must not be mistaken for a crash.
                #
                # RViz frequently segfaults (exit -11) while Ogre tears down its
                # render window, so a plain Ctrl+C or a stop of the view looks
                # exactly like the startup abort this handler exists for. Left
                # unguarded, the handler would relaunch RViz into a launch that
                # is already going away - and print "RViz died, restarting" on
                # every normal stop, which is a warning that fires on correct
                # operation and teaches the operator to ignore it.
                if code == 0:
                    return [LogInfo(msg='[operator] RViz closed')]
                if getattr(context, 'is_shutdown', False):
                    return [LogInfo(msg=f'[operator] RViz stopped (exit {code} '
                                        f'during shutdown)')]
                if attempt < 2:
                    return [
                        LogInfo(msg=f'[operator] RViz died (exit {code}) - '
                                    f'restarting it once. This is usually the '
                                    f'VMware OpenGL driver still initialising.'),
                        *rviz_actions(attempt + 1),
                    ]
                return [LogInfo(
                    msg=f'[operator] RViz died again (exit {code}) and will '
                        f'not be restarted. SLAM is still running and holding '
                        f'the view lock, so there is no window and no way to '
                        f'start another view. Stop the view, then start it '
                        f'again. If it keeps dying, the OpenGL driver is the '
                        f'suspect - this VM renders in software.')]

            return [
                node,
                RegisterEventHandler(
                    OnProcessExit(target_action=node, on_exit=on_exit)),
            ]

        actions.extend(rviz_actions())

    return actions


def generate_launch_description():
    return LaunchDescription([
        # FALSE by default, because the console owns the gaze HUD.
        #
        # The console's "Start gaze" button launches it with the camera the
        # operator picked and the source they chose. If this launch also starts
        # one, there are two trackers on one machine: two processes opening
        # webcams (which fail, because a camera opens exclusively), two
        # publishers on /gaze/point and two on /servo/gaze, so the head gets
        # commands from both and the point flickers between two estimates.
        #
        # Measured: `ps` showed two gaze_hud_node processes, one from 02:14
        # (console) and one from 02:21 (this launch), both running.
        #
        # Pass gaze:=true only when running WITHOUT the console.
        DeclareLaunchArgument('gaze', default_value='false',
                              choices=['true', 'false'],
                              description='Start a gaze HUD here. Leave false '
                                          'when the console is driving it - '
                                          'two HUDs fight over the camera and '
                                          'the servo topics.'),
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
