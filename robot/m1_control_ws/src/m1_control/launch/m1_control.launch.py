#!/usr/bin/env python3
"""Robot-side control stack: arbiter + EEG bridge (+ optional keyboard teleop).

Deliberately separate from `m1_robot_bringup.launch.py`:

* bringup = hardware (driver, odometry, camera, LiDAR)
* control = decision making (arbiter, input sources)

Keeping them apart means you can restart the control layer after a code change
without power-cycling the chassis driver, and a hardware restart does not
silently re-arm a controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    params_file = os.path.join(
        get_package_share_directory('m1_control'), 'config', 'm1_params.yaml')

    actions = [
        Node(
            package='m1_control',
            executable='control_arbiter_node',
            name='control_arbiter_node',
            output='screen',
            parameters=[params_file],
        ),
    ]

    if arg('eeg').lower() == 'true':
        actions.append(Node(
            package='m1_control',
            executable='eeg_udp_bridge',
            name='eeg_udp_bridge',
            output='screen',
            arguments=[
                '--port', arg('eeg_port'),
                '--speed', arg('eeg_speed'),
                '--steer', arg('eeg_steer'),
                '--link-timeout', arg('eeg_link_timeout'),
            ],
        ))
        # The sound player follows the EEG: an eyebrow raise is the least
        # reliable of the three gestures, so it gets the only job where a
        # misfire costs nothing but a noise. It is not gated on `eeg` because
        # /eeg/music can also be published by hand while debugging.
        actions.append(Node(
            package='m1_control',
            executable='sound_player_node',
            name='sound_player_node',
            output='screen',
            parameters=[{'sound_dir': arg('sound_dir')}],
        ))
    else:
        actions.append(LogInfo(msg='[control] EEG bridge OFF'))

    if arg('teleop').lower() == 'true':
        actions.append(Node(
            package='m1_control',
            executable='wasd_teleop_node',
            name='wasd_teleop_node',
            output='screen',
            parameters=[params_file],
            # Keyboard teleop owns the terminal; do not spawn it headless.
            emulate_tty=True,
        ))

    # ---------------- audio relay -------------------------------------------
    # Started here so the operator console's Voice panel has something to talk
    # to. It stays muted until /audio/enable arrives, so the robot's
    # microphone is never live by accident - a real concern in someone's home.
    # Ports are derived from --role (robot listens on 9201, sends to 9200), so
    # they are deliberately not launch arguments: a typo there would silently
    # give one-way audio.
    if arg('audio').lower() == 'true':
        actions.append(Node(
            package='m1_control',
            executable='audio_bridge',
            name='audio_bridge',
            output='screen',
            arguments=[
                '--role', 'robot',
                '--peer-ip', arg('audio_peer'),
                '--device-in', arg('audio_device'),
                '--device-out', arg('audio_device'),
            ],
        ))
    else:
        actions.append(LogInfo(msg='[control] audio bridge OFF'))

    # ---------------- gaze pipeline ------------------------------------------
    # These were in NO launch file at all, so the gaze path never worked unless
    # you started two nodes by hand. The console's Calibrate button then said
    # "no gaze source" with no indication that nothing was running.
    #
    # The split is deliberate:
    #
    #   gaze_udp_bridge    turns UDP from the eye tracker into ROS topics.
    #                      Lives here because it publishes /gaze/raw, and the
    #                      operator console calibrates against that.
    #   gaze_control_node  turns a gaze point into a velocity command. Lives
    #                      here because it is a CONTROL node: its output feeds
    #                      the arbiter, and the arbiter is on the robot.
    #
    # gaze_hud_node is NOT started here - it needs a display and a webcam, so
    # it belongs on the operator PC (m1_operator.launch.py).
    if arg('gaze').lower() == 'true':
        actions.append(Node(
            package='m1_control',
            executable='gaze_udp_bridge',
            name='gaze_udp_bridge',
            output='screen',
            arguments=['--port', arg('gaze_port')],
        ))
        actions.append(Node(
            package='m1_control',
            executable='gaze_control_node',
            name='gaze_control_node',
            output='screen',
            parameters=[params_file],
        ))
    else:
        actions.append(LogInfo(msg='[control] gaze pipeline OFF'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sound_dir', default_value='',
                              description='Folder of sound files for the EEG '
                                          'eyebrow-raise gesture. Empty means '
                                          'auto-detect: /root/sound in the '
                                          'container, /home/pi/sound outside'),
        DeclareLaunchArgument('eeg', default_value='true',
                              choices=['true', 'false'],
                              description='Start the EEG UDP -> /cmd_vel/eeg bridge'),
        DeclareLaunchArgument('eeg_port', default_value='9000'),
        DeclareLaunchArgument('eeg_speed', default_value='0.15'),
        DeclareLaunchArgument('eeg_steer', default_value='0.60'),
        DeclareLaunchArgument('eeg_link_timeout', default_value='1.5'),
        DeclareLaunchArgument('teleop', default_value='false',
                              choices=['true', 'false'],
                              description='Start keyboard teleop (needs a TTY)'),
        DeclareLaunchArgument('audio', default_value='true',
                              choices=['true', 'false'],
                              description='Start the 2-way voice relay'),
        DeclareLaunchArgument('audio_peer', default_value='10.0.0.102',
                              description='Operator PC address for voice UDP'),
        DeclareLaunchArgument('audio_device', default_value='plughw:2,0',
                              description='ALSA device for the USB headset '
                                          '(card 2 on this robot)'),
        DeclareLaunchArgument('gaze', default_value='true',
                              choices=['true', 'false'],
                              description='Start the gaze UDP bridge and the '
                                          'gaze -> /cmd_vel/gaze controller'),
        DeclareLaunchArgument('gaze_port', default_value='9100',
                              description='UDP port the eye tracker sends to'),
        OpaqueFunction(function=launch_setup),
    ])
