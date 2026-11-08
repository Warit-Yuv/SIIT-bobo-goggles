#!/usr/bin/env python3
"""Robot-side bringup for the ROSMASTER M1.

Composes the hardware nodes explicitly instead of including Yahboom's
`laser_bringup_launch.py`, for three reasons:

1. **No joystick node.** `yahboomcar_bringup_M1_launch.py` starts
   `yahboom_joy_M1`, which publishes /cmd_vel. That fights the arbiter
   (handbook issue #2: "robot drives erratically"). Only the arbiter may
   publish /cmd_vel, so the joy node is deliberately left out.
2. **Optional LiDAR.** The LiDAR is started only when its device node
   actually exists, so an unplugged / re-enumerated sensor cannot abort
   bringup or leave the rest of the robot dead.
3. **No RViz on the Pi.** The operator PC does the visualisation.

Everything is resolved at launch time via OpaqueFunction, so `lidar:=auto`
checks the real filesystem rather than trusting an environment variable.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def _device_present(path: str) -> bool:
    """True if the device node or a udev symlink to it exists."""
    return os.path.exists(path)


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    lidar_mode = arg('lidar').lower()          # auto | true | false
    camera_mode = arg('camera').lower()
    lidar_device = arg('lidar_device')
    video_device = arg('video_device')
    throttle_rate = arg('throttle_rate')
    robot_type = os.environ.get('ROBOT_TYPE', 'M1')
    camera_type = os.environ.get('CAMERA_TYPE', 'usb')

    actions = []

    # ---------------- 1. robot_state_publisher (URDF / TF tree) -------------
    desc_share = get_package_share_directory('yahboomcar_description')
    if camera_type == 'usb':
        model = os.path.join(desc_share, 'urdf', 'yahboomcar_M1_usb.urdf.xacro')
    else:
        model = os.path.join(desc_share, 'urdf', 'yahboomcar_M1.urdf.xacro')

    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', model]),
                                                value_type=str),
            'use_sim_time': False,
        }],
    ))

    # The PTZ camera joints (cam1/cam2) are revolute, so robot_state_publisher
    # needs joint states before it can publish cam1_Link -> cam2_Link. Without
    # this the camera frames are missing from the TF tree and RViz cannot place
    # the image. The gimbal is driven through /Servo, not through these joints,
    # so a neutral 0 rad here is correct (the URDF origin already holds the
    # real mounting offsets).
    actions.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    ))

    # ---------------- 2. STM32 chassis driver -------------------------------
    actions.append(Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_M1',
        name='Mcnamu_driver_M1',
        output='screen',
        parameters=[{'car_type': robot_type}],
    ))

    # ---------------- 3. Odometry base node ---------------------------------
    actions.append(Node(
        package='yahboomcar_base_node',
        executable='base_node_M1',
        name='base_node_M1',
        output='screen',
        parameters=[{
            'pub_odom_tf': False,      # EKF owns odom -> base_footprint
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
        }],
    ))

    # ---------------- 4. IMU filter + EKF fusion ----------------------------
    imu_cfg = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'param', 'imu_filter_param.yaml')
    if os.path.exists(imu_cfg):
        actions.append(Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[imu_cfg],
        ))
    else:
        actions.append(LogInfo(msg='[bringup] imu_filter_param.yaml missing - IMU filter skipped'))

    # ---------------- 4b. EKF (odom -> base_footprint) ----------------------
    # Uses the Layer-4 parameter file, not the vendor one. The vendor config
    # fuses yaw from BOTH the wheels and the IMU, so a hand-turn with the
    # wheels off the ground moves /odom by 9% of what the IMU actually saw and
    # slam_toolbox never triggers. See config/ekf_m1.yaml for the measurement.
    ekf_params = os.path.join(
        get_package_share_directory('m1_control'), 'config', 'ekf_m1.yaml')
    try:
        get_package_share_directory('robot_localization')
        if os.path.exists(ekf_params):
            actions.append(Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_params],
                remappings=[('/odometry/filtered', '/odom')],
            ))
            actions.append(LogInfo(
                msg='[bringup] EKF ON (IMU owns yaw - see config/ekf_m1.yaml)'))
        else:
            # Falling back to the vendor launch is better than no odometry at
            # all, but say so loudly: the rotation bug will be present.
            actions.append(LogInfo(
                msg=f'[bringup] WARNING {ekf_params} missing - falling back to '
                    f'the vendor EKF, which ignores IMU yaw when the wheels '
                    f'are not turning'))
            rl_share = get_package_share_directory('robot_localization')
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rl_share, 'launch', 'ekf_M1_launch.py'))))
    except Exception as exc:  # noqa: BLE001
        actions.append(LogInfo(
            msg=f'[bringup] EKF unavailable ({exc}) - odom TF disabled'))

    # ---------------- 5. LiDAR (optional, auto-detected) --------------------
    lidar_share = None
    try:
        lidar_share = get_package_share_directory('ydlidar_ros2_driver')
    except Exception:  # noqa: BLE001
        pass

    if lidar_mode == 'false':
        start_lidar = False
    elif lidar_mode == 'true':
        start_lidar = True
    else:  # auto
        start_lidar = _device_present(lidar_device)

    if start_lidar and lidar_share is None:
        actions.append(LogInfo(
            msg='[bringup] ydlidar_ros2_driver not installed - LiDAR skipped'))
        start_lidar = False

    if start_lidar and not _device_present(lidar_device):
        actions.append(LogInfo(
            msg=f'[bringup] LiDAR requested but {lidar_device} is absent - '
                f'skipped, rest of the robot continues'))
        start_lidar = False

    if start_lidar:
        params_file = os.path.join(
            get_package_share_directory('m1_control'), 'config',
            'ydlidar_tmini_plus.yaml')
        actions.append(LogInfo(
            msg=f'[bringup] LiDAR ON ({lidar_device}) - scan on /scan'))

        # The vendor launch is an IncludeLaunchDescription around a
        # LifecycleNode. We start that node directly instead so we can attach
        # respawn: the Tmini MCU is not always ready the instant USB
        # enumeration finishes, and a first attempt can fail with
        # "cannot retrieve YDLidar health code". With respawn the node retries
        # by itself and the failure becomes invisible. It also recovers
        # automatically when the LiDAR is unplugged and plugged back in.
        actions.append(LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='/',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
            respawn=True,
            respawn_delay=5.0,
        ))
        # Vendor-proven static transform: the driver publishes frame_id "laser",
        # while the URDF mesh link is "laser_Link".
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_link_to_laser',
            arguments=['0.0060585', '0', '0.14912',
                       '0', '3.1415', '3.1415', 'base_link', 'laser'],
        ))
    else:
        actions.append(LogInfo(
            msg='[bringup] LiDAR OFF - SLAM will not work until it is connected '
                'and the launch is restarted with lidar:=true'))

    # ---------------- 6. Camera + throttler ---------------------------------
    if camera_mode == 'true':
        if not _device_present(video_device):
            actions.append(LogInfo(
                msg=f'[bringup] Camera requested but {video_device} is absent - '
                    f'skipped'))
        else:
            actions.append(LogInfo(
                msg=f'[bringup] Camera ON ({video_device}) 640x480 YUYV @ 30 fps'))
            actions.append(Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam',
                output='screen',
                parameters=[{
                    'video_device': video_device,
                    'image_width': 640,
                    'image_height': 480,
                    'pixel_format': 'yuyv2rgb',   # native 30 Hz, not 120 Hz MJPG
                    'framerate': 30.0,
                    'camera_name': 'usb_cam',
                    # cam2_Link is the URDF tilt-stage frame, so the TF tree is
                    # already complete - no extra static transform needed.
                    'camera_frame_id': 'cam2_Link',
                    'brightness': 50,
                    'contrast': 32,
                    'saturation': 64,
                    'autoexposure': True,
                }],
            ))
            actions.append(Node(
                package='m1_control',
                executable='image_throttle_node',
                name='image_throttle_node',
                output='screen',
                parameters=[{
                    'input_topic': '/image_raw/compressed',
                    'output_topic': '/image_throttled/compressed',
                    'target_rate_hz': float(throttle_rate),
                }],
            ))
    else:
        actions.append(LogInfo(msg='[bringup] Camera OFF'))

    # ---------------- 7. Optional RViz (usually on the PC instead) ----------
    if arg('rviz').lower() == 'true':
        rviz_cfg = os.path.join(desc_share, 'rviz', 'yahboomcar.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar', default_value='auto',
            choices=['auto', 'true', 'false'],
            description='Start the LiDAR: auto = only if the device exists'),
        DeclareLaunchArgument(
            'lidar_device', default_value='/dev/rplidar',
            description='YDLIDAR Tmini serial device (udev symlink)'),
        DeclareLaunchArgument(
            'camera', default_value='true',
            choices=['true', 'false'],
            description='Start the USB camera + image throttler'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'throttle_rate', default_value='20.0',
            description='Max compressed frames/s published over Wi-Fi'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Run RViz on the robot (normally on the PC instead)'),
        OpaqueFunction(function=launch_setup),
    ])
