from glob import glob
import os

from setuptools import setup

package_name = 'm1_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSMASTER M1 Team',
    maintainer_email='pi@raspberrypi.local',
    description='Multi-modal teleoperation overlay for the Yahboom ROSMASTER M1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_arbiter_node = m1_control.control_arbiter_node:main',
            'wasd_teleop_node = m1_control.wasd_teleop_node:main',
            'gaze_hud_node = m1_control.gaze_hud_node:main',
            'eeg_udp_bridge = m1_control.eeg_udp_bridge:main',
            'image_throttle_node = m1_control.image_throttle_node:main',
            'sensor_sync_node = m1_control.sensor_sync_node:main',
            'gaze_control_node = m1_control.gaze_control_node:main',
            'gaze_udp_bridge = m1_control.gaze_udp_bridge:main',
        ],
    },
)
