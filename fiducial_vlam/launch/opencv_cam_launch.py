import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


# Full teleop launch: driver, base, joystick, etc.


def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(cmd=['rviz2', '-d', 'src/fiducial_vlam/fiducial_vlam/cfg/default.rviz'], output='screen'),
        # ExecuteProcess(cmd=['rviz2'], output='screen'),

        Node(package='opencv_cam', node_executable='opencv_cam_main', output='screen',
             node_name='opencv_cam', parameters=[{
                'file': False,
                # 'filename': '/home/peter/VID_20191219_210200.mp4',
                'fps': 15,
            }]),

        # Node(package='charuco_ros2', node_executable='charuco_ros2', output='screen'),
    ])

# 'filename': '/home/peter/Videos/my_video-1.mkv',
# 'filename': '/home/peter/Videos/VID_20190717_164536.mp4',
