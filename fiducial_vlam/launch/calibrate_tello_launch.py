import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

package_name = 'fiducial_vlam'
package_share_directory = get_package_share_directory(package_name);
package_launch_directory = os.path.join(package_share_directory, 'launch')
package_cfg_directory = os.path.join(package_share_directory, 'cfg')

pi30_tello_driver_args = [{
    'drone_ip': '192.168.0.30',
    'command_port': '11002',
    'drone_port': '8889',
    'data_port': '13002',
    'video_port': '14002'
}]

pi21_tello_driver_args = [{
    'drone_ip': '192.168.0.21',
    'command_port': '38065',
    'drone_port': '8889',
    'data_port': '8890',
    'video_port': '11111'
}]

vloc_args = [{
    'loc_calibrate_not_localize' : 1,
}]


def generate_launch_description():

    entities = [
        Node(package='tello_driver', node_executable='tello_driver_main', output='screen',
             parameters=pi21_tello_driver_args),

        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             parameters=vloc_args),
    ]

    return LaunchDescription(entities)

