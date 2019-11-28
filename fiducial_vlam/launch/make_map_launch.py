
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

map_filename = os.path.join(package_cfg_directory, 'fiducial_marker_locations_created.yaml')
rviz_config_filename = os.path.join(package_cfg_directory, package_name + '.rviz')

print ("map_filename: ", map_filename)
print ("rviz_config_filename: ", rviz_config_filename)


vloc_args = [{
    'use_sim_time': False,  # Use /clock if available
    'publish_tfs': 1,  # Publish drone and camera /tf
    'stamp_msgs_with_current_time': 1,  # Use incoming message time, not now()
    # 'base_frame_id': 'base_link' + suffix,
    'map_init_pose_z': -0.035,
    # 'camera_frame_id': 'camera_link' + suffix,
    # 'base_odometry_pub_topic': 'filtered_odom',
    # 'sub_camera_info_best_effort_not_reliable': 1,
    'publish_image_marked': 1,
}]

vmap_args = [{
    'use_sim_time': False,  # Use /clock if available
    'publish_tfs': 1,  # Publish marker /tf
    'marker_length': 0.1627,  # Marker length
    'marker_map_save_full_filename': map_filename,  # Puts the new map in cfg directory
    'make_not_use_map': 1  # Don't save a map to disk
}]



def generate_launch_description():

    entities = [
        ExecuteProcess(cmd=['rviz2', '-d', rviz_config_filename], output='screen'),

        Node(package='fiducial_vlam', node_executable='vloc_node', output='screen',
             parameters=vloc_args),
        Node(package='fiducial_vlam', node_executable='vmap_node', output='screen',
             parameters=vmap_args),

    ]

    return LaunchDescription(entities)

