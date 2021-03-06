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

created_map_filename = os.path.join(package_cfg_directory, 'fiducial_marker_locations_test_sam.yaml')
existing_map_filename = os.path.join(package_cfg_directory, 'fiducial_marker_locations_test_sam_existing.yaml')
rviz_config_filename = os.path.join(package_cfg_directory, package_name + '.rviz')

print("created_map_filename: ", created_map_filename)
print("existing_map_filename: ", existing_map_filename)
print("rviz_config_filename: ", rviz_config_filename)

corner_measurement_sigma = 1.0

tello_args = [{
    'drone_ip': '192.168.0.30',
    'command_port': 11002,
    'drone_port': 8889,
    'data_port': 13002,
    'video_port': 14002
}]

vloc_args = [{
    'use_sim_time': False,  # Use /clock if available
    'psl_publish_tfs': 1,  # Publish drone and camera /tf
    'psl_stamp_msgs_with_current_time': 1,  # Use incoming message time, not now()
    # 'psl_base_frame_id': 'base_link' + suffix,
    'map_init_pose_z': -0.035,
    # 'psl_camera_frame_id': 'camera_link' + suffix,
    # 'psl_base_odometry_pub_topic': 'filtered_odom',
    'psl_sub_camera_info_best_effort_not_reliable': 1,
    'psl_publish_camera_tf_per_marker': 0,
    'psl_publish_image_marked': 1,
}]

vmap_args = [{
    'use_sim_time': False,  # Use /clock if available
    'psm_publish_tfs': 1,  # Publish marker /tf
    'map_marker_length': 0.1627,  # Marker length
    'map_save_filename': created_map_filename,
    'map_load_filename': existing_map_filename,
    'map_corner_measurement_sigma': corner_measurement_sigma,
    'map_init_id': 1,
}]


def generate_launch_description():
    entities = [
        ExecuteProcess(cmd=['rviz2', '-d', rviz_config_filename], output='screen'),

        Node(package='tello_driver', node_executable='tello_driver_main', output='screen',
             parameters=tello_args),

        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             parameters=vloc_args),
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             parameters=vmap_args),
    ]

    return LaunchDescription(entities)
