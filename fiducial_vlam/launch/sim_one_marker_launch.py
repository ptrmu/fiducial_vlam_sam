import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

vlam_package_share_directory = get_package_share_directory('fiducial_vlam')
sim_package_share_directory = get_package_share_directory('sim_fiducial')

launch_directory = os.path.join(vlam_package_share_directory, 'launch')
worlds_directory = os.path.join(sim_package_share_directory, 'worlds')
sdf_directory = os.path.join(sim_package_share_directory, 'sdf')

map_filename = os.path.join(worlds_directory, 'one_marker_map.yaml')
world_filename = os.path.join(worlds_directory, 'one_marker.world')
forward_camera_sdf = os.path.join(sdf_directory, 'forward_camera.sdf')

corner_measurement_sigma = 0.7

vloc_args = [{
    'use_sim_time': False,  # Don't use /clock
    'mel_publish_tfs': 1,  # Publish drone and camera /tf
    'mel_stamp_msgs_with_current_time': 1,  # Stamp with now()
    'map_init_pose_z': 0,
    'mel_sub_camera_info_best_effort_not_reliable': 1,
    'mel_publish_tfs_per_marker': 0,
    'mel_publish_image_marked': 1,
    'mel_camera_frame_id': 'forward_camera',
    'mel_publish_base_pose': 1,
    'mel_publish_camera_odom': 1,
    'mel_publish_base_odom': 1,
}]

vmap_args = [{
    'use_sim_time': False,  # Don't use /clock
    'mem_publish_tfs': 1,  # Publish marker /tf
    'map_marker_length': 0.1778,  # Marker length
    'map_save_filename': map_filename,
    'map_load_filename': map_filename,
    'corner_measurement_sigma': corner_measurement_sigma,
}]


# Before running Gazebo, prepare the environment:
# export GAZEBO_MODEL_PATH=${PWD}/install/sim_fiducial/share/sim_fiducial/models
# . /usr/share/gazebo/setup.sh


def generate_launch_description():
    entities = [
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_filename
        ], output='screen'),

        # Add forward-facing camera to the simulation
        Node(package='sim_fiducial', node_executable='inject_entity.py', output='screen',
             arguments=[forward_camera_sdf, '0', '0', '0', '0', '0', '0']),

        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             parameters=vloc_args, node_namespace='forward_camera'),
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             parameters=vmap_args),
    ]

    return LaunchDescription(entities)
