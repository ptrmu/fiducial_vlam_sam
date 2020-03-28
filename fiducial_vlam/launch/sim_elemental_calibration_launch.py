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

world_name = "charuco_12x9_aruco_5x5_target"
created_map_filename = os.path.join(worlds_directory, 'sim_make_map.yaml')
existing_map_filename = os.path.join(worlds_directory,  world_name + '_map.yaml')
world_filename = os.path.join(worlds_directory, world_name + '.world')
forward_camera_sdf = os.path.join(sdf_directory, 'forward_camera.sdf')
elemental_camera_sdf = os.path.join(sdf_directory, 'elemental_camera.sdf')

corner_measurement_sigma = 2.0

vloc_args = [{
    'use_sim_time': False,  # Don't use /clock
    'publish_tfs': 1,  # Publish drone and camera /tf
    'stamp_msgs_with_current_time': 1,  # Stamp with now()
    'map_init_pose_z': 0,
    'sub_camera_info_best_effort_not_reliable': 1,
    'publish_tfs_per_marker': 0,
    'publish_image_marked': 1,
    'camera_frame_id': 'forward_camera',
    'publish_base_pose': 1,
    'publish_camera_odom': 1,
    'publish_base_odom': 1,
    'cv4_corner_refinement_method': 2,
    'localize_camera_sam_not_cv': 1,
    'elemental_parameter': 5,
}]

vmap_args = [{
    'use_sim_time': False,  # Don't use /clock
    'publish_tfs': 1,  # Publish marker /tf
    'marker_length': 0.1775,  # Marker length
    'marker_map_save_full_filename': created_map_filename,
    'marker_map_load_full_filename': existing_map_filename,
    'corner_measurement_sigma': corner_measurement_sigma,
    'build_marker_map_skip_images' : 3,
    'build_marker_map_cmd': 'none',
}]



def generate_launch_description():
    entities = [
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_filename
        ], output='screen'),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', node_executable='joy_node', output='screen'),

        # Add forward-facing camera to the simulation
        Node(package='sim_fiducial', node_executable='inject_entity.py', output='screen',
             arguments=[elemental_camera_sdf, '0', '0', '0', '0', '0', '0']),

        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             parameters=vloc_args, node_namespace='forward_camera'),
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             parameters=vmap_args),
    ]

    return LaunchDescription(entities)
