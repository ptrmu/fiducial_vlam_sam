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

world_name = "circle_of_markers"
created_map_filename = os.path.join(worlds_directory, 'sim_make_map.yaml')
existing_map_filename = os.path.join(worlds_directory, world_name + '_map.yaml')
world_filename = os.path.join(worlds_directory, world_name + '.world')

# camera_sdf = os.path.join(sdf_directory, 'forward_camera.sdf')
# camera_sdf = os.path.join(sdf_directory, 'elemental_camera.sdf')
camera_sdf = os.path.join(sdf_directory, 'dual_camera.sdf')

corner_measurement_sigma = 2.0

vdet_left_args = [{
    'det_aruco_dictionary_id': 9,  # aruco marker dictionary
    'det_marker_length': 0.1775,  # Marker length
    'det_t_base_camera_y': 0.2,
    'det_pub_observations_frame_id': 'base_link',
    'det_pub_camera_info_frame_id': 'left_camera'
}]

vdet_right_args = [{
    'det_aruco_dictionary_id': 9,  # aruco marker dictionary
    'det_marker_length': 0.1775,  # Marker length
    'det_t_base_camera_y': -0.2,
    'det_pub_observations_frame_id': 'base_link',
    'det_pub_camera_info_frame_id': 'right_camera'
}]

vloc_args = [{
    'loc_camera_algorithm': 1,
    'loc_corner_refinement_method': 2,
    'loc_aruco_dictionary_id': 9,  # aruco marker dictionary
    'psl_pub_tf_camera_enable': True,  # Publish drone and camera /tf
    'psl_sub_camera_info_topic': 'right/camera_info',
    'psl_sub_camera_info_best_effort_not_reliable': True,
    'psl_sub_image_raw_topic': 'right/image_raw',
    'psl_sub_image_raw_best_effort_not_reliable': True,
    'psl_pub_tf_camera_child_frame_id': 'forward_camera',
    'psl_pub_camera_odom_enable': False,
    'psl_pub_base_pose_enable': True,
    'psl_pub_base_odom_enable': False,
    'psl_pub_tf_camera_per_marker_enable': True,
    'psl_pub_tf_marker_per_marker_enable': True,
}]

vmap_args = [{
    'map_marker_length': 0.1775,  # Marker length
    'map_save_filename': created_map_filename,
    'map_load_filename': existing_map_filename,
    'map_corner_measurement_sigma': corner_measurement_sigma,
    'map_cmd': '',
    'bmm_algorithm': 1,
    'bmm_recorded_observations_name': 'recorded_observations.yaml'
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
        # Node(package='joy', node_executable='joy_node', output='screen'),

        # Add camera to the simulation
        Node(package='sim_fiducial', executable='inject_entity.py', output='screen',
             arguments=[camera_sdf, '0', '0', '0', '0', '0', '0']),

        Node(package='fiducial_vlam', executable='vdet_main', output='screen',
             parameters=vdet_left_args, namespace='/dual_camera/left'),
        Node(package='fiducial_vlam', executable='vdet_main', output='screen',
             parameters=vdet_right_args, namespace='/dual_camera/right'),

        # Node(package='fiducial_vlam', executable='vloc_main', output='screen',
        #      parameters=vloc_args, namespace='dual_camera_x'),
        # Node(package='fiducial_vlam', executable='vmap_main', output='screen',
        #      parameters=vmap_args),
    ]

    return LaunchDescription(entities)
