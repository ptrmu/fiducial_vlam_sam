import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# A single camera scanning markers
#   Messages:
#       /forward_camera/camera_info, gazebo->vloc_node, msg::CameraInfo, frame_id: camera_frame
#       /forward_camera/image_raw, gazebo->vloc_node, msg::Image, frame_id: camera_frame
#       /fiducial_map, vmap_node->vloc_node, msg::Map, frame_id: map
#       /fiducial_visuals, vmap_node->, msg::Markers, frame_id: map
#           /fiducial_observations, vloc_node->vmap_node, msg::ObservationsSynced, frame_id: camera_frame, imager_frame
#           /forward_camera/image_marked, vloc_node->, msg::Image, frame_id: camera_frame
#           /forward_camera/camera_pose, vloc_node->, msg::PoseWithCovariance, frame_id: map
#           /forward_camera/camera_odom, vloc_node->, msg::Odometry, frame_id: map, camera_frame
#           /forward_camera/base_pose, vloc_node->, msg::PoseWithCovariance, frame_id: map
#           /forward_camera/base_odom, vloc_node->, msg::Odometry, frame_id: map, base_link

vlam_package_share_directory = get_package_share_directory('fiducial_vlam')
sim_package_share_directory = get_package_share_directory('sim_fiducial')

launch_directory = os.path.join(vlam_package_share_directory, 'launch')
worlds_directory = os.path.join(sim_package_share_directory, 'worlds')
sdf_directory = os.path.join(sim_package_share_directory, 'sdf')

world_name = "circle_of_markers"
created_map_filename = os.path.join(worlds_directory, 'sim_make_map.yaml')
existing_map_filename = os.path.join(worlds_directory, world_name + '_map.yaml')
world_filename = os.path.join(worlds_directory, world_name + '.world')
forward_camera_sdf = os.path.join(sdf_directory, 'forward_camera.sdf')
elemental_camera_sdf = os.path.join(sdf_directory, 'elemental_camera.sdf')

corner_measurement_sigma = 2.0

vloc_args = [{
    'loc_camera_algorithm': 1,
    'loc_gtsam_factor_type': 2,
    'det_corner_refinement_method': 2,
    'det_pub_observations_topic': '/fiducial_observations',
    # 'psl_pub_tf_camera_enable': True,  # Publish drone and camera /tf
    # 'psl_sub_camera_info_best_effort_not_reliable': True,
    # 'psl_sub_image_raw_best_effort_not_reliable': True,
    # 'psl_pub_tf_camera_child_frame_id': 'camera_frame',
    # 'psl_pub_camera_odom_enable': False,
    # 'psl_pub_base_pose_enable': True,
    # 'psl_pub_base_odom_enable': False,
    # 'psl_pub_tf_camera_per_marker_enable': True,
    # 'psl_pub_tf_marker_per_marker_enable': True,
}]

vmap_args = [{
    'map_marker_length': 0.1775,  # Marker length
    'map_aruco_dictionary_id': 9,  # aruco marker dictionary
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
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide injection endpoints
            world_filename
        ], output='screen'),

        # Joystick driver, generates /namespace/joy messages
        # Node(package='joy', node_executable='joy_node', output='screen'),

        # Add forward-facing camera to the simulation
        # Node(package='sim_fiducial', executable='inject_entity.py', output='screen',
        #      arguments=[forward_camera_sdf, '0', '0', '0', '0', '0', '0']),
        Node(package='sim_fiducial', executable='inject_entity.py', output='screen',
             arguments=[elemental_camera_sdf, '0', '0', '0', '0', '0', '0']),

        Node(package='fiducial_vlam', executable='vloc_main', output='screen',
             parameters=vloc_args, namespace='forward_camera'),
        Node(package='fiducial_vlam', executable='vmap_main', output='screen',
             parameters=vmap_args),
    ]

    return LaunchDescription(entities)
