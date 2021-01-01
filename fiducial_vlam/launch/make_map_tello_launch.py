import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

vlam_package_share_directory = get_package_share_directory('fiducial_vlam')
vlam_cfg_directory = os.path.join(vlam_package_share_directory, 'cfg')

corner_measurement_sigma = 2.0
created_map_filename = "nook_walls_map_new.yaml"
existing_map_filename = "nook_walls_map_new.yaml"

opencv_tello_29e_params = [{
    'drone_ip': '192.168.0.35',
    'command_port': 38065,
    'drone_port': 8889,
    'data_port': 8890,
    'video_port': 11111,
    'image_raw_frame_id': 'camera',
    'pub_image_raw_best_effort_not_reliable': 0,
    'pub_camera_info_best_effort_not_reliable': 0
}]


vloc_args = [{
    'use_sim_time': False,  # Don't use /clock
    'psl_publish_tfs': 1,  # Publish drone and camera /tf
    'loc_aruco_dictionary_id': 0,  # aruco marker dictionary
    'psl_stamp_msgs_with_current_time': 0,  # Stamp with now()
    'psl_sub_camera_info_best_effort_not_reliable': 0,
    'psl_publish_camera_tf_per_marker': 0,
    'psl_publish_marker_tf_per_marker' : 0,
    'psl_publish_image_marked': 1,
    'psl_camera_frame_id': 'camera',
    'psl_publish_base_pose': 1,
    'psl_publish_camera_odom': 1,
    'psl_publish_base_odom': 1,
    'loc_cv4_corner_refinement_method': 2,
    'loc_camera_sam_not_cv': 0,
}]

vmap_args = [{
    'use_sim_time': False,  # Don't use /clock
    'psm_publish_tfs': 1,  # Publish marker /tf
    'map_marker_length': 0.21,  # Marker length
    'map_save_filename': created_map_filename,
    'map_load_filename': existing_map_filename,
    'map_corner_measurement_sigma': corner_measurement_sigma,
    'map_skip_images': 5,
    'map_cmd': 'none',
}]

def generate_launch_description():
    entities = [
        Node(package='tello_driver', executable='tello_driver_main', output='screen',
             parameters=opencv_tello_29e_params),

        Node(package='fiducial_vlam', executable='vloc_main', output='screen',
             parameters=vloc_args),
        Node(package='fiducial_vlam', executable='vmap_main', output='screen',
             parameters=vmap_args),
    ]

    return LaunchDescription(entities)
