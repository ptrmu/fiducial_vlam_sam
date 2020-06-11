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

opencv_zed_params = [{
    'file': False,
    'index': 0,
    'half_image': 2, # 1 - left sensor, 2 - right sensor (when looking in the same direction as the ZED)
    'camera_info_path': os.path.join(vlam_cfg_directory, "camera_info_ZED_right_image.yaml"),
    'vcp_property0': 'CAP_PROP_FRAME_WIDTH', 'vcp_value0': 2560.,
    'vcp_property1': 'CAP_PROP_FRAME_HEIGHT', 'vcp_value1': 720.,
}]


opencv_lifecam_params = [{
    'file': False,
    'index': 2,
    'vcp_property0': 'CAP_PROP_FRAME_WIDTH', 'vcp_value0': 1280.,
    'vcp_property1': 'CAP_PROP_FRAME_HEIGHT', 'vcp_value1': 720.,
    'vcp_property2': 'CAP_PROP_AUTOFOCUS', 'vcp_value2': 0.,
    'vcp_property3': 'CAP_PROP_FOCUS', 'vcp_value3': 1.0,
}]


vloc_args = [{
    'use_sim_time': False,  # Don't use /clock
    'psl_publish_tfs': 1,  # Publish drone and camera /tf
    'loc_aruco_dictionary_id': 0,  # aruco marker dictionary
    'psl_stamp_msgs_with_current_time': 0,  # Stamp with now()
    'psl_sub_camera_info_best_effort_not_reliable': 1,
    'psl_publish_tfs_per_marker': 0,
    'psl_publish_image_marked': 1,
    'psl_camera_frame_id': 'forward_camera',
    'psl_publish_base_pose': 1,
    'psl_publish_camera_odom': 1,
    'psl_publish_base_odom': 1,
    'loc_cv4_corner_refinement_method': 2,
    'loc_camera_sam_not_cv': 1,
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
        Node(package='opencv_cam', node_executable='opencv_cam_main', output='screen',
             node_name='opencv_cam', parameters=opencv_zed_params),

        Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
             parameters=vloc_args),
        Node(package='fiducial_vlam', node_executable='vmap_main', output='screen',
             parameters=vmap_args),
    ]

    return LaunchDescription(entities)
