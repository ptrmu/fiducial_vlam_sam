import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


opencv_zed_params = [{
    'file': False,
    'index': 0,
    'half_image': 2, # 1 - left sensor, 2 - right sensor (when looking in the same direction as the ZED)
    'vcp_property0': 'CAP_PROP_FRAME_WIDTH', 'vcp_value0': 2560.,
    'vcp_property1': 'CAP_PROP_FRAME_HEIGHT', 'vcp_value1': 720.,
}]


opencv_lifecam_params = [{
    'file': False,
    'index': 0,
    'vcp_property0': 'CAP_PROP_FRAME_WIDTH', 'vcp_value0': 1280.,
    'vcp_property1': 'CAP_PROP_FRAME_HEIGHT', 'vcp_value1': 720.,
    'vcp_property2': 'CAP_PROP_AUTOFOCUS', 'vcp_value2': 0.,
    'vcp_property3': 'CAP_PROP_FOCUS', 'vcp_value3': 1.0,
}]

vloc_args = [{
    'loc_calibrate_not_loocalize' : 1,
}]

def generate_launch_description():

    entities = [
        Node(package='opencv_cam', node_executable='opencv_cam_main', output='screen',
             node_name='opencv_cam', parameters=opencv_zed_params),

        # Node(package='fiducial_vlam', node_executable='vloc_main', output='screen',
        #      parameters=vloc_args),
    ]

    return LaunchDescription(entities)

