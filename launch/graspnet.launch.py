"""Launch RealSense D435i and the GraspNet ROS node.

Camera topics match graspnet_node subscribers:
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw

MoveIt2 and RViz are not started here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_ws_setup = LaunchConfiguration('ros_ws_setup')
    config = PathJoinSubstitution([
        FindPackageShare('graspnet_ros2'), 'config', 'graspnet.yaml'
    ])
    realsense_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_ws_setup',
            default_value='',
            description='Path to the ROS workspace install/setup.bash that '
                        'provides graspnet_ros2; exported as ROS_WS_SETUP.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': '',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false',
                'rgb_camera.color_profile': '1280,720,30',
                'depth_module.depth_profile': '1280,720,30',
            }.items(),
        ),
        Node(
            package='graspnet_ros2',
            executable='graspnet_node.py',
            name='graspnet_node',
            output='screen',
            parameters=[config],
            additional_env={'ROS_WS_SETUP': ros_ws_setup},
        ),
    ])