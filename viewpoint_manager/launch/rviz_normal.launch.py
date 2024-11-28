import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Launch RViz visualization node
        Node(
            package='rviz',
            executable='rviz',
            name='rvizvisualisation',
            output='log',
            arguments=['-d', os.path.join(get_package_share_directory('viewpoint_manager'), 'config', 'viewpoint_manager.rviz')]
        ),

        # Launch static_transform_publisher node (8 parameters)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_53',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'navigation']
        ),
    ])
