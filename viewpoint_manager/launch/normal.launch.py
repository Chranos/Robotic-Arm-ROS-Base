import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('fov_h', default_value='70.0', description='Horizontal field of view'),
        DeclareLaunchArgument('fov_w', default_value='85.0', description='Vertical field of view'),
        DeclareLaunchArgument('max_dist', default_value='3.2', description='Maximum distance'),
        DeclareLaunchArgument('resolution_', default_value='0.2', description='Resolution'),

        # Launch the viewpoint_manager node with its parameters
        Node(
            package='viewpoint_manager',
            executable='viewpoint_manager_node',
            name='viewpoint_manager',
            output='screen',
            parameters=[{
                # Viewpoint Manager Node parameters
                'viewpoint_manager_node/cloud_path': os.path.join(get_package_share_directory('viewpoint_manager'), 'resource', 'office_surface.pcd'),
                'viewpoint_manager_node/free_cloud_path': os.path.join(get_package_share_directory('viewpoint_manager'), 'resource', 'office_free.pcd'),
                'viewpoint_manager_node/downsampled_size': 0.3,
                'viewpoint_manager_node/min_box_z': 1.5,
                'viewpoint_manager_node/max_box_z': 2.5,
                'viewpoint_manager_node/downsampled_size_for_viewpoint': 0.35,

                # Mapping parameters
                'hcmap/resolution': LaunchConfiguration('resolution_'),  # Argument passed
                'hcmap/interval': 1.0,
                'hcmap/plane_thickness': 3.0,
                'hcmap/dilateRadius': 0.5,
                'hcmap/checkScale': 5.0,
                'hcmap/checkSize': 7,
                'hcmap/inflateVoxel': 4,

                # Perception parameters
                'perception_utils/top_angle': LaunchConfiguration('fov_h') + ' * 3.1415926 / 360.0',  # Ensure the angle conversion
                'perception_utils/left_angle': LaunchConfiguration('fov_w') + ' * 3.1415926 / 360.0',  # Ensure the angle conversion
                'perception_utils/right_angle': LaunchConfiguration('fov_w') + ' * 3.1415926 / 360.0',  # Ensure the angle conversion
                'perception_utils/max_dist': LaunchConfiguration('max_dist'),
                'perception_utils/vis_dist': 0.9,

                # Viewpoint Manager parameters
                'viewpoint_manager/visible_range': LaunchConfiguration('max_dist'),
                'viewpoint_manager/viewpoints_distance': 2.5,
                'viewpoint_manager/fov_h': LaunchConfiguration('fov_h'),
                'viewpoint_manager/fov_w': LaunchConfiguration('fov_w'),
                'viewpoint_manager/pitch_upper': 70.0,
                'viewpoint_manager/pitch_lower': -85.0,
                'viewpoint_manager/zGround': True,
                'viewpoint_manager/GroundPos': 0.0,
                'viewpoint_manager/safeHeight': 0.5,
                'viewpoint_manager/safe_radius': 0.4,
                'viewpoint_manager/attitude_type': 'yaw',  # 'yaw' or 'all'
                'viewpoint_manager/max_iter_num': 3,
                'viewpoint_manager/pose_update': False,
            }]
        ),
    ])
