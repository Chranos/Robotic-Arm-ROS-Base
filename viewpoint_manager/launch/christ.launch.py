import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('fov_h', default_value='55.0', description='Horizontal Field of View'),
        DeclareLaunchArgument('fov_w', default_value='75.0', description='Vertical Field of View'),
        DeclareLaunchArgument('max_dist', default_value='8.0', description='Maximum distance for viewpoints'),
        DeclareLaunchArgument('resolution_', default_value='0.4', description='Resolution for the map'),

        # Viewpoint Manager Node
        Node(
            package='viewpoint_manager',
            executable='skeleton_viewpoint_node',
            name='skeleton_viewpoint_node',
            output='screen',
            parameters=[{
                'rosa_main/estimation_num': 1000,
                'rosa_main/pcd': os.path.join(FindPackageShare('rosa').find('rosa'), 'data/christ.pcd'),
                'rosa_main/radius': 0.1,
                'rosa_main/th_mah': 0.01,
                'rosa_main/delta': 0.01,
                'rosa_main/num_drosa': 5,
                'rosa_main/num_dcrosa': 2,
                'rosa_main/k_KNN': 6,
                'rosa_main/sample_r': 0.05,
                'rosa_main/alpha': 0.3,
                'rosa_main/pt_downsample_size': 0.02,
                'rosa_main/estimation_number': 10,
                'rosa_main/upper_bound_angle_inner_decomp': 45.0,
                'rosa_main/upper_bound_length_inner_decomp': 1.0,
                'rosa_main/Prune': True,
                'rosa_main/lower_bound_length': 0.2,
                'rosa_main/lower_bound_prune_angle': 75.0,
                'rosa_main/upper_bound_original_points_num': 10000,
                'rosa_main/Ground': False,

                # Mapping parameters
                'hcmap/resolution': LaunchConfiguration('resolution_'),
                'hcmap/interval': 1.0,
                'hcmap/plane_thickness': 3.0,
                'hcmap/dilateRadius': 0.5,
                'hcmap/checkScale': 5.0,
                'hcmap/checkSize': 3,
                'hcmap/inflateVoxel': 3,

                # Perception parameters
                'perception_utils/top_angle': LaunchConfiguration('fov_h') + '*3.1415926/360.0',
                'perception_utils/left_angle': LaunchConfiguration('fov_w') + '*3.1415926/360.0',
                'perception_utils/right_angle': LaunchConfiguration('fov_w') + '*3.1415926/360.0',
                'perception_utils/max_dist': LaunchConfiguration('max_dist'),
                'perception_utils/vis_dist': 1.8,

                # Viewpoint Manager parameters
                'viewpoint_manager/visible_range': LaunchConfiguration('max_dist'),
                'viewpoint_manager/viewpoints_distance': 6.0,
                'viewpoint_manager/fov_h': LaunchConfiguration('fov_h'),
                'viewpoint_manager/fov_w': LaunchConfiguration('fov_w'),
                'viewpoint_manager/pitch_upper': 70.0,
                'viewpoint_manager/pitch_lower': -85.0,
                'viewpoint_manager/zGround': False,
                'viewpoint_manager/GroundPos': -7.4,
                'viewpoint_manager/safeHeight': 1.5,
                'viewpoint_manager/safe_radius': 3.0,
                'viewpoint_manager/attitude_type': 'all',  # attitude_type : 1) yaw 2) all -> (pitch, yaw)
                'viewpoint_manager/max_iter_num': 2,
                'viewpoint_manager/pose_update': True,
            }],
        ),
    ])
