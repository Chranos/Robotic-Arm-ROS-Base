import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('fov_h', default_value='55.0', description='Horizontal field of view'),
        DeclareLaunchArgument('fov_w', default_value='75.0', description='Vertical field of view'),
        DeclareLaunchArgument('max_dist', default_value='11.0', description='Maximum distance'),
        DeclareLaunchArgument('resolution_', default_value='0.4', description='Resolution'),

        # Launch the viewpoint_manager node with its parameters
        Node(
            package='viewpoint_manager',
            executable='skeleton_viewpoint_node',
            name='skeleton_viewpoint_node',
            output='screen',
            parameters=[{
                # Skeleton-based Space Decomposition Parameters
                'rosa_main/estimation_num': 2000,
                'rosa_main/pcd': os.path.join(get_package_share_directory('hierarchical_coverage_planner'), 'data', 'MBS.pcd'),
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
                'rosa_main/Prune': False,
                'rosa_main/lower_bound_length': 0.1,
                'rosa_main/lower_bound_prune_angle': 75.0,
                'rosa_main/upper_bound_original_points_num': 10000,
                'rosa_main/Ground': True,

                # Mapping parameters
                'hcmap/resolution': LaunchConfiguration('resolution_'),  # Argument passed
                'hcmap/interval': 1.0,
                'hcmap/plane_thickness': 3.0,
                'hcmap/dilateRadius': 0.5,
                'hcmap/checkScale': 5.0,
                'hcmap/checkSize': 7,
                'hcmap/inflateVoxel': 4,

                # Perception parameters
                'perception_utils/top_angle': LaunchConfiguration('fov_h') * 3.1415926 / 360.0,
                'perception_utils/left_angle': LaunchConfiguration('fov_w') * 3.1415926 / 360.0,
                'perception_utils/right_angle': LaunchConfiguration('fov_w') * 3.1415926 / 360.0,
                'perception_utils/max_dist': LaunchConfiguration('max_dist'),
                'perception_utils/vis_dist': 3.0,

                # Viewpoint Manager parameters
                'hcplanner/mesh': os.path.join(get_package_share_directory('hierarchical_coverage_planner'), 'data', 'mesh', 'MBS.obj'),
                'hcplanner/fullcloud': os.path.join(get_package_share_directory('hierarchical_coverage_planner'), 'data', 'MBSmore.pcd'),
                'hcplanner/model_downsample_size': 2.0,
                'hcplanner/sample_step4normal': 1.0,
                'viewpoint_manager/visible_range': LaunchConfiguration('max_dist'),
                'viewpoint_manager/viewpoints_distance': 8.0,
                'viewpoint_manager/fov_h': LaunchConfiguration('fov_h'),
                'viewpoint_manager/fov_w': LaunchConfiguration('fov_w'),
                'viewpoint_manager/pitch_upper': 70.0,
                'viewpoint_manager/pitch_lower': -85.0,
                'viewpoint_manager/zGround': True,
                'viewpoint_manager/GroundPos': -10.5,
                'viewpoint_manager/safeHeight': 1.5,
                'viewpoint_manager/safe_radius': 3.0,
                'viewpoint_manager/attitude_type': 'all',  # attitude_type : 1) yaw 2) all -> (pitch, yaw)
                'viewpoint_manager/max_iter_num': 10,
            }]
        ),
    ])
