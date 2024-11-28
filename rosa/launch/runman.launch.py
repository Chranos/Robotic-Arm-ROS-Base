# <launch>

# <node pkg ="rosa" name ="rosa_exec" type ="rosa_exec" output = "screen" required ="true">
#   <param name="rosa_main/estimation_num" value="2000" type="int"/>
  
#   <!-- /* RUNMAN */ -->
#   <param name="rosa_main/pcd" value="$(find rosa)/data/camel.pcd" type="string"/>
#   <param name="rosa_main/radius" value="0.1" type="double"/>
#   <param name="rosa_main/th_mah" value="0.01" type="double"/>
#   <param name="rosa_main/delta" value="0.01" type="double"/>
#   <param name="rosa_main/num_drosa" value="5" type="int"/>
#   <param name="rosa_main/num_dcrosa" value="2" type="int"/>
#   <param name="rosa_main/k_KNN" value="6" type="int"/>
#   <param name="rosa_main/sample_r" value="0.05" type="double"/>
#   <param name="rosa_main/alpha" value="0.5" type="double"/>
#   <param name="rosa_main/pt_downsample_size" value="0.02" type="double"/>
#   <param name="rosa_main/estimation_number" value="5" type="int"/>
#   <param name="rosa_main/upper_bound_angle_inner_decomp" value="45.0" type="double"/>
#   <param name="rosa_main/upper_bound_length_inner_decomp" value="1.0" type="double"/>
#   <param name="rosa_main/Prune" value="false" type="bool"/>
#   <param name="rosa_main/lower_bound_length" value="0.2" type="double"/>
#   <param name="rosa_main/lower_bound_prune_angle" value="75.0" type="double"/>
#   <param name="rosa_main/upper_bound_original_points_num" value="10000" type="int"/>
#   <param name="rosa_main/Ground" value="false" type="bool"/>
# </node>

# </launch>

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 查找 rosa 包的共享目录
    rosa_share = FindPackageShare(package='rosa').find('rosa')
    pcd_file_path = os.path.join(rosa_share, 'data', 'camel.pcd')

    return LaunchDescription([
        Node(
            package='rosa',
            executable='rosa_exec',
            name='rosa_exec',
            output='screen',
            parameters=[{
                'rosa_main/estimation_num': 2000,
                'rosa_main/pcd': pcd_file_path,  # 动态查找 pcd 文件路径
                'rosa_main/radius': 0.1,
                'rosa_main/th_mah': 0.01,
                'rosa_main/delta': 0.01,
                'rosa_main/num_drosa': 5,
                'rosa_main/num_dcrosa': 2,
                'rosa_main/k_KNN': 6,
                'rosa_main/sample_r': 0.05,
                'rosa_main/alpha': 0.5,
                'rosa_main/pt_downsample_size': 0.02,
                'rosa_main/estimation_number': 5,
                'rosa_main/upper_bound_angle_inner_decomp': 45.0,
                'rosa_main/upper_bound_length_inner_decomp': 1.0,
                'rosa_main/Prune': False,
                'rosa_main/lower_bound_length': 0.2,
                'rosa_main/lower_bound_prune_angle': 75.0,
                'rosa_main/upper_bound_original_points_num': 10000,
                'rosa_main/Ground': False,
            }],
        )
    ])
