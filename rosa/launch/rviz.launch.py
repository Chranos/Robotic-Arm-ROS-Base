# <launch>
#   <node name="rvizvisualisation" pkg="rviz" type="rviz" output="log" args="-d $(find rosa)/config/traj.rviz" />
#   <node pkg="tf" type="static_transform_publisher" name="tf_53" args="0 0 0 0 0 0 world navigation 100" />
# </launch>
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 查找 rosa 包的共享目录
    rosa_share = FindPackageShare(package='rosa').find('rosa')
    rviz_config_path = os.path.join(rosa_share, 'config', 'traj.rviz')

    return LaunchDescription([
        # rviz 可视化节点
        # ExecuteProcess(
        #     cmd=['rviz2', '-d', rviz_config_path],
        #     output='log',
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rvizvisualisation',
            output='log',
            arguments=['-d', rviz_config_path],
        ),
        # static_transform_publisher 节点
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_53',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0','1.0', 'world', 'navigation'],
        )
    ])
