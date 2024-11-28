# Robotic-Arm-ROS-Base

# Test

```shell

source install/setup.bash \
ros2 launch rosa rviz.launch.py

source install/setup.bash \
ros2 launch rosa ${SCENE}.launch.py (e.g., redbird.launch)


source install/setup.bash \
ros2 launch viewpoint_manager rviz.launch.py
source install/setup.bash \ 
ros2 launch viewpoint_manager mbs.launch.py


source install/setup.bash \
ros2 launch hierarchical_coverage_planner rviz.launch.py
source install/setup.bash \
ros2 launch hierarchical_coverage_planner mbs.launch.py

```