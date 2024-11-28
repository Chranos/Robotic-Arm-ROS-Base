/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the execution file of SSD, which individually implements the 
 *                   SSD module.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Planner is free software: you can redistribute it and/or 
 *                   modify it under the terms of the GNU Lesser General Public 
 *                   License as published by the Free Software Foundation, 
 *                   either version 3 of the License, or (at your option) any 
 *                   later version.
 *                   FC-Planner is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty 
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

// #include <rosa/rosa_main.h>
// #include <ros/ros.h>

// using namespace predrecon;

// int main(int argc, char** argv){
//     ros::init(argc, argv, "rosa_exec");
//     ros::NodeHandle nh("~");

//     ROSA_main rm;
//     rm.init(nh);
//     rm.main();
//     ros::Duration(1.0).sleep();
//     ros::spin();
//     return 0; 
// }

#include <rclcpp/rclcpp.hpp>

#include <rosa/rosa_main.h>  // 假设这个头文件仍然适用
#define PCL_NO_PRECOMPILE

using namespace predrecon;

int main(int argc, char** argv) {
    // ROS2初始化
    rclcpp::init(argc, argv);

    // 创建节点对象，ROS2中没有类似NodeHandle的概念，直接创建节点对象
    auto node = std::make_shared<rclcpp::Node>("rosa_exec");

    // 实例化你的 ROSA_main 类
    ROSA_main rm;
    rm.init(node);  // 假设 ROSA_main 的 init() 方法可以接受 std::shared_ptr<rclcpp::Node> 作为参数
    rm.main();

    // ROS2中的 sleep 使用 WallRate 或者定时器，但最简单的方式是直接使用睡眠
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ROS2中的spin
    rclcpp::spin(node);

    // 退出时需要调用 shutdown
    rclcpp::shutdown();

    return 0;
}
