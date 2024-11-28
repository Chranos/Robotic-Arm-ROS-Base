# `PlanningVisualization` 类总结

该代码文件是一个基于 **ROS2** 的可视化类 `PlanningVisualization` 的实现，主要用于处理路径规划与点云数据的可视化任务。通过生成和发布3D几何形状、点云数据和路径信息，开发者能够在 **ROS2** 环境中进行调试和观察。

## 主要功能和模块

### 1. 点云可视化
- 代码通过 **`pcl::PointCloud`** 库管理点云数据，并将其转换为 **ROS2** 消息格式（`sensor_msgs::msg::PointCloud2`）。
- 关键函数：
  - `publishOptArea`：发布优化区域的点云数据。
  - `publishSurface`：发布表面点云。
  - `publishVisiblePoints`：发布可见点云。

### 2. 路径与几何可视化
- 使用 **`visualization_msgs::msg::Marker`** 和 **`visualization_msgs::msg::MarkerArray`** 创建3D模型、箭头、线段等几何形状，帮助可视化路径、点位和其他视觉元素。
- 关键函数：
  - `publishYawTraj`：发布航向轨迹。
  - `publishTravelTraj`：发布旅行路径。
  - `publishGlobalBoundary`：发布全局边界路径。

### 3. 方向与法线可视化
- 通过函数 **`publishRevisedNormal`** 和 **`publishROSAOrientation`** 发布法线向量，以便显示点云的法线方向。
- 通过 **`geometry_msgs::msg::Point`** 定义3D点，使用 **`Marker::ARROW`** 标记类型展示方向。

### 4. 随机颜色生成
- 使用 **`rand()`** 生成随机RGB颜色值，在可视化元素中应用，以区分不同的路径和对象。

### 5. 路径与空间分解
- 代码中包含路径规划相关函数，例如：
  - `publishLocalPath`：发布局部路径信息。
  - `publishGlobalSeq`：发布全局路径序列。
  
### 6. 姿态和视角发布
- 函数 **`publishCheckNeigh`** 和 **`publishCheckCP`** 发布姿态和视角数据，展示不同点的空间关系和摄像机的视角。

## ROS2 特性

### 1. 时间机制
- 使用 **`rclcpp::Clock().now()`** 替换 **ROS1** 中的 `ros::Time::now()`，确保时间戳与 **ROS2** 标准一致。

### 2. 消息类型更新
- 所有消息类型改为 **`msg`** 命名空间下的 **ROS2** 消息类型，如 `visualization_msgs::msg::MarkerArray` 和 `sensor_msgs::msg::PointCloud2`。

### 3. Publisher 改进
- 发布器使用了 **`SharedPtr`**，通过 `->publish()` 来发布消息，这是 **ROS2** 的标准做法。

## 总结
该代码实现了路径规划和点云处理的可视化功能。其核心在于通过 **ROS2** 环境发布各种3D标记、点云和路径信息，帮助开发者在 **RViz** 等工具中查看和调试路径规划过程。

- **点云可视化**：发布并显示点云。
- **路径可视化**：通过3D标记显示规划路径和边界。
- **法线和方向**：展示点云的法线方向和位姿。

通过这些功能，开发者能够更加直观地理解路径规划和点云处理的过程，并在仿真环境中调试与验证。
