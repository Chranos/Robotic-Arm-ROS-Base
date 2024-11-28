#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h> 
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <chrono>

#include <fstream>

namespace predrecon {
MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

int MapROS::sgn(double& x)
{
  if (x>=0)
    return 1;
  else
    return -1;
}

void MapROS::init() {
  // node_.param("map_ros/fx", fx_, -1.0);
  // node_.param("map_ros/fy", fy_, -1.0);
  // node_.param("map_ros/cx", cx_, -1.0);
  // node_.param("map_ros/cy", cy_, -1.0);
  // node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  // node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  // node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  // node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  // node_.param("map_ros/skip_pixel", skip_pixel_, -1);

  // node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  // node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
  // node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
  // node_.param("map_ros/show_occ_time", show_occ_time_, false);
  // node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  // node_.param("map_ros/show_all_map", show_all_map_, false);
  // node_.param("map_ros/frame_id", frame_id_, string("world"));


  node_->declare_parameter("map_ros/fx", -1.0);
  node_->declare_parameter("map_ros/fy", -1.0);
  node_->declare_parameter("map_ros/cx", -1.0);
  node_->declare_parameter("map_ros/cy", -1.0);
  node_->declare_parameter("map_ros/depth_filter_maxdist", -1.0);
  node_->declare_parameter("map_ros/depth_filter_mindist", -1.0);
  node_->declare_parameter("map_ros/depth_filter_margin", -1);
  node_->declare_parameter("map_ros/k_depth_scaling_factor", -1.0);
  node_->declare_parameter("map_ros/skip_pixel", -1);

  node_->declare_parameter("map_ros/esdf_slice_height", -0.1);
  node_->declare_parameter("map_ros/visualization_truncate_height", -0.1);
  node_->declare_parameter("map_ros/visualization_truncate_low", -0.1);
  node_->declare_parameter("map_ros/show_occ_time", false);
  node_->declare_parameter("map_ros/show_esdf_time", false);
  node_->declare_parameter("map_ros/show_all_map", false);
  node_->declare_parameter("map_ros/frame_id", std::string("world"));

  // 获取参数值
  node_->get_parameter("map_ros/fx", fx_);
  node_->get_parameter("map_ros/fy", fy_);
  node_->get_parameter("map_ros/cx", cx_);
  node_->get_parameter("map_ros/cy", cy_);
  node_->get_parameter("map_ros/depth_filter_maxdist", depth_filter_maxdist_);
  node_->get_parameter("map_ros/depth_filter_mindist", depth_filter_mindist_);
  node_->get_parameter("map_ros/depth_filter_margin", depth_filter_margin_);
  node_->get_parameter("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_);
  node_->get_parameter("map_ros/skip_pixel", skip_pixel_);

  node_->get_parameter("map_ros/esdf_slice_height", esdf_slice_height_);
  node_->get_parameter("map_ros/visualization_truncate_height", visualization_truncate_height_);
  node_->get_parameter("map_ros/visualization_truncate_low", visualization_truncate_low_);
  node_->get_parameter("map_ros/show_occ_time", show_occ_time_);
  node_->get_parameter("map_ros/show_esdf_time", show_esdf_time_);
  node_->get_parameter("map_ros/show_all_map", show_all_map_);
  node_->get_parameter("map_ros/frame_id", frame_id_);


  // K matrix
  K_depth_.setZero();
  K_depth_(0, 0) = fx_; //fx
  K_depth_(1, 1) = fy_; //fy
  K_depth_(0, 2) = cx_; //cx
  K_depth_(1, 2) = cy_; //cy
  K_depth_(2, 2) = 1.0;

  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  // proj_points_.reserve(640 * 480 / map_->mp_->skip_pixel_ / map_->mp_->skip_pixel_);
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  // esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
  // vis_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);

  // map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
  // map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
  // map_local_inflate_pub_ =
  //     node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  // unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  // esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  // update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
  // depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);
  // lidar_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/lidar_cloud", 10);
  // // global planning vis pub
  // arrow_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/sdf_map/normal", 120);
  // vp_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/sdf_map/vp_global", 120);
  // init_tour_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/sdf_map/init_global", 10);
  // pred_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/pred_cloud", 10);

  // depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/depth", 50));
  // cloud_sub_.reset(
  //     new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
  // pose_sub_.reset(
  //     new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/map_ros/pose", 25));

  // sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
  //     MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  // sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
  // sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
  //     MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  // sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  // map_start_time_ = ros::Time::now();

  // 创建定时器
  esdf_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&MapROS::updateESDFCallback, this));
  vis_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&MapROS::visCallback, this));

  // 创建发布者
  map_all_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/sdf_map/update_range", 10);
  depth_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/depth_cloud", 10);
  lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/lidar_cloud", 10);

  // 创建MarkerArray类型的发布者
  arrow_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sdf_map/normal", 120);
  vp_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sdf_map/vp_global", 120);
  init_tour_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sdf_map/init_global", 10);
  pred_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/pred_cloud", 10);

  // 创建订阅者和同步器
  // depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node_.get(), "/map_ros/depth", 50);

  // 自定义 QoS，设置队列大小为50
  rclcpp::QoS custom_qos = rclcpp::SensorDataQoS().keep_last(50);

  // 将 QoS 转换为 rmw_qos_profile_t
  rmw_qos_profile_t qos_profile = custom_qos.get_rmw_qos_profile();

  // 使用转换后的 QoS 进行订阅
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(
      node_.get(), "/map_ros/depth", qos_profile));
// rmw_qos_profile_t qos = rmw_qos_profile_default;
// auto subscriber = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
//     node_.get(), "your_topic", qos);



  // cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(node_.get(), "/map_ros/cloud", custom_qos);
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_.get(),"/map_ros/cloud", qos_profile));


  custom_qos = rclcpp::SensorDataQoS().keep_last(25);
  qos_profile = custom_qos.get_rmw_qos_profile();
  // pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(node_.get(), "/map_ros/pose", custom_qos);
  pose_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(node_.get(),"/map_ros/pose", qos_profile));



  // 创建消息同步器
  sync_image_pose_ = std::make_shared<message_filters::Synchronizer<MapROS::SyncPolicyImagePose>>(
      MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_);
  sync_image_pose_->registerCallback(std::bind(&MapROS::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

  sync_cloud_pose_ = std::make_shared<message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>>(
      MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_);
  sync_cloud_pose_->registerCallback(std::bind(&MapROS::cloudPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 获取当前时间
  map_start_time_ = node_->now();



  // set sample param
  // r_min = 1.5;
  // r_max = 3.5;
  // z_range = 2.5;
  // theta_sample = 45.0*M_PI/180.0;
  // node_.param("global/sample_phi_range_", phi_sample, -1.0);
  // a_step = 30.0*M_PI/180.0;
  // theta_thre = 25.0*M_PI/180.0;
  // r_num = 2;
  // z_step = 2;

  // min_dist = 0.75;
  // refine_num = 7;
  // refine_radius = 5.0;
  // top_view_num = 15;
  // max_decay = 0.8;
  // downsample_c = 10.0;

  // double resolution_ = map_->getResolution();
  // Eigen::Vector3d origin, size;
  // map_->getRegion(origin, size);
  // raycaster_.reset(new RayCaster);
  // raycaster_->setParams(resolution_, origin);
  r_min = 1.5;
  r_max = 3.5;
  z_range = 2.5;
  theta_sample = 45.0 * M_PI / 180.0;

  // 使用 declare_parameter 和 get_parameter 代替原来的 param
  node_->declare_parameter("global/sample_phi_range_", -1.0);
  node_->get_parameter("global/sample_phi_range_", phi_sample);

  a_step = 30.0 * M_PI / 180.0;
  theta_thre = 25.0 * M_PI / 180.0;
  r_num = 2;
  z_step = 2;

  min_dist = 0.75;
  refine_num = 7;
  refine_radius = 5.0;
  top_view_num = 15;
  max_decay = 0.8;
  downsample_c = 10.0;

  double resolution_ = map_->getResolution();
  Eigen::Vector3d origin, size;
  map_->getRegion(origin, size);

  // 使用 std::make_shared 创建智能指针
  raycaster_.reset(new RayCaster);
  raycaster_->setParams(resolution_, origin);




}

// void MapROS::visCallback(const ros::TimerEvent& e) {
//   publishMapLocal();
//   if (show_all_map_) {
//   // Limit the frequency of all map
//     static double tpass = 0.0;
//     tpass += (e.current_real - e.last_real).toSec();
//     if (tpass > 0.1) {
//       publishMapAll();
//       tpass = 0.0;
//     }
//   }
//   // publishPred_INTERNAL();
//   // publishPredCloud();
//   // publishESDF();

//   // publishUpdateRange();
//   // publishDepth();
// }

void MapROS::visCallback() {
  publishMapLocal();

  if (show_all_map_) {
    // 使用静态变量记录经过的时间
    static double tpass = 0.0;

    // 获取当前时间
    static rclcpp::Time last_time = node_->now();
    rclcpp::Time current_time = node_->now();

    // 计算时间间隔并累加到tpass
    tpass += (current_time - last_time).seconds();

    // 更新last_time为当前时间
    last_time = current_time;

    // 如果时间间隔大于0.1秒，发布全地图
    if (tpass > 0.1) {
      publishMapAll();
      tpass = 0.0;  // 重置时间累计器
    }
  }

  // 其他可选的发布操作
  // publishPred_INTERNAL();
  // publishPredCloud();
  // publishESDF();
  // publishUpdateRange();
  // publishDepth();
}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



// void MapROS::updateESDFCallback(const ros::TimerEvent& /*event*/) {
//   if (!esdf_need_update_) return;
//   auto t1 = ros::Time::now();

//   map_->updateESDF3d();
//   esdf_need_update_ = false;

//   auto t2 = ros::Time::now();
//   esdf_time_ += (t2 - t1).toSec();
//   max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
//   esdf_num_++;
//   if (show_esdf_time_)
//     ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
//              max_esdf_time_);
// }

void MapROS::updateESDFCallback() {
  if (!esdf_need_update_) return;

  // 获取当前时间
  auto t1 = node_->now();

  // 更新ESDF
  map_->updateESDF3d();
  esdf_need_update_ = false;

  // 获取当前时间
  auto t2 = node_->now();

  // 计算时间差
  esdf_time_ += (t2 - t1).seconds();
  max_esdf_time_ = std::max(max_esdf_time_, (t2 - t1).seconds());
  esdf_num_++;

  // 打印日志
  if (show_esdf_time_) {
    RCLCPP_WARN(node_->get_logger(), "ESDF t: cur: %lf, avg: %lf, max: %lf", 
                (t2 - t1).seconds(), esdf_time_ / esdf_num_, max_esdf_time_);
  }
}



// void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
//                                const nav_msgs::OdometryConstPtr& pose) {
//   // AirSim only
//   Eigen::Quaterniond airsim_q_;
//   airsim_q_.w() = cos(0.25*M_PI);
//   airsim_q_.x() = 0.0;
//   airsim_q_.y() = 0.0;
//   airsim_q_.z() = sin(0.25*M_PI);

//   Eigen::Vector3d body_pos_, camera2body_XYZ;
//   camera2body_XYZ << 0.125, 0, 0;// hyper param
//   body_pos_(0) = pose->pose.pose.position.x;
//   body_pos_(1) = pose->pose.pose.position.y;
//   body_pos_(2) = pose->pose.pose.position.z;

//   Eigen::Quaterniond body_q_;
//   Eigen::Matrix3d Rotation_matrix, camera2body_rotation;
//   body_q_ = Eigen::Quaterniond(pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
//                                  pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);
//   // body_q_ = airsim_q_*body_q_;
//   Rotation_matrix = body_q_.toRotationMatrix();
//   camera2body_rotation << 0, 0, 1,
//                           -1, 0, 0,
//                           0, -1, 0;
//   Eigen::Quaterniond c_to_b(camera2body_rotation);

//   camera_pos_ = Rotation_matrix * camera2body_XYZ + body_pos_;
//   camera_q_ = body_q_;

//   if (!map_->isInMap(camera_pos_))  // exceed mapped region
//     return;
  
//   cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
//   if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
//     (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
//   cv_ptr->image.copyTo(*depth_image_);
  
//   auto t1 = ros::Time::now();
//   // generate point cloud, update map
//   proessDepthImage();
//   // ------------------------------------------------

//   map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
//   if (local_updated_) {
//     map_->clearAndInflateLocalMap();
//     esdf_need_update_ = true;
//     local_updated_ = false;
//   }

//   auto t2 = ros::Time::now();
//   fuse_time_ += (t2 - t1).toSec();
//   max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
//   fuse_num_ += 1;
//   if (show_occ_time_)
//     ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
//              max_fuse_time_);
// }


void MapROS::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                               const nav_msgs::msg::Odometry::ConstSharedPtr& pose) {
  // AirSim only
  Eigen::Quaterniond airsim_q_;
  airsim_q_.w() = cos(0.25 * M_PI);
  airsim_q_.x() = 0.0;
  airsim_q_.y() = 0.0;
  airsim_q_.z() = sin(0.25 * M_PI);

  Eigen::Vector3d body_pos_, camera2body_XYZ;
  camera2body_XYZ << 0.125, 0, 0;  // hyper param
  body_pos_(0) = pose->pose.pose.position.x;
  body_pos_(1) = pose->pose.pose.position.y;
  body_pos_(2) = pose->pose.pose.position.z;

  Eigen::Quaterniond body_q_;
  Eigen::Matrix3d Rotation_matrix, camera2body_rotation;
  body_q_ = Eigen::Quaterniond(pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
                               pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);
  // body_q_ = airsim_q_*body_q_;
  Rotation_matrix = body_q_.toRotationMatrix();
  camera2body_rotation << 0, 0, 1,
                          -1, 0, 0,
                          0, -1, 0;
  Eigen::Quaterniond c_to_b(camera2body_rotation);

  camera_pos_ = Rotation_matrix * camera2body_XYZ + body_pos_;
  camera_q_ = body_q_;

  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  // 获取当前时间
  auto t1 = node_->now();

  // generate point cloud, update map
  proessDepthImage();

  // 更新地图
  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  // 获取当前时间
  auto t2 = node_->now();

  // 计算融合时间
  fuse_time_ += (t2 - t1).seconds();
  max_fuse_time_ = std::max(max_fuse_time_, (t2 - t1).seconds());
  fuse_num_ += 1;

  // 打印日志
  if (show_occ_time_) {
    RCLCPP_WARN(node_->get_logger(), "Fusion t: cur: %lf, avg: %lf, max: %lf", 
                (t2 - t1).seconds(), fuse_time_ / fuse_num_, max_fuse_time_);
  }
}



// void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
//                                const nav_msgs::OdometryConstPtr& pose) {
//   camera_pos_(0) = pose->pose.pose.position.x;
//   camera_pos_(1) = pose->pose.pose.position.y;
//   camera_pos_(2) = pose->pose.pose.position.z;
//   camera_q_ = Eigen::Quaterniond(pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
//                                  pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);

//   // Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();

//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   pcl::fromROSMsg(*msg, cloud);

//   int num = cloud.points.size();

//   map_->inputPointCloud(cloud, num, camera_pos_);

//   if (local_updated_) {
//     map_->clearAndInflateLocalMap();
//     esdf_need_update_ = true;
//     local_updated_ = false;
//   }
// }

void MapROS::cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                               const nav_msgs::msg::Odometry::ConstSharedPtr& pose) {
  // 获取相机位姿
  camera_pos_(0) = pose->pose.pose.position.x;
  camera_pos_(1) = pose->pose.pose.position.y;
  camera_pos_(2) = pose->pose.pose.position.z;

  // 从姿态信息中提取四元数
  camera_q_ = Eigen::Quaterniond(pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
                                 pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);

  // 将ROS的PointCloud2消息转换为PCL点云
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // 获取点云的大小
  int num = cloud.points.size();

  // 将点云输入到地图中
  map_->inputPointCloud(cloud, num, camera_pos_);

  // 如果局部地图有更新，则进行处理
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}



void MapROS::proessDepthImage() {
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      // TODO: simplify the logic here
      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;
      // fixed depth bug in AirSim
      Eigen::Vector3d normal_uvd = Eigen::Vector3d(u, v, 1.0);
      Eigen::Vector3d normal_xyz = K_depth_.inverse() * normal_uvd;
      double length = normal_xyz.norm();
      Eigen::Vector3d xyz = normal_xyz / length * depth;
      pt_cur << xyz(0), xyz(2), -xyz(1);

      // pt_cur(0) = (u - cx_) * depth / fx_;
      // pt_cur(1) = (v - cy_) * depth / fy_;
      // pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  publishDepth();
}

// void MapROS::publishMapAll() {
//   pcl::PointXYZ pt;
//   pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
//   for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
//     for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
//       for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
//         if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
//           Eigen::Vector3d pos;
//           map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
//           if (pos(2) > visualization_truncate_height_) continue;
//           if (pos(2) < visualization_truncate_low_) continue;
//           pt.x = pos(0);
//           pt.y = pos(1);
//           pt.z = pos(2);
//           cloud1.push_back(pt);
//         }
//       }
//   cloud1.width = cloud1.points.size();
//   cloud1.height = 1;
//   cloud1.is_dense = true;
//   cloud1.header.frame_id = frame_id_;
//   sensor_msgs::PointCloud2 cloud_msg;
//   pcl::toROSMsg(cloud1, cloud_msg);
//   map_all_pub_.publish(cloud_msg);

//   // Output time and known volumn
//   double time_now = (ros::Time::now() - map_start_time_).toSec();
//   double known_volumn = 0;

//   for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
//     for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
//       for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
//         if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3)
//           known_volumn += 0.1 * 0.1 * 0.1;
//       }
//   ofstream file("package://percep_manager/output/"
//                 "volume.txt",
//                 ios::app);
//   file << "time:" << time_now << ",vol:" << known_volumn << std::endl;
// }

void MapROS::publishMapAll() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  
  for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;

  // 将PCL点云转换为ROS2的PointCloud2消息
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);

  // 设置消息的时间戳
  cloud_msg.header.stamp = node_->now();

  // 发布点云
  map_all_pub_->publish(cloud_msg);

  // Output time and known volume
  double time_now = (node_->now() - map_start_time_).seconds();
  double known_volumn = 0;

  for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3)
          known_volumn += 0.1 * 0.1 * 0.1;  // 假设体素的大小为0.1米
      }

  // 以文本文件形式输出体积信息
  std::ofstream file("/home/chranos/FCtest/FEtest/src/plan_env/percep_manager/output/volume.txt", std::ios::app);
  file << "time:" << time_now << ",vol:" << known_volumn << std::endl;
}



// void MapROS::publishMapLocal() {
//   pcl::PointXYZ pt;
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   pcl::PointCloud<pcl::PointXYZ> cloud2;
//   Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
//   Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
//   map_->boundIndex(min_cut);
//   map_->boundIndex(max_cut);

//   // for (int z = min_cut(2); z <= max_cut(2); ++z)
//   for (int x = min_cut(0); x <= max_cut(0); ++x)
//     for (int y = min_cut(1); y <= max_cut(1); ++y)
//       for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
//         if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
//           // Occupied cells
//           Eigen::Vector3d pos;
//           map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
//           if (pos(2) > visualization_truncate_height_) continue;
//           if (pos(2) < visualization_truncate_low_) continue;

//           pt.x = pos(0);
//           pt.y = pos(1);
//           pt.z = pos(2);
//           cloud.push_back(pt);
//         }
//         // else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)
//         // {
//         //   // Inflated occupied cells
//         //   Eigen::Vector3d pos;
//         //   map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
//         //   if (pos(2) > visualization_truncate_height_)
//         //     continue;
//         //   if (pos(2) < visualization_truncate_low_)
//         //     continue;

//         //   pt.x = pos(0);
//         //   pt.y = pos(1);
//         //   pt.z = pos(2);
//         //   cloud2.push_back(pt);
//         // }
//       }

//   cloud.width = cloud.points.size();
//   cloud.height = 1;
//   cloud.is_dense = true;
//   cloud.header.frame_id = frame_id_;
//   cloud2.width = cloud2.points.size();
//   cloud2.height = 1;
//   cloud2.is_dense = true;
//   cloud2.header.frame_id = frame_id_;
//   sensor_msgs::PointCloud2 cloud_msg;

//   pcl::toROSMsg(cloud, cloud_msg);
//   map_local_pub_.publish(cloud_msg);
//   pcl::toROSMsg(cloud2, cloud_msg);
//   map_local_inflate_pub_.publish(cloud_msg);
// }


void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  // 遍历局部地图中的点云
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          // 如果占据值大于最小阈值，则认为该体素被占据
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  // 设置点云的基本属性
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;

  // 将PCL点云转换为ROS2的PointCloud2消息
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  // 设置时间戳
  cloud_msg.header.stamp = node_->now();

  // 发布局部地图点云
  map_local_pub_->publish(cloud_msg);

  // 将膨胀的点云发布
  pcl::toROSMsg(cloud2, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_local_inflate_pub_->publish(cloud_msg);
}



// void MapROS::publishPredCloud()
// {
//   pcl::PointCloud<pcl::PointXYZ> cloud_pred;
//   cloud_pred = copy_cloud_GHPR;

//   cloud_pred.width = cloud_pred.points.size();
//   cloud_pred.height = 1;
//   cloud_pred.is_dense = true;
//   cloud_pred.header.frame_id = frame_id_;
  
//   sensor_msgs::PointCloud2 cloud_msg;
//   pcl::toROSMsg(cloud_pred, cloud_msg);
//   pred_pub_.publish(cloud_msg);
// }

void MapROS::publishPredCloud()
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = copy_cloud_GHPR;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = frame_id_;

  // 将PCL点云转换为ROS2的PointCloud2消息
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);

  // 设置时间戳
  cloud_msg.header.stamp = node_->now();

  // 发布点云
  pred_pub_->publish(cloud_msg);
}


// void MapROS::publishPred_INTERNAL() {
//   pcl::PointXYZ pt;
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   // Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
//   // Eigen::Vector3i max_cut = map_->md_->local_bound_max_;

//   Eigen::Vector3i min_cut;
//   Eigen::Vector3i max_cut;
//   map_->posToIndex(map_->mp_->map_min_boundary_, min_cut);
//   map_->posToIndex(map_->mp_->map_max_boundary_, max_cut);
//   map_->boundIndex(max_cut);
//   map_->boundIndex(min_cut);

//   for (int x = min_cut(0); x <= max_cut(0); ++x)
//     for (int y = min_cut(1); y <= max_cut(1); ++y)
//       for (int z = min_cut(2); z <= max_cut(2); ++z) {
//         // if ((map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3 && map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->min_occupancy_log_))
//         // if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 2*1e-3)
//         if (map_->md_->occupancy_buffer_pred_[map_->toAddress(x, y, z)] == 1)
//         {
//           Eigen::Vector3d pos;
//           map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
//           // if (pos(2) > visualization_truncate_height_) continue;
//           if (pos(2) < visualization_truncate_low_) continue;
//           pt.x = pos(0);
//           pt.y = pos(1);
//           pt.z = pos(2);
//           cloud.push_back(pt);
//         }
//       }
//   cloud.width = cloud.points.size();
//   cloud.height = 1;
//   cloud.is_dense = true;
//   cloud.header.frame_id = frame_id_;
//   sensor_msgs::PointCloud2 cloud_msg;
//   pcl::toROSMsg(cloud, cloud_msg);
//   unknown_pub_.publish(cloud_msg);
// }



void MapROS::publishPred_INTERNAL() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 获取地图的边界索引
  Eigen::Vector3i min_cut;
  Eigen::Vector3i max_cut;
  map_->posToIndex(map_->mp_->map_min_boundary_, min_cut);
  map_->posToIndex(map_->mp_->map_max_boundary_, max_cut);
  map_->boundIndex(max_cut);
  map_->boundIndex(min_cut);

  // 遍历所有体素，并根据 occupancy_buffer_pred_ 的状态筛选点云
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (map_->md_->occupancy_buffer_pred_[map_->toAddress(x, y, z)] == 1) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) < visualization_truncate_low_) continue;  // 过滤低于可视化阈值的点
          
          // 将体素的位置添加到点云中
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  // 设置点云的基本属性
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;

  // 将PCL点云转换为ROS2的PointCloud2消息
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  // 手动设置时间戳
  cloud_msg.header.stamp = node_->now();

  // 发布点云消息
  unknown_pub_->publish(cloud_msg);
}


// void MapROS::publishDepth() {
//   pcl::PointXYZ pt;
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   for (int i = 0; i < proj_points_cnt; ++i) {
//     cloud.push_back(point_cloud_.points[i]);
//   }
//   cloud.width = cloud.points.size();
//   cloud.height = 1;
//   cloud.is_dense = true;
//   cloud.header.frame_id = frame_id_;
//   sensor_msgs::PointCloud2 cloud_msg;
//   pcl::toROSMsg(cloud, cloud_msg);
//   depth_pub_.publish(cloud_msg);
// }


void MapROS::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 将投影点云添加到cloud
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_.points[i]);
  }

  // 设置点云的属性
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;

  // 将PCL点云转换为ROS2的PointCloud2消息
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  // 手动设置时间戳
  cloud_msg.header.stamp = node_->now();

  // 发布点云消息
  depth_pub_->publish(cloud_msg);
}


// void MapROS::publishUpdateRange() {
//   Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
//   visualization_msgs::Marker mk;
//   map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
//   map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

//   cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
//   cube_scale = esdf_max_pos - esdf_min_pos;
//   mk.header.frame_id = frame_id_;
//   mk.header.stamp = ros::Time::now();
//   mk.type = visualization_msgs::Marker::CUBE;
//   mk.action = visualization_msgs::Marker::ADD;
//   mk.id = 0;
//   mk.pose.position.x = cube_pos(0);
//   mk.pose.position.y = cube_pos(1);
//   mk.pose.position.z = cube_pos(2);
//   mk.scale.x = cube_scale(0);
//   mk.scale.y = cube_scale(1);
//   mk.scale.z = cube_scale(2);
//   mk.color.a = 0.3;
//   mk.color.r = 1.0;
//   mk.color.g = 0.0;
//   mk.color.b = 0.0;
//   mk.pose.orientation.w = 1.0;
//   mk.pose.orientation.x = 0.0;
//   mk.pose.orientation.y = 0.0;
//   mk.pose.orientation.z = 0.0;

//   update_range_pub_.publish(mk);
// }

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::msg::Marker mk;

  // 获取地图边界
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  // 计算中心位置和缩放比例
  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;

  // 设置Marker的基本属性
  mk.header.frame_id = frame_id_;
  mk.header.stamp = node_->now();  // 手动设置时间戳
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = 0;

  // 设置Marker的位姿和缩放
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  // 设置颜色
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  // 设置默认的方向
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  // 发布Marker消息
  update_range_pub_->publish(mk);
}


// void MapROS::publishESDF() {
//   double dist;
//   pcl::PointCloud<pcl::PointXYZI> cloud;
//   pcl::PointXYZI pt;

//   const double min_dist = 0.0;
//   const double max_dist = 3.0;

//   Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
//                                                                           map_->mp_->local_map_margin_,
//                                                                           map_->mp_->local_map_margin_);
//   Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
//                                                                           map_->mp_->local_map_margin_,
//                                                                           map_->mp_->local_map_margin_);
//   map_->boundIndex(min_cut);
//   map_->boundIndex(max_cut);

//   for (int x = min_cut(0); x <= max_cut(0); ++x)
//     for (int y = min_cut(1); y <= max_cut(1); ++y) {
//       Eigen::Vector3d pos;
//       map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
//       pos(2) = esdf_slice_height_;
//       dist = map_->getDistance(pos);
//       dist = min(dist, max_dist);
//       dist = max(dist, min_dist);
//       pt.x = pos(0);
//       pt.y = pos(1);
//       pt.z = -0.2;
//       pt.intensity = (dist - min_dist) / (max_dist - min_dist);
//       cloud.push_back(pt);
//     }

//   cloud.width = cloud.points.size();
//   cloud.height = 1;
//   cloud.is_dense = true;
//   cloud.header.frame_id = frame_id_;
//   sensor_msgs::PointCloud2 cloud_msg;
//   pcl::toROSMsg(cloud, cloud_msg);

//   esdf_pub_.publish(cloud_msg);

//   // ROS_INFO("pub esdf");
// }

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  // 定义地图边界
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  // 遍历地图的x和y方向，计算距离并填充点云
  for (int x = min_cut(0); x <= max_cut(0); ++x) {
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;  // 设置ESDF的切片高度
      dist = map_->getDistance(pos);  // 获取点的ESDF距离
      dist = std::min(dist, max_dist);
      dist = std::max(dist, min_dist);

      // 设置点的坐标和强度
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;  // 假设的平面高度
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);  // 归一化强度值
      cloud.push_back(pt);
    }
  }

  // 设置点云的属性
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;

  // 将PCL点云转换为ROS2的PointCloud2消息
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  // 设置时间戳
  cloud_msg.header.stamp = node_->now();

  // 发布点云
  esdf_pub_->publish(cloud_msg);

  // ROS_INFO("pub esdf");  // ROS2中可以使用RCLCPP_INFO
}


}