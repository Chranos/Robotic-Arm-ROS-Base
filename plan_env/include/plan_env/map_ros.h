#ifndef _MAP_ROS_H
#define _MAP_ROS_H

// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/time_synchronizer.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <ros/ros.h>

// #include <cv_bridge/cv_bridge.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>

// #include <memory>
// #include <random>
// #include <pcl/io/pcd_io.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <pcl/common/common.h>
// #include <string>
// #include <pcl/filters/conditional_removal.h>
// #include <vector>
// #include <chrono>
// #include <Eigen/Core>
// #include <pcl/common/centroid.h>
// #include <pcl/kdtree/kdtree_flann.h>  
// #include <pcl/surface/mls.h>         
// #include <pcl/features/normal_3d_omp.h>
// #include <pcl/features/normal_3d.h> 
// #include <pcl/io/ply_io.h>
// #include <boost/thread/thread.hpp>
// #include <pcl/surface/gp3.h>
// #include <pcl/surface/poisson.h>
// #include <pcl/surface/convex_hull.h>
// #include <math.h>

// using std::shared_ptr;
// using std::unique_ptr;
// using std::normal_distribution;
// using std::default_random_engine;
// typedef pcl::PointXYZ PointType;
// typedef pcl::Normal NormalType;
// using namespace std;

// class RayCaster;


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>  // ROS2的头文件

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>    // ROS2的消息头文件更新
#include <nav_msgs/msg/odometry.hpp>             // ROS2中的Odometry消息
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <string>
#include <pcl/filters/conditional_removal.h>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/convex_hull.h>
#include <cmath> 

using std::shared_ptr;
using std::unique_ptr;
using std::normal_distribution;
using std::default_random_engine;
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
using namespace std;

class RayCaster; 


// namespace predrecon {
// class SDFMap;
// struct VP_global {
//   // Position and heading
//   Eigen::Vector3d pos_g;
//   double yaw_g;
//   // Fraction of the cluster that can be covered
//   // double fraction_;
//   int visib_num_g;
// };
// struct cluster_normal
// {
//   vector<Eigen::Vector3d> global_cells_;
//   Eigen::Vector3d center_;
//   Eigen::Vector3d normal_;
//   vector<VP_global> vps_global;
//   int id_g;
//   list<vector<Eigen::Vector3d>> paths_;
//   list<double> costs_;
// };

// class MapROS {
// public:
//   MapROS();
//   ~MapROS();
//   void setMap(SDFMap* map);
//   void init();
//   int sgn(double& x);

// private:
//   void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
//                          const nav_msgs::OdometryConstPtr& pose);
//   void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
//                          const nav_msgs::OdometryConstPtr& pose);
//   void updateESDFCallback(const ros::TimerEvent& /*event*/);
//   void visCallback(const ros::TimerEvent& /*event*/);
//   // -----------------------
//   void publishMapAll();
//   void publishMapLocal();
//   void publishESDF();
//   void publishUpdateRange();
//   void publishPredCloud();
//   void publishPred_INTERNAL();
//   void publishDepth();

//   void proessDepthImage();

//   SDFMap* map_;
//   // may use ExactTime?
//   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
//       SyncPolicyImagePose;
//   typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
//   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
//                                                           nav_msgs::Odometry>
//       SyncPolicyCloudPose;
//   typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

//   ros::NodeHandle node_;
//   shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
//   shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
//   shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> pose_sub_;
//   SynchronizerImagePose sync_image_pose_;
//   SynchronizerCloudPose sync_cloud_pose_;

//   ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
//       update_range_pub_, depth_pub_, lidar_pub_, arrow_pub_, vp_pub_, init_tour_pub_, pred_pub_;
//   ros::Timer esdf_timer_, vis_timer_;

//   // params, depth projection
//   Eigen::Matrix3d K_depth_;
//   double cx_, cy_, fx_, fy_;
//   double depth_filter_maxdist_, depth_filter_mindist_;
//   int depth_filter_margin_;
//   double k_depth_scaling_factor_;
//   int skip_pixel_;
//   string frame_id_;
//   // msg publication
//   double esdf_slice_height_;
//   double visualization_truncate_height_, visualization_truncate_low_;
//   bool show_esdf_time_, show_occ_time_;
//   bool show_all_map_;

//   // data
//   // flags of map state
//   bool local_updated_, esdf_need_update_;
//   // input
//   Eigen::Vector3d camera_pos_;
//   Eigen::Quaterniond camera_q_;
//   unique_ptr<cv::Mat> depth_image_;
//   vector<Eigen::Vector3d> proj_points_;
//   int proj_points_cnt;
//   double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
//   int fuse_num_, esdf_num_;
//   pcl::PointCloud<pcl::PointXYZ> point_cloud_;

//   normal_distribution<double> rand_noise_;
//   default_random_engine eng_;

//   ros::Time map_start_time_;

//   // prediction point cloud
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prediction;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_GHPR;
//   pcl::PointCloud<pcl::PointXYZ> copy_cloud_GHPR;
//   vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> uni_cluster;
//   vector<Eigen::Matrix3f> uni_normal;
//   Eigen::Vector4f viewpoint;
//   // temp point cloud
//   pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inverse;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cec;
//   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals;

//   // uniform cluster container
//   vector<cluster_normal> uni_cnv;
//   vector<cluster_normal> uni_open_cnv;
//   // global planning path
//   vector<Eigen::Vector3d> global_tour;
//   vector<Eigen::Vector3d> global_points;
//   vector<double> global_yaws;
//   vector<vector<Eigen::Vector3d>> global_n_points;
//   vector<vector<double>> global_n_yaws;
//   vector<Eigen::Vector3d> refined_n_points;
//   vector<double> refined_n_yaws;
//   vector<int> refined_ids;
//   vector<Eigen::Vector3d> unrefined_points;
//   vector<Eigen::Vector3d> refined_tour;
//   // Param
//   double r_min, r_max, z_range, theta_sample, phi_sample, a_step, theta_thre; // sample radius and angle range
//   int r_num, z_step;
//   double min_dist, refine_radius, max_decay, downsample_c;
//   int refine_num, top_view_num;

//   unique_ptr<RayCaster> raycaster_;

//   friend SDFMap;
// };
// }


namespace predrecon {
class SDFMap;
struct VP_global {
  // Position and heading
  Eigen::Vector3d pos_g;
  double yaw_g;
  // Fraction of the cluster that can be covered
  int visib_num_g;
};
struct cluster_normal {
  vector<Eigen::Vector3d> global_cells_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  vector<VP_global> vps_global;
  int id_g;
  list<vector<Eigen::Vector3d>> paths_;
  list<double> costs_;
};

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init();
  int sgn(double& x);

private:
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void updateESDFCallback();
  void visCallback();
  // -----------------------
  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishPredCloud();
  void publishPred_INTERNAL();
  void publishDepth();

  void proessDepthImage();

  SDFMap* map_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImagePose;
  typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                          nav_msgs::msg::Odometry>
      SyncPolicyCloudPose;
  typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> pose_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_local_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_local_inflate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_all_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr update_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr init_tour_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pred_pub_;


  // // 创建发布者
  // map_all_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_all", 10);
  // map_local_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local", 10);
  // map_local_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  // unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/unknown", 10);
  // esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);
  // update_range_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/sdf_map/update_range", 10);
  // depth_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/depth_cloud", 10);
  // lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/lidar_cloud", 10);

  // // 创建MarkerArray类型的发布者
  // arrow_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sdf_map/normal", 120);
  // vp_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sdf_map/vp_global", 120);
  // init_tour_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sdf_map/init_global", 10);
  // pred_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/pred_cloud", 10);


  rclcpp::TimerBase::SharedPtr esdf_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  Eigen::Matrix3d K_depth_;
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  std::string frame_id_;

  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;

  bool local_updated_, esdf_need_update_;

  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  std::unique_ptr<cv::Mat> depth_image_;
  std::vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  std::normal_distribution<double> rand_noise_;
  std::default_random_engine eng_;

  rclcpp::Time map_start_time_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prediction;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_GHPR;
  pcl::PointCloud<pcl::PointXYZ> copy_cloud_GHPR;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> uni_cluster;
  std::vector<Eigen::Matrix3f> uni_normal;
  Eigen::Vector4f viewpoint;

  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform;
  pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inverse;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cec;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals;

  std::vector<cluster_normal> uni_cnv;
  std::vector<cluster_normal> uni_open_cnv;

  std::vector<Eigen::Vector3d> global_tour;
  std::vector<Eigen::Vector3d> global_points;
  std::vector<double> global_yaws;
  std::vector<std::vector<Eigen::Vector3d>> global_n_points;
  std::vector<std::vector<double>> global_n_yaws;
  std::vector<Eigen::Vector3d> refined_n_points;
  std::vector<double> refined_n_yaws;
  std::vector<int> refined_ids;
  std::vector<Eigen::Vector3d> unrefined_points;
  std::vector<Eigen::Vector3d> refined_tour;

  double r_min, r_max, z_range, theta_sample, phi_sample, a_step, theta_thre;
  int r_num, z_step;
  double min_dist, refine_radius, max_decay, downsample_c;
  int refine_num, top_view_num;

  std::unique_ptr<RayCaster> raycaster_;

  friend SDFMap;
};




}


#endif