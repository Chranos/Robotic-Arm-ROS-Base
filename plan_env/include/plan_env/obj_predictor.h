#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

// #include <Eigen/Eigen>
// #include <algorithm>
// #include <geometry_msgs/PoseStamped.h>
// #include <iostream>
// #include <list>
// #include <ros/ros.h>
// #include <visualization_msgs/Marker.h>
// #include <nav_msgs/Odometry.h>

// using std::cout;
// using std::endl;
// using std::list;
// using std::shared_ptr;
// using std::unique_ptr;
// using std::vector;

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;


namespace predrecon {
class PolynomialPrediction;
typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;
typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;

/* ========== prediction polynomial ========== */
class PolynomialPrediction {
private:
  vector<Eigen::Matrix<double, 6, 1>> polys;
  double t1, t2;  // start / end

public:
  PolynomialPrediction(/* args */) {
  }
  ~PolynomialPrediction() {
  }

  void setPolynomial(vector<Eigen::Matrix<double, 6, 1>>& pls) {
    polys = pls;
  }
  void setTime(double t1, double t2) {
    this->t1 = t1;
    this->t2 = t2;
  }

  bool valid() {
    return polys.size() == 3;
  }

  /* note that t should be in [t1, t2] */
  Eigen::Vector3d evaluate(double t) {
    Eigen::Matrix<double, 6, 1> tv;
    tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

    return pt;
  }

  Eigen::Vector3d evaluateConstVel(double t) {
    Eigen::Matrix<double, 2, 1> tv;
    tv << 1.0, pow(t, 1);

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

    return pt;
  }
};

/* ========== subscribe and record object history ========== */
// class ObjHistory {
// public:
//   static int skip_num_;
//   static int queue_size_;
//   static ros::Time global_start_time_;

//   ObjHistory() {
//   }
//   ~ObjHistory() {
//   }

//   void init(int id);

//   void poseCallback(const nav_msgs::OdometryConstPtr& msg);

//   void clear() {
//     history_.clear();
//   }

//   void getHistory(list<Eigen::Vector4d>& his) {
//     his = history_;
//   }

// private:
//   list<Eigen::Vector4d> history_;  // x,y,z;t
//   int skip_;
//   int obj_idx_;
//   Eigen::Vector3d scale_;
// };

class ObjHistory {
public:
  static int skip_num_;
  static int queue_size_;
  static rclcpp::Time global_start_time_;  // ROS2中的rclcpp::Time

  ObjHistory() {
  }
  ~ObjHistory() {
  }

  void init(int id);

  // 回调函数中使用ROS2的消息类型
  void poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void clear() {
    history_.clear();
  }

  void getHistory(std::list<Eigen::Vector4d>& his) {
    his = history_;
  }

private:
  std::list<Eigen::Vector4d> history_;  // x,y,z;t
  int skip_;
  int obj_idx_;
  Eigen::Vector3d scale_;
};


/* ========== predict future trajectory using history ========== */
// class ObjPredictor {
// private:
//   ros::NodeHandle node_handle_;

//   int obj_num_;
//   double lambda_;
//   double predict_rate_;

//   vector<ros::Subscriber> pose_subs_;
//   ros::Subscriber marker_sub_;
//   ros::Timer predict_timer_;
//   vector<shared_ptr<ObjHistory>> obj_histories_;

//   /* share data with planner */
//   ObjPrediction predict_trajs_;
//   ObjScale obj_scale_;
//   vector<bool> scale_init_;

//   void markerCallback(const visualization_msgs::MarkerConstPtr& msg);

//   void predictCallback(const ros::TimerEvent& e);
//   void predictPolyFit();
//   void predictConstVel();

// public:
//   ObjPredictor(/* args */);
//   ObjPredictor(ros::NodeHandle& node);
//   ~ObjPredictor();

//   void init();

//   ObjPrediction getPredictionTraj();
//   ObjScale getObjScale();

//   typedef shared_ptr<ObjPredictor> Ptr;
// };

class ObjPredictor {
private:
  std::shared_ptr<rclcpp::Node> node_handle_;  // 替换ros::NodeHandle为rclcpp::Node::SharedPtr

  int obj_num_;
  double lambda_;
  double predict_rate_;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> pose_subs_;  // 替换ros::Subscriber为rclcpp::Subscription
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;      // 替换ros::Subscriber为rclcpp::Subscription
  rclcpp::TimerBase::SharedPtr predict_timer_;  // 替换ros::Timer为rclcpp::TimerBase::SharedPtr
  std::vector<std::shared_ptr<ObjHistory>> obj_histories_;

  /* 与planner共享的数据 */
  ObjPrediction predict_trajs_;
  ObjScale obj_scale_;
  std::vector<bool> scale_init_;

  void markerCallback(const visualization_msgs::msg::Marker::ConstSharedPtr& msg);  // 替换MarkerConstPtr为ConstSharedPtr

  void predictCallback();  // ROS2中Timer不再使用TimerEvent
  void predictPolyFit();
  void predictConstVel();

public:
  ObjPredictor();
  ObjPredictor(rclcpp::Node::SharedPtr node);  // 替换NodeHandle为rclcpp::Node::SharedPtr
  ~ObjPredictor();

  void init();

  ObjPrediction getPredictionTraj();
  ObjScale getObjScale();

  typedef std::shared_ptr<ObjPredictor> Ptr;
};



}  // namespace fast_planner

#endif