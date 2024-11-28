#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <utility>
#include <memory>  // for shared_ptr and unique_ptr

#include <plan_env/obj_predictor.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace predrecon {
class SDFMap;

class EDTEnvironment {
private:
  /* data */
  ObjPrediction obj_prediction_;
  ObjScale obj_scale_;
  double resolution_inv_;
  
  // Method to calculate distance to box
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
  
  // Method to calculate minimum distance to all boxes
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
  EDTEnvironment(/* args */) {
  }

  ~EDTEnvironment() {
  }

  shared_ptr<SDFMap> sdf_map_;

  // Method to initialize environment
  void init();
  
  // Set the SDF map
  void setMap(shared_ptr<SDFMap>& map);
  
  // Set object prediction data
  void setObjPrediction(ObjPrediction prediction);
  
  // Set object scale data
  void setObjScale(ObjScale scale);
  
  // Evaluate the Euclidean Distance Transform (EDT) with gradient
  void evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time, double& dist, Eigen::Vector3d& grad);
  
  // Coarse evaluation of the EDT
  double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);

  // Deprecated methods
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  
  // Trilinear interpolation with gradient
  void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff, double& value,
                            Eigen::Vector3d& grad);

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace predrecon

#endif

// #ifndef _EDT_ENVIRONMENT_H_
// #define _EDT_ENVIRONMENT_H_

// #include <Eigen/Eigen>
// #include <iostream>
// #include <utility>

// #include <plan_env/obj_predictor.h>

// using std::cout;
// using std::endl;
// using std::list;
// using std::pair;
// using std::shared_ptr;
// using std::unique_ptr;
// using std::vector;

// namespace predrecon {
// class SDFMap;

// class EDTEnvironment {
// private:
//   /* data */
//   ObjPrediction obj_prediction_;
//   ObjScale obj_scale_;
//   double resolution_inv_;
//   double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
//   double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

// public:
//   EDTEnvironment(/* args */) {
//   }
//   ~EDTEnvironment() {
//   }

//   shared_ptr<SDFMap> sdf_map_;

//   void init();
//   void setMap(shared_ptr<SDFMap>& map);
//   void setObjPrediction(ObjPrediction prediction);
//   void setObjScale(ObjScale scale);
//   void evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time, double& dist, Eigen::Vector3d& grad);
//   double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);

//   // deprecated
//   void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
//   void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff, double& value,
//                             Eigen::Vector3d& grad);

//   typedef shared_ptr<EDTEnvironment> Ptr;
// };

// }  // namespace predrecon

// #endif