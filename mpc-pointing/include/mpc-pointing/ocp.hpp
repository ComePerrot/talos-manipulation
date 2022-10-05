#ifndef SOBEC_OCP_P
#define SOBEC_OCP_P

#include <memory.h>

#include <pinocchio/fwd.hpp>

// include pinocchio first

#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/model_factory.hpp"

namespace sobec {
struct OCPSettings_Point {
  size_t horizon_length;
  ModelMakerSettings modelMakerSettings;

  void readParamsFromYamlString(std::string &StringToParse);
  void readParamsFromYamlFile(const std::string &Filename);
};

class OCP_Point {
 private:
  OCPSettings_Point settings_;
  RobotDesigner designer_;
  ModelMaker modelMaker_;
  DDP solver_;

  bool initialized_ = false;

  // prealocated memory:
  std::vector<Eigen::VectorXd> warm_xs_;
  std::vector<Eigen::VectorXd> warm_us_;

  // OCP Problem Maker private functions
  void buildSolver(const Eigen::VectorXd x0, pinocchio::SE3 oMtarget,
                   const ModelMakerSettings modelMakerSettings);
  void solveFirst(const Eigen::VectorXd x);

  // OCP Problem Helper private functions
  AMA ama(const unsigned long time);
  IAM iam(const unsigned long time);
  DAM dam(const unsigned long time);
  Cost costs(const unsigned long time);
  ADA ada(const unsigned long time);

 public:
  OCP_Point(const OCPSettings_Point &OCPSettings,
            const RobotDesigner &designer);

  void initialize(const Eigen::VectorXd &x0, const pinocchio::SE3 &oMtarget);
  void solve(const Eigen::VectorXd &measured_x);

  // OCP Problem Helper public functions
  void recede();
  void setBalancingTorques();
  void changeTarget(const size_t index,
                    const Eigen::Ref<const Eigen::Vector3d> position);
  void updateGoalPosition(const Eigen::Ref<const Eigen::Vector3d> position);
  void updateGoalRotation(const Eigen::Ref<const Eigen::Matrix3d> rotation);
  void changeGoalCostActivation(const size_t index, const bool value);
  void changeGoaleTrackingWeights(double weight);

  // Setters and Getters

  Eigen::VectorXd get_torque();
  Eigen::MatrixXd get_gain();

  ModelMaker &get_modelMaker() { return (modelMaker_); };
  size_t get_initialized() { return (initialized_); };
  size_t get_horizonLength() { return (settings_.horizon_length); };
};

}  // namespace sobec

#endif  // SOBEC_OCP_P
