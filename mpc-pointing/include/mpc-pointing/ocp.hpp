#ifndef OCP_P
#define OCP_P

#include <memory.h>

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include "mpc-pointing/fwd.hpp"

namespace mpc_p {
using namespace crocoddyl;
struct OCPSettings_Point {
  size_t horizon_length;
  ModelMakerSettings modelMakerSettings;

  void readParamsFromYamlString(std::string &StringToParse);
  void readParamsFromYamlFile(const std::string &Filename);
};

class OCP_Point {
 private:
  OCPSettings_Point settings_;
  RobotWrapper designer_;
  ModelMaker modelMaker_;
  DDP solver_;

  bool initialized_ = false;

  // prealocated memory:
  std::vector<VectorXd> warm_xs_;
  std::vector<VectorXd> warm_us_;

  // OCP Problem Maker private functions
  void buildSolver(const VectorXd x0, SE3 oMtarget,
                   const ModelMakerSettings &modelMakerSettings);
  void solveFirst(const VectorXd x);

  // OCP Problem Helper private functions
  ActionModel ama(const unsigned long time);
  IntegratedActionModel iam(const unsigned long time);
  DifferentialActionModel dam(const unsigned long time);
  CostModelSum costs(const unsigned long time);
  ActionData ada(const unsigned long time);

 public:
  OCP_Point(const OCPSettings_Point &OCPSettings, const RobotWrapper &designer);

  void initialize(const ConstVectorRef &x0, const SE3 &oMtarget);
  void solve(const ConstVectorRef &measured_x);

  // OCP Problem Helper public functions
  void recede();
  void setBalancingTorques();
  void changeTarget(const size_t index,
                    const Eigen::Ref<const Eigen::Vector3d> &position);
  void updateGoalPosition(const Eigen::Ref<const Eigen::Vector3d> &position);
  void updateGoalRotation(const Eigen::Ref<const Eigen::Matrix3d> &rotation);
  void changeGoalCostActivation(const size_t index, const bool value);
  void changeGoaleTrackingWeights(double weight);

  // Setters and Getters

  const VectorXd get_torque();
  const MatrixXd get_gain();

  ModelMaker &get_modelMaker() { return (modelMaker_); };
  size_t get_initialized() { return (initialized_); };
  size_t get_horizonLength() { return (settings_.horizon_length); };
};

}  // namespace mpc_p

#endif  // OCP_P
