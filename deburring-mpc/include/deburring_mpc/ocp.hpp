///////////////////////////////////////////////////////////////////////////////
// BSD 2-Clause License
//
// Copyright (C) 2022-2023, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef DEBURRING_OCP
#define DEBURRING_OCP

#include <memory.h>

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic-flat-log.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solver-base.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
// #include <crocoddyl/multibody/frames.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/robot_designer.hpp"

namespace deburring {

struct OCP_debugData {
 public:
  std::vector<Eigen::VectorXd> xi;
  std::vector<Eigen::VectorXd> ui;

  std::vector<Eigen::VectorXd> xs;
  std::vector<Eigen::VectorXd> us;
  std::vector<crocoddyl::SolverFDDP::MatrixXdRowMajor> K;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &xi;
    ar &ui;

    ar &xs;
    ar &us;
    ar &K;
  }
};

struct OCPSettings {
  size_t horizon_length;
  // Timing
  double time_step = 0.01;

  // Croco configuration
  double w_state_reg = 0;
  double w_control_reg = 0;
  double w_state_limits = 0;
  double w_control_limit = 0;
  double w_com_pos = 0;
  double w_gripper_pos = 0;
  double w_gripper_rot = 0;
  double w_gripper_vel = 0;

  double limit_scale = 1;
  bool limit_speed = false;

  Eigen::VectorXd state_weights;
  Eigen::VectorXd control_weights;

  void readParamsFromYamlString(const std::string &string_to_parse);
  void readParamsFromYamlFile(const std::string &filename);
};

/**
 * @brief Interface with Crocoddyl
 *
 * This class handles the calls to Crocoddyl to manage the OCP.
 */
class OCP {
 private:
  OCPSettings settings_;
  RobotDesigner designer_;
  DDP solver_;

  boost::shared_ptr<crocoddyl::StateMultibody> state_;
  boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;

  bool is_initialized_ = false;

  // prealocated memory:
  std::vector<VectorXd> warm_xs_;
  std::vector<VectorXd> warm_us_;

  // OCP Problem Maker private functions
  void buildSolver(const VectorXd x0, SE3 oMtarget);
  ActionModel formulatePointingTask();
  ActionModel formulateTerminalPointingTask();
  void setArmature(DifferentialActionModel DAM);
  void defineFeetContact(Contact &contact_collector);
  void defineStateRegularization(CostModelSum &cost_collector,
                                 const double w_state_reg);
  void defineControlRegularization(CostModelSum &cost_collector,
                                   const double w_control_reg);
  void defineStateLimits(CostModelSum &cost_collector,
                         const double w_state_limits, const double limit_scale,
                         const bool limit_speed);
  void defineControlLimits(CostModelSum &cost_collector,
                           const double w_control_limit,
                           const double limit_scale);
  void defineCoMPosition(CostModelSum &cost_collector, const double w_com_pos);
  void defineGripperPlacement(CostModelSum &cost_collector,
                              const double w_gripper_pos,
                              const double w_gripper_rot);
  void defineGripperVelocity(CostModelSum &cost_collector,
                             const double w_gripper_vel);

  // OCP Problem Helper private functions
  ActionModel ama(const unsigned long time);
  IntegratedActionModel iam(const unsigned long time);
  DifferentialActionModel dam(const unsigned long time);
  CostModelSum costs(const unsigned long time);
  ActionData ada(const unsigned long time);

 public:
  OCP(const OCPSettings &ocp_settings, const RobotDesigner &designer);

  void initialize(const ConstVectorRef &x0, const SE3 &oMtarget);
  void reset(const ConstVectorRef &x0, const SE3 &oMtarget);
  void solveFirst(const VectorXd x);
  void solve(const ConstVectorRef &measured_x);

  // OCP Problem Helper public functions
  void setWarmStart(const std::vector<VectorXd> &warm_xs,
                    const std::vector<VectorXd> &warm_us);
  void recede();
  void setBalancingTorques();
  void changeTarget(const size_t index,
                    const Eigen::Ref<const Eigen::Vector3d> &position);
  void updateGoalPosition(const Eigen::Ref<const Eigen::Vector3d> &position);
  void updateGoalRotation(const Eigen::Ref<const Eigen::Matrix3d> &rotation);
  void changeGoalCostActivation(const size_t index, const bool value);
  void changeGoalTrackingWeights(double weight);
  void changePostureReference(const size_t index,
                              const Eigen::Ref<const VectorXd> reference);
  const VectorXd &getFinalPosture();

  // Debug
  void printCosts();

  std::vector<OCP_debugData> debugDataOCP_;

  void logData(const std::vector<Eigen::VectorXd> &x_init,
               const std::vector<Eigen::VectorXd> &u_init,
               const std::vector<Eigen::VectorXd> &xs,
               const std::vector<Eigen::VectorXd> &us,
               const std::vector<crocoddyl::SolverFDDP::MatrixXdRowMajor> &K);
  void dumpToFile(std::string fileName);
  std::vector<OCP_debugData> fetchFromFile(std::string fileName);
  void reprOCP(const unsigned long time);

  // Setters and Getters
  boost::shared_ptr<crocoddyl::StateMultibody> get_state() { return state_; }
  const VectorXd get_torque();
  const MatrixXd get_gain();

  OCPSettings &get_settings() { return settings_; };
  DDP get_solver() { return (solver_); };
  size_t get_is_initialized() { return (is_initialized_); };
  size_t get_horizon_length() { return (settings_.horizon_length); };
};

}  // namespace deburring

#endif  // DEBURRING_OCP
