#ifndef MPC_P_DESIGNER
#define MPC_P_DESIGNER

#include <pinocchio/fwd.hpp>
// Include pinocchio first
#include <Eigen/Dense>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <vector>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "deburring_mpc/fwd.hpp"

namespace mpc_p {

struct RobotDesignerSettings {
 public:
  std::string urdfPath = "";
  std::string srdfPath = "";
  std::string robotDescription = "";
  std::vector<std::string> controlledJointsNames;

  std::string leftFootName = "";
  std::string rightFootName = "";
};

class RobotDesigner {
 private:
  RobotDesignerSettings settings_;

  std::vector<unsigned long> controlled_joints_id_;
  unsigned long leftFootId_, rightFootId_, EndEffectorId_;

  pinocchio::Model rModelComplete_, rModel_;
  pinocchio::Data rDataComplete_, rData_;
  // std::vector<pinocchio::JointIndex> pinocchioControlledJoints_;

  Eigen::VectorXd q0Complete_, q0_;
  Eigen::VectorXd v0Complete_, v0_;
  Eigen::VectorXd x0_;

  Eigen::Vector3d com_position_;
  Eigen::Vector3d LF_position_;
  Eigen::Vector3d RF_position_;

  // Memori allocations
  double mass_ = 0;

 public:
  RobotDesigner();
  RobotDesigner(const RobotDesignerSettings &settings);
  void initialize(const RobotDesignerSettings &settings);
  bool initialized_ = false;

  void updateReducedModel(const Eigen::VectorXd &x);
  void updateCompleteModel(const Eigen::VectorXd &x);

  void updateModelLimits(const Eigen::VectorXd lowerPositionLimit,
                         const Eigen::VectorXd upperPositionLimit);
  void addEndEffectorFrame(std::string endEffectorName, std::string parentName,
                           pinocchio::SE3 endEffectorPlacement);

  const pinocchio::SE3 &get_LF_frame();
  const pinocchio::SE3 &get_RF_frame();
  const pinocchio::SE3 &get_EndEff_frame();

  double getRobotMass();

  const pinocchio::Model &get_rModel() { return rModel_; }
  const pinocchio::Model &get_rModelComplete() { return rModelComplete_; }
  const pinocchio::Data &get_rData() { return rData_; }
  const pinocchio::Data &get_rDataComplete() { return rDataComplete_; }
  const Eigen::VectorXd &get_q0() { return q0_; }
  const Eigen::VectorXd &get_v0() { return v0_; }
  const Eigen::VectorXd &get_q0Complete() { return q0Complete_; }
  const Eigen::VectorXd &get_v0Complete() { return v0Complete_; }
  const Eigen::VectorXd &get_x0() { return x0_; }

  const std::string &get_LF_name() { return settings_.leftFootName; }
  const std::string &get_RF_name() { return settings_.rightFootName; }
  const pinocchio::FrameIndex &get_LF_id() { return leftFootId_; }
  const pinocchio::FrameIndex &get_RF_id() { return rightFootId_; }
  const pinocchio::FrameIndex &get_EndEff_id() { return EndEffectorId_; }
  const RobotDesignerSettings &get_settings() { return settings_; }
  const std::vector<unsigned long> &get_controlledJointsIDs() {
    return controlled_joints_id_;
  }

  const Eigen::Vector3d &get_LF_position() { return LF_position_; }
  const Eigen::Vector3d &get_RF_position() { return RF_position_; }
  const Eigen::Vector3d &get_com_position() { return com_position_; }
};

}  // namespace mpc_p
#endif  // MPC_P_DESIGNER
