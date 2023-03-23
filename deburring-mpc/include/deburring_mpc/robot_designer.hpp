#ifndef MPC_P_DESIGNER
#define MPC_P_DESIGNER

#include <pinocchio/fwd.hpp>
// Include pinocchio first
#include <Eigen/Dense>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <vector>

#include "deburring_mpc/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace mpc_p {

struct RobotDesignerSettings {
 public:
  std::string urdf_path = "";
  std::string srdf_path = "";
  std::string robot_description = "";
  std::vector<std::string> controlled_joints_names;

  std::string left_foot_name = "";
  std::string right_foot_name = "";
};

class RobotDesigner {
 private:
  RobotDesignerSettings settings_;

  std::vector<unsigned long> controlled_joints_ids_;
  unsigned long left_foot_id_, right_foot_id_, end_effector_id_;

  pinocchio::Model rmodel_complete_, rmodel_;
  pinocchio::Data rdata_complete_, rdata_;
  // std::vector<pinocchio::JointIndex> pinocchioControlledJoints_;

  Eigen::VectorXd q0_complete_, q0_;
  Eigen::VectorXd v0_complete_, v0_;
  Eigen::VectorXd x0_;

  Eigen::Vector3d com_position_;
  Eigen::Vector3d lf_position_;
  Eigen::Vector3d rf_position_;

  bool is_initialized_ = false;

  // Memori allocations
  double robot_mass_ = 0;

 public:
  RobotDesigner();
  RobotDesigner(const RobotDesignerSettings &settings);
  void initialize(const RobotDesignerSettings &settings);

  void updateReducedModel(const Eigen::VectorXd &x);
  void updateCompleteModel(const Eigen::VectorXd &x);

  void updateModelLimits(const Eigen::VectorXd lower_position_limit,
                         const Eigen::VectorXd upper_Position_limit);
  void addEndEffectorFrame(std::string end_effector_name, std::string parent_name,
                           pinocchio::SE3 end_effector_placement);

  // accessors
  double get_robot_mass();

  const pinocchio::SE3 &get_lf_frame();
  const pinocchio::SE3 &get_rf_frame();
  const pinocchio::SE3 &get_end_effector_frame();

  bool get_is_initialized() { return is_initialized_; }

  const pinocchio::Model &get_rmodel() { return rmodel_; }
  const pinocchio::Model &get_rmodel_complete() { return rmodel_complete_; }
  const pinocchio::Data &get_rdata() { return rdata_; }
  const pinocchio::Data &get_rdata_complete() { return rdata_complete_; }
  const Eigen::VectorXd &get_q0() { return q0_; }
  const Eigen::VectorXd &get_v0() { return v0_; }
  const Eigen::VectorXd &get_q0_complete() { return q0_complete_; }
  const Eigen::VectorXd &get_v0_complete() { return v0_complete_; }
  const Eigen::VectorXd &get_x0() { return x0_; }

  const std::string &get_lf_name() { return settings_.left_foot_name; }
  const std::string &get_rf_name() { return settings_.right_foot_name; }
  const pinocchio::FrameIndex &get_lf_id() { return left_foot_id_; }
  const pinocchio::FrameIndex &get_rf_id() { return right_foot_id_; }
  const pinocchio::FrameIndex &get_end_effector_id() { return end_effector_id_; }
  const RobotDesignerSettings &get_settings() { return settings_; }
  const std::vector<unsigned long> &get_controlled_joints_ids() {
    return controlled_joints_ids_;
  }

  const Eigen::Vector3d &get_lf_position() { return lf_position_; }
  const Eigen::Vector3d &get_rf_position() { return rf_position_; }
  const Eigen::Vector3d &get_com_position() { return com_position_; }
};

}  // namespace mpc_p
#endif  // MPC_P_DESIGNER
