#ifndef SOBEC_MPC_P
#define SOBEC_MPC_P

#include <pinocchio/fwd.hpp>

// include pinocchio first

#include "sobec/pointing/ocp-pointing.hpp"
#include "sobec/walk-with-traj/designer.hpp"

namespace sobec {

struct MPCSettings_Point {
  // Timing parameters
  size_t T_initialization;
  size_t T_stabilization;
  size_t T_drilling;

  // Mocap
  int use_mocap;

  // GainScheduling
  int use_gainScheduling;
  double gainSchedulig_slope;
  double maxGoalWeight;

  // Target
  Eigen::Vector3d targetPos;
  std::vector<Eigen::Vector3d> holes_offsets;
  double backwardOffset;
  double tolerance;

  void readParamsFromYamlString(std::string& StringToParse);
  void readParamsFromYamlFile(const std::string& Filename);
};

class MPC_Point {
 private:
  MPCSettings_Point settings_;
  RobotDesigner designer_;
  OCP_Point OCP_;

  Eigen::VectorXd x0_;
  Eigen::VectorXd u0_;
  Eigen::MatrixXd K0_;

  // MPC State
  size_t current_hole_ = 0;
  int drilling_state_ = 0;
  size_t iteration_ = 0;
  double goal_weight_ = 0;

  // Target related variables
  size_t number_holes_;
  std::vector<pinocchio::SE3> holes_offsets_;
  std::vector<pinocchio::SE3>
      list_oMhole_;  // Holes position in the robot frame
  pinocchio::SE3 backwardOffset_ = pinocchio::SE3::Identity();

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  pinocchio::SE3 oMtarget_;
  pinocchio::SE3 oMbackwardHole_;
  pinocchio::SE3 tool_se3_hole_;
  double position_error_ = 0;
  std::vector<unsigned long> controlled_joints_id_;
  Eigen::VectorXd x_internal_;

 private:
  void setTarget(pinocchio::SE3 toolMtarget);
  void updateTarget(pinocchio::SE3 toolMtarget);
  void updateOCP();
  void setHolesPlacement();

 public:
  MPC_Point(const MPCSettings_Point &settings,
            const OCPSettings_Point &OCPSettings, const RobotDesigner &design);

  void initialize(const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
                  pinocchio::SE3 toolMtarget);

  void iterate(const Eigen::VectorXd &x0, pinocchio::SE3 toolMtarget);

  void iterate(const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current,
               pinocchio::SE3 toolMtarget);

  const Eigen::VectorXd &shapeState(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &v);

  // getters and setters
  MPCSettings_Point &get_settings() { return settings_; }

  const Eigen::VectorXd &get_x0() const { return x0_; }

  const Eigen::VectorXd &get_u0() const { return u0_; }

  const Eigen::MatrixXd &get_K0() const { return K0_; }

  OCP_Point &get_OCP() { return OCP_; }
  void set_OCP(const OCP_Point &OCP) { OCP_ = OCP; }

  RobotDesigner &get_designer() { return designer_; }
  void set_designer(const RobotDesigner &designer) { designer_ = designer; }
};
}  // namespace sobec

#endif  // SOBEC_MPC_P
