#ifndef DEBURRING_MPC
#define DEBURRING_MPC

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/robot_designer.hpp"
#include "deburring_mpc/ocp.hpp"

namespace deburring {

struct MPC_command {
 public:
  Eigen::VectorXd us0;
  Eigen::MatrixXd K0;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &us0;
    ar &K0;
  }
};

struct MPC_debugData {
 public:
  Eigen::VectorXd x_input;

  MPC_command output;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &x_input;

    ar &output;
  }
};

struct MPCSettings {
  // Timing parameters
  size_t T_initialization;
  size_t T_stabilization;
  size_t T_drilling;

  // Mocap
  int use_mocap;

  // GainScheduling
  int use_gain_scheduling;
  double gain_schedulig_slope;
  double gain_schedulig_max_weight;

  // Target
  Vector3d target_position;
  std::vector<Vector3d> holes_offsets;
  double backward_offset;
  double precision_threshold;

  void readParamsFromYamlString(std::string &string_to_parse);
  void readParamsFromYamlFile(const std::string &filename);
};

class MPC {
 private:
  MPCSettings settings_;
  RobotDesigner designer_;
  OCP OCP_;

  VectorXd x0_;
  VectorXd u0_;
  MatrixXd K0_;

  // MPC State
  size_t current_hole_ = 0;
  int drilling_state_ = 0;
  size_t iteration_ = 0;
  double goal_weight_ = 0;

  // Target related variables
  size_t number_holes_;
  std::vector<SE3> holes_offsets_;
  std::vector<SE3> list_oMhole_;  // Holes position in the robot frame
  SE3 backward_offset_ = SE3::Identity();

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  SE3 oMtarget_;
  SE3 oMdisengaged_target_;
  SE3 toolMhole_;
  double position_error_ = 0;
  std::vector<unsigned long> controlled_joints_ids_;
  VectorXd x_internal_;

 private:
  void setTarget(const SE3 &toolMtarget);
  void setHolesPlacement();
  void updateTarget(const SE3 &toolMtarget);
  void updateOCP();

 public:
  MPC(const MPCSettings &mpc_settings,
            const OCPSettings &ocp_settings, const RobotDesigner &designer);

  void initialize(const ConstVectorRef &q0, const ConstVectorRef &v0,
                  const SE3 &toolMtarget);

  void iterate(const VectorXd &x0, const SE3 &toolMtarget);

  void iterate(const ConstVectorRef &q_current, const ConstVectorRef &v_current,
               const SE3 &toolMtarget);

  const VectorXd &shapeState(const ConstVectorRef &q, const ConstVectorRef &v);

  // Debug
  std::vector<MPC_debugData> debugDataMPC_;

  void logData(const Eigen::Ref<const Eigen::VectorXd> x_input,
               const Eigen::Ref<const Eigen::VectorXd> us0,
               const Eigen::Ref<const Eigen::MatrixXd> K0);

  void dumpToFile(std::string name);
  std::vector<MPC_debugData> fetchFromFile(std::string name);

  // getters and setters
  MPCSettings &get_settings() { return settings_; }

  int get_drilling_state() { return drilling_state_; }

  const VectorXd &get_x0() const { return x0_; }

  const VectorXd &get_u0() const { return u0_; }

  const MatrixXd &get_K0() const { return K0_; }

  const SE3 &get_target_frame() const {
    return list_oMhole_[current_hole_];
  }

  OCP &get_OCP() { return OCP_; }
  void set_OCP(const OCP &OCP) { OCP_ = OCP; }

  RobotDesigner &get_designer() { return designer_; }
  void set_designer(const RobotDesigner &designer) { designer_ = designer; }
};
}  // namespace deburring

#endif  // DEBURRING_MPC
