#ifndef MPC_P
#define MPC_P

#include "deburring_mpc/fwd.hpp"
#include "deburring_mpc/robot_designer.hpp"
#include "deburring_mpc/ocp.hpp"

namespace mpc_p {

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
  Vector3d targetPos;
  std::vector<Vector3d> holes_offsets;
  double backwardOffset;
  double tolerance;

  void readParamsFromYamlString(std::string &StringToParse);
  void readParamsFromYamlFile(const std::string &Filename);
};

class MPC_Point {
 private:
  MPCSettings_Point settings_;
  RobotDesigner designer_;
  OCP_Point OCP_;

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
  SE3 backwardOffset_ = SE3::Identity();

  // Security management
  bool initialized_ = false;

  // Memory preallocations:
  SE3 oMtarget_;
  SE3 oMbackwardHole_;
  SE3 tool_se3_hole_;
  double position_error_ = 0;
  std::vector<unsigned long> controlled_joints_ids_;
  VectorXd x_internal_;

 private:
  void setTarget(const SE3 &toolMtarget);
  void setHolesPlacement();
  void updateTarget(const SE3 &toolMtarget);
  void updateOCP();

 public:
  MPC_Point(const MPCSettings_Point &settings,
            const OCPSettings_Point &OCPSettings, const RobotDesigner &designer);

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
  MPCSettings_Point &get_settings() { return settings_; }

  int get_drillingState() { return drilling_state_; }

  const VectorXd &get_x0() const { return x0_; }

  const VectorXd &get_u0() const { return u0_; }

  const MatrixXd &get_K0() const { return K0_; }

  const pinocchio::SE3 &get_Target_frame() const {
    return list_oMhole_[current_hole_];
  }

  OCP_Point &get_OCP() { return OCP_; }
  void set_OCP(const OCP_Point &OCP) { OCP_ = OCP; }

  RobotDesigner &get_designer() { return designer_; }
  void set_designer(const RobotDesigner &designer) { designer_ = designer; }
};
}  // namespace mpc_p

#endif  // MPC_P
