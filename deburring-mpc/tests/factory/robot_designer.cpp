#include "robot_designer.hpp"

namespace deburring {
namespace unittest {
deburring::RobotDesigner RobotDesignerFactory::createRobotDesigner() {
  //  Robot designer
  deburring::RobotDesignerSettings designerSettings =
      deburring::RobotDesignerSettings();

  designerSettings.controlled_joints_names = {
      "root_joint",        "leg_left_1_joint",  "leg_left_2_joint",
      "leg_left_3_joint",  "leg_left_4_joint",  "leg_left_5_joint",
      "leg_left_6_joint",  "leg_right_1_joint", "leg_right_2_joint",
      "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint",
      "leg_right_6_joint", "torso_1_joint",     "torso_2_joint",
      "arm_left_1_joint",  "arm_left_2_joint",  "arm_left_3_joint",
      "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",
      "arm_left_7_joint",  "arm_right_1_joint", "arm_right_2_joint",
      "arm_right_3_joint", "arm_right_4_joint"};
  designerSettings.left_foot_name = "left_sole_link";
  designerSettings.right_foot_name = "right_sole_link";
  designerSettings.urdf_path =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/robots/"
      "talos_reduced.urdf";
  designerSettings.srdf_path =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/srdf/"
      "talos.srdf";

  deburring::RobotDesigner pinWrapper =
      deburring::RobotDesigner(designerSettings);

  pinocchio::SE3 gripperMtool = pinocchio::SE3::Identity();
  gripperMtool.translation().x() = 0;
  gripperMtool.translation().y() = -0.02;
  gripperMtool.translation().z() = -0.0825;
  pinWrapper.addEndEffectorFrame("deburring_tool",
                                 "gripper_left_fingertip_3_link", gripperMtool);

  std::vector<double> lowerPositionLimit = {
      // Base
      -1, -1, -1, -1, -1, -1, -1,
      // Left leg
      -0.35, -0.52, -2.10, 0.0, -1.31, -0.52,
      // Right leg
      -1.57, -0.52, -2.10, 0.0, -1.31, -0.52,
      // Torso
      -1.3, -0.1,
      // Left arm
      -1.57, 0.2, -2.44, -2.1, -2.53, -1.3, -0.6,
      // Right arm
      -0.4, -2.88, -2.44, -2};
  std::vector<double> upperPositionLimit = {// Base
                                            1, 1, 1, 1, 1, 1, 1,
                                            // Left leg
                                            1.57, 0.52, 0.7, 2.62, 0.77, 0.52,
                                            // Right leg
                                            0.35, 0.52, 0.7, 2.62, 0.77, 0.52,
                                            // Torso
                                            1.3, 0.78,
                                            // Left arm
                                            0.52, 2.88, 2.44, 0, 2.53, 1.3, 0.6,
                                            // Right arm
                                            1.57, -0.2, 2.44, 0};

  std::vector<double>::size_type size_limit = lowerPositionLimit.size();
  pinWrapper.updateModelLimits(
      Eigen::VectorXd::Map(lowerPositionLimit.data(), (Eigen::Index)size_limit),
      Eigen::VectorXd::Map(upperPositionLimit.data(),
                           (Eigen::Index)size_limit));

  return (pinWrapper);
}
}  // namespace unittest
}  // namespace deburring