#include <ros/ros.h>

#include <sobec/pointing/mpc-pointing.hpp>
#include <sobec/walk-with-traj/designer.hpp>

#include "talos_manipulation/ros-mpc-interface.h"

sobec::RobotDesigner buildRobotDesigner() {
  // Settings
  sobec::RobotDesignerSettings designerSettings =
      sobec::RobotDesignerSettings();
  designerSettings.leftFootName = "right_sole_link";
  designerSettings.rightFootName = "left_sole_link";
  designerSettings.urdfPath =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/robots/"
      "talos_reduced.urdf";
  designerSettings.srdfPath =
      "/opt/openrobots/share/example-robot-data/robots/talos_data/srdf/"
      "talos.srdf";

  designerSettings.controlledJointsNames.push_back("root_joint");
  designerSettings.controlledJointsNames.push_back("leg_left_1_joint");
  designerSettings.controlledJointsNames.push_back("leg_left_2_joint");
  designerSettings.controlledJointsNames.push_back("leg_left_3_joint");
  designerSettings.controlledJointsNames.push_back("leg_left_4_joint");
  designerSettings.controlledJointsNames.push_back("leg_left_5_joint");
  designerSettings.controlledJointsNames.push_back("leg_left_6_joint");
  designerSettings.controlledJointsNames.push_back("leg_right_1_joint");
  designerSettings.controlledJointsNames.push_back("leg_right_2_joint");
  designerSettings.controlledJointsNames.push_back("leg_right_3_joint");
  designerSettings.controlledJointsNames.push_back("leg_right_4_joint");
  designerSettings.controlledJointsNames.push_back("leg_right_5_joint");
  designerSettings.controlledJointsNames.push_back("leg_right_6_joint");
  designerSettings.controlledJointsNames.push_back("torso_1_joint");
  designerSettings.controlledJointsNames.push_back("torso_2_joint");
  designerSettings.controlledJointsNames.push_back("arm_left_1_joint");
  designerSettings.controlledJointsNames.push_back("arm_left_2_joint");
  designerSettings.controlledJointsNames.push_back("arm_left_3_joint");
  designerSettings.controlledJointsNames.push_back("arm_left_4_joint");
  designerSettings.controlledJointsNames.push_back("arm_left_5_joint");
  designerSettings.controlledJointsNames.push_back("arm_left_6_joint");
  designerSettings.controlledJointsNames.push_back("arm_left_7_joint");
  designerSettings.controlledJointsNames.push_back("arm_right_1_joint");
  designerSettings.controlledJointsNames.push_back("arm_right_2_joint");
  designerSettings.controlledJointsNames.push_back("arm_right_3_joint");
  designerSettings.controlledJointsNames.push_back("arm_right_4_joint");

  std::cout << "Building designer:" << std::endl;
  sobec::RobotDesigner designer = sobec::RobotDesigner(designerSettings);
  std::cout << "  Done" << std::endl;

  pinocchio::SE3 gripper_SE3_tool = pinocchio::SE3::Identity();
  gripper_SE3_tool.translation().x() = 0;
  gripper_SE3_tool.translation().y() = -0.02;
  gripper_SE3_tool.translation().z() = -0.0825;

  std::cout << "Adding end effector frame:" << std::endl;
  designer.addEndEffectorFrame(
      "deburring_tool", "gripper_left_fingertip_3_link", gripper_SE3_tool);
  std::cout << "  Done" << std::endl;

  return (designer);
}

sobec::MPC_Point buildMPC(const sobec::RobotDesigner& pinWrapper,
                          std::string parameterFile) {
  sobec::OCPSettings_Point ocpSettings = sobec::OCPSettings_Point();
  sobec::MPCSettings_Point mpcSettings = sobec::MPCSettings_Point();

  ocpSettings.readParamsFromYamlFile(parameterFile);
  mpcSettings.readParamsFromYamlFile(parameterFile);

  sobec::MPC_Point mpc = sobec::MPC_Point(mpcSettings, ocpSettings, pinWrapper);

  return (mpc);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "talos_manipulation");
  ros::NodeHandle nh("~");

  
  
  // Initialize MPC
  MPC.initialize(pinWrapper.get_q0Complete(), pinWrapper.get_v0Complete(),
                 pinocchio::SE3::Identity());

  while (ros::ok()) {
    ros::spinOnce();
  }

  return (0);
}