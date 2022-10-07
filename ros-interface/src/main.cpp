#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <mpc-pointing/mpc.hpp>
#include <sobec/walk-with-traj/designer.hpp>

#include "ros-interface/ros-mpc-interface.h"

ros_wbmpc_msgs::SensorDataConstPtr sensor_measurments;

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

mpc_p::MPC_Point buildMPC(const sobec::RobotDesigner& pinWrapper,
                          std::string parameterFile) {
  mpc_p::OCPSettings_Point ocpSettings = mpc_p::OCPSettings_Point();
  mpc_p::MPCSettings_Point mpcSettings = mpc_p::MPCSettings_Point();

  ocpSettings.readParamsFromYamlFile(parameterFile);
  mpcSettings.readParamsFromYamlFile(parameterFile);

  mpc_p::MPC_Point mpc = mpc_p::MPC_Point(mpcSettings, ocpSettings, pinWrapper);

  return (mpc);
}

void SensorCb(const ros_wbmpc_msgs::SensorDataConstPtr& msg) {
  // ROS_INFO_STREAM("Received joint state from subscriber");
  sensor_measurments = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "talos_manipulation");
  ros::NodeHandle nh;

  // Define ROS Communication elements
  ros::TransportHints hints;
  hints.tcpNoDelay(true);
  //  tf2 listener
  pinocchio::SE3 toolMtarget;
  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //  sensor subscriber
  ros::Subscriber sensor_sub =
      nh.subscribe("sensor_robot", 1, &SensorCb, hints);

  //  command publisher
  ros_wbmpc_msgs::ControlDataConstPtr control_data;
  Eigen::Affine3d eigenTransform;
  boost::shared_ptr<
      realtime_tools::RealtimePublisher<ros_wbmpc_msgs::ControlData>>
      command_pub;
  command_pub.reset(
      new realtime_tools::RealtimePublisher<ros_wbmpc_msgs::ControlData>(
          nh, "command", 1));

  // Define MPC
  sobec::RobotDesigner pinWrapper = buildRobotDesigner();
  mpc_p::MPC_Point MPC = buildMPC(pinWrapper, "test");

  // Initialize MPC
  try {
    transformStamped = tfBuffer.lookupTransform("target", "tool", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
  }
  tf::transformMsgToEigen(transformStamped.transform, eigenTransform);
  toolMtarget =
      pinocchio::SE3(eigenTransform.rotation(), eigenTransform.translation());
  MPC.initialize(sensor_measurments["q"], sensor_measurments["v"], toolMtarget);

  while (ros::ok()) {
    try {
      transformStamped =
          tfBuffer.lookupTransform("target", "tool", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
    }

    tf::transformMsgToEigen(transformStamped.transform, eigenTransform);
    toolMtarget =
        pinocchio::SE3(eigenTransform.rotation(), eigenTransform.translation());

    MPC.iterate(sensor_measurments["q"], sensor_measurments["v"], toolMtarget);

    // ROS_Interface.send_Command(MPC.get_x0(), MPC.get_u0(), MPC.get_K0());
    if (command_pub->trylock()) {
      command_pub->msg_ = control_data;
      command_pub->unlockAndPublish();
    }

    ros::spinOnce();
  }

  return (0);
}
