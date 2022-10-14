#include <mpc-pointing/mpc.hpp>
#include <sobec/walk-with-traj/designer.hpp>
// Must be included first

#include <realtime_tools/realtime_publisher.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros_wbmpc_msgs/Control.h>
#include <ros_wbmpc_msgs/Sensor.h>

#include "ros-interface/ros-mpc-interface.h"

sobec::RobotDesigner buildRobotDesigner(ros::NodeHandle nh) {
  // Settings
  sobec::RobotDesignerSettings designerSettings =
      sobec::RobotDesignerSettings();
  nh.getParam("left_foot_name", designerSettings.leftFootName);
  nh.getParam("right_foot_name", designerSettings.rightFootName);
  nh.getParam("urdf", designerSettings.urdfPath);
  nh.getParam("srdf", designerSettings.srdfPath);
  nh.getParam("controlled_joints", designerSettings.controlledJointsNames);

  std::vector<double> gripperTtool;
  nh.getParam("tool_frame_pos", gripperTtool);
  pinocchio::SE3 gripperMtool = pinocchio::SE3();
  gripperMtool.translation().x() = gripperTtool[0];
  gripperMtool.translation().y() = gripperTtool[1];
  gripperMtool.translation().z() = gripperTtool[2];

  ROS_INFO_STREAM("Building robot designer");
  sobec::RobotDesigner designer = sobec::RobotDesigner(designerSettings);

  ROS_INFO_STREAM("Adding end effector frame to the robot model");
  designer.addEndEffectorFrame("deburring_tool",
                               "gripper_left_fingertip_3_link", gripperMtool);

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

void SensorCb(const ros_wbmpc_msgs::SensorConstPtr& msg,
              Eigen::VectorXd& jointPos, Eigen::VectorXd& jointVel) {
  // ROS_INFO_STREAM("Received joint state from subscriber");
  Eigen::Vector3d base_position, base_linear_vel, base_angular_vel;
  Eigen::Quaterniond base_quaternion;

  // Base State
  tf::pointMsgToEigen(msg->base_state.pose.pose.position, base_position);
  tf::quaternionMsgToEigen(msg->base_state.pose.pose.orientation,
                           base_quaternion);
  base_quaternion.normalize();

  tf::vectorMsgToEigen(msg->base_state.twist.twist.linear, base_linear_vel);
  tf::vectorMsgToEigen(msg->base_state.twist.twist.angular, base_angular_vel);

  for (int i = 0; i < 3; i++) {
    jointPos[i] = base_position[i];
    jointVel[i] = base_linear_vel[i];
    jointVel[i + 3] = base_angular_vel[i];
  }

  jointPos[3] = base_quaternion.x();
  jointPos[4] = base_quaternion.y();
  jointPos[5] = base_quaternion.z();
  jointPos[6] = base_quaternion.w();

  // Joint States
  for (size_t i = 0; i < msg->joint_state.position.size(); i++) {
    jointPos[(Eigen::Index)i] = msg->joint_state.position[i];
    jointVel[(Eigen::Index)i] = msg->joint_state.velocity[i];
  }
}

void mapToTF(const Eigen::VectorXd& u0, const Eigen::MatrixXd& K0,
             ros_wbmpc_msgs::Control& control) {
  tf::matrixEigenToMsg(u0, control.feedforward);
  tf::matrixEigenToMsg(K0, control.feedback_gain);
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
  Eigen::VectorXd jointPos;
  Eigen::VectorXd jointVel;
  ros::Subscriber sensor_sub = nh.subscribe<ros_wbmpc_msgs::Sensor>(
      "sensor_state", 1, boost::bind(&SensorCb, _1, jointPos, jointVel));

  //  command publisher
  ros_wbmpc_msgs::Control control_data;
  Eigen::Affine3d eigenTransform;
  boost::shared_ptr<realtime_tools::RealtimePublisher<ros_wbmpc_msgs::Control>>
      command_pub;
  command_pub.reset(
      new realtime_tools::RealtimePublisher<ros_wbmpc_msgs::Control>(
          nh, "command", 1));

  // Define MPC
  std::string paramFile;
  nh.getParam("settings_file", paramFile);
  paramFile = ros::package::getPath("ros-interface") + "/config/" + paramFile;

  sobec::RobotDesigner pinWrapper = buildRobotDesigner(nh);
  mpc_p::MPC_Point MPC = buildMPC(pinWrapper, paramFile);

  // Initialize MPC
  while ((jointPos.norm() == 0) || (jointPos.norm() == 0)) {
    ROS_INFO_THROTTLE(0.5, "Receiving joint state from the robot");
    ros::spinOnce();
  }

  if (MPC.get_settings().use_mocap > 0) {
    try {
      transformStamped =
          tfBuffer.lookupTransform("target", "tool", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
    }
    tf::transformMsgToEigen(transformStamped.transform, eigenTransform);
    toolMtarget =
        pinocchio::SE3(eigenTransform.rotation(), eigenTransform.translation());
  } else {
    toolMtarget = pinocchio::SE3::Identity();
  }

  MPC.initialize(jointPos, jointVel, toolMtarget);

  while (ros::ok()) {
    if (MPC.get_settings().use_mocap > 0) {
      try {
        transformStamped =
            tfBuffer.lookupTransform("target", "tool", ros::Time(0));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
      }

      tf::transformMsgToEigen(transformStamped.transform, eigenTransform);
      toolMtarget = pinocchio::SE3(eigenTransform.rotation(),
                                   eigenTransform.translation());
    }

    MPC.iterate(jointPos, jointVel, toolMtarget);
    mapToTF(MPC.get_u0(), MPC.get_K0(), control_data);
    // ROS_Interface.send_Command(MPC.get_x0(), MPC.get_u0(), MPC.get_K0());
    if (command_pub->trylock()) {
      command_pub->msg_ = control_data;
      command_pub->unlockAndPublish();
    }

    ros::spinOnce();
  }

  return (0);
}
