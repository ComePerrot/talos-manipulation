#include <mpc-pointing/mpc.hpp>
#include <sobec/walk-with-traj/designer.hpp>
// Must be included first

#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

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
  pinocchio::SE3 gripperMtool = pinocchio::SE3::Identity();
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

mpc_p::MPC_Point buildMPC(ros::NodeHandle nh,
                          const sobec::RobotDesigner& pinWrapper) {
  std::string parameterFileName;
  nh.getParam("settings_file", parameterFileName);
  std::string parameterFilePath =
      ros::package::getPath("ros-interface") + "/config/";
  std::string parameterFile = parameterFilePath + parameterFileName;

  mpc_p::OCPSettings_Point ocpSettings = mpc_p::OCPSettings_Point();
  mpc_p::MPCSettings_Point mpcSettings = mpc_p::MPCSettings_Point();

  ocpSettings.readParamsFromYamlFile(parameterFile);
  mpcSettings.readParamsFromYamlFile(parameterFile);

  mpc_p::MPC_Point mpc = mpc_p::MPC_Point(mpcSettings, ocpSettings, pinWrapper);

  return (mpc);
}

class MOCAP_Interface {
 public:
  MOCAP_Interface() {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // while (toolMtarget_.isIdentity()) {
    //   readTF();
    //   ROS_INFO_THROTTLE(0.5, "Receiving target position from the MOCAP");
    //   ros::spinOnce();
    // }
  }

  pinocchio::SE3& get_toolMtarget() {
    readTF();
    return (toolMtarget_);
  }

 private:
  void readTF() {
    try {
      transformStamped =
          tf_buffer_->lookupTransform("target", "tool", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
    }
    tf::transformMsgToEigen(transformStamped.transform, eigenTransform);
    toolMtarget_ =
        pinocchio::SE3(eigenTransform.rotation(), eigenTransform.translation());
  }
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Affine3d eigenTransform;
  pinocchio::SE3 toolMtarget_ = pinocchio::SE3::Identity();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_mpc_pointing");
  ros::NodeHandle nh;

  // Robot Desginer & MPC
  sobec::RobotDesigner pinWrapper = buildRobotDesigner(nh);
  mpc_p::MPC_Point MPC = buildMPC(nh, pinWrapper);

  // Mocap Interface
  MOCAP_Interface Mocap = MOCAP_Interface();

  // Robot Interface
  ROS_MPC_Interface Robot = ROS_MPC_Interface(nh);

  // Initialize MPC
  int use_mocap = MPC.get_settings().use_mocap;
  pinocchio::SE3 toolMtarget;

  if (use_mocap > 0) {
    toolMtarget = Mocap.get_toolMtarget();
  } else {
    toolMtarget = pinocchio::SE3::Identity();
  }

  Eigen::VectorXd x0 = Robot.get_robotState();
  MPC.initialize(x0.head(MPC.get_designer().get_rModel().nq),
                 x0.tail(MPC.get_designer().get_rModel().nv), toolMtarget);

  while (ros::ok()) {
    if (use_mocap > 0) {
      toolMtarget = Mocap.get_toolMtarget();
    }

    // Solving MPC iteration
    MPC.iterate(Robot.get_robotState(), toolMtarget);

    // Sending command to robot
    Robot.update(MPC.get_u0(), MPC.get_K0());

    ros::spinOnce();
  }

  return (0);
}
