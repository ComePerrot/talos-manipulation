#include <deburring_mpc/mpc.hpp>
// Must be included first

#include <pal_statistics/pal_statistics_macros.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "deburring_ros_interface/custom_registration_utils.h"
#include "deburring_ros_interface/ros_interface.h"

deburring::RobotDesigner buildRobotDesigner(ros::NodeHandle nh) {
  // Settings
  deburring::RobotDesignerSettings designer_settings =
      deburring::RobotDesignerSettings();
  nh.getParam("left_foot_name", designer_settings.left_foot_name);
  nh.getParam("right_foot_name", designer_settings.right_foot_name);
  nh.getParam("urdf", designer_settings.urdf_path);
  nh.getParam("srdf", designer_settings.srdf_path);
  nh.getParam("controlled_joints", designer_settings.controlled_joints_names);

  std::vector<double> gripperTtool;
  nh.getParam("tool_frame_pos", gripperTtool);
  pinocchio::SE3 gripperMtool = pinocchio::SE3::Identity();
  gripperMtool.translation().x() = gripperTtool[0];
  gripperMtool.translation().y() = gripperTtool[1];
  gripperMtool.translation().z() = gripperTtool[2];

  ROS_INFO_STREAM("Building robot designer");
  deburring::RobotDesigner designer =
      deburring::RobotDesigner(designer_settings);

  ROS_INFO_STREAM("Adding end effector frame to the robot model");
  designer.addEndEffectorFrame("deburring_tool",
                               "gripper_left_fingertip_3_link", gripperMtool);

  bool use_custom_limits;
  nh.getParam("custom_limits", use_custom_limits);

  if (use_custom_limits) {
    ROS_INFO_STREAM("Updating Limits");
    // Loading custom model limits
    std::vector<double> lower_position_limits;
    nh.getParam("lowerPositionLimit", lower_position_limits);
    std::vector<double> upper_position_limits;
    nh.getParam("upperPositionLimit", upper_position_limits);

    std::vector<double>::size_type size_limit = lower_position_limits.size();

    designer.updateModelLimits(
        Eigen::VectorXd::Map(lower_position_limits.data(),
                             (Eigen::Index)size_limit),
        Eigen::VectorXd::Map(upper_position_limits.data(),
                             (Eigen::Index)size_limit));
  }

  return (designer);
}

deburring::MPC buildMPC(ros::NodeHandle nh,
                        const deburring::RobotDesigner& pinWrapper) {
  std::string parameterFileName;
  nh.getParam("settings_file", parameterFileName);
  std::string parameterFilePath =
      ros::package::getPath("deburring_ros_interface") + "/config/";
  std::string parameterFile = parameterFilePath + parameterFileName;

  deburring::OCPSettings ocpSettings = deburring::OCPSettings();
  deburring::MPCSettings mpcSettings = deburring::MPCSettings();

  ocpSettings.readParamsFromYamlFile(parameterFile);
  mpcSettings.readParamsFromYamlFile(parameterFile);

  deburring::MPC mpc = deburring::MPC(mpcSettings, ocpSettings, pinWrapper);

  return (mpc);
}

class MoCapInterface {
 public:
  MoCapInterface() {
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
      transform_stamped_ =
          tf_buffer_->lookupTransform("target", "tool", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
    }
    tf::transformMsgToEigen(transform_stamped_.transform, eigen_transform_);
    toolMtarget_ = pinocchio::SE3(eigen_transform_.rotation(),
                                  eigen_transform_.translation());
  }

  // TF variables
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Target position wrt tool
  pinocchio::SE3 toolMtarget_ = pinocchio::SE3::Identity();

  // Memmory allocation
  geometry_msgs::TransformStamped transform_stamped_;
  Eigen::Affine3d eigen_transform_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_mpc_pointing");
  ros::NodeHandle nh;
  pal_statistics::RegistrationsRAII registered_variables;

  // Robot Desginer & MPC
  deburring::RobotDesigner pin_wrapper = buildRobotDesigner(nh);
  deburring::MPC MPC = buildMPC(nh, pin_wrapper);

  // Mocap Interface
  MoCapInterface Mocap = MoCapInterface();

  // Robot Interface
  DeburringROSInterface Robot = DeburringROSInterface(nh);

  // Initialize MPC
  int use_mocap = MPC.get_settings().use_mocap;
  pinocchio::SE3 toolMtarget;

  if (use_mocap > 0) {
    toolMtarget = Mocap.get_toolMtarget();
  } else {
    toolMtarget = pinocchio::SE3::Identity();
  }

  Eigen::VectorXd x = Robot.get_robotState();
  MPC.initialize(x.head(MPC.get_designer().get_rmodel().nq),
                 x.tail(MPC.get_designer().get_rmodel().nv), toolMtarget);

  REGISTER_VARIABLE("/introspection_data", "end_effector_actual_position",
                    &MPC.get_designer().get_end_effector_frame().translation(),
                    &registered_variables);
  REGISTER_VARIABLE("/introspection_data", "end_effector_desired_position",
                    &MPC.get_target_frame().translation(),
                    &registered_variables);

  Eigen::VectorXd u0;
  Eigen::MatrixXd K0;

  ros::Rate r(static_cast<double>(1 / MPC.get_OCP().get_settings().time_step));
  while (ros::ok()) {
    ros::spinOnce();

    // Get state from Robot intergace
    x = Robot.get_robotState();

    // Solving MPC iteration
    MPC.iterate(x, toolMtarget);

    // Sending command to robot
    u0 = MPC.get_u0();
    K0 = MPC.get_K0();

    Robot.update(u0, K0);

    PUBLISH_STATISTICS("/introspection_data");

    r.sleep();
  }

  return (0);
}
