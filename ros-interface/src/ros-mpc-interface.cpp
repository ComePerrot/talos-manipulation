#include "ros-interface/ros-mpc-interface.h"

ROS_MPC_Interface::ROS_MPC_Interface() {
  tf2_ros::TransformListener tfListener(tfBuffer);
  
}

Eigen::Affine3d ROS_MPC_Interface::get_TargetPlacement() {
  try {
    transformStamped = tfBuffer.lookupTransform("target", "tool", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }

  tf::transformMsgToEigen(transformStamped.transform, eigenTransform);

  return (eigenTransform);
};

std::map<std::string, Eigen::VectorXd> get_RobotState() {
  std::map<std::string, Eigen::VectorXd> map;
  return (map);
};
