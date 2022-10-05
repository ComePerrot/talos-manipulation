#ifndef ROS_MPC_INTERFACE
#define ROS_MPC_INTERFACE

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <map>
#include <string>

class ROS_MPC_Interface {
 public:
  ROS_MPC_Interface();

  void send_Command(Eigen::VectorXd x0, Eigen::VectorXd u0, Eigen::VectorXd K0);

  Eigen::Affine3d get_TargetPlacement();
  std::map<std::string, Eigen::VectorXd> get_RobotState();

 private:
  geometry_msgs::TransformStamped transformStamped;
  Eigen::Affine3d eigenTransform;

  tf2_ros::Buffer tfBuffer;
};

#endif  // ROS_MPC_INTERFACE
