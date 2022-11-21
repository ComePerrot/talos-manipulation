#ifndef ROS_MPC_INTERFACE
#define ROS_MPC_INTERFACE

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <linear_feedback_controller_msgs/Control.h>
#include <linear_feedback_controller_msgs/Sensor.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <linear_feedback_controller_msgs/eigen_conversions.hpp>
#include <map>
#include <string>

class ROS_MPC_Interface {
 public:
  ROS_MPC_Interface();
  void load(ros::NodeHandle nh);
  void update(const Eigen::VectorXd& u0, const Eigen::MatrixXd& K0);

  Eigen::VectorXd& get_robotState();

 private:
  void SensorCb(const linear_feedback_controller_msgs::SensorConstPtr& msg);
  void mapEigenToTF(const Eigen::VectorXd& u0, const Eigen::MatrixXd& K0);

  ros::Subscriber sensor_sub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<
      linear_feedback_controller_msgs::Control>>
      command_pub_;

  linear_feedback_controller_msgs::Sensor sensor_msg_;
  linear_feedback_controller_msgs::Control control_msg_;

  linear_feedback_controller_msgs::Eigen::Sensor sensor_eigen_;
  Eigen::VectorXd jointStates_;

  // prealocated memory

  Eigen::VectorXd jointPos_, jointVel_;
  Eigen::Vector3d base_position_, base_linear_vel_, base_angular_vel_;
  Eigen::Quaterniond base_quaternion_;
};

#endif  // ROS_MPC_INTERFACE
