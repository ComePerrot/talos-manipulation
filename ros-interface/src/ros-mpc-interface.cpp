#include "ros-interface/ros-mpc-interface.h"

ROS_MPC_Interface::ROS_MPC_Interface() {}

void ROS_MPC_Interface::load(ros::NodeHandle nh) {
  ros::TransportHints hints;
  hints.tcpNoDelay(true);

  // Robot sensor subscriber
  sensor_sub_ = nh.subscribe<linear_feedback_controller_msgs::Sensor>(
      "sensor_state", 1, &ROS_MPC_Interface::SensorCb, this, hints);

  // Control publisher
  command_pub_.reset(
      new realtime_tools::RealtimePublisher<
          linear_feedback_controller_msgs::Control>(nh, "command", 1));

  while ((jointPos_.norm() == 0) || (jointPos_.norm() == 0)) {
    ROS_INFO_THROTTLE(0.5, "Receiving joint state from the robot");
    ros::spinOnce();
  }
}

void ROS_MPC_Interface::update(const Eigen::VectorXd& u0,
                               const Eigen::MatrixXd& K0) {
  mapEigenToTF(u0, K0);

  if (command_pub_->trylock()) {
    control_msg_.header.stamp = ros::Time::now();
    command_pub_->msg_ = control_msg_;
    command_pub_->unlockAndPublish();
  }
}

Eigen::VectorXd& ROS_MPC_Interface::get_robotState() {
  // Copying message to keep track of the initial state that is used to compute
  // control
  control_msg_.initial_state = sensor_msg_;

  // Converting from ros message to Eigen only when its necessary
  linear_feedback_controller_msgs::sensorMsgToEigen(sensor_msg_, sensor_eigen_);
  jointStates_ << sensor_eigen_.base_pose, sensor_eigen_.joint_state.position,
      sensor_eigen_.base_twist, sensor_eigen_.joint_state.velocity;

  return (jointStates_);
}

void ROS_MPC_Interface::SensorCb(
    const linear_feedback_controller_msgs::SensorConstPtr& msg) {
  sensor_msg_ = *msg;
}

void ROS_MPC_Interface::mapEigenToTF(const Eigen::VectorXd& u0,
                                     const Eigen::MatrixXd& K0) {
  tf::matrixEigenToMsg(u0, control_msg_.feedforward);
  tf::matrixEigenToMsg(K0, control_msg_.feedback_gain);
}