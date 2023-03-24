#include "deburring_ros_interface/ros_interface.h"

DeburringROSInterface::DeburringROSInterface(ros::NodeHandle nh) {
  ros::TransportHints hints;
  hints.tcpNoDelay(true);

  // Robot sensor subscriber
  sensor_sub_ = nh.subscribe<linear_feedback_controller_msgs::Sensor>(
      "/linear_feedback_controller/sensor_state", 1, &DeburringROSInterface::SensorCb, this, hints);

  // Control publisher
  command_pub_.reset(
      new realtime_tools::RealtimePublisher<
          linear_feedback_controller_msgs::Control>(nh, "/linear_feedback_controller/desired_control", 1));

  ros::Rate r(1);  // Rate for reading inital state from the robot
  while (sensor_msg_.header.stamp.toNSec() == 0) {
    // No measurments have been received if message time stamp is zero
    ROS_INFO_STREAM("Waiting for sensor measurments from the robot");
    ros::spinOnce();

    r.sleep();
  }

  joint_states_.resize(
      7                                                // Base pose
      + 6                                              // Base twist
      + (long)sensor_msg_.joint_state.position.size()  // Joint positions
      + (long)sensor_msg_.joint_state.velocity.size()  // Joint velocity
  );
}

void DeburringROSInterface::update(const Eigen::VectorXd& u0,
                               const Eigen::MatrixXd& K0) {
  mapControlToMsg(u0, K0);

  if (command_pub_->trylock()) {
    control_msg_.header.stamp = ros::Time::now();
    command_pub_->msg_ = control_msg_;
    command_pub_->unlockAndPublish();
  }
}

Eigen::VectorXd& DeburringROSInterface::get_robotState() {
  mapMsgToJointSates();
  return (joint_states_);
}

void DeburringROSInterface::SensorCb(
    const linear_feedback_controller_msgs::SensorConstPtr& msg) {
  sensor_msg_ = *msg;
}

void DeburringROSInterface::mapMsgToJointSates() {
  // Copying message to keep track of the initial state that is used to compute
  // control
  control_msg_.initial_state = sensor_msg_;

  // Converting from ros message to Eigen only when its necessary
  linear_feedback_controller_msgs::sensorMsgToEigen(sensor_msg_, sensor_eigen_);
  joint_states_ << sensor_eigen_.base_pose, sensor_eigen_.joint_state.position,
      sensor_eigen_.base_twist, sensor_eigen_.joint_state.velocity;
}

void DeburringROSInterface::mapControlToMsg(const Eigen::VectorXd& u0,
                                        const Eigen::MatrixXd& K0) {
  tf::matrixEigenToMsg(u0, control_msg_.feedforward);
  tf::matrixEigenToMsg(K0, control_msg_.feedback_gain);
}