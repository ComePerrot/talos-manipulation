#include "ros-interface/ros-mpc-interface.h"

ROS_MPC_Interface::ROS_MPC_Interface() {}

void ROS_MPC_Interface::load(ros::NodeHandle nh) {
  ros::TransportHints hints;
  hints.tcpNoDelay(true);

  // Robot sensor subscriber
  sensor_sub_ = nh.subscribe<ros_wbmpc_msgs::Sensor>(
      "sensor_state", 1, &ROS_MPC_Interface::SensorCb, this, hints);

  // Control publisher
  command_pub_.reset(
      new realtime_tools::RealtimePublisher<ros_wbmpc_msgs::Control>(
          nh, "command", 1));

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

void ROS_MPC_Interface::SensorCb(const ros_wbmpc_msgs::SensorConstPtr& msg) {
  control_msg_.initial_state = *msg;

  // Base State
  tf::pointMsgToEigen(msg->base_state.pose.pose.position, base_position_);
  tf::quaternionMsgToEigen(msg->base_state.pose.pose.orientation,
                           base_quaternion_);
  base_quaternion_.normalize();

  tf::vectorMsgToEigen(msg->base_state.twist.twist.linear, base_linear_vel_);
  tf::vectorMsgToEigen(msg->base_state.twist.twist.angular, base_angular_vel_);

  for (int i = 0; i < 3; i++) {
    jointPos_[i] = base_position_[i];
    jointVel_[i] = base_linear_vel_[i];
    jointVel_[i + 3] = base_angular_vel_[i];
  }

  jointPos_[3] = base_quaternion_.x();
  jointPos_[4] = base_quaternion_.y();
  jointPos_[5] = base_quaternion_.z();
  jointPos_[6] = base_quaternion_.w();

  // Joint States
  for (size_t i = 0; i < msg->joint_state.position.size(); i++) {
    jointPos_[(Eigen::Index)i] = msg->joint_state.position[i];
    jointVel_[(Eigen::Index)i] = msg->joint_state.velocity[i];
  }
}

void ROS_MPC_Interface::mapEigenToTF(const Eigen::VectorXd& u0,
                                     const Eigen::MatrixXd& K0) {
  tf::matrixEigenToMsg(u0, control_msg_.feedforward);
  tf::matrixEigenToMsg(K0, control_msg_.feedback_gain);
}