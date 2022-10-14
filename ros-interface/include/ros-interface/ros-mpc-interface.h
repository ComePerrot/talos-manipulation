#ifndef ROS_MPC_INTERFACE
#define ROS_MPC_INTERFACE

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <ros_wbmpc_msgs/Control.h>
#include <ros_wbmpc_msgs/Sensor.h>

#include <Eigen/Core>
#include <map>
#include <string>

class ROS_MPC_Interface {
 public:
  ROS_MPC_Interface();
  void load(ros::NodeHandle nh);
  void update(const Eigen::VectorXd& u0, const Eigen::MatrixXd& K0);

  Eigen::VectorXd& get_jointPos() { return (jointPos_); };
  Eigen::VectorXd& get_jointVel() { return (jointVel_); };

 private:
  void SensorCb(const ros_wbmpc_msgs::SensorConstPtr& msg);
  void mapEigenToTF(const Eigen::VectorXd& u0, const Eigen::MatrixXd& K0);

  ros::Subscriber sensor_sub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<ros_wbmpc_msgs::Control>>
      command_pub_;

  ros_wbmpc_msgs::Control control_msg_;

  Eigen::VectorXd jointPos_;
  Eigen::VectorXd jointVel_;

  Eigen::Vector3d base_position_, base_linear_vel_, base_angular_vel_;
  Eigen::Quaterniond base_quaternion_;
};

#endif  // ROS_MPC_INTERFACE
