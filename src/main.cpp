#include <ros/ros.h>

#include <sobec/pointing/mpc_pointing.hpp>

#include "talos_manipulation/ros-mpc-interface.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "talos_manipulation");
  ros::NodeHandle nh("~");

  // Initialize MPC interface
  ROS_MPC_Interface MPC_Interface;

  // Initialize MPC
  sobec::MPC_Point MPC = sobec::MPC_Point();

  

  while (ros::ok()) {
    ros::spinOnce();
  }

  return (0);
}