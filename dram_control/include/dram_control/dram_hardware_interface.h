#ifndef DRAM_HARDWARE_INTERFACE_H
#define DRAM_HARDWARE_INTERFACE_H


#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>

namespace dram_hardware_interface
{     
class DramHardwareInterface : public hardware_interface::RobotHW
{   


public:
//ros::Subscriber sub;

  DramHardwareInterface(ros::NodeHandle &node_handle, ros::NodeHandle& private_node_handle, ros::NodeHandle& nh);
  ~DramHardwareInterface() = default;

  //std_msgs::String message;
 //dram_hardware_interface::DramHardwareInterface::hardware_interface obj;
//void statesCallback(const std_msgs::String::ConstPtr& msg)
//{
 //ROS_INFO("%s", message.data.c_str());
//ROS_INFO("will read something!");
   //js= message;
//ROS_INFO("I heard: [%s]", msg->data.c_str());

//}

 hardware_interface::RobotHW obja;
 //void statesCallback(const std_msgs::String::ConstPtr& msg);
//sub = node_handle_.subscribe("/states", 1000, &DramHardwareInterface::statesCallback, this);

private:

  void setupJoint(const std::string& name, int index);
  void update(const ros::TimerEvent& e);
  void read();
  void write(const ros::Duration& elapsed_time);
  
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_soft_limits_interface_;

  //DramHardwareInterface obj;

  std::array<double, 4> joint_positions_ = {};
  std::array<double, 4> joint_velocities_ = {};
  std::array<double, 4> joint_efforts_ = {};
  std::array<double, 4> joint_velocity_commands_ = {};

ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Timer update_timer_;
ros::NodeHandle nh_;
  double loop_hz_ = 20.0;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

};

}

#endif
