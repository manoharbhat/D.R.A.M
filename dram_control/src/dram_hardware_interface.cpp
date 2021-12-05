


#include "dram_control/dram_hardware_interface.h"

#include <cmath>

namespace dram_hardware_interface
{

  std_msgs::String js;
using namespace std::string_literals;
using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;
	auto tt = 9990;



constexpr double rotationsToRadians(double rots)
{
    return rots * 2.0 * M_PI;
}

constexpr double rpsToRadPerSec(double rps)
{
    return rps * 2.0 * M_PI;
}

constexpr double radPerSecTORPS(double rad_per_sec)
{
    return rad_per_sec * 0.5 * M_1_PI;
}

DramHardwareInterface::DramHardwareInterface(ros::NodeHandle &node_handle, ros::NodeHandle& private_node_handle, ros::NodeHandle& nh)
    : node_handle_(node_handle),
      private_node_handle_(private_node_handle),
      nh_(nh)
{
    setupJoint("joint_l_f", 0);
    setupJoint("joint_l_b", 1);
    setupJoint("joint_r_f", 2);
    setupJoint("joint_r_b", 3);

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocity_joint_soft_limits_interface_);


    controller_manager_.reset(new controller_manager::ControllerManager(this, node_handle_));
    node_handle_.param("/dram/hardware_interface/loop_hz", loop_hz_, loop_hz_);
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);// 1.0/ loop_hz_
    update_timer_ = node_handle_.createTimer(update_freq, &DramHardwareInterface::update, this);

}

void DramHardwareInterface::setupJoint(const std::string& name, int index)
{
    JointStateHandle joint_state_handle(name, &joint_positions_[index], &joint_velocities_[index], &joint_efforts_[index]);
    joint_state_interface_.registerHandle(joint_state_handle);

    JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_commands_[index]);
    JointLimits limits;
    SoftJointLimits soft_limits;
    getJointLimits(name, node_handle_, limits);
    VelocityJointSoftLimitsHandle joint_limits_handle(joint_velocity_handle, limits, soft_limits);
    velocity_joint_soft_limits_interface_.registerHandle(joint_limits_handle);
    velocity_joint_interface_.registerHandle(joint_velocity_handle);


}

void DramHardwareInterface::update(const ros::TimerEvent& e)
{
  
    auto elapsed_time = ros::Duration(e.current_real - e.last_real);


  controller_manager_->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
read();
//while (true){    read();}
  
}
//while true(){    read(elapsed_time);}

void DramHardwareInterface::read()
{ 

ros::Subscriber sub = nh_.subscribe<std_msgs::String>("/states", 1,[&](const std_msgs::String::ConstPtr& msg){ js=*msg; });
	
//ROS_INFO("I subscribed something!");

 //ROS_INFO("%s", js.data.c_str());

	std::vector<std::string> tokens;
        boost::split(tokens, js.data, boost::is_any_of(",$\n"));

        tokens.erase(std::remove(tokens.begin(), tokens.end(), ""),
                     tokens.end());

        if (tokens.size() != 8) {
            ROS_WARN_STREAM(
                "Received message did not contain the right number of tokens. Expected 8, but got "
                    << tokens.size() << "\n" << js);
            return;
        }

       joint_positions_[0] = rotationsToRadians(atof(tokens[0].c_str()));
       joint_positions_[1] = rotationsToRadians(atof(tokens[1].c_str()));
       joint_positions_[2] = rotationsToRadians(atof(tokens[2].c_str()));
       joint_positions_[3] = rotationsToRadians(atof(tokens[3].c_str()));
	joint_velocities_[0] = rpsToRadPerSec(atof(tokens[4].c_str()));
	joint_velocities_[1] = rpsToRadPerSec(atof(tokens[5].c_str()));
	joint_velocities_[2] = rpsToRadPerSec(atof(tokens[6].c_str()));
	joint_velocities_[3] = rpsToRadPerSec(atof(tokens[7].c_str()));
       // joint_positions_[1] = rotationsToRadians(tokens[2]);

       // joint_positions_[2] = rotationsToRadians(tokens[1]);
        //joint_positions_[3] = rotationsToRadians(tokens[3]);

        //joint_velocities_[0] = rpsToRadPerSec(tokens[4]);
        //joint_velocities_[1] = rpsToRadPerSec(tokens[6]);
        //joint_velocities_[2] = rpsToRadPerSec(tokens[5]);
        //joint_velocities_[3] = rpsToRadPerSec(tokens[7]);

 ros::spin();
}


void DramHardwareInterface::write(const ros::Duration& elapsed_time)
{
   	ros::Publisher pub = node_handle_.advertise<std_msgs::String>("/wheel_vel", 1000);
        velocity_joint_soft_limits_interface_.enforceLimits(elapsed_time);

        const auto left_front_rpm = radPerSecTORPS(joint_velocity_commands_[0]);
        const auto right_front_rpm = radPerSecTORPS(joint_velocity_commands_[2]);

        const auto left_back_rpm = radPerSecTORPS(joint_velocity_commands_[1]);
        const auto right_back_rpm = radPerSecTORPS(joint_velocity_commands_[3]);

int count = 0;
	while (ros::ok())
	{
		std_msgs::String velo;

    std::stringstream ss;
    //ss << std::to_string(left_front_rpm) + ", " + std::to_string(10) + ", " +std::to_string(left_back_rpm) + ", " + std::to_string(right_back_rpm)<< count;
ss << std::to_string(left_front_rpm) + ", " + std::to_string(right_front_rpm) + ", " +std::to_string(left_back_rpm) + ", " + std::to_string(right_back_rpm)<< count;
    velo.data = ss.str();
//std::to_string(right_front_rpm)
   // ROS_INFO("%s", velo.data.c_str());
		
			//wheel_vel.data[0]= left_front_rpm;
			//	wheel_vel.data[1]= right_front_rpm;
			//	wheel_vel.data[2]= left_back_rpm;
			//	wheel_vel.data[3]= right_back_rpm;
		
		//Publish array
		pub.publish(velo);
		//Let the world know
		//ROS_INFO("I published something!");
		//Do this.

}

read();       
}



}
