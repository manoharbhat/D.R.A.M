#include "dram_control/dram_hardware_interface.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{ 	
    ros::init(argc, argv, "dram_hardware_interface");
    ros::CallbackQueue callback_queue;
    ros::NodeHandle node_handle;
ros::NodeHandle nh;

    ros::NodeHandle private_node_handle("~");
    node_handle.setCallbackQueue(&callback_queue);
   dram_hardware_interface::DramHardwareInterface hardware_interface(node_handle, private_node_handle,nh);
    ros::MultiThreadedSpinner spinner;
    spinner.spin(&callback_queue);
	ros::spin();


    return 0;

}
