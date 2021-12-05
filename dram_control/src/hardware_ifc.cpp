

#include "dram_control/dram_hardware.h"

#include <cmath>

namespace dram_hardware_interface
{
using namespace std::string_literals;
using namespace hardware_interface;




void DramHardwareInterface::tryToOpenPort()
{
    try {
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(115200);

        auto serial_timeout = serial::Timeout::simpleTimeout(tt);
        //auto serial_timeout = serial::Timeout::max();
        serial_port_.setTimeout(serial_timeout);
        serial_port_.open();

        ROS_INFO_STREAM("Connected to motor control arduino on serial port " << port_name_);
        return;
    } catch (const std::exception& e)
    {
        ROS_WARN_STREAM_THROTTLE(60, "Could not open serial port, " << port_name_ << ": " << e.what());
    }
}
int main()
{

serial_port_.setPort("/dev/ttyACM0");
serial_port_.setBaudrate(115200);

auto serial_timeout = serial::Timeout::simpleTimeout(1000);
serial_port_.setTimeout(serial_timeout);
serial_port_.open();

 const auto out_message = "$1234\n";

        serial_port_.write(out_message);
 const auto in_message = serial_port.readline();
 std::cout << in_message << std::endl;

return 0;
}

}






