#include <ros/ros.h>
#include "thor_hardware_interface/thor_hardware_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thor_hardware_interface_node");
    ros::NodeHandle node_handle;
    ros::Rate rate(1 / 0.010);
    ThorHardwareInterface thor;

    while (ros::ok())
    {
        thor.read();
        thor.updateControllerManager();
        thor.write();
        rate.sleep();
    }

    return 0;
}

