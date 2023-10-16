#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include "thor_hardware_interface/thor_hardware_interface.h"

int main(int argc, char **argv) {
    try {
    ros::init(argc, argv, "thor_hardware_interface_node");
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    ThorHardwareInterface thor(nh);
    controller_manager::ControllerManager cm(&thor, nh);

    ros::AsyncSpinner spinner(1, &queue);
    spinner.start();

    ros::Time ts = ros::Time::now();
    ros::Rate rate(20);
    
    while (ros::ok()) {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();

        thor.read();
        cm.update(ts, d);
        thor.write();

        rate.sleep();
    }

    spinner.stop();

   }
    catch (const std::exception& e) {
        ROS_ERROR("Caught exception: %s", e.what());
    }
    catch (...) {
        ROS_ERROR("Caught unknown exception");
    }
    return 0;
}
