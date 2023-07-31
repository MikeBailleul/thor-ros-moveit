#include "thor_hardware_interface/thor_hardware_interface.h"

ThorHardwareInterface::ThorHardwareInterface()
{
    ROS_INFO("Initializing ThorHardwareInterface");
    for (int i = 0; i < 6; i++)
    {
        cmd[i] = 0;
        pos[i] = 0;
        vel[i] = 0;
        eff[i] = 0;
        char joint_name[10];
        sprintf(joint_name, "joint%d", i + 1);
        ROS_INFO("Registering handle for %s", joint_name);
        hardware_interface::JointStateHandle state_handle(joint_name, &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_name), &cmd[i]);
        jnt_pos_interface.registerHandle(pos_handle);
    }

    ROS_INFO("Registering interfaces");
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    ROS_INFO("Creating ControllerManager");
    controller_manager.reset(new controller_manager::ControllerManager(this));
}

void ThorHardwareInterface::read() 
{
  
  // You need to implement reading from your actual hardware here
  static int counter = 0;
    if (counter % 100 == 0) {
    ROS_INFO("Reading from hardware");
        for (int i = 0; i < 6; i++) {
            ROS_INFO_STREAM("Command for joint " << i << ": " << cmd[i]);
        }
    }
    counter++;
}

void ThorHardwareInterface::write() 
{
  
  // And writing to your actual hardware here
  static int counter = 0;
    if (counter % 100 == 0) {
    ROS_INFO("Writing to hardware");
        for (int i = 0; i < 6; i++) {
            ROS_INFO_STREAM("Command for joint " << i << ": " << cmd[i]);
        }
    }
    counter++;

}

void ThorHardwareInterface::updateControllerManager()
{
    //ROS_INFO("Updating ControllerManager");
    controller_manager->update(ros::Time::now(), ros::Duration(0.01));
}

