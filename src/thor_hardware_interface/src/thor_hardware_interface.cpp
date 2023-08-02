#include "thor_hardware_interface/thor_hardware_interface.h"

ThorHardwareInterface::ThorHardwareInterface()
{
    joint_names = {"joint_base_art1_yaw", "joint_art1_art2_pitch", "joint_art2_art3_pitch", 
                    "joint_art3_art4_roll", "joint_art4_art5_pitch", "joint_art5_art6_roll"};


    ROS_INFO("Initializing ThorHardwareInterface");
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        cmd[i] = 0;
        pos[i] = 0;
        vel[i] = 0;
        eff[i] = 0;
        
        ROS_INFO("Registering handle for %s", joint_names[i].c_str());
        
        hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
        joint_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle(joint_names[i]), &cmd[i]);
        joint_pos_interface.registerHandle(pos_handle);
    }

    ROS_INFO("Registering interfaces");
    registerInterface(&joint_state_interface);
    registerInterface(&joint_pos_interface);
    
    ROS_INFO("Creating ControllerManager");
    controller_manager.reset(new controller_manager::ControllerManager(this));
}

void ThorHardwareInterface::read() 
{
  
  // You need to implement reading from your actual hardware here
  static int counter = 0;
    if (counter % 100 == 0) {
    ROS_INFO("Reading from hardware");
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            ROS_INFO_STREAM("Pos for " << joint_names[i].c_str() << ": " << pos[i]);
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
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            ROS_INFO_STREAM("Command for " << joint_names[i].c_str() << ": " << cmd[i]);
        }
    }
    counter++;

}

void ThorHardwareInterface::updateControllerManager()
{
    //ROS_INFO("Updating ControllerManager");
    controller_manager->update(ros::Time::now(), ros::Duration(0.01));
}

