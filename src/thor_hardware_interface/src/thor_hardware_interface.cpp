#include "thor_hardware_interface/thor_hardware_interface.h"

ThorHardwareInterface::ThorHardwareInterface() {
    ROS_INFO("Initializing ThorHardwareInterface");

    ros::NodeHandle nh;
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    joint_names = {"joint_base_art1_yaw",
                   "joint_art1_art2_pitch",
                   "joint_art2_art3_pitch",
                   "joint_art3_art4_roll",
                   "joint_art4_art5_pitch",
                   "joint_art5_art6_roll",
                   "joint_hand_finger_left",
                   "joint_hand_finger_right"};

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
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
}

void ThorHardwareInterface::read() {
    ROS_INFO("Reading hardware sensors and computing positions");
    
    for (std::size_t i = 0; i < joint_names.size() - 1; ++i) {
        ROS_INFO_STREAM("Pos for " << joint_names[i].c_str() << ": " << cmd[i]);
        
        if (i < joint_names.size() - 2) {
            pos[i] = cmd[i];
        } else {
            pos[i] = cmd[i];
            pos[i+1] = cmd[i];
        }
    }

    // Create a JointState message and publish it to the /joint_states topic
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name = joint_names;
    joint_state_msg.position.assign(pos, pos + joint_names.size());
    joint_state_msg.velocity.assign(vel, vel + joint_names.size());
    joint_state_msg.effort.assign(eff, eff + joint_names.size());

    joint_state_pub.publish(joint_state_msg);
}

void ThorHardwareInterface::write() {
    ROS_INFO("Receiving commands to be sent towards arduino/robot");

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO_STREAM("Command for " << joint_names[i].c_str() << ": " << cmd[i]);
    }
}
