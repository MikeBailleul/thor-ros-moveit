#ifndef THOR_HARDWARE_INTERFACE_H
#define THOR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

class ThorHardwareInterface : public hardware_interface::RobotHW {
private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;
    std::vector<std::string> joint_names;
    ros::Publisher joint_state_pub;

    double cmd[8];

    double pos[8];
    double vel[8];
    double eff[8];

public:
    ThorHardwareInterface();
    void read();
    void write();
};

#endif // THOR_HARDWARE_INTERFACE_H
