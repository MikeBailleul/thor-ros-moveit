#ifndef THOR_HARDWARE_INTERFACE_H
#define THOR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

class ThorHardwareInterface : public hardware_interface::RobotHW
{
private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    std::unique_ptr<controller_manager::ControllerManager> controller_manager;
    double cmd[6];
    double pos[6];
    double vel[6];
    double eff[6];

public:
    ThorHardwareInterface();
    void updateControllerManager();
    void read();
    void write();
};

#endif // THOR_HARDWARE_INTERFACE_H

