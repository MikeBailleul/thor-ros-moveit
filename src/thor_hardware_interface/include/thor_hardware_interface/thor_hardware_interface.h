#ifndef THOR_HARDWARE_INTERFACE_H
#define THOR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <queue>

class ThorHardwareInterface : public hardware_interface::RobotHW {
private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;
    std::vector<std::string> joint_names;
    ros::Publisher joint_state_pub;

    // used for serial communication /!\ DO NOT change order of declaration
    ros::Timer timer;
    boost::asio::io_service io_service;
    boost::asio::serial_port port;
    boost::asio::io_service::work work;
    std::thread io_thread;
    boost::asio::streambuf read_buffer; 
    std::queue<std::string> commandQueue;

    // arduino command to send i.e. write() & sendCommandFromQueue()
    std::string lastArduinoCommandArm;
    std::string lastArduinoCommandGripper;
    // arduino state received i.e. read()
    std::string lastArduinoStateArm;
    std::string lastArduinoStateGripper;

    double cmd[8];
    double pos[8];
    double vel[8];
    double eff[8];
    

public:
    ThorHardwareInterface(ros::NodeHandle &nh);
    ~ThorHardwareInterface();

    // read functions
    void read();
    void startAsyncRead();
    void handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred);

    // write functions
    void write();
    void sendCommandFromQueue();
    void timerQueueCallback(const ros::TimerEvent&);
    std::string convertMoveItCmdToGrblArmCommand();
    std::string convertMoveItCmdToGrblGripperCommand();
};

#endif // THOR_HARDWARE_INTERFACE_H
