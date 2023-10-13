#include "thor_hardware_interface/thor_hardware_interface.h"
#include <libserial/SerialPortConstants.h>
#include <SerialStream.h>
#include <SerialPort.h>
#include <cmath>

double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

double mapRange(double val, double oldMin, double oldMax, double newMin, double newMax) {
    return (val - oldMin) / (oldMax - oldMin) * (newMax - newMin) + newMin;
}

ThorHardwareInterface::ThorHardwareInterface() {
    ROS_INFO("Initializing ThorHardwareInterface");


    // ------------ Setup the serial connection ------------
    port = new LibSerial::SerialStream();
    port->Open("/dev/ttyACM0", std::ios::in | std::ios::out);
    port->SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    if (!port->IsOpen() || !port->good()) {
        ROS_ERROR("Could not open serial port.");
        // Handle the error, e.g., by throwing an exception or exiting
    }


    // ------------ Setup ROS ------------
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
    // ROS_INFO("Reading hardware sensors and computing positions");
    
    for (std::size_t i = 0; i < joint_names.size() - 1; ++i) {
        // ROS_INFO_STREAM("Pos for " << joint_names[i].c_str() << ": " << cmd[i]);
        
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
    // ROS_INFO("Receiving commands to be sent towards arduino/robot");

     // ------------- Arm arduino command -------------
    std::string arduinoCommandArm = "G0 ";
    for (std::size_t i = 0; i < joint_names.size() - 2; ++i) {
        double degrees = radiansToDegrees(cmd[i]);
        std::string degreesStr;
        double convertedYDegreeTranslation;
        double convertedZDegreeTranslation;
        double convertedYZDegreeRotation;

        switch (i)
        {
        case 0:
            degreesStr = std::to_string(radiansToDegrees(cmd[i]));

            arduinoCommandArm += "A" + degreesStr;
            break;

        case 1:
            degreesStr = std::to_string(radiansToDegrees(cmd[i]));

            arduinoCommandArm += " B" + degreesStr + " C" + degreesStr;
            break;

        case 2:
            degreesStr = std::to_string(radiansToDegrees(cmd[i]));

            arduinoCommandArm += " D" + degreesStr;
            break;

        case 3:
            degreesStr = std::to_string(radiansToDegrees(cmd[i]));

            arduinoCommandArm += " X" + degreesStr;
            break;

        case 4:
            // translation move: convert [-90;90] => [-6;6]
            convertedYDegreeTranslation = mapRange(degrees, -90, 90, -6, 6);
            convertedZDegreeTranslation = -mapRange(degrees, -90, 90, -6, 6);
            break;

        case 5:
            // rotation move: convert [-180;180] => [-26;26]
            convertedYZDegreeRotation = mapRange(degrees, -180, 180, -26, 26);
            
            arduinoCommandArm += " Y" + std::to_string(convertedYDegreeTranslation + convertedYZDegreeRotation)
                            + " Z" + std::to_string(convertedZDegreeTranslation + convertedYZDegreeRotation);
            break;
        
        default:
            break;
        }
    }

    arduinoCommandArm += "\n";

    if (arduinoCommandArm != lastArduinoCommandArm) {
        ROS_INFO_STREAM("Arduino command for ARM: " << arduinoCommandArm);
        lastArduinoCommandArm = arduinoCommandArm;
        *port << arduinoCommandArm;

        ros::Duration(0.2).sleep();
    }

    // ------------- Gripper arduino command -------------
    double gripperMove = cmd[6];
    std::string arduinoCommandGripper = "M3 S" + std::to_string(mapRange(gripperMove, 0, 0.06, 255, 0)) + "\n";

    if (arduinoCommandGripper != lastArduinoCommandGripper) {
        ROS_INFO_STREAM("Arduino command for GRIPPER: " << arduinoCommandGripper);
        lastArduinoCommandGripper = arduinoCommandGripper;
        *port << arduinoCommandGripper;

        ros::Duration(0.2).sleep();
    }
    
}

// Ensure you close the port in the destructor
ThorHardwareInterface::~ThorHardwareInterface() {
    if (port && port->IsOpen()) {
        port->Close();
    }
    delete port;
}