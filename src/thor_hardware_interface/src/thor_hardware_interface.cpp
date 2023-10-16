#include "thor_hardware_interface/thor_hardware_interface.h"
#include <cmath>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <regex>
#include <vector>

double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

double mapRange(double val, double oldMin, double oldMax, double newMin, double newMax) {
    return (val - oldMin) / (oldMax - oldMin) * (newMax - newMin) + newMin;
}

bool extractArmPositions(const std::string& input, std::vector<double>& mposValue) {
    std::regex pos_pattern("MPos:([^,>]+),([^,>]+),([^,>]+),([^,>]+),([^,>]+),([^,>]+),([^,>]+)");
    std::smatch matches;

    if (std::regex_search(input, matches, pos_pattern) && matches.size() == 8) {
        for (size_t i = 1; i < matches.size(); ++i) {
            mposValue.push_back(std::stod(matches[i].str()));
        }
        return true;
    }
    return false;
}

double extractGripperPosition(const std::string& input) {
    std::stringstream ss(input);
    std::string temp;
    double number = 0.0;

    while (ss >> temp) {
        if (temp.find("S") != std::string::npos) {
            std::string num_str = temp.substr(1);  // remove 'S'
            number = std::stod(num_str);  // convert to double
            break;
        }
    }
    return number;
}

// Convert (MPos + M3S) format in degrees => MoveIt format in radians
void convertMposAndM3sToMoveIt(const std::vector<double>& mposValues, const double gripperM3sValue, double result[]) {
    if (mposValues.size() != 7) {
        ROS_ERROR("Issue with the size of mposValues");
        return;
    }

    // calculation for articulations 1, 2, 3 and 4
    result[0] = degreesToRadians(mposValues[0]);
    result[1] = degreesToRadians(mposValues[1]);  // mposValues[1] == mposValues[2]
    result[2] = degreesToRadians(mposValues[3]);
    result[3] = degreesToRadians(mposValues[4]);

    // calculation for articulations 5 and 6
    double motorY = mposValues[5];// -32
    double motorZ = mposValues[6];// -20

    double translationMove = (motorY - motorZ) / 2;
    double rotationMove = (motorY + motorZ) / 2;

    // Inverse of: [-6;6] => [-90;90] and convert to Rad
    result[4] = degreesToRadians(mapRange(translationMove, -6, 6, -90, 90));
    // Inverse of: [-26;26] => [-180;180] and convert to Rad
    result[5] = degreesToRadians(mapRange(rotationMove, -26, 26, -180, 180));

    // calculation for gripper
    double gripperPosition = mapRange(gripperM3sValue, 255, 0, 0, 0.06);
    result[6] = gripperPosition;
    result[7] = gripperPosition;
}

ThorHardwareInterface::ThorHardwareInterface(ros::NodeHandle &nh) : 
    io_service(), 
    work(io_service), 
    port(io_service) 
{
    ROS_INFO("Initializing ThorHardwareInterface");

    // ------------ Setup the serial connection ------------

    try {
        port.open("/dev/ttyACM0");
        ros::Duration(0.5).sleep();
        port.set_option(boost::asio::serial_port_base::baud_rate(115200));
        // Flush/reset the port
        boost::asio::write(port, boost::asio::buffer("", 0));
    } catch (const boost::system::system_error& e) {
        ROS_ERROR("Could not open serial port: %s", e.what());
        // Handle the error, e.g., by throwing an exception or exiting
    } catch (const std::exception& e) {
        ROS_ERROR("An error occurred: %s", e.what());
        // Handle the general error
    } catch (...) {
        ROS_ERROR("An unknown error occurred.");
        // Handle unknown exceptions
    }

    // Kick off async reading from the serial port
    startAsyncRead();

    // Start IO service in the background
    io_thread = std::thread([this] {
        io_service.run();
    });

    // ------------ Setup ROS ------------
    timer = nh.createTimer(ros::Duration(0.1), &ThorHardwareInterface::timerQueueCallback, this);
    
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

void ThorHardwareInterface::startAsyncRead() {
    // Flush/reset the port
    boost::asio::write(port, boost::asio::buffer("", 0));
    boost::system::error_code ec;
    boost::asio::write(port, boost::asio::buffer("?\n", 2), ec);
    if (ec) {
        ROS_ERROR_STREAM("Error sending instruction: " << ec.message());
        return;
    }

    std::size_t bufferSize = read_buffer.size();
    ROS_ERROR_STREAM("Buffer size: " << bufferSize);

    boost::asio::async_read_until(port, read_buffer, '\n', 
        boost::bind(&ThorHardwareInterface::handleRead, this, 
        boost::asio::placeholders::error, 
        boost::asio::placeholders::bytes_transferred));
}

void ThorHardwareInterface::handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred) {
    if (!ec) {
        std::istream is(&read_buffer);
        std::string line;
        std::getline(is, line);
        if (!line.empty()) {
            // Process the line of data from the serial port here
            // For now, we'll just print it
            ROS_ERROR_STREAM("Received: " << line);

            if (line[0] == '<') {
                lastArduinoStateArm = line;
            }
            
            read_buffer.consume(bytes_transferred);
        }
        // Continue async reading
        ros::Duration(0.2).sleep(); 
        // boost::this_thread::sleep(boost::posix_time::milliseconds(200));  // 200ms delay

        startAsyncRead();
    } else {
        ROS_ERROR_STREAM("Error during async read: " << ec.message());
    }
}

void ThorHardwareInterface::read() {
    std::vector<double> mposValues;
    if (extractArmPositions(lastArduinoStateArm, mposValues)) {
        double gripperPosition = extractGripperPosition(lastArduinoStateGripper);
        convertMposAndM3sToMoveIt(mposValues, gripperPosition, pos);

        // Create a JointState message and publish it to the /joint_states topic
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.name = joint_names;
        joint_state_msg.position.assign(pos, pos + joint_names.size());
        joint_state_msg.velocity.assign(vel, vel + joint_names.size());
        joint_state_msg.effort.assign(eff, eff + joint_names.size());

        joint_state_pub.publish(joint_state_msg);
    }
    
    // OLD used to fake the reading
    // for (std::size_t i = 0; i < joint_names.size() - 1; ++i) {
    //     // ROS_INFO_STREAM("Pos for " << joint_names[i].c_str() << ": " << cmd[i]);
        
    //     if (i < joint_names.size() - 2) {
    //         pos[i] = cmd[i];
    //     } else {
    //         pos[i] = cmd[i];
    //         pos[i+1] = cmd[i];
    //     }
    // }
}

void ThorHardwareInterface::write() {
     // ------------- Arm arduino command -------------
    std::string arduinoCommandArm = convertMoveItCmdToGrblArmCommand();
    if (arduinoCommandArm != lastArduinoCommandArm) {
        lastArduinoCommandArm = arduinoCommandArm;
        commandQueue.push(arduinoCommandArm);
    }

    // ------------- Gripper arduino command -------------
    std::string arduinoCommandGripper = convertMoveItCmdToGrblGripperCommand();
    if (arduinoCommandGripper != lastArduinoCommandGripper) {
        lastArduinoCommandGripper = arduinoCommandGripper;
        commandQueue.push(arduinoCommandGripper);
    }
}

void ThorHardwareInterface::sendCommandFromQueue() {
    if (!commandQueue.empty()) {
        std::string cmdToExec = commandQueue.front();
        ROS_INFO_STREAM("Sending command to Arduino: " << cmdToExec);

        if (cmdToExec.compare(0, 4, "M3 S") == 0) {
            // we save the gripper command as a "state" because no ack reply from arduino
            lastArduinoStateGripper = cmdToExec;
        }

        boost::system::error_code ec;
        boost::asio::write(port, boost::asio::buffer(cmdToExec), ec);
        if (ec) {
            ROS_ERROR_STREAM("Error sending command to arduino: " << ec.message());
        }

        commandQueue.pop();
    }
}

void ThorHardwareInterface::timerQueueCallback(const ros::TimerEvent&) {
    sendCommandFromQueue();
}

// Convert MoveIt format in radians => GRBL arm command in degrees (G0 A_ B_ C_ D_ X_ Y_ Z_)
std::string ThorHardwareInterface::convertMoveItCmdToGrblArmCommand() {
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
    return arduinoCommandArm;
}

// Convert MoveIt format => GRBL gripper command (M3 S_.__)
std::string ThorHardwareInterface::convertMoveItCmdToGrblGripperCommand() {
    double gripperMove = cmd[6];
    return "M3 S" + std::to_string(mapRange(gripperMove, 0, 0.06, 255, 0)) + "\n";
}

// Ensure you close the port in the destructor
ThorHardwareInterface::~ThorHardwareInterface() {
    io_service.stop();
    io_thread.join();

    if (port.is_open()) {
        port.close();
    }
}