#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <robolaunch_msgs/msg/velocity.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <future> 
#include <iostream>
#include <string.h>
#include <cstring>
#include <stdexcept>

#include <regex>

#include <libusb-1.0/libusb.h>
#include <cassert>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <features.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <serial/serial.h>

using namespace std;

namespace Robolaunch
{
    class MotorController: public rclcpp::Node
    {
        private:

        public:
            MotorController();
            ~MotorController();

            rclcpp::Subscription<robolaunch_msgs::msg::Velocity>::SharedPtr _motors_velocity_subscriber;

            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _motor_acceleration_subscriber;
    
            rclcpp::Publisher<robolaunch_msgs::msg::Velocity>::SharedPtr _motors_velocity_output_publisher;
    
            rclcpp::TimerBase::SharedPtr _motor_controller_read_timer, _motor_controller_write_timer;

            void motorsVelocityCallback(const robolaunch_msgs::msg::Velocity::SharedPtr message);

            int settingBaudrate(int baudrate);

            void prepareTermios(termios* tty, int serial_port, int baudrate);

            void clearMotorControllerBuffer(int serial_port);

            void readMotorsAndMotorControllerValues(void);

            void writeMotorsValues(void);

            void makeSenseOfMotorControllerValues(string serial_data);

            robolaunch_msgs::msg::Velocity _motors_velocity_message;

            robolaunch_msgs::msg::Velocity _motors_velocity_output_message;

            struct termios _motor_controller_tty;

            serial::Serial _serial_connection;

            int _motor_controller_serial_port;

            string _motor_controller_serial_port_name;

            int _motor_controller_baudrate = B19200;

            int _motor_controller_bytes = 0;

            char _motor_controller_buffer[256];

            int _left_motor_direction, _right_motor_direction;

            string _motor_controller_serial_port_group;

            string _motor_controller_id_vendor, _motor_controller_id_vendor_id, _motor_controller_id_model_id, _motor_controller_id_path;

            std::string::const_iterator _motor_controller_regex_iterator_start, _motor_controller_regex_iterator_end;

            std::regex _motors_speed_pattern;
            std::smatch _motors_speed_matches;

            string _motor_controller_readed_data = "";

            string _motor_controller_serial_data = "";

            
    };
}

#endif