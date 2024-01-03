#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("r\r");
}

void ArduinoComms::readEncoderValues(int &int_val_1, int &int_val_2, int &int_val_3, int &int_val_4)
{
    std::string response = sendMsg("e\r");
    std::istringstream iss(response);
    std::string val_1, val_2, val_3, val_4;

    iss >> val_1 >> val_2 >> val_3 >> val_4;

    int_val_1 = -1*std::atoi(val_1.c_str());
    int_val_2 = -1*std::atoi(val_2.c_str());
    int_val_3 = -1*std::atoi(val_3.c_str());
    int_val_4 = -1*std::atoi(val_4.c_str());


    //std::string delimiter = " ";
    //size_t del_pos = response.find(delimiter);
    //std::string token_1 = response.substr(0, del_pos);
    //std::string token_2 = response.substr(del_pos + delimiter.length());

    //val_1 = -1*std::atoi(token_1.c_str());
    //val_2 = -1*std::atoi(token_2.c_str());




}

void ArduinoComms::setMotorValues(float val_1, float val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}