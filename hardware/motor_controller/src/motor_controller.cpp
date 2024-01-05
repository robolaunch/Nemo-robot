#include <motor_controller/motor_controller.hpp>

using namespace Robolaunch;
using namespace std;


MotorController::MotorController():
Node("motor_controller_node")
{
    _motors_velocity_subscriber = this->create_subscription<robolaunch_msgs::msg::Velocity>("/motors_velocity", 1000, bind(&MotorController::motorsVelocityCallback, this, placeholders::_1));

    _motors_velocity_output_publisher = this->create_publisher<robolaunch_msgs::msg::Velocity>("/motors_velocity_output", 1);

    _motor_controller_read_timer = this->create_wall_timer(1ms, bind(&MotorController::readMotorsAndMotorControllerValues, this));

    _motor_controller_write_timer = this->create_wall_timer(10ms, bind(&MotorController::writeMotorsValues, this));
    
    declare_parameter("motor_controller.port_name", "/dev/ttyUSB0");
    declare_parameter("motor_controller.baudrate", 115200);
    declare_parameter("motor_controller.left_motor_direction", 1);
    declare_parameter("motor_controller.right_motor_direction", 1);
    declare_parameter("motor_controller.serial_port_group", "/dev/tty");
    declare_parameter("motor_controller.id_vendor", "-");
    declare_parameter("motor_controller.id_vendor_id", "-");
    declare_parameter("motor_controller.id_model_id", "-");
    declare_parameter("motor_controller.id_path", "-");

    _motor_controller_serial_port_name = this->get_parameter("motor_controller.port_name").as_string();
    _motor_controller_baudrate = this->get_parameter("motor_controller.baudrate").as_int();

    _left_motor_direction = this->get_parameter("motor_controller.left_motor_direction").as_int();
    _right_motor_direction = this->get_parameter("motor_controller.right_motor_direction").as_int();

    _motor_controller_serial_port_group = this->get_parameter("motor_controller.serial_port_group").as_string();
    _motor_controller_id_vendor = this->get_parameter("motor_controller.id_vendor").as_string();
    _motor_controller_id_vendor_id = this->get_parameter("motor_controller.id_vendor_id").as_string();
    _motor_controller_id_model_id = this->get_parameter("motor_controller.id_model_id").as_string();
    _motor_controller_id_path = this->get_parameter("motor_controller.id_path").as_string();

    _motor_controller_baudrate = this->settingBaudrate(_motor_controller_baudrate);

    _motor_controller_serial_port = open(_motor_controller_serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    rclcpp::sleep_for(chrono::milliseconds(2000));

    _motors_speed_pattern = std::regex("ML:([-+]?[0-9]+\.[0-9]+),MR:([-+]?[0-9]+\.[0-9]+)");




    // Prepare serial port
    prepareTermios(&_motor_controller_tty, _motor_controller_serial_port, _motor_controller_baudrate);
    //reset encoder
    write(_motor_controller_serial_port, ("e \r"), strlen("e /r"));

}



MotorController::~MotorController()
{
    close(_motor_controller_serial_port);
}



void MotorController::motorsVelocityCallback(const robolaunch_msgs::msg::Velocity::SharedPtr message)
{
    _motors_velocity_message = *message;
}



int MotorController::settingBaudrate(int baudrate)
{
    switch (baudrate)
    {
        case 9600:
            std::cout << "Baudrate of Serial Port 9600: " << std::endl;
            baudrate = B9600;
            break;

        case 19200:
            std::cout << "Baudrate of Serial Port 19200: " << std::endl;
            baudrate = B19200;
            break;

        case 38400:
            std::cout << "Baudrate of Serial Port 38400: " << std::endl;
            baudrate = B38400; 
            break;

        case 57600:
            std::cout << "Baudrate of Serial Port 57600: " << std::endl;
            baudrate = B57600;
            break;

        case 115200:
            std::cout << "Baudrate of Serial Port 115200: " << std::endl;
            baudrate = B115200;
            break;
        case 230400:
            std::cout << "Baudrate of Serial Port 230400: " << std::endl;
            baudrate = B230400;
            break;
        default:
            std::cout << "Baudrate of Serial Port 9600: " << std::endl;
            baudrate = B9600;
            break;
    }
    return baudrate;
}



void MotorController::prepareTermios(termios* tty, int serial_port, int baudrate)
{
    if(tcgetattr(serial_port, tty) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Can not open motor controller serial port!");
        rclcpp::shutdown();
    }

    cfsetospeed(tty, (speed_t)baudrate);
    cfsetispeed(tty, (speed_t)baudrate);

    tty->c_cflag     &=  ~PARENB;            // Make 8n1
    tty->c_cflag     &=  ~CSTOPB;
    tty->c_cflag     &=  ~CSIZE;
    tty->c_cflag     |=  CS8;

    tty->c_cflag     &=  ~CRTSCTS;           // no flow control
    tty->c_cc[VMIN]   =  1;                  // read doesn't block
    tty->c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty->c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    cfmakeraw(tty);
    tcflush(serial_port, TCIFLUSH);

    if(tcsetattr(serial_port, TCSANOW, tty) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Can not configure motor controller serial port!");
        rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Serial ports configuration is completed.");
}



void MotorController::readMotorsAndMotorControllerValues(void)
{
    try
    {
       _motor_controller_bytes = read(_motor_controller_serial_port, _motor_controller_buffer, sizeof(_motor_controller_buffer));
       int motor_controller_byte_length = string(_motor_controller_buffer, _motor_controller_bytes).length();
       if(_motor_controller_bytes != -1 && motor_controller_byte_length != 0)
       {
            _motor_controller_readed_data = _motor_controller_readed_data + string(_motor_controller_buffer, _motor_controller_bytes);
            if(_motor_controller_readed_data.size() > 100)
            {
                //_motor_controller_serial_data = regex_replace(_motor_controller_readed_data, regex(" "), ""); It will be tested
                // RCLCPP_INFO_STREAM(this->get_logger(), "Serial Data: " << _motor_controller_readed_data);
                makeSenseOfMotorControllerValues(_motor_controller_readed_data);
                _motor_controller_readed_data = "";
            }
       }
       else
       {
        RCLCPP_INFO_STREAM(this->get_logger(), "Motor Controller serial port is closed. Please check your usb connection.");
       }
    }
    catch(const std::exception& e)
    {
        //std::cerr << e.what() << '\n';
    }

}


void MotorController::makeSenseOfMotorControllerValues(string serial_data)
{
    _motor_controller_regex_iterator_start = serial_data.cbegin();
    _motor_controller_regex_iterator_end = serial_data.cend();

    while(std::regex_search(_motor_controller_regex_iterator_start, _motor_controller_regex_iterator_end, _motors_speed_matches, _motors_speed_pattern))
    {
        _motor_controller_regex_iterator_start = _motors_speed_matches[0].second;

        _motors_velocity_output_message.header.stamp = this->get_clock()->now();
        _motors_velocity_output_message.header.frame_id = "motor_controller";

        _motors_velocity_output_message.left_motor_velocity.data = stof(_motors_speed_matches[1].str());
        _motors_velocity_output_message.right_motor_velocity.data = stof(_motors_speed_matches[2].str());
    }

    _motors_velocity_output_publisher->publish(_motors_velocity_output_message);
}



void MotorController::writeMotorsValues(void)
{
    write(_motor_controller_serial_port, ("m " + to_string(_left_motor_direction * _motors_velocity_message.left_motor_velocity.data) + " " + to_string(_right_motor_direction * _motors_velocity_message.right_motor_velocity.data) + "\r").c_str(), strlen(("m " + to_string(_left_motor_direction * _motors_velocity_message.left_motor_velocity.data) + " " + to_string(_right_motor_direction * _motors_velocity_message.right_motor_velocity.data) + "\r").c_str()));
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MotorController>());
    return 0;
}