#include <inverse_kinematics/inverse_kinematics.hpp>

using namespace Robolaunch;
using namespace std;



InverseKinematics::InverseKinematics():
Node("inverse_kinematics_node")
{
    _cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1000, bind(&InverseKinematics::cmdVellCallback, this, placeholders::_1));
    
    _motors_velocity_publisher = this->create_publisher<robolaunch_msgs::msg::Velocity>("/motors_velocity", 1);

    _timer = this->create_wall_timer(10ms, bind(&InverseKinematics::getMotorsVelocityFromCMDVel, this));

    declare_parameter("wheel_specs.wheel_radius", 0.199);
    declare_parameter("wheel_specs.gear_ratio", 1.0);
    declare_parameter("body_specs.base_width", 0.400);
    declare_parameter("motor_frame_id", "motor");

    _wheel_radius = this->get_parameter("wheel_specs.wheel_radius").as_double();
    _gear_ratio = this->get_parameter("wheel_specs.gear_ratio").as_double();
    _base_width = this->get_parameter("body_specs.base_width").as_double();
    _motor_frame_id = this->get_parameter("motor_frame_id").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "Wheel Radius: " << _wheel_radius);
    RCLCPP_INFO_STREAM(this->get_logger(), "Gear Ratio: " << _gear_ratio);
    RCLCPP_INFO_STREAM(this->get_logger(), "Base Width: " << _base_width);
    RCLCPP_INFO_STREAM(this->get_logger(), "Frame ID: " << _motor_frame_id);
}



InverseKinematics::~InverseKinematics()
{

}



void InverseKinematics::cmdVellCallback(const geometry_msgs::msg::Twist::SharedPtr message)
{
    _linear_velocity = message->linear.x;
    _angular_velocity = message->angular.z;
}


void InverseKinematics::getMotorsVelocityFromCMDVel(void)
{
    _motors_velocity_message.header.frame_id = _motor_frame_id;
    _motors_velocity_message.header.stamp = this->get_clock()->now();

    _wl = (((_linear_velocity - ((_angular_velocity * _base_width) / 2.0)) / _wheel_radius) * 180 / PI) / 6;
    _wr = (((_linear_velocity + ((_angular_velocity * _base_width) / 2.0)) / _wheel_radius) * 180 / PI) / 6;

    _vl = 2 * PI / 60 * _wheel_radius * _wl;
    _vr = 2 * PI / 60 * _wheel_radius * _wr;

    //_vl = (2 * _linear_velocity - _angular_velocity * _base_width) / (2 * _wheel_radius);
    //_vr = (2 * _linear_velocity + _angular_velocity * _base_width) / (2 * _wheel_radius);  

    _reference_left_motor_velocity = _vl * _gear_ratio;
    _reference_right_motor_velocity = _vr * _gear_ratio; 

    _motors_velocity_message.left_motor_velocity.data = _reference_left_motor_velocity;
    _motors_velocity_message.right_motor_velocity.data = _reference_right_motor_velocity;

    _motors_velocity_publisher->publish(_motors_velocity_message);
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<InverseKinematics>());
    return 0;
}
