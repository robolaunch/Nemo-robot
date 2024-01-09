#include <forward_kinematics/forward_kinematics.hpp>

using namespace Robolaunch;


ForwardKinematics::ForwardKinematics():
Node("forward_kinematics_node")
{
    _motors_velocity_subscriber = this->create_subscription<robolaunch_msgs::msg::Velocity>("/motors_velocity_output", 1000, std::bind(&ForwardKinematics::motorsVelocityCallback, this, placeholders::_1));
    _rpm_subscriber = this->create_subscription<robolaunch_msgs::msg::RPM>("/motor_rpm_output", 1000, std::bind(&ForwardKinematics::rpmCallback, this, placeholders::_1));
    _odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

    _timer = this->create_wall_timer(1ms, std::bind(&ForwardKinematics::calculateOdometryAndPublish, this));

    declare_parameter("wheel_specs.wheel_radius", 0.1);
    declare_parameter("wheel_specs.gear_ratio", 1.0);
    declare_parameter("body_specs.base_width", 0.1);

    _wheel_radius = this->get_parameter("wheel_specs.wheel_radius").as_double();
    _gear_ratio = this->get_parameter("wheel_specs.gear_ratio").as_double();
    _base_width = this->get_parameter("body_specs.base_width").as_double();

    _odometry_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO_STREAM(this->get_logger(), "Wheel Radius: " <<  _wheel_radius);
    RCLCPP_INFO_STREAM(this->get_logger(), "Gear Ratio: " <<  _gear_ratio);
    RCLCPP_INFO_STREAM(this->get_logger(), "Base Width: " <<  _base_width);
}



ForwardKinematics::~ForwardKinematics()
{

}



void ForwardKinematics::motorsVelocityCallback(const robolaunch_msgs::msg::Velocity::SharedPtr message)
{
    _first_time = message->header.stamp;

    _left_motor_velocity = message->left_motor_velocity.data;
    _right_motor_velocity = message->right_motor_velocity.data;
    
    _motors_velocity_message_came = true;
}



void ForwardKinematics::rpmCallback(const robolaunch_msgs::msg::RPM::SharedPtr message)
{
    _first_time = message->header.stamp;

    _left_motor_rpm = message->left_motor_rpm.data;
    _right_motor_rpm = message->right_motor_rpm.data;

    _rpm_message_came = true;
}



auto ForwardKinematics::createQuaternionMessageFromYaw(double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    // return tf2::toMsg(quaternion);
    return quaternion;
}


void ForwardKinematics::calculateOdometryAndPublish(void)
{
    if(_motors_velocity_message_came)
    {
        _dt = (_first_time.seconds() - _last_time.seconds());
        _last_time = _first_time;

        // _v_right_wheel = (2 * PI * _wheel_radius) * (_right_motor_rpm / _gear_ratio) / 60.0;
        // _v_left_wheel = (2 * PI * _wheel_radius) * (_left_motor_rpm / _gear_ratio) / 60.0;

        _v_left_wheel = _left_motor_velocity;
        _v_right_wheel = _right_motor_velocity;
        

        _v = (_v_right_wheel + _v_left_wheel) / 2.0;

        _v_theta = (_v_right_wheel - _v_left_wheel) / _base_width;
        
        _position_theta = _position_theta + _v_theta * _dt;

        _v_x = _v * cos(_position_theta);
        _v_y = _v * sin(_position_theta);

        _position_x = _position_x + _v_x * _dt;
        _position_y = _position_y + _v_y * _dt;

        // Odometry Message for odom topic.
        _odometry_message.header.stamp =     this->get_clock()->now();
        _odometry_message.header.frame_id = "odom";
        _odometry_message.child_frame_id = "base_footprint";

        _odometry_message.pose.pose.position.x = _position_x;
        _odometry_message.pose.pose.position.y = _position_y;
        _odometry_message.pose.pose.position.z = 0.0;

        _odometry_quaternion = createQuaternionMessageFromYaw(_position_theta);
        _odometry_message.pose.pose.orientation.x = _odometry_quaternion.x();
        _odometry_message.pose.pose.orientation.y = _odometry_quaternion.y();
        _odometry_message.pose.pose.orientation.z = _odometry_quaternion.z();
        _odometry_message.pose.pose.orientation.w = _odometry_quaternion.w();

        _odometry_message.twist.twist.linear.x = _v;
        _odometry_message.twist.twist.angular.z = _v_theta;

        _odometry_publisher->publish(_odometry_message);
        
        // Odometry Transform Stamped for odometry broadcast
        _odometry_transform_stamped.header.stamp =     this->get_clock()->now();
        _odometry_transform_stamped.header.frame_id = "odom";
        _odometry_transform_stamped.child_frame_id = "base_footprint";

        _odometry_transform_stamped.transform.translation.x = _position_x;
        _odometry_transform_stamped.transform.translation.y = _position_y;
        _odometry_transform_stamped.transform.translation.z = 0.0;

        _odometry_transform_stamped.transform.rotation.x = _odometry_quaternion.getX();
        _odometry_transform_stamped.transform.rotation.y = _odometry_quaternion.getY();
        _odometry_transform_stamped.transform.rotation.z = _odometry_quaternion.getZ();
        _odometry_transform_stamped.transform.rotation.w = _odometry_quaternion.getW();

        _odometry_broadcaster->sendTransform(_odometry_transform_stamped);

        _motors_velocity_message_came = false;
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ForwardKinematics>());
    return 0;
}