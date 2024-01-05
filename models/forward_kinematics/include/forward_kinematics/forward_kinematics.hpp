#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robolaunch_msgs/msg/velocity.hpp>
#include <robolaunch_msgs/msg/rpm.hpp>
#include <rclcpp/time.hpp>


#define PI 3.141592653589793238462

using namespace std;


namespace Robolaunch
{
    class ForwardKinematics: public rclcpp::Node
    {
        private:

        public:
            ForwardKinematics();
            ~ForwardKinematics();

            rclcpp::Subscription<robolaunch_msgs::msg::Velocity>::SharedPtr _motors_velocity_subscriber;

            rclcpp::Subscription<robolaunch_msgs::msg::RPM>::SharedPtr _rpm_subscriber;

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odometry_publisher;

            rclcpp::TimerBase::SharedPtr _timer;

            std::unique_ptr<tf2_ros::TransformBroadcaster> _odometry_broadcaster;

            void motorsVelocityCallback(const robolaunch_msgs::msg::Velocity::SharedPtr message);

            void rpmCallback(const robolaunch_msgs::msg::RPM::SharedPtr message);

            auto createQuaternionMessageFromYaw(double yaw);

            void calculateOdometryAndPublish(void);

            nav_msgs::msg::Odometry _odometry_message;

            tf2::Quaternion _odometry_quaternion;
            
            geometry_msgs::msg::TransformStamped _odometry_transform_stamped;

            robolaunch_msgs::msg::Velocity _motors_velocity_message;
            
            robolaunch_msgs::msg::RPM _rpm_message;
            
            double _dt;

            double _position_x, _position_y, _position_theta;

            double _v_right_wheel, _v_left_wheel;

            double _w_left_wheel, _w_right_wheel;

            double _v, _v_theta;

            double _v_x, _v_y;

            double _base_width, _wheel_radius, _gear_ratio;

            double _left_motor_velocity, _right_motor_velocity;

            double _left_motor_rpm, _right_motor_rpm;

            bool _motors_velocity_message_came = false;

            bool _rpm_message_came = false;

            rclcpp::Time _first_time, _last_time;

            double _rate;            
    };
}


#endif