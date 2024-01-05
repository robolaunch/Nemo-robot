#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>

#include <string.h>
#include <geometry_msgs/msg/twist.hpp>
#include <robolaunch_msgs/msg/velocity.hpp>
#include <robolaunch_msgs/msg/rpm.hpp>

#define PI 3.141592653589793238462

using namespace std;


namespace Robolaunch
{
    class InverseKinematics: public rclcpp::Node
    {
        private:

        public:
            InverseKinematics();
            ~InverseKinematics();

            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_subscriber;

            rclcpp::Publisher<robolaunch_msgs::msg::Velocity>::SharedPtr _motors_velocity_publisher;

            rclcpp::TimerBase::SharedPtr _timer;
            
            void cmdVellCallback(const geometry_msgs::msg::Twist::SharedPtr message);

            void getMotorsVelocityFromCMDVel(void);

            robolaunch_msgs::msg::Velocity _motors_velocity_message;

            float _linear_velocity, _angular_velocity;

            float _wheel_radius, _base_width, _gear_ratio;

            string _motor_frame_id;

            float _vl, _vr;

            float _wl, _wr;

            float _reference_left_motor_velocity, _reference_right_motor_velocity;
    };
}

#endif