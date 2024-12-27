#ifndef JOYSTICK_TELEOP_TELEOP_JOYSTICK_HPP_
#define JOYSTICK_TELEOP_TELEOP_JOYSTICK_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick_teleop {

class TeleopJoystick : public rclcpp::Node {
public:
    TeleopJoystick();

private:
    void joyCallback(sensor_msgs::msg::Joy::SharedPtr joy_msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  joy_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    int   linear_axis_;
    int   angular_axis_;
    float linear_scale_;
    float angular_scale_;
    float linear_deadzone_;
    float angular_deadzone_;
};

}  // namespace joystick_teleop

#endif  // JOYSTICK_TELEOP_TELEOP_JOYSTICK_HPP_
