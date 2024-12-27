#include "joystick_teleop.hpp"
#include <cstdlib>

namespace joystick_teleop {

TeleopJoystick::TeleopJoystick() :
    Node("teleop_joystick"),
    linear_axis_(1),
    angular_axis_(0),
    linear_scale_(0.21),
    angular_scale_(2.7),
    linear_deadzone_(0.2),
    angular_deadzone_(0.2) {
    this->declare_parameter("axis_linear", linear_axis_);
    this->declare_parameter("axis_angular", angular_axis_);
    this->declare_parameter("scale_linear", linear_scale_);
    this->declare_parameter("scale_angular", angular_scale_);

    this->declare_parameter("linear_deadzone", linear_deadzone_);
    this->declare_parameter("angular_deadzone", angular_deadzone_);

    this->get_parameter("axis_linear", linear_axis_);
    this->get_parameter("axis_angular", angular_axis_);
    this->get_parameter("scale_linear", linear_scale_);
    this->get_parameter("scale_angular", angular_scale_);

    this->get_parameter("linear_deadzone", linear_deadzone_);
    this->get_parameter("angular_deadzone", angular_deadzone_);

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&TeleopJoystick::joyCallback, this, std::placeholders::_1)
    );

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "TeleopJoystick node initialized");
    RCLCPP_INFO(
        this->get_logger(),
        "TeleopJoystick parameters: axis_linear=%d, axis_angular=%d, scale_linear=%.2f, scale_angular=%.2f, "
        "deadzone_linear=%.2f, deadzone_angular=%.2f",
        linear_axis_, angular_axis_, linear_scale_, angular_scale_, linear_deadzone_, angular_deadzone_
    );
}

void TeleopJoystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    auto twist_msg = geometry_msgs::msg::Twist();
    auto linear_speed = joy_msg->axes[linear_axis_];
    auto angular_speed = joy_msg->axes[angular_axis_];

    linear_speed = abs(linear_speed) < linear_deadzone_ ? 0 : linear_speed;
    angular_speed = abs(angular_speed) < angular_deadzone_ ? 0 : angular_speed;

    twist_msg.linear.x = linear_scale_ * linear_speed;
    twist_msg.angular.z = angular_scale_ * angular_speed;

    cmd_vel_publisher_->publish(twist_msg);

    RCLCPP_DEBUG(
        this->get_logger(), "Publishing Twist: linear=%.2f, angular=%.2f", twist_msg.linear.x, twist_msg.angular.z
    );
}

}  // namespace joystick_teleop
