#include "joystick_teleop.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joystick_teleop::TeleopJoystick>());
    rclcpp::shutdown();
    return 0;
}
