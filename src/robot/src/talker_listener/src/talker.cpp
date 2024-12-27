#include "talker.hpp"

Talker::Talker() : Node("talker") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    constexpr std::chrono::milliseconds publish_interval(500);
    timer_ = this->create_wall_timer(publish_interval, [this] { timer_callback(); });
}

void Talker::timer_callback() {
    std_msgs::msg::String message;
    message.data = "Hello, world! " + std::to_string(counter_++);
    RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", message.data.c_str());
    publisher_->publish(message);
}
