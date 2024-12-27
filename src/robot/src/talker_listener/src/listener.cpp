#include "listener.hpp"

Listener::Listener() : Node("listener") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1)
    );
}

void Listener::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Recebido: '%s'", msg->data.c_str());
}
