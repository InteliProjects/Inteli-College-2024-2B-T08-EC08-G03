#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <cmath>

class BatteryStateFilter : public rclcpp::Node {
public:
    BatteryStateFilter() 
        : Node("battery_state_filter"), last_percentage_(-1.0) {
        subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", 10,
            std::bind(&BatteryStateFilter::battery_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
            "/filtered_battery_state", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&BatteryStateFilter::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "BatteryStateFilter iniciado com callback a cada 30 segundos.");
    }

private:
    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        last_received_msg_ = msg;
    }

    void timer_callback() {
        if (last_received_msg_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Nenhuma mensagem de bateria recebida ainda.");
            return;
        }

        float current_percentage = last_received_msg_->percentage;

        if (last_percentage_ < 0 || std::abs(current_percentage - last_percentage_) >= 10) {
            RCLCPP_INFO(this->get_logger(), "Publicando novo estado da bateria: %.2f%%",
                        current_percentage);
            publisher_->publish(*last_received_msg_);

            last_percentage_ = current_percentage;
        } else {
            RCLCPP_INFO(this->get_logger(), "Sem mudanças significativas na bateria: %.2f%%.",
                        current_percentage);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::BatteryState::SharedPtr last_received_msg_;
    float last_percentage_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryStateFilter>());
    rclcpp::shutdown();
    return 0;
}