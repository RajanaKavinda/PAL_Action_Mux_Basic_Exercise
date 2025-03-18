#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class Int32Publisher : public rclcpp::Node {
public:
    Int32Publisher() : Node("int32_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&Int32Publisher::publish_message, this));
    }

private:
    void publish_message() {
        auto message = std_msgs::msg::Int32();
        message.data = 42;
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Int32Publisher>());
    rclcpp::shutdown();
    return 0;
}
