#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_publisher");
    auto pub = node->create_publisher<std_msgs::msg::Int32>("trigger_action", 10);
    rclcpp::Rate rate(2);  // 2 Hz
    while (rclcpp::ok()) {
        auto msg = std_msgs::msg::Int32();
        msg.data = rand() % 10;
        pub->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Published: %d", msg.data);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
