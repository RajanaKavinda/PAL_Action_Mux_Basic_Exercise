#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/int32.hpp"
#include "action_proj_interfaces/action/simple_action.hpp"

using SimpleAction = action_proj_interfaces::action::SimpleAction;
using SimpleActionGoalHandle = rclcpp_action::ClientGoalHandle<SimpleAction>;
using namespace std::placeholders;

class SimpleActionClient : public rclcpp::Node {
public:
    SimpleActionClient() : Node("action_client") 
    {
        action_client_ = rclcpp_action::create_client<SimpleAction>(this, "simple_action");

        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/goal_topic", 10, std::bind(&SimpleActionClient::goal_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Simple action client has been started. Send a goal to /goal_topic.");
    }

private:
    void goal_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Action server not available.");
            return;
        }

        // Check if there is an active goal
        if (active_goal_handle_) {
            auto future_cancel = action_client_->async_cancel_goal(active_goal_handle_);
            RCLCPP_INFO(this->get_logger(), "Previous goal preempted.");
        }

        // Create a goal
        auto goal_msg = SimpleAction::Goal();
        goal_msg.goal = msg->data;

        // Add a callback to get the result of the goal
        auto send_goal_options = rclcpp_action::Client<SimpleAction>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&SimpleActionClient::goal_result_callback, this, _1);
        send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goal_response_callback, this, _1);

        // Send the goal
        action_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Sent new goal: %d", msg->data);
    }

    void goal_result_callback(const SimpleActionGoalHandle::WrappedResult &result) {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal completed successfully.");
        } 
        else if(status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_INFO(this->get_logger(), "Goal was aborted.");
        } 
        else if(status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_INFO(this->get_logger(), "Goal was canceled.");
        }
        int final_result = result.result->result;
        RCLCPP_INFO(this->get_logger(), "Result: %d", final_result);

        active_goal_handle_.reset();
    }

    void goal_response_callback(const SimpleActionGoalHandle::SharedPtr &goal_handle) {
        if (goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server.");
            active_goal_handle_ = goal_handle;
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "Goal rejected by server.");
        }
    }

    rclcpp_action::Client<SimpleAction>::SharedPtr action_client_;
    rclcpp_action::ClientGoalHandle<SimpleAction>::SharedPtr active_goal_handle_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleActionClient>());
    rclcpp::shutdown();
    return 0;
}
