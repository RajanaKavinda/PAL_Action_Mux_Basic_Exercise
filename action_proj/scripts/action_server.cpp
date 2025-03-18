#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_proj_interfaces/action/simple_action.hpp"

using SimpleAction = action_proj_interfaces::action::SimpleAction;
using SimpleActionGoalHandle = rclcpp_action::ServerGoalHandle<SimpleAction>;
using namespace std::placeholders;

class SimpleActionServer : public rclcpp::Node {
public:
    SimpleActionServer() : Node("action_server") 
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        action_server_ = rclcpp_action::create_server<SimpleAction>(
            this,
            "simple_action",
            std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
            std::bind(&SimpleActionServer::handle_cancel, this, _1),
            std::bind(&SimpleActionServer::handle_accepted, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_);
        RCLCPP_INFO(this->get_logger(), "Simple action server has been started");
    }

private:
    rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const SimpleAction::Goal> goal) 
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received new goal: %d", goal->goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<SimpleActionGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<SimpleActionGoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute(goal_handle);
    }

    void execute(const std::shared_ptr<SimpleActionGoalHandle> goal_handle) {

        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        RCLCPP_INFO(this->get_logger(), "Processing goal...");
        auto result = std::make_shared<SimpleAction::Result>();

        for (int i = 0; i < 5; ++i) {
            if (goal_handle->is_canceling()) {
                result->result = -1;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Cancelled.");
                return;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        result->result = 1;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal completed successfully.");
    }

    rclcpp_action::Server<SimpleAction>::SharedPtr action_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<SimpleActionGoalHandle> goal_handle_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
