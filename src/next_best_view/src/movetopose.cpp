#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <kinova_action_interfaces/action/move_to_pose.hpp>  // Adjust based on your package name

class MoveToPose : public rclcpp::Node {
public:
    using MoveToPoseAction = kinova_action_interfaces::action::MoveToPose;  // Adjust namespace if necessary
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPoseAction>;

    explicit MoveToPose(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("move_to_pose_server", options) {
    }

    void initialize() {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "manipulator");  // Adjust group name if necessary

        action_server_ = rclcpp_action::create_server<MoveToPoseAction>(
            this,
            "move_to_pose",
            std::bind(&MoveToPose::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveToPose::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveToPose::handle_accepted, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "MoveToPose action server initialized");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp_action::Server<MoveToPoseAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MoveToPoseAction::Goal> goal
    ) {
        RCLCPP_INFO(this->get_logger(), "Received goal to move to pose: x=%.2f, y=%.2f, z=%.2f",
                    goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
        (void)uuid;  // Unused parameter
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle
    ) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel move to pose");
        (void)goal_handle;  // Unused parameter
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        std::thread{std::bind(&MoveToPose::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToPoseAction::Feedback>();
        auto result = std::make_shared<MoveToPoseAction::Result>();

        // Configure MoveIt parameters
        move_group_interface_->setMaxVelocityScalingFactor(0.3);
        move_group_interface_->setMaxAccelerationScalingFactor(0.3);
        move_group_interface_->setPlanningTime(2.0);
        move_group_interface_->setNumPlanningAttempts(5);
        move_group_interface_->setGoalPositionTolerance(0.01);
        move_group_interface_->setGoalOrientationTolerance(0.01);

        // Set the target pose for the end effector
        geometry_msgs::msg::Pose target_pose = goal->target_pose;
        std::string end_effector_link = move_group_interface_->getEndEffectorLink();
        move_group_interface_->setPoseTarget(target_pose, end_effector_link);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool plan_success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!plan_success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan to target pose (x=%.2f, y=%.2f, z=%.2f)",
                         target_pose.position.x, target_pose.position.y, target_pose.position.z);
            feedback->status = "Planning failed";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->succeed(result);
            return;
        }

        feedback->status = "Starting movement to target pose";
        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Goal canceled before execution");
            result->success = false;
            goal_handle->canceled(result);
            return;
        }

        // Execute the plan (blocking call)
        auto execute_result = move_group_interface_->execute(my_plan);

        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            feedback->status = "Movement completed successfully";
            goal_handle->publish_feedback(feedback);
            result->success = true;
            RCLCPP_INFO(this->get_logger(), "Successfully moved to target pose (x=%.2f, y=%.2f, z=%.2f)",
                        target_pose.position.x, target_pose.position.y, target_pose.position.z);
        } else {
            feedback->status = "Movement failed";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to execute plan to target pose (x=%.2f, y=%.2f, z=%.2f)",
                         target_pose.position.x, target_pose.position.y, target_pose.position.z);
        }

        goal_handle->succeed(result);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPose>();
    node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}