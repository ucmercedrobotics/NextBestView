#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "kinova_action_interfaces/action/move_to.hpp"  // Change to the correct action file path

// Define a class for the Action Server
class MoveToActionServer : public rclcpp::Node
{
public:
  // Define the action type and goal handle type
  using MoveTo = kinova_action_interfaces::action::MoveTo;
  using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

  MoveToActionServer() : Node("move_to_action_server") // Initialize the node
  {
    // Create the action server
    action_server_ = rclcpp_action::create_server<MoveTo>(
      this,                                   // Node pointer
      "move_to_action",                       // Action name
      std::bind(&MoveToActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),  // Goal handler
      std::bind(&MoveToActionServer::handle_cancel, this, std::placeholders::_1),  // Cancel handler
      std::bind(&MoveToActionServer::handle_accepted, this, std::placeholders::_1) // Accepted handler
    );

    // Create a MoveGroupInterface for controlling the robot
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "manipulator");
  }

private:
  // Declare the action server and MoveGroupInterface as shared pointers
  rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  // Handle incoming goal requests
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, // Goal ID (not used here)
    std::shared_ptr<const MoveTo::Goal> goal) // Goal message
  {
    // Check if the command is valid
    if (goal->command == "MoveTo_position" || goal->command == "MoveTo_objectposition") {
      RCLCPP_INFO(this->get_logger(), "Accepted goal with command: %s", goal->command.c_str());
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept the goal
    } else {
      RCLCPP_WARN(this->get_logger(), "Rejected goal with invalid command: %s", goal->command.c_str());
      return rclcpp_action::GoalResponse::REJECT; // Reject the goal
    }
  }

  // Handle cancel requests
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveTo>)
  {
    RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT; // Accept the cancel request
  }

  // Handle accepted goals
  void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
  {
    // Start a new thread to execute the goal
    std::thread{std::bind(&MoveToActionServer::execute, this, goal_handle)}.detach();
  }

  // Execute the goal
  void execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
  {
    auto goal = goal_handle->get_goal(); // Get the goal message
    move_group_interface_->setPoseTarget(goal->pose); // Set the target pose

    // Plan the movement to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    auto result = std::make_shared<MoveTo::Result>(); // Create a result message

    if (success) {
      move_group_interface_->execute(plan); // Execute the planned movement
      result->result = "Pose is reachable";
      send_feedback(goal_handle, "SUCCESS"); // Send success feedback
    } else {
      result->result = "Pose is not reachable";
      send_feedback(goal_handle, "FAILED"); // Send failure feedback
    }

    goal_handle->succeed(result); // Mark the goal as succeeded
  }

  // Send feedback to the client
  void send_feedback(const std::shared_ptr<GoalHandleMoveTo> goal_handle, const std::string &feedback_msg)
  {
    auto feedback = std::make_shared<MoveTo::Feedback>(); // Create a feedback message
    feedback->feedback = feedback_msg;
    goal_handle->publish_feedback(feedback); // Publish the feedback
    RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback_msg.c_str()); // Log the feedback
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // Initialize ROS 2
  rclcpp::spin(std::make_shared<MoveToActionServer>()); // Create and spin the action server
  rclcpp::shutdown(); // Shutdown ROS 2
  return 0;
}