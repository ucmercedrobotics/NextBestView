#include "next_best_view/moveto.hpp"

MoveToNode::MoveToNode()
    : Node("move_to_action_server")  // Initialize the node
{
  // Create the action server
  action_server_ = rclcpp_action::create_server<MoveTo>(
      this,              // Node pointer
      "move_to_action",  // Action name
      std::bind(&MoveToNode::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),  // Goal handler
      std::bind(&MoveToNode::handle_cancel, this,
                std::placeholders::_1),  // Cancel handler
      std::bind(&MoveToNode::handle_accepted, this,
                std::placeholders::_1)  // Accepted handler
  );

  // Create a MoveGroupInterface for controlling the robot
  move_group_interface_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          std::shared_ptr<MoveToNode>(this), "manipulator");
}

// Handle incoming goal requests
rclcpp_action::GoalResponse MoveToNode::handle_goal(
    const rclcpp_action::GoalUUID &uuid,       // Goal ID (not used here)
    std::shared_ptr<const MoveTo::Goal> goal)  // Goal message
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request: %s",
              goal->task.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // Accept the goal
}

// Handle cancel requests
rclcpp_action::CancelResponse MoveToNode::handle_cancel(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;  // Accept the cancel request
}

// Handle accepted goals
void MoveToNode::handle_accepted(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
  // Start a new thread to execute the goal
  std::thread{std::bind(&MoveToNode::execute, this, goal_handle)}.detach();
}

// Execute the goal
void MoveToNode::execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle) {
  auto goal = goal_handle->get_goal();               // Get the goal message
  move_group_interface_->setPoseTarget(goal->pose);  // Set the target pose

  // Plan the movement to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  auto result = std::make_shared<MoveTo::Result>();  // Create a result message

  if (success) {
    move_group_interface_->execute(plan);  // Execute the planned movement
    result->result = "Pose is reachable";
    send_feedback(goal_handle, "SUCCESS");  // Send success feedback
  } else {
    result->result = "Pose is not reachable";
    send_feedback(goal_handle, "FAILED");  // Send failure feedback
  }

  goal_handle->succeed(result);  // Mark the goal as succeeded
}

// Send feedback to the client
void MoveToNode::send_feedback(
    const std::shared_ptr<GoalHandleMoveTo> goal_handle,
    const std::string &feedback_msg) {
  (void)goal_handle;
  auto feedback =
      std::make_shared<MoveTo::Feedback>();  // Create a feedback message
  feedback->processing_status = feedback_msg;
  goal_handle->publish_feedback(feedback);  // Publish the feedback
  RCLCPP_INFO(this->get_logger(), "Feedback: %s",
              feedback_msg.c_str());  // Log the feedback
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<MoveToNode>());  // Create and spin the action server
  rclcpp::shutdown();                   // Shutdown ROS 2
  return 0;
}
