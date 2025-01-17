#include "next_best_view/moveto.hpp"

MoveToNode::MoveToNode()
    : Node("move_to_action_server")  // Initialize the node
{
  // Create the action server
  action_server_ = rclcpp_action::create_server<MoveTo>(
      this,                              // Node pointer
      "/next_best_view/move_to_action",  // Action name
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
              goal->task_name.c_str());
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
  auto result = std::make_shared<MoveTo::Result>();  // Create a result message

  geometry_msgs::msg::PoseStamped efl_pose =
      move_group_interface_->getCurrentPose();

  RCLCPP_INFO(this->get_logger(), "Current efl pose: %f, %f, %f",
              efl_pose.pose.position.x, efl_pose.pose.position.y,
              efl_pose.pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Current efl orientation: %f, %f, %f, %f",
              efl_pose.pose.orientation.x, efl_pose.pose.orientation.y,
              efl_pose.pose.orientation.z, efl_pose.pose.orientation.w);

  if (strncmp(goal->movement_link.c_str(), goal->END_EFFECTOR_LINK.c_str(),
              strlen(goal->END_EFFECTOR_LINK.c_str())) == 0) {
    // TODO: add support for orientation updating as well, right now only
    // (x,y,z)
    // TODO: add support for dynamic tf values
    // efl_pose.pose.position.x += goal->pose.position.x;
    // efl_pose.pose.position.y += goal->pose.position.y;
    // efl_pose.pose.position.z += goal->pose.position.z;
    ;
  } else if (strncmp(goal->movement_link.c_str(), goal->BASE_LINK.c_str(),
                     strlen(goal->BASE_LINK.c_str())) == 0) {
    // we actually move with respect to the end effector, but it's without
    // current position computation
    efl_pose.pose.position = goal->pose.position;
    efl_pose.pose.orientation = goal->pose.orientation;
  } else if (strncmp(goal->movement_link.c_str(),
                     goal->CAMERA_DEPTH_FRAME.c_str(),
                     strlen(goal->CAMERA_DEPTH_FRAME.c_str())) == 0) {
    // TODO: we don't do anything with this right now BUT we need to figure out
    // how to determine
    geometry_msgs::msg::PoseStamped camera_pose =
        move_group_interface_->getCurrentPose(goal->CAMERA_DEPTH_FRAME);
    // we don't do anything here since it's already absolute movement by default
    RCLCPP_INFO(this->get_logger(), "Current camera pose: %f, %f, %f",
                camera_pose.pose.position.x, camera_pose.pose.position.y,
                camera_pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(),
                "Current camera orientation: %f, %f, %f, %f",
                camera_pose.pose.orientation.x, camera_pose.pose.orientation.y,
                camera_pose.pose.orientation.z, camera_pose.pose.orientation.w);
    // TODO: add support for orientation updating as well, right now only
    // (x,y,z)
    // TODO: add support for dynamic tf values
    // efl_pose.pose.position.x -= (X_TF_STATIC_CAMERA_LINK +
    // goal->pose.position.x); efl_pose.pose.position.y -=
    // (Y_TF_STATIC_CAMERA_LINK + goal->pose.position.y);
    // efl_pose.pose.position.z += (goal->pose.position.z -
    // OBJECT_KEEP_DISTANCE);
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Bad link requested for movement: %s. Expected: %s, %s, %s",
                 goal->movement_link.c_str(), goal->END_EFFECTOR_LINK.c_str(),
                 goal->BASE_LINK.c_str(), goal->CAMERA_DEPTH_FRAME.c_str());
    send_feedback(goal_handle, "FAILED");
    return;
  }

  move_group_interface_->setPoseTarget(efl_pose);  // Set the target pose

  // Plan the movement to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group_interface_->execute(plan);   // Execute the planned movement
    send_feedback(goal_handle, "SUCCESS");  // Send success feedback
    goal_handle->succeed(result);           // Mark the goal as succeeded
  } else {
    send_feedback(goal_handle, "FAILED");  // Send failure feedback
  }
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
