#include "next_best_view/moveto.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

  geometry_msgs::msg::PoseStamped eel_pose =
      move_group_interface_->getCurrentPose();
  geometry_msgs::msg::Pose target_pose;

  RCLCPP_INFO(this->get_logger(), "Current efl pose: %f, %f, %f",
              eel_pose.pose.position.x, eel_pose.pose.position.y,
              eel_pose.pose.position.z);
  RCLCPP_INFO(this->get_logger(), "Current efl orientation: %f, %f, %f, %f",
              eel_pose.pose.orientation.x, eel_pose.pose.orientation.y,
              eel_pose.pose.orientation.z, eel_pose.pose.orientation.w);

  if (strncmp(goal->movement_link.c_str(), goal->END_EFFECTOR_LINK.c_str(),
              strlen(goal->END_EFFECTOR_LINK.c_str())) == 0) {
    // TODO: add support for orientation updating as well, right now only
    // (x,y,z)
    target_pose = compute_relative_goal_pose(eel_pose.pose, goal->pose);

  } else if (strncmp(goal->movement_link.c_str(), goal->BASE_LINK.c_str(),
                     strlen(goal->BASE_LINK.c_str())) == 0) {
    // we move with respect to the base_link world
    // no transformation required as it expects input is with respect to base
    // link
    target_pose.position = goal->pose.position;
    target_pose.orientation = goal->pose.orientation;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Bad link requested for movement: %s. Expected: %s, %s",
                 goal->movement_link.c_str(), goal->END_EFFECTOR_LINK.c_str(),
                 goal->BASE_LINK.c_str());
    send_feedback(goal_handle, "FAILED");
    return;
  }

  // move to pose wrt end effector link
  // NOTE: I tried doing this with other links as reference and it works
  //       HOWEVER, after one movement, the error builds up and fails goal
  //       tolerance the next move
  move_group_interface_->setPoseTarget(target_pose);

  // Plan the movement to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    moveit::core::MoveItErrorCode ret =
        move_group_interface_->execute(plan);  // Execute the planned movement
    if (ret == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      result->success = true;
    } else {
      result->success = false;
      send_feedback(goal_handle, "Execution failed");  // Send failure feedback
    }
    send_feedback(goal_handle, "SUCCESS");  // Send success feedback
  } else {
    result->success = false;
    send_feedback(goal_handle, "Planning failed");  // Send failure feedback
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

geometry_msgs::msg::Pose MoveToNode::compute_relative_goal_pose(
    const geometry_msgs::msg::Pose &current_pose,
    const geometry_msgs::msg::Pose &goal_pose) {
  geometry_msgs::msg::Pose target_pose;
  // Current orientation as a tf2::Quaternion
  tf2::Quaternion current_orientation;
  tf2::fromMsg(current_pose.orientation, current_orientation);

  // convert goals into tf objects
  tf2::Vector3 relative_translation(goal_pose.position.x, goal_pose.position.y,
                                    goal_pose.position.z);
  tf2::Quaternion relative_orientation(
      goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z,
      goal_pose.orientation.w);

  // Combine the current orientation with the relative orientation
  tf2::Quaternion target_orientation =
      current_orientation * relative_orientation;
  target_orientation.normalize();

  // Transform the relative translation into the world frame
  tf2::Matrix3x3 rotation_matrix(current_orientation);
  tf2::Vector3 transformed_translation = rotation_matrix * relative_translation;

  // Compute the target pose
  target_pose.position.x =
      current_pose.position.x + transformed_translation.x();
  target_pose.position.y =
      current_pose.position.y + transformed_translation.y();
  target_pose.position.z =
      current_pose.position.z + transformed_translation.z();
  target_pose.orientation = tf2::toMsg(target_orientation);

  return target_pose;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<MoveToNode>());  // Create and spin the action server
  rclcpp::shutdown();                   // Shutdown ROS 2
  return 0;
}
