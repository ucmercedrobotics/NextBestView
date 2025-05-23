#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "kinova_action_interfaces/action/move_to.hpp"  // Change to the correct action file path

// Define the action type and goal handle type
using MoveTo = kinova_action_interfaces::action::MoveTo;
using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

// Define a class for the Action Server
class MoveToNode : public rclcpp::Node {
 public:
  MoveToNode();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_interface_;

 private:
  // Declare the action server and MoveGroupInterface as shared pointers
  rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // START MoveTo action server functionality
  // Handle incoming goal requests
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,        // Goal ID (not used here)
      std::shared_ptr<const MoveTo::Goal> goal);  // Goal message

  // Handle cancel requests
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMoveTo> goal_handle);

  // Handle accepted goals
  void handle_accepted(const std::shared_ptr<GoalHandleMoveTo> goal_handle);

  // Execute the goal
  void execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle);

  // Send feedback to the client
  void send_feedback(const std::shared_ptr<GoalHandleMoveTo> goal_handle,
                     const std::string& feedback_msg);
  // END MoveTo action server functionality
  // helper quaternion conversion
  geometry_msgs::msg::Pose compute_relative_goal_pose(
      const geometry_msgs::msg::Pose& current_pose,
      const geometry_msgs::msg::Pose& goal_pose);
};
