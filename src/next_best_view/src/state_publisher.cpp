/**
* Program to print end-effector pose
*/
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


#include <geometry_msgs/msg/pose.hpp>


int main(int argc, char* argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Declare Node
  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("get_eef_pose",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
        true));

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "manipulator");

  // print current pose
  geometry_msgs::msg::Pose current_pose =
    move_group_interface.getCurrentPose().pose;

  // Set loop rate
rclcpp::Rate loop_rate(1.0); // 1 Hz

while (rclcpp::ok()) {
  // Get and print current pose
  geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);

  // Sleep to maintain loop rate
  loop_rate.sleep();
}

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}