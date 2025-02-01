#include <inttypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "kinova_action_interfaces/action/next_best_view.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NextBestViewServer : public rclcpp::Node {
 public:
  using Nbv = kinova_action_interfaces::action::NextBestView;
  using GoalHandleNbv = rclcpp_action::ServerGoalHandle<Nbv>;

  explicit NextBestViewServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("nbv_server", options) {
    // Create a MoveGroupInterface for controlling the robot
    move_group_interface_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<NextBestViewServer>(this), "manipulator");

    // Initialize MoveIt Visual Tools
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        this->shared_from_this(), "base_link", "moveit_visual_markers",
        move_group_interface_->getRobotModel());

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    // Create action server
    action_server_ = rclcpp_action::create_server<Nbv>(
        this, "next_best_view",
        std::bind(&NextBestViewServer::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&NextBestViewServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&NextBestViewServer::handle_accepted, this,
                  std::placeholders::_1));

    // Create a publisher for the /save_trigger topic
    save_trigger_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("/save_trigger", 10);
  }

 private:
  rclcpp_action::Server<Nbv>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr save_trigger_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Nbv::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received number of location:  %d",
                goal->location_number);
    RCLCPP_INFO(this->get_logger(),
                "Received object location: x: %.2f  y: %.2f z: %.2f number of "
                "locations: %d",
                goal->object_position.x, goal->object_position.y,
                goal->object_position.z, goal->location_number);

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    // Create collision objects for the robot to avoid
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id =
        move_group_interface_->getPlanningFrame();
    collision_object.id = "sphere";
    shape_msgs::msg::SolidPrimitive sphere_primitive;
    sphere_primitive.type = sphere_primitive.SPHERE;
    sphere_primitive.dimensions.resize(1);
    sphere_primitive.dimensions[0] = 0.2;  // Sphere radius

    // Define the pose of the sphere
    geometry_msgs::msg::Pose sphere_pose;
    sphere_pose.orientation.w = 1.0;
    sphere_pose.position.x = goal->object_position.x;
    sphere_pose.position.y = goal->object_position.y;
    sphere_pose.position.z = goal->object_position.z;

    collision_object.primitives.push_back(sphere_primitive);
    collision_object.primitive_poses.push_back(sphere_pose);
    collision_object.operation = collision_object.ADD;

    // Create a desk collision object
    moveit_msgs::msg::CollisionObject collision_desk;
    collision_desk.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_desk.id = "desk";
    shape_msgs::msg::SolidPrimitive desk_primitive;
    desk_primitive.type = desk_primitive.BOX;
    desk_primitive.dimensions.resize(3);
    desk_primitive.dimensions[0] = 1.0;  // Length (X)
    desk_primitive.dimensions[1] = 1.0;  // Width (Y)
    desk_primitive.dimensions[2] = 0.2;  // Height (Z)

    // Define the pose of the desk
    geometry_msgs::msg::Pose desk_pose;
    desk_pose.orientation.w = 1.0;
    desk_pose.position.x = 0.5;
    desk_pose.position.y = 0.0;
    desk_pose.position.z = -0.1;

    collision_desk.primitives.push_back(desk_primitive);
    collision_desk.primitive_poses.push_back(desk_pose);
    collision_desk.operation = collision_desk.ADD;

    // Add both collision objects to the scene at once
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    collision_objects.push_back(collision_desk);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNbv> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    using namespace std::placeholders;
    // Spin up a new thread to execute the goal
    std::thread{std::bind(&NextBestViewServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing Next Best View");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Nbv::Feedback>();
    auto& message = feedback->processing_status;
    message = "Creating view locations...";
    auto result = std::make_shared<Nbv::Result>();
    std::vector<geometry_msgs::msg::Pose> poses_nbv;

    // Clear previous markers
    visual_tools_->deleteAllMarkers();

    // Ensure location_number is greater than 0 to avoid division by zero
    if (goal->location_number <= 0) {
      result->success = false;
      goal_handle->succeed(result);
      RCLCPP_ERROR(this->get_logger(), "Invalid number of locations");
      return;
    }

    // Create a local copy of object_position
    geometry_msgs::msg::Point object_position = goal->object_position;

    // Generate the points using the local copy
    poses_nbv = generateCirclePoints(goal->location_number, object_position,
                                     goal->distance);

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Next Best View succeeded");
    }
  }

  std::vector<geometry_msgs::msg::Pose> generateCirclePoints(
      int32_t location_number,
      geometry_msgs::msg::Point&
          object_position,  // Pass by non-const reference
      float distance) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int max_z_adjustments = 3;  // Maximum number of z-axis adjustments
    float z_increment =
        0.10;  // Amount to increment the object's z-position each time
    int attempt = 0;

    while (attempt < max_z_adjustments) {
      nbv_poses.clear();  // Clear the previous poses

      if (location_number <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid number of locations");
        return nbv_poses;
      }

      // Calculate the angular step for evenly distributing points
      double angle_step = 2.0 * M_PI / static_cast<double>(location_number);

      // Generate the points
      for (int32_t i = 0; i < location_number; ++i) {
        double angle = i * angle_step;

        // Position calculation
        geometry_msgs::msg::Pose pose;
        pose.position.x = object_position.x + distance * cos(angle);
        pose.position.y = object_position.y + distance * sin(angle);
        pose.position.z = object_position.z;

        // Calculate direction vector (z-axis of end effector)
        Eigen::Vector3d z_axis_target(object_position.x - pose.position.x,
                                      object_position.y - pose.position.y,
                                      object_position.z - pose.position.z);
        z_axis_target.normalize();

        // Define the z-axis of the base frame (assuming base frame z-axis is
        // [0, 0, 1])
        Eigen::Vector3d z_axis_base(0.0, 0.0, 1.0);

        // Align the y-axis of the end effector with the z-axis of the base
        Eigen::Vector3d y_axis_target = z_axis_base;

        // Ensure the y-axis is orthogonal to the z-axis
        if (y_axis_target.dot(z_axis_target) != 0) {
          y_axis_target =
              y_axis_target - y_axis_target.dot(z_axis_target) * z_axis_target;
          y_axis_target.normalize();
        }

        // Compute the x-axis using the cross product of y and z
        Eigen::Vector3d x_axis_target = y_axis_target.cross(z_axis_target);
        x_axis_target.normalize();

        // Construct the rotation matrix
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix.col(0) = x_axis_target;
        rotation_matrix.col(1) = y_axis_target;
        rotation_matrix.col(2) = z_axis_target;

        // Convert the rotation matrix to a quaternion
        Eigen::Quaterniond quat(rotation_matrix);
        quat.normalize();

        // Populate the pose orientation
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        // Add the pose to the list (regardless of whether it is reachable)
        nbv_poses.push_back(pose);
      }

      // Try to visit all generated poses
      std::vector<geometry_msgs::msg::Pose> successfully_visited_poses;
      for (const auto& pose : nbv_poses) {
        // Set the planner to an OMPL planner (e.g., RRTConnect)
        move_group_interface_->setPlannerId(
            "RRTConnect");  // Use RRTConnect planner

        // Plan and execute the motion to the target pose
        move_group_interface_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success_arm = (move_group_interface_->plan(my_plan_arm) ==
                            moveit::core::MoveItErrorCode::SUCCESS);

        if (success_arm) {
          visual_tools_->publishZArrow(pose, rviz_visual_tools::BLUE,
                                       rviz_visual_tools::MEDIUM);
          visual_tools_->trigger();
          move_group_interface_->execute(my_plan_arm);
          move_group_interface_->setStartStateToCurrentState();

          // Publish "true" to /save_trigger to start saving
          std_msgs::msg::Bool save_msg;
          save_msg.data = true;
          save_trigger_pub_->publish(save_msg);
          RCLCPP_INFO(this->get_logger(), "Started saving point clouds.");

          // Wait for 5 seconds
          std::this_thread::sleep_for(std::chrono::seconds(15));

          // Publish "false" to /save_trigger to stop saving
          save_msg.data = false;
          save_trigger_pub_->publish(save_msg);
          RCLCPP_INFO(this->get_logger(), "Stopped saving point clouds.");

          // Add the pose to the successfully visited list
          successfully_visited_poses.push_back(pose);
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to plan to pose at (%.2f, %.2f, %.2f)",
                       pose.position.x, pose.position.y, pose.position.z);
          visual_tools_->publishZArrow(pose, rviz_visual_tools::RED,
                                       rviz_visual_tools::MEDIUM);
          visual_tools_->trigger();
        }
      }

      // Check if all positions were successfully reached
      if (successfully_visited_poses.size() == location_number) {
        RCLCPP_INFO(this->get_logger(),
                    "Successfully visited all %d locations.", location_number);
        return successfully_visited_poses;  // Return the successfully visited
                                            // poses
      } else {
        // Adjust the object position along the z-axis and retry
        object_position.z +=
            z_increment;  // Increase z by the specified increment
        attempt++;
        RCLCPP_WARN(this->get_logger(),
                    "Only %zu/%d poses reached. Adjusting object position to "
                    "z=%.2f and retrying...",
                    successfully_visited_poses.size(), location_number,
                    object_position.z);
      }
    }

    // If maximum attempts are reached, return the poses that were successfully
    // visited
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to reach all positions after %d attempts",
                 max_z_adjustments);
    return nbv_poses;  // Return the last generated poses (even if not all were
                       // visited)
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<NextBestViewServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
