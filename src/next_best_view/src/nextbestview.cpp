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
#include <random>

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
    move_group_interface_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<NextBestViewServer>(this), "manipulator");

    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        this->shared_from_this(), "base_link", "moveit_visual_markers",
        move_group_interface_->getRobotModel());

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    action_server_ = rclcpp_action::create_server<Nbv>(
        this, "next_best_view/next_best_view",
        std::bind(&NextBestViewServer::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&NextBestViewServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&NextBestViewServer::handle_accepted, this,
                  std::placeholders::_1));

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
    RCLCPP_INFO(this->get_logger(),
                "Received goal - object location: x: %.2f y: %.2f z: %.2f, "
                "distance: %.2f, number of locations: %d",
                goal->object_position.x, goal->object_position.y,
                goal->object_position.z, goal->distance, goal->location_number);

    if (goal->location_number <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid number of locations: %d", goal->location_number);
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id =
        move_group_interface_->getPlanningFrame();
    collision_object.id = "sphere";
    shape_msgs::msg::SolidPrimitive sphere_primitive;
    sphere_primitive.type = sphere_primitive.SPHERE;
    sphere_primitive.dimensions.resize(1);
    sphere_primitive.dimensions[0] = 0.2;

    geometry_msgs::msg::Pose sphere_pose;
    sphere_pose.orientation.w = 1.0;
    sphere_pose.position.x = goal->object_position.x;
    sphere_pose.position.y = goal->object_position.y;
    sphere_pose.position.z = goal->object_position.z;

    collision_object.primitives.push_back(sphere_primitive);
    collision_object.primitive_poses.push_back(sphere_pose);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::msg::CollisionObject collision_desk;
    collision_desk.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_desk.id = "desk";
    shape_msgs::msg::SolidPrimitive desk_primitive;
    desk_primitive.type = desk_primitive.BOX;
    desk_primitive.dimensions.resize(3);
    desk_primitive.dimensions[0] = 1.0;
    desk_primitive.dimensions[1] = 1.0;
    desk_primitive.dimensions[2] = 0.2;

    geometry_msgs::msg::Pose desk_pose;
    desk_pose.orientation.w = 1.0;
    desk_pose.position.x = 0.5;
    desk_pose.position.y = 0.0;
    desk_pose.position.z = -0.1;

    collision_desk.primitives.push_back(desk_primitive);
    collision_desk.primitive_poses.push_back(desk_pose);
    collision_desk.operation = collision_desk.ADD;

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
    std::thread{std::bind(&NextBestViewServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing Next Best View");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Nbv::Feedback>();
    auto& message = feedback->processing_status;
    message = "Generating hemisphere poses...";
    goal_handle->publish_feedback(feedback);
    auto result = std::make_shared<Nbv::Result>();

    //Setup Moveit
    move_group_interface_->setMaxVelocityScalingFactor(0.2);
    move_group_interface_->setMaxAccelerationScalingFactor(0.2);
    move_group_interface_->setPlanningTime(2.0);
    move_group_interface_->setNumPlanningAttempts(5);
    move_group_interface_->setGoalPositionTolerance(0.001);  // 1mm
    move_group_interface_->setGoalOrientationTolerance(0.01); // ~0.57 degrees

    visual_tools_->deleteAllMarkers();

    // Generate 50 candidate poses on the upper hemisphere (Z positive)
    std::vector<geometry_msgs::msg::Pose> candidate_poses =
        generateHemispherePoints(goal->object_position, goal->distance);

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    // Identify reachable poses with distance check
    std::vector<geometry_msgs::msg::Pose> reachable_poses;
    for (const auto& pose : candidate_poses) {
        // Distance check from base (origin)
        double distance_from_base = sqrt(pose.position.x*pose.position.x +
                                        pose.position.y*pose.position.y +
                                        pose.position.z*pose.position.z);
        if (distance_from_base > 0.891) { //do not take locations out of workspace (maximum reach is 891 mm)
            visual_tools_->publishZArrow(pose, rviz_visual_tools::RED,
                                         rviz_visual_tools::MEDIUM);
            visual_tools_->trigger();
            continue;  // Not reachable, skip planning
        }

        move_group_interface_->setPlannerId("RRTConnect");
        move_group_interface_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success = (move_group_interface_->plan(my_plan_arm) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            reachable_poses.push_back(pose);
            visual_tools_->publishZArrow(pose, rviz_visual_tools::GREEN,
                                         rviz_visual_tools::MEDIUM);
        } else {
            visual_tools_->publishZArrow(pose, rviz_visual_tools::RED,
                                         rviz_visual_tools::MEDIUM);
        }
        visual_tools_->trigger();
    }

    if (reachable_poses.empty()) {
        result->success = false;
        RCLCPP_ERROR(this->get_logger(), "No reachable poses found");
        goal_handle->succeed(result);
        return;
    }

    // Select random subset of reachable poses
    int target_locations = std::min(goal->location_number, static_cast<int>(reachable_poses.size()));
    std::vector<geometry_msgs::msg::Pose> selected_poses = selectRandomPoses(reachable_poses, target_locations);

    message = "Moving to selected poses...";
    goal_handle->publish_feedback(feedback);

    // Execute movement to selected poses
    for (const auto& pose : selected_poses) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled during execution");
            return;
        }

        move_group_interface_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
        bool success = (move_group_interface_->plan(my_plan_arm) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group_interface_->execute(my_plan_arm);
            move_group_interface_->setStartStateToCurrentState();

            std_msgs::msg::Bool save_msg;
            save_msg.data = true;
            save_trigger_pub_->publish(save_msg);
            RCLCPP_INFO(this->get_logger(), "Started saving point clouds at pose (%.2f, %.2f, %.2f)",
                        pose.position.x, pose.position.y, pose.position.z);

            std::this_thread::sleep_for(std::chrono::seconds(5));

            save_msg.data = false;
            save_trigger_pub_->publish(save_msg);
            RCLCPP_INFO(this->get_logger(), "Stopped saving point clouds.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to replan to pose (%.2f, %.2f, %.2f)",
                         pose.position.x, pose.position.y, pose.position.z);
        }
    }

    result->success = true;
    RCLCPP_INFO(this->get_logger(), "Visited %zu of %d requested locations",
                selected_poses.size(), goal->location_number);

    if (rclcpp::ok()) {
        goal_handle->succeed(result);
    }
  }

  std::vector<geometry_msgs::msg::Pose> generateHemispherePoints(
      const geometry_msgs::msg::Point& object_position, float distance) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    const int total_poses = 50;
    double golden_ratio = (1.0 + sqrt(5.0)) / 2.0;

    // Generate points on the upper hemisphere (Z positive relative to object center)
    for (int i = 0; i < total_poses; ++i) {
      double theta = 2.0 * M_PI * i / golden_ratio;  // Azimuthal angle
      double phi = acos(1.0 - (2.0 * i + 1.0) / (2.0 * total_poses));  // Polar angle (0 to pi/2 for upper hemisphere)

      // Ensure Z is positive relative to object center for upper hemisphere
      if (phi > M_PI/2) continue;  // Skip lower hemisphere

      geometry_msgs::msg::Pose pose;
      pose.position.x = object_position.x + distance * sin(phi) * cos(theta);
      pose.position.y = object_position.y + distance * sin(phi) * sin(theta);
      pose.position.z = object_position.z + distance * cos(phi);  // Ensures Z >= object_position.z for upper hemisphere

      // Orientation: Z-axis toward object center, Y-axis upward, X-axis left and parallel to XY plane
      Eigen::Vector3d z_axis_target(object_position.x - pose.position.x,
                                    object_position.y - pose.position.y,
                                    object_position.z - pose.position.z);
      z_axis_target.normalize();

      Eigen::Vector3d y_axis_target(0.0, 0.0, 1.0);  // Global Z-axis (up)
      if (fabs(y_axis_target.dot(z_axis_target)) > 0.99) {
        y_axis_target = Eigen::Vector3d(0.0, 1.0, 0.0);
      }
      y_axis_target = y_axis_target - y_axis_target.dot(z_axis_target) * z_axis_target;
      y_axis_target.normalize();

      Eigen::Vector3d x_axis_target = y_axis_target.cross(z_axis_target);
      x_axis_target.normalize();

      Eigen::Matrix3d rotation_matrix;
      rotation_matrix.col(0) = x_axis_target;  // X-axis (left)
      rotation_matrix.col(1) = y_axis_target;  // Y-axis (up)
      rotation_matrix.col(2) = z_axis_target;  // Z-axis (forward)

      Eigen::Quaterniond quat(rotation_matrix);
      quat.normalize();

      pose.orientation.x = quat.x();
      pose.orientation.y = quat.y();
      pose.orientation.z = quat.z();
      pose.orientation.w = quat.w();

      nbv_poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Generated %zu hemisphere poses", nbv_poses.size());
    return nbv_poses;
  }

  std::vector<geometry_msgs::msg::Pose> selectRandomPoses(
      const std::vector<geometry_msgs::msg::Pose>& poses, int count) {
    std::vector<geometry_msgs::msg::Pose> selected_poses;
    if (poses.empty() || count <= 0) return selected_poses;

    std::vector<size_t> indices(poses.size());
    std::iota(indices.begin(), indices.end(), 0);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    int selected_count = std::min(count, static_cast<int>(poses.size()));
    selected_poses.reserve(selected_count);
    for (int i = 0; i < selected_count; ++i) {
      selected_poses.push_back(poses[indices[i]]);
    }

    RCLCPP_INFO(this->get_logger(), "Selected %zu random poses out of %zu reachable poses",
                selected_poses.size(), poses.size());
    return selected_poses;
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