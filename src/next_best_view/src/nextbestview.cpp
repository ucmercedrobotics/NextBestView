#include <inttypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
    // Declare and load parameters from YAML
    this->declare_parameter("height", 2.0);
    this->declare_parameter("total_poses", 180);
    this->declare_parameter("sleep_time", 5);
    this->declare_parameter("z_increments", 10);
    this->declare_parameter("cylinder_diameter", 0.5);

    height_ = this->get_parameter("height").as_double();
    total_poses_ = this->get_parameter("total_poses").as_int();
    sleep_time_ = this->get_parameter("sleep_time").as_int();
    n_z_ = this->get_parameter("z_increments").as_int();
    cylinder_diameter_ = this->get_parameter("cylinder_diameter").as_double();

    // Calculate cylinder radius from diameter
    cylinder_radius_ = cylinder_diameter_ / 2.0;
    // Calculate number of theta increments (angular divisions)
    n_theta_ = total_poses_ / n_z_;

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

  // Parameters loaded from YAML
  double height_;          // Height of cylinder and poses
  int total_poses_;        // Total number of cylindrical poses
  int sleep_time_;         // Sleep time between point cloud saves
  int n_z_;                // Number of Z increments
  double cylinder_diameter_; // Diameter of the collision cylinder
  double cylinder_radius_;  // Calculated radius of the collision cylinder
  int n_theta_;            // Calculated number of angular divisions

  // Inline definition of moveToPose (unchanged)
  bool moveToPose(const geometry_msgs::msg::Pose& target_pose,
    bool constrain_joints = false,
    const std::vector<double>& fixed_joint_values = {}) {
    moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
    if (!current_state) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current state");
      return false;
    }

    if (constrain_joints && fixed_joint_values.size() >= 3) {
      const moveit::core::JointModelGroup* joint_model_group =
          move_group_interface_->getRobotModel()->getJointModelGroup("manipulator");
      std::vector<std::string> joint_names = joint_model_group->getJointModelNames();

      RCLCPP_INFO(this->get_logger(), "Constraining joints: %s, %s, %s",
                  joint_names[0].c_str(), joint_names[1].c_str(), joint_names[2].c_str());

      for (size_t i = 0; i < 3 && i < joint_names.size(); ++i) {
        const moveit::core::JointModel* joint_model =
            joint_model_group->getJointModel(joint_names[i]);
        current_state->setJointPositions(joint_model, &fixed_joint_values[i]);
        current_state->enforceBounds();
      }
    }

    std::vector<double> joint_values;
    bool found_ik = current_state->setFromIK(
        move_group_interface_->getRobotModel()->getJointModelGroup("manipulator"),
        target_pose, 1.0);
    if (!found_ik) {
      RCLCPP_ERROR(this->get_logger(), "No IK solution found for pose (%.2f, %.2f, %.2f)",
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      return false;
    }

    current_state->copyJointGroupPositions("manipulator", joint_values);
    move_group_interface_->setPlannerId("PTP");
    move_group_interface_->setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success = (move_group_interface_->plan(my_plan_arm) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to pose (%.2f, %.2f, %.2f)",
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      return false;
    }

    move_group_interface_->execute(my_plan_arm);
    move_group_interface_->setStartStateToCurrentState();
    return true;
  }

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

    // Define the cylinder collision object using loaded parameters
    moveit_msgs::msg::CollisionObject collision_cylinder;
    collision_cylinder.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_cylinder.id = "cylinder";
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = cylinder_primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[0] = height_;       // Use loaded height
    cylinder_primitive.dimensions[1] = cylinder_radius_; // Use calculated radius

    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;
    cylinder_pose.position.x = goal->object_position.x;
    cylinder_pose.position.y = goal->object_position.y;
    cylinder_pose.position.z = goal->object_position.z;

    collision_cylinder.primitives.push_back(cylinder_primitive);
    collision_cylinder.primitive_poses.push_back(cylinder_pose);
    collision_cylinder.operation = collision_cylinder.ADD;

    // Define the desk collision object (unchanged)
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
    desk_pose.position.x = 0.4;
    desk_pose.position.y = 0.0;
    desk_pose.position.z = -0.1;

    collision_desk.primitives.push_back(desk_primitive);
    collision_desk.primitive_poses.push_back(desk_pose);
    collision_desk.operation = collision_desk.ADD;

    // Combine collision objects
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_cylinder);
    collision_objects.push_back(collision_desk);

    // Apply to the planning scene
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
    message = "Generating cylinder poses...";
    goal_handle->publish_feedback(feedback);
    auto result = std::make_shared<Nbv::Result>();

    move_group_interface_->setMaxVelocityScalingFactor(0.2);
    move_group_interface_->setMaxAccelerationScalingFactor(0.2);
    move_group_interface_->setPlanningTime(2.0);
    move_group_interface_->setNumPlanningAttempts(5);
    move_group_interface_->setGoalPositionTolerance(0.001);
    move_group_interface_->setGoalOrientationTolerance(0.01);

    visual_tools_->deleteAllMarkers();

    std::vector<geometry_msgs::msg::Pose> candidate_poses =
        generateCylinderPoints(goal->object_position, goal->distance);
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    std::vector<geometry_msgs::msg::Pose> reachable_poses;
    for (const auto& pose : candidate_poses) {
      double distance_from_base = sqrt(pose.position.x * pose.position.x +
                                       pose.position.y * pose.position.y +
                                       pose.position.z * pose.position.z);
      if (distance_from_base > 0.891) {
        visual_tools_->publishZArrow(pose, rviz_visual_tools::YELLOW,
                                     rviz_visual_tools::MEDIUM);
        visual_tools_->trigger();
        continue;
      }

      moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
      if (!current_state) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state");
        continue;
      }

      std::vector<double> joint_values;
      bool found_ik = current_state->setFromIK(
          move_group_interface_->getRobotModel()->getJointModelGroup("manipulator"),
          pose, 1.0);
      if (found_ik) {
        current_state->copyJointGroupPositions("manipulator", joint_values);
        move_group_interface_->setPlannerId("PTP");
        move_group_interface_->setJointValueTarget(joint_values);
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

    int target_locations = std::min(goal->location_number, static_cast<int>(reachable_poses.size()));
    std::vector<geometry_msgs::msg::Pose> selected_poses = selectRandomPoses(reachable_poses, target_locations);

    message = "Moving to selected poses...";
    goal_handle->publish_feedback(feedback);

    for (const auto& initial_pose : selected_poses) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled during execution");
        return;
      }

      if (!moveToPose(initial_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to initial pose (%.2f, %.2f, %.2f)",
                     initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
        continue;
      }

      std_msgs::msg::Bool save_msg;
      save_msg.data = true;
      save_trigger_pub_->publish(save_msg);
      RCLCPP_INFO(this->get_logger(), "Started saving point clouds at initial pose (%.2f, %.2f, %.2f)",
                  initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
      std::this_thread::sleep_for(std::chrono::seconds(sleep_time_)); // Use loaded sleep time
      save_msg.data = false;
      save_trigger_pub_->publish(save_msg);
      RCLCPP_INFO(this->get_logger(), "Stopped saving point clouds.");

      moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
      std::vector<double> current_joint_values;
      current_state->copyJointGroupPositions("manipulator", current_joint_values);
      std::vector<double> fixed_joints = {current_joint_values[0], current_joint_values[1], current_joint_values[2]};

      geometry_msgs::msg::Pose down_pose = initial_pose;
      Eigen::Quaterniond initial_quat(
          initial_pose.orientation.w, initial_pose.orientation.x,
          initial_pose.orientation.y, initial_pose.orientation.z);
      Eigen::Quaterniond rotation_down(Eigen::AngleAxisd(-M_PI / 12, Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond down_quat = initial_quat * rotation_down;
      down_pose.orientation.x = down_quat.x();
      down_pose.orientation.y = down_quat.y();
      down_pose.orientation.z = down_quat.z();
      down_pose.orientation.w = down_quat.w();

      if (!moveToPose(down_pose, true, fixed_joints)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to down pose (%.2f, %.2f, %.2f)",
                     down_pose.position.x, down_pose.position.y, down_pose.position.z);
        continue;
      }

      save_msg.data = true;
      save_trigger_pub_->publish(save_msg);
      RCLCPP_INFO(this->get_logger(), "Started saving point clouds at down pose (%.2f, %.2f, %.2f)",
                  down_pose.position.x, down_pose.position.y, down_pose.position.z);
      std::this_thread::sleep_for(std::chrono::seconds(sleep_time_)); // Use loaded sleep time
      save_msg.data = false;
      save_trigger_pub_->publish(save_msg);
      RCLCPP_INFO(this->get_logger(), "Stopped saving point clouds.");

      geometry_msgs::msg::Pose up_pose = down_pose;
      Eigen::Quaterniond down_quat_current(
          down_pose.orientation.w, down_pose.orientation.x,
          down_pose.orientation.y, down_pose.orientation.z);
      Eigen::Quaterniond rotation_up(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitX()));
      Eigen::Quaterniond up_quat = down_quat_current * rotation_up;
      up_pose.orientation.x = up_quat.x();
      up_pose.orientation.y = up_quat.y();
      up_pose.orientation.z = up_quat.z();
      up_pose.orientation.w = up_quat.w();

      if (!moveToPose(up_pose, true, fixed_joints)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to up pose (%.2f, %.2f, %.2f)",
                     up_pose.position.x, up_pose.position.y, up_pose.position.z);
        continue;
      }

      save_msg.data = true;
      save_trigger_pub_->publish(save_msg);
      RCLCPP_INFO(this->get_logger(), "Started saving point clouds at up pose (%.2f, %.2f, %.2f)",
                  up_pose.position.x, up_pose.position.y, up_pose.position.z);
      std::this_thread::sleep_for(std::chrono::seconds(sleep_time_)); // Use loaded sleep time
      save_msg.data = false;
      save_trigger_pub_->publish(save_msg);
      RCLCPP_INFO(this->get_logger(), "Stopped saving point clouds.");
    }

    result->success = true;
    RCLCPP_INFO(this->get_logger(), "Visited %zu of %d requested locations",
                selected_poses.size(), goal->location_number);

    if (rclcpp::ok()) {
      goal_handle->succeed(result);
    }
  }

  std::vector<geometry_msgs::msg::Pose> generateCylinderPoints(
      const geometry_msgs::msg::Point& object_position, float radius) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;

    for (int k = 0; k < total_poses_; ++k) { // Use loaded total_poses
      int j = k % n_theta_;  // Angle index
      int i = k / n_theta_;  // Height index
      double theta = 2.0 * M_PI * j / n_theta_;             // Angle from 0 to 2π
      double z_offset = -height_ / 2.0 + height_ * i / (n_z_ - 1.0); // Height from -height/2 to +height/2

      geometry_msgs::msg::Pose pose;
      pose.position.x = object_position.x + radius * cos(theta);
      pose.position.y = object_position.y + radius * sin(theta);
      pose.position.z = object_position.z + z_offset;

      Eigen::Vector3d z_axis_target(object_position.x - pose.position.x,
                                    object_position.y - pose.position.y,
                                    object_position.z - pose.position.z);
      z_axis_target.normalize();

      Eigen::Vector3d y_axis_target(0.0, 0.0, 1.0);
      if (fabs(y_axis_target.dot(z_axis_target)) > 0.99) {
        y_axis_target = Eigen::Vector3d(0.0, 1.0, 0.0);
      }
      y_axis_target = y_axis_target - y_axis_target.dot(z_axis_target) * z_axis_target;
      y_axis_target.normalize();

      Eigen::Vector3d x_axis_target = y_axis_target.cross(z_axis_target);
      x_axis_target.normalize();

      Eigen::Matrix3d rotation_matrix;
      rotation_matrix.col(0) = x_axis_target;
      rotation_matrix.col(1) = y_axis_target;
      rotation_matrix.col(2) = z_axis_target;

      Eigen::Quaterniond quat(rotation_matrix);
      quat.normalize();

      pose.orientation.x = quat.x();
      pose.orientation.y = quat.y();
      pose.orientation.z = quat.z();
      pose.orientation.w = quat.w();

      nbv_poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Generated %zu cylinder poses", nbv_poses.size());
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