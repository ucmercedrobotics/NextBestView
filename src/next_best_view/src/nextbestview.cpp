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
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <random>
#include <limits>

#include "kinova_action_interfaces/action/next_best_view.hpp"
#include "kinova_action_interfaces/action/save_point_cloud.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NextBestViewServer : public rclcpp::Node {
 public:
  using Nbv = kinova_action_interfaces::action::NextBestView;
  using GoalHandleNbv = rclcpp_action::ServerGoalHandle<Nbv>;

  /** Constructor: Declare parameters only */
  explicit NextBestViewServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("nbv_server", options) {
    this->declare_parameter("cylinder_height", 2.0);
    this->declare_parameter("cylinder_total_poses", 180);
    this->declare_parameter("cylinder_z_increments", 10);
    this->declare_parameter("cylinder_diameter", 0.5);

    this->declare_parameter("hemisphere_radius", 1.0);
    this->declare_parameter("hemisphere_total_poses", 100);

    this->declare_parameter("sphere_radius", 1.0);
    this->declare_parameter("sphere_total_poses", 200);

    this->declare_parameter("rectangle_length", 1.0);
    this->declare_parameter("rectangle_width", 1.0);
    this->declare_parameter("rectangle_height", 1.0);
    this->declare_parameter("rectangle_total_poses", 100);

    this->declare_parameter("plane_width", 1.0);
    this->declare_parameter("plane_height", 1.0);
    this->declare_parameter("plane_total_poses", 100);
  }

  /** Initialize MoveIt and ROS components after construction */
  void initialize() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "manipulator");

    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), "base_link", "moveit_visual_markers",
        move_group_interface_->getRobotModel());

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    action_server_ = rclcpp_action::create_server<Nbv>(
        this, "next_best_view/next_best_view",
        std::bind(&NextBestViewServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NextBestViewServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&NextBestViewServer::handle_accepted, this, std::placeholders::_1));

    save_point_cloud_client_ = rclcpp_action::create_client<kinova_action_interfaces::action::SavePointCloud>(
        this, "save_point_cloud");
  }

 private:
  // Member variables
  rclcpp_action::Server<Nbv>::SharedPtr action_server_;
  rclcpp_action::Client<kinova_action_interfaces::action::SavePointCloud>::SharedPtr save_point_cloud_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  // Shape-specific parameters
  double cylinder_height_;
  int cylinder_total_poses_;
  int cylinder_z_increments_;
  double cylinder_diameter_;
  double cylinder_radius_;
  int cylinder_n_theta_;

  double hemisphere_radius_;
  int hemisphere_total_poses_;

  double sphere_radius_;
  int sphere_total_poses_;

  double rectangle_length_;
  double rectangle_width_;
  double rectangle_height_;
  int rectangle_total_poses_;

  double plane_width_;
  double plane_height_;
  int plane_total_poses_;

  /** Compute Euclidean distance between two poses */
  double distanceBetweenPoses(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2) {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    double dz = p1.position.z - p2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  /** Solve TSP using a greedy algorithm starting from start_pose */
  std::vector<geometry_msgs::msg::Pose> solveTSP(const std::vector<geometry_msgs::msg::Pose>& poses,
                                                 const geometry_msgs::msg::Pose& start_pose) {
    std::vector<geometry_msgs::msg::Pose> tour;
    std::vector<bool> visited(poses.size(), false);
    geometry_msgs::msg::Pose current = start_pose;

    while (tour.size() < poses.size()) {
      double min_dist = std::numeric_limits<double>::max();
      int next_index = -1;
      for (size_t i = 0; i < poses.size(); ++i) {
        if (!visited[i]) {
          double dist = distanceBetweenPoses(current, poses[i]);
          if (dist < min_dist) {
            min_dist = dist;
            next_index = i;
          }
        }
      }
      if (next_index != -1) {
        current = poses[next_index];
        tour.push_back(current);
        visited[next_index] = true;
      } else {
        break;
      }
    }
    return tour;
  }

  /** Load shape-specific parameters based on the goal */
  void loadShapeParameters(const std::string& shape) {
    if (shape == "cylinder") {
      cylinder_height_ = this->get_parameter("cylinder_height").as_double();
      cylinder_total_poses_ = this->get_parameter("cylinder_total_poses").as_int();
      cylinder_z_increments_ = this->get_parameter("cylinder_z_increments").as_int();
      cylinder_diameter_ = this->get_parameter("cylinder_diameter").as_double();
      cylinder_radius_ = cylinder_diameter_ / 2.0;
      cylinder_n_theta_ = cylinder_total_poses_ / cylinder_z_increments_;
    } else if (shape == "hemisphere") {
      hemisphere_radius_ = this->get_parameter("hemisphere_radius").as_double();
      hemisphere_total_poses_ = this->get_parameter("hemisphere_total_poses").as_int();
    } else if (shape == "sphere") {
      sphere_radius_ = this->get_parameter("sphere_radius").as_double();
      sphere_total_poses_ = this->get_parameter("sphere_total_poses").as_int();
    } else if (shape == "rectangle") {
      rectangle_length_ = this->get_parameter("rectangle_length").as_double();
      rectangle_width_ = this->get_parameter("rectangle_width").as_double();
      rectangle_height_ = this->get_parameter("rectangle_height").as_double();
      rectangle_total_poses_ = this->get_parameter("rectangle_total_poses").as_int();
    } else if (shape == "plane") {
      plane_width_ = this->get_parameter("plane_width").as_double();
      plane_height_ = this->get_parameter("plane_height").as_double();
      plane_total_poses_ = this->get_parameter("plane_total_poses").as_int();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown shape: %s", shape.c_str());
    }
  }

  /** Generate poses based on shape type */
  std::vector<geometry_msgs::msg::Pose> generatePoses(const std::string& shape,
                                                      const geometry_msgs::msg::Point& object_position,
                                                      float distance) {
    if (shape == "cylinder") {
      return generateCylinderPoints(object_position, distance);
    } else if (shape == "hemisphere") {
      return generateHemispherePoints(object_position, distance);
    } else if (shape == "sphere") {
      return generateSpherePoints(object_position, distance);
    } else if (shape == "rectangle") {
      return generateRectanglePoints(object_position, distance);
    } else if (shape == "plane") {
      return generatePlanePoints(object_position, distance);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown shape: %s", shape.c_str());
      return {};
    }
  }

  /** Generate cylinder poses */
  std::vector<geometry_msgs::msg::Pose> generateCylinderPoints(const geometry_msgs::msg::Point& object_position,
                                                               float radius) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    for (int k = 0; k < cylinder_total_poses_; ++k) {
      int j = k % cylinder_n_theta_;
      int i = k / cylinder_n_theta_;
      double theta = 2.0 * M_PI * j / cylinder_n_theta_;
      double z_offset = -cylinder_height_ / 2.0 + cylinder_height_ * i / (cylinder_z_increments_ - 1.0);

      geometry_msgs::msg::Pose pose;
      pose.position.x = object_position.x + radius * cos(theta);
      pose.position.y = object_position.y + radius * sin(theta);
      pose.position.z = object_position.z + z_offset;

      setOrientation(pose, object_position);
      nbv_poses.push_back(pose);
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu cylinder poses", nbv_poses.size());
    return nbv_poses;
  }

  /** Generate hemisphere poses */
  std::vector<geometry_msgs::msg::Pose> generateHemispherePoints(const geometry_msgs::msg::Point& object_position,
                                                                 float radius) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_phi = static_cast<int>(sqrt(hemisphere_total_poses_ / 2));
    int n_theta = 2 * n_phi;

    for (int i = 0; i < n_phi; ++i) {
      double phi = M_PI / 2.0 * i / (n_phi - 1.0);
      for (int j = 0; j < n_theta; ++j) {
        double theta = 2.0 * M_PI * j / n_theta;
        geometry_msgs::msg::Pose pose;
        pose.position.x = object_position.x + radius * sin(phi) * cos(theta);
        pose.position.y = object_position.y + radius * sin(phi) * sin(theta);
        pose.position.z = object_position.z + radius * cos(phi);

        setOrientation(pose, object_position);
        nbv_poses.push_back(pose);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu hemisphere poses", nbv_poses.size());
    return nbv_poses;
  }

  /** Generate sphere poses */
  std::vector<geometry_msgs::msg::Pose> generateSpherePoints(const geometry_msgs::msg::Point& object_position,
                                                             float radius) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_phi = static_cast<int>(sqrt(sphere_total_poses_ / 2));
    int n_theta = 2 * n_phi;

    for (int i = 0; i < n_phi; ++i) {
      double phi = M_PI * i / (n_phi - 1.0);
      for (int j = 0; j < n_theta; ++j) {
        double theta = 2.0 * M_PI * j / n_theta;
        geometry_msgs::msg::Pose pose;
        pose.position.x = object_position.x + radius * sin(phi) * cos(theta);
        pose.position.y = object_position.y + radius * sin(phi) * sin(theta);
        pose.position.z = object_position.z + radius * cos(phi);

        setOrientation(pose, object_position);
        nbv_poses.push_back(pose);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu sphere poses", nbv_poses.size());
    return nbv_poses;
  }

  /** Generate rectangle poses */
  std::vector<geometry_msgs::msg::Pose> generateRectanglePoints(const geometry_msgs::msg::Point& object_position,
                                                                float distance) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_x = static_cast<int>(cbrt(rectangle_total_poses_));
    int n_y = n_x;
    int n_z = rectangle_total_poses_ / (n_x * n_y);

    for (int i = 0; i < n_x; ++i) {
      double x = -rectangle_length_ / 2.0 + rectangle_length_ * i / (n_x - 1.0);
      for (int j = 0; j < n_y; ++j) {
        double y = -rectangle_width_ / 2.0 + rectangle_width_ * j / (n_y - 1.0);
        for (int k = 0; k < n_z; ++k) {
          double z = -rectangle_height_ / 2.0 + rectangle_height_ * k / (n_z - 1.0);
          geometry_msgs::msg::Pose pose;
          pose.position.x = object_position.x + x + distance * (x > 0 ? 1 : -1);
          pose.position.y = object_position.y + y + distance * (y > 0 ? 1 : -1);
          pose.position.z = object_position.z + z;

          setOrientation(pose, object_position);
          nbv_poses.push_back(pose);
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu rectangle poses", nbv_poses.size());
    return nbv_poses;
  }

  /** Generate plane poses */
  std::vector<geometry_msgs::msg::Pose> generatePlanePoints(const geometry_msgs::msg::Point& object_position,
                                                            float distance) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_width = static_cast<int>(std::sqrt(plane_total_poses_));
    int n_height = plane_total_poses_ / n_width;

    double norm = std::sqrt(object_position.x * object_position.x +
                            object_position.y * object_position.y +
                            object_position.z * object_position.z);
    if (norm < 1e-6) {
      RCLCPP_ERROR(this->get_logger(), "Object position is too close to base link");
      return {};
    }

    Eigen::Vector3d unit_vec = Eigen::Vector3d(object_position.x / norm,
                                               object_position.y / norm,
                                               object_position.z / norm);
    Eigen::Vector3d plane_center = Eigen::Vector3d(object_position.x, object_position.y, object_position.z) -
                                   distance * unit_vec;

    double xy_norm = std::sqrt(object_position.x * object_position.x +
                               object_position.y * object_position.y);
    Eigen::Vector3d normal_vector;
    if (xy_norm < 1e-6) {
      normal_vector = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
      normal_vector = Eigen::Vector3d(object_position.x / xy_norm, object_position.y / xy_norm, 0.0);
    }

    Eigen::Vector3d height_axis(0.0, 0.0, 1.0);
    Eigen::Vector3d width_axis = height_axis.cross(normal_vector).normalized();

    for (int i = 0; i < n_width; ++i) {
      double w_offset = -plane_width_ / 2.0 + plane_width_ * i / (n_width - 1);
      for (int j = 0; j < n_height; ++j) {
        double h_offset = -plane_height_ / 2.0 + plane_height_ * j / (n_height - 1);
        geometry_msgs::msg::Pose pose;
        pose.position.x = plane_center.x() + w_offset * width_axis.x() + h_offset * height_axis.x();
        pose.position.y = plane_center.y() + w_offset * width_axis.y() + h_offset * height_axis.y();
        pose.position.z = plane_center.z() + w_offset * width_axis.z() + h_offset * height_axis.z();
        setOrientation(pose, object_position);
        nbv_poses.push_back(pose);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu plane poses", nbv_poses.size());
    return nbv_poses;
  }

  /** Set orientation of the pose to face the object */
  void setOrientation(geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Point& object_position) {
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
  }

  /** Handle incoming action goal */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const Nbv::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal - shape: %s, object location: x: %.2f y: %.2f z: %.2f, distance: %.2f, locations: %d",
                goal->shape_type.c_str(), goal->object_position.x, goal->object_position.y,
                goal->object_position.z, goal->distance, goal->location_number);

    if (goal->location_number <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid number of locations: %d", goal->location_number);
      return rclcpp_action::GoalResponse::REJECT;
    }

    loadShapeParameters(goal->shape_type);

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_object.id = "object";
    shape_msgs::msg::SolidPrimitive primitive;
    geometry_msgs::msg::Pose object_pose;
    object_pose.orientation.w = 1.0;
    object_pose.position = goal->object_position;

    if (goal->shape_type == "cylinder") {
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[0] = cylinder_height_;
      primitive.dimensions[1] = cylinder_radius_;
    } else {
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.1;
      primitive.dimensions[1] = 0.1;
      primitive.dimensions[2] = 0.1;
    }

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {collision_object};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /** Execute the action goal with rotation functionality */
  void execute(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Nbv::Feedback>();
    auto& message = feedback->processing_status;
    message = "Generating poses for " + goal->shape_type + "...";
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
        generatePoses(goal->shape_type, goal->object_position, goal->distance);

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
        visual_tools_->publishZArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
        visual_tools_->trigger();
        continue;
      }

      moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
      if (!current_state || !current_state->setFromIK(
              move_group_interface_->getRobotModel()->getJointModelGroup("manipulator"), pose, 1.0)) {
        visual_tools_->publishZArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
        visual_tools_->trigger();
        continue;
      }

      std::vector<double> joint_values;
      current_state->copyJointGroupPositions("manipulator", joint_values);
      move_group_interface_->setPlannerId("PTP");
      move_group_interface_->setJointValueTarget(joint_values);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      bool success = (move_group_interface_->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success) {
        reachable_poses.push_back(pose);
        visual_tools_->publishZArrow(pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
      } else {
        visual_tools_->publishZArrow(pose, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
      }
      visual_tools_->trigger();
    }

    if (reachable_poses.empty()) {
      result->success = false;
      RCLCPP_ERROR(this->get_logger(), "No reachable poses found");
      goal_handle->succeed(result);
      return;
    }

    std::vector<geometry_msgs::msg::Pose> selected_poses;
    if (goal->visit_all) {
      geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
      selected_poses = solveTSP(reachable_poses, current_pose);
    } else {
      int target_locations = std::min(goal->location_number, static_cast<int>(reachable_poses.size()));
      selected_poses = selectRandomPoses(reachable_poses, target_locations);
    }

    message = "Moving to selected poses with rotations...";
    goal_handle->publish_feedback(feedback);

    for (const auto& pose : selected_poses) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled during execution");
        return;
      }

      // Move to initial pose without constraints
      if (!moveToPose(pose)) {
        RCLCPP_WARN(this->get_logger(), "Failed to move to initial pose (%.2f, %.2f, %.2f), skipping",
                    pose.position.x, pose.position.y, pose.position.z);
        continue;
      }

      // Save point cloud at initial orientation
      if (!savePointCloud()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud at initial orientation for pose (%.2f, %.2f, %.2f), continuing",
                     pose.position.x, pose.position.y, pose.position.z);
        continue;
      }

      // Get current joint values for joints 1, 2, 3
      moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions("manipulator", joint_values);
      std::vector<double> fixed_joints = {joint_values[0], joint_values[1], joint_values[2]};

      // Rotate +15 degrees on x-axis with joint constraints
      geometry_msgs::msg::Pose rotated_pose_plus = rotatePose(pose, 15.0);
      RCLCPP_INFO(this->get_logger(), "Attempting to move to +15° rotation around x-axis");
      if (!moveToPose(rotated_pose_plus, true, fixed_joints)) {
        RCLCPP_WARN(this->get_logger(), "Failed to move to +15 degree rotation for pose (%.2f, %.2f, %.2f), skipping rotation",
                    pose.position.x, pose.position.y, pose.position.z);
      } else {
        if (!savePointCloud()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud at +15 degree rotation for pose (%.2f, %.2f, %.2f), continuing",
                       pose.position.x, pose.position.y, pose.position.z);
        }
      }

      // Rotate -15 degrees on x-axis with joint constraints
      geometry_msgs::msg::Pose rotated_pose_minus = rotatePose(pose, -15.0);
      RCLCPP_INFO(this->get_logger(), "Attempting to move to -15° rotation around x-axis");
      if (!moveToPose(rotated_pose_minus, true, fixed_joints)) {
        RCLCPP_WARN(this->get_logger(), "Failed to move to -15 degree rotation for pose (%.2f, %.2f, %.2f), skipping rotation",
                    pose.position.x, pose.position.y, pose.position.z);
      } else {
        if (!savePointCloud()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud at -15 degree rotation for pose (%.2f, %.2f, %.2f), continuing",
                       pose.position.x, pose.position.y, pose.position.z);
        }
      }
    }

    result->success = true;
    if (goal->visit_all) {
      RCLCPP_INFO(this->get_logger(), "Visited all %zu reachable locations with rotations", selected_poses.size());
    } else {
      RCLCPP_INFO(this->get_logger(), "Visited %zu of %d requested locations with rotations",
                  selected_poses.size(), goal->location_number);
    }
    goal_handle->succeed(result);
  }

  /** Move to a specified pose */
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

  /** Save the point cloud at the current pose using the save_point_cloud action */
  bool savePointCloud() {
    if (!save_point_cloud_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Save point cloud action server not available");
      return false;
    }

    auto goal = kinova_action_interfaces::action::SavePointCloud::Goal();
    goal.start_saving = true;

    auto send_goal_options = rclcpp_action::Client<kinova_action_interfaces::action::SavePointCloud>::SendGoalOptions();

    auto goal_handle_future = save_point_cloud_client_->async_send_goal(goal, send_goal_options);

    if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Save point cloud action did not accept the goal within timeout");
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Save point cloud goal was rejected");
      return false;
    }

    auto result_future = save_point_cloud_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Save point cloud action did not complete within timeout");
      return false;
    }

    auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (wrapped_result.result->success) {
        RCLCPP_INFO(this->get_logger(), "Point cloud saved successfully");
        return true;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud (action succeeded but result indicates failure)");
        return false;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Save point cloud action failed with code: %d",
                   static_cast<int>(wrapped_result.code));
      return false;
    }
  }

  /** Rotate a pose around the x-axis by a specified angle */
  geometry_msgs::msg::Pose rotatePose(const geometry_msgs::msg::Pose& original_pose, double angle_degrees) {
    double angle_radians = angle_degrees * M_PI / 180.0;
    Eigen::Quaterniond original_quat(
        original_pose.orientation.w,
        original_pose.orientation.x,
        original_pose.orientation.y,
        original_pose.orientation.z
    );

    // Create rotation quaternion for x-axis rotation
    Eigen::Quaterniond rotation_quat(Eigen::AngleAxisd(angle_radians, Eigen::Vector3d::UnitX()));

    // Apply rotation to the original orientation
    Eigen::Quaterniond new_quat = original_quat * rotation_quat;
    new_quat.normalize();

    geometry_msgs::msg::Pose rotated_pose = original_pose;
    rotated_pose.orientation.x = new_quat.x();
    rotated_pose.orientation.y = new_quat.y();
    rotated_pose.orientation.z = new_quat.z();
    rotated_pose.orientation.w = new_quat.w();

    return rotated_pose;
  }

  /** Handle goal cancellation */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /** Handle accepted goal */
  void handle_accepted(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    std::thread{std::bind(&NextBestViewServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  /** Select random poses from a list */
  std::vector<geometry_msgs::msg::Pose> selectRandomPoses(const std::vector<geometry_msgs::msg::Pose>& poses,
                                                          int count) {
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
    return selected_poses;
  }
};

/** Main function */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<NextBestViewServer>();
  action_server->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}