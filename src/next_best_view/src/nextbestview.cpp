#include "next_best_view/nextbestview.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <iostream>
#include <cmath>

NextBestView::NextBestView(const rclcpp::NodeOptions& options)
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

void NextBestView::initialize() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "manipulator");

    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), "base_link", "moveit_visual_markers",
        move_group_interface_->getRobotModel());

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    action_server_ = rclcpp_action::create_server<Nbv>(
        this, "next_best_view/next_best_view",
        std::bind(&NextBestView::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NextBestView::handle_cancel, this, std::placeholders::_1),
        std::bind(&NextBestView::handle_accepted, this, std::placeholders::_1));

    save_point_cloud_client_ = rclcpp_action::create_client<kinova_action_interfaces::action::SavePointCloud>(
        this, "save_pointcloud");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

double NextBestView::distanceBetweenPoses(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2) {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    double dz = p1.position.z - p2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<geometry_msgs::msg::Pose> NextBestView::solveTSP(const std::vector<geometry_msgs::msg::Pose>& poses,
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

void NextBestView::loadShapeParameters(const std::string& shape) {
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

std::vector<geometry_msgs::msg::Pose> NextBestView::generatePoses(const std::string& shape,
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

std::vector<geometry_msgs::msg::Pose> NextBestView::generateCylinderPoints(const geometry_msgs::msg::Point& object_position,
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

std::vector<geometry_msgs::msg::Pose> NextBestView::generateHemispherePoints(const geometry_msgs::msg::Point& object_position,
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

std::vector<geometry_msgs::msg::Pose> NextBestView::generateSpherePoints(const geometry_msgs::msg::Point& object_position,
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

std::vector<geometry_msgs::msg::Pose> NextBestView::generateRectanglePoints(const geometry_msgs::msg::Point& object_position,
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

std::vector<geometry_msgs::msg::Pose> NextBestView::generatePlanePoints(const geometry_msgs::msg::Point& object_position,
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

bool NextBestView::computeEndEffectorPose(const geometry_msgs::msg::Pose& desired_camera_pose,
                                                geometry_msgs::msg::Pose& end_effector_pose) {
    std::string end_effector_link = move_group_interface_->getEndEffectorLink();
    geometry_msgs::msg::TransformStamped T_ec;
    try {
        T_ec = tf_buffer_->lookupTransform(end_effector_link, "camera_color_frame", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transform from %s to camera_color_frame: %s",
                     end_effector_link.c_str(), ex.what());
        return false;
    }

    tf2::Transform T_ec_tf;
    tf2::fromMsg(T_ec.transform, T_ec_tf);

    tf2::Transform T_wc;
    tf2::fromMsg(desired_camera_pose, T_wc);

    tf2::Transform T_ec_inv = T_ec_tf.inverse();
    tf2::Transform T_we = T_wc * T_ec_inv;

    tf2::toMsg(T_we, end_effector_pose);
    return true;
}

geometry_msgs::msg::Pose NextBestView::getCurrentCameraPose() {
    std::string end_effector_link = move_group_interface_->getEndEffectorLink();
    geometry_msgs::msg::PoseStamped current_end_effector_pose = move_group_interface_->getCurrentPose(end_effector_link);
    geometry_msgs::msg::TransformStamped T_ec;
    try {
        T_ec = tf_buffer_->lookupTransform(end_effector_link, "camera_color_frame", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transform from %s to camera_color_frame: %s",
                     end_effector_link.c_str(), ex.what());
        return geometry_msgs::msg::Pose();
    }
    tf2::Transform T_we, T_ec_tf, T_wc;
    tf2::fromMsg(current_end_effector_pose.pose, T_we);
    tf2::fromMsg(T_ec.transform, T_ec_tf);
    T_wc = T_we * T_ec_tf;
    geometry_msgs::msg::Pose current_camera_pose;
    tf2::toMsg(T_wc, current_camera_pose);
    return current_camera_pose;
}

void NextBestView::setOrientation(geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Point& object_position) {
    Eigen::Vector3d z_axis_target(object_position.x - pose.position.x,
                                  object_position.y - pose.position.y,
                                  object_position.z - pose.position.z);
    z_axis_target.normalize();

    Eigen::Vector3d x_axis_target;
    if (std::abs(z_axis_target.z()) > 0.99) {
        x_axis_target = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
        x_axis_target = Eigen::Vector3d(0.0, 0.0, 1.0).cross(z_axis_target);
    }
    x_axis_target.normalize();

    Eigen::Vector3d y_axis_target = z_axis_target.cross(x_axis_target);
    y_axis_target.normalize();

    if (y_axis_target.z() > 0) {
        y_axis_target = -y_axis_target;
        x_axis_target = y_axis_target.cross(z_axis_target);
        x_axis_target.normalize();
    }

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

rclcpp_action::GoalResponse NextBestView::handle_goal(const rclcpp_action::GoalUUID& uuid,
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

    moveit_msgs::msg::CollisionObject rectangle_collision_object;
    rectangle_collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
    rectangle_collision_object.id = "rectangle_object";
    shape_msgs::msg::SolidPrimitive rectangle_primitive;
    rectangle_primitive.type = rectangle_primitive.BOX;
    rectangle_primitive.dimensions.resize(3);
    rectangle_primitive.dimensions[0] = 1.2;
    rectangle_primitive.dimensions[1] = 1.2;
    rectangle_primitive.dimensions[2] = 0.2;
    geometry_msgs::msg::Pose rectangle_pose;
    rectangle_pose.orientation.w = 1.0;
    rectangle_pose.position.x = -0.4;
    rectangle_pose.position.y = 0.4;
    rectangle_pose.position.z = -0.1;
    rectangle_collision_object.primitives.push_back(rectangle_primitive);
    rectangle_collision_object.primitive_poses.push_back(rectangle_pose);
    rectangle_collision_object.operation = rectangle_collision_object.ADD;

    moveit_msgs::msg::CollisionObject control_panel_object;
    control_panel_object.header.frame_id = move_group_interface_->getPlanningFrame();
    control_panel_object.id = "control_panel_object";
    shape_msgs::msg::SolidPrimitive control_panel_primitive;
    control_panel_primitive.type = control_panel_primitive.BOX;
    control_panel_primitive.dimensions.resize(3);
    control_panel_primitive.dimensions[0] = 0.3;
    control_panel_primitive.dimensions[1] = 1.2;
    control_panel_primitive.dimensions[2] = 1.0;
    geometry_msgs::msg::Pose control_panel_pose;
    control_panel_pose.orientation.w = 1.0;
    control_panel_pose.position.x = -0.8;
    control_panel_pose.position.y = 0.4;
    control_panel_pose.position.z = 0.3;
    control_panel_object.primitives.push_back(control_panel_primitive);
    control_panel_object.primitive_poses.push_back(control_panel_pose);
    control_panel_object.operation = control_panel_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {
        collision_object, rectangle_collision_object, control_panel_object};
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(collision_objects);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void NextBestView::execute(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Nbv::Feedback>();
    auto& message = feedback->processing_status;
    message = "Generating poses for " + goal->shape_type + "...";
    goal_handle->publish_feedback(feedback);
    auto result = std::make_shared<Nbv::Result>();

    move_group_interface_->setMaxVelocityScalingFactor(0.3);
    move_group_interface_->setMaxAccelerationScalingFactor(0.3);
    move_group_interface_->setPlanningTime(2.0);
    move_group_interface_->setNumPlanningAttempts(5);
    move_group_interface_->setGoalPositionTolerance(0.01);
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
    std::string end_effector_link = move_group_interface_->getEndEffectorLink();
    for (const auto& camera_pose : candidate_poses) {
        double distance_from_base = sqrt(camera_pose.position.x * camera_pose.position.x +
                                         camera_pose.position.y * camera_pose.position.y +
                                         camera_pose.position.z * camera_pose.position.z);
        if (distance_from_base > 0.891) {
            visual_tools_->publishZArrow(camera_pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
            visual_tools_->trigger();
            continue;
        }

        geometry_msgs::msg::Pose end_effector_pose;
        if (!computeEndEffectorPose(camera_pose, end_effector_pose)) {
            visual_tools_->publishZArrow(camera_pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
            visual_tools_->trigger();
            continue;
        }

        moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
        if (!current_state || !current_state->setFromIK(
                move_group_interface_->getRobotModel()->getJointModelGroup("manipulator"),
                end_effector_pose, end_effector_link, 1.0)) {
            visual_tools_->publishZArrow(camera_pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
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
            reachable_poses.push_back(camera_pose);
            visual_tools_->publishZArrow(camera_pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
        } else {
            visual_tools_->publishZArrow(camera_pose, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
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
        geometry_msgs::msg::Pose current_camera_pose = getCurrentCameraPose();
        selected_poses = solveTSP(reachable_poses, current_camera_pose);
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

        if (!moveToPose(pose)) {
            RCLCPP_WARN(this->get_logger(), "Failed to move to initial pose (%.2f, %.2f, %.2f), skipping",
                        pose.position.x, pose.position.y, pose.position.z);
            continue;
        }

        if (!savePointCloud()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud at initial orientation for pose (%.2f, %.2f, %.2f), continuing",
                         pose.position.x, pose.position.y, pose.position.z);
            continue;
        }

        moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
        std::vector<double> joint_values;
        current_state->copyJointGroupPositions("manipulator", joint_values);
        std::vector<double> fixed_joints = {joint_values[0], joint_values[1], joint_values[2]};

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

    moveToHome();

    result->success = true;
    if (goal->visit_all) {
        RCLCPP_INFO(this->get_logger(), "Visited all %zu reachable locations with rotations", selected_poses.size());
    } else {
        RCLCPP_INFO(this->get_logger(), "Visited %zu of %d requested locations with rotations",
                    selected_poses.size(), goal->location_number);
    }
    goal_handle->succeed(result);
}

bool NextBestView::moveToPose(const geometry_msgs::msg::Pose& desired_camera_pose,
                                    bool constrain_joints,
                                    const std::vector<double>& fixed_joint_values) {
    geometry_msgs::msg::Pose end_effector_pose;
    if (!computeEndEffectorPose(desired_camera_pose, end_effector_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute end-effector pose for camera pose (%.2f, %.2f, %.2f)",
                     desired_camera_pose.position.x, desired_camera_pose.position.y, desired_camera_pose.position.z);
        return false;
    }

    moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
    if (!current_state) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state");
        return false;
    }

    const moveit::core::JointModelGroup* jmg =
        move_group_interface_->getRobotModel()->getJointModelGroup("manipulator");

    if (constrain_joints && fixed_joint_values.size() >= 3) {
        std::vector<std::string> joint_names = jmg->getJointModelNames();
        for (size_t i = 0; i < 3 && i < joint_names.size(); ++i) {
            current_state->setJointPositions(joint_names[i], &fixed_joint_values[i]);
            current_state->enforceBounds();
        }
    }

    std::vector<double> current_joints;
    current_state->copyJointGroupPositions(jmg, current_joints);

    double min_displacement = std::numeric_limits<double>::max();
    std::vector<double> best_solution;
    int num_attempts = 10;
    double timeout = 1.0;
    std::string end_effector_link = move_group_interface_->getEndEffectorLink();

    for (int attempt = 0; attempt < num_attempts; ++attempt) {
        moveit::core::RobotStatePtr temp_state = std::make_shared<moveit::core::RobotState>(*current_state);
        temp_state->setToRandomPositions(jmg);
        if (temp_state->setFromIK(jmg, end_effector_pose, end_effector_link, timeout)) {
            std::vector<double> solution;
            temp_state->copyJointGroupPositions(jmg, solution);
            double displacement = 0.0;
            for (size_t i = 0; i < solution.size(); ++i) {
                displacement += std::abs(solution[i] - current_joints[i]);
            }
            if (displacement < min_displacement) {
                min_displacement = displacement;
                best_solution = solution;
            }
        }
    }

    if (best_solution.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No IK solution found for camera pose (%.2f, %.2f, %.2f) after %d attempts",
                     desired_camera_pose.position.x, desired_camera_pose.position.y,
                     desired_camera_pose.position.z, num_attempts);
        return false;
    }

    move_group_interface_->setPlannerId("RRTConnect");
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setNumPlanningAttempts(10);
    move_group_interface_->setJointValueTarget(best_solution);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success = (move_group_interface_->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan to camera pose (%.2f, %.2f, %.2f)",
                     desired_camera_pose.position.x, desired_camera_pose.position.y, desired_camera_pose.position.z);
        return false;
    }

    move_group_interface_->execute(my_plan_arm);
    move_group_interface_->setStartStateToCurrentState();
    return true;
}

bool NextBestView::savePointCloud() {
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
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped_result.result->success) {
        RCLCPP_INFO(this->get_logger(), "Point cloud saved successfully");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud");
        return false;
    }
}

geometry_msgs::msg::Pose NextBestView::rotatePose(const geometry_msgs::msg::Pose& original_pose, double angle_degrees) {
    double angle_radians = angle_degrees * M_PI / 180.0;
    Eigen::Quaterniond original_quat(
        original_pose.orientation.w,
        original_pose.orientation.x,
        original_pose.orientation.y,
        original_pose.orientation.z
    );

    Eigen::Quaterniond rotation_quat(Eigen::AngleAxisd(angle_radians, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond new_quat = original_quat * rotation_quat;
    new_quat.normalize();

    geometry_msgs::msg::Pose rotated_pose = original_pose;
    rotated_pose.orientation.x = new_quat.x();
    rotated_pose.orientation.y = new_quat.y();
    rotated_pose.orientation.z = new_quat.z();
    rotated_pose.orientation.w = new_quat.w();

    return rotated_pose;
}

rclcpp_action::CancelResponse NextBestView::handle_cancel(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NextBestView::handle_accepted(const std::shared_ptr<GoalHandleNbv> goal_handle) {
    std::thread{std::bind(&NextBestView::execute, this, std::placeholders::_1), goal_handle}.detach();
}

std::vector<geometry_msgs::msg::Pose> NextBestView::selectRandomPoses(const std::vector<geometry_msgs::msg::Pose>& poses,
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

bool NextBestView::moveToHome() {
    move_group_interface_->setNamedTarget("Home");
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    bool success = (move_group_interface_->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group_interface_->execute(home_plan);
        RCLCPP_INFO(this->get_logger(), "Moved to Home position successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan or execute move to Home position");
    }
    return success;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<NextBestView>();
  action_server->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}