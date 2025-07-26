#include "next_best_view/reachability_check.hpp"
#include <cmath>
#include <functional>

// Constructor implementation
ReachablePosesService::ReachablePosesService(const rclcpp::NodeOptions& options)
    : Node("reachable_poses_service", options) {
}

// Initialize method implementation
void ReachablePosesService::initialize() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "manipulator");

    service_ = this->create_service<kinova_action_interfaces::srv::ReachabilityCheck>(
        "check_reachable_poses",
        std::bind(&ReachablePosesService::handle_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Reachable Poses Service initialized");
}

// Compute end-effector pose implementation
bool ReachablePosesService::computeEndEffectorPose(const geometry_msgs::msg::Pose& desired_camera_pose,
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

// Handle request implementation
void ReachablePosesService::handle_request(const std::shared_ptr<kinova_action_interfaces::srv::ReachabilityCheck::Request> request,
                                           std::shared_ptr<kinova_action_interfaces::srv::ReachabilityCheck::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request with %zu camera poses", request->camera_poses.size());

    moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(
        move_group_interface_->getRobotModel());
    const moveit::core::JointModelGroup* jmg = move_group_interface_->getRobotModel()->getJointModelGroup("manipulator");
    std::string end_effector_link = move_group_interface_->getEndEffectorLink();

    for (const auto& camera_pose : request->camera_poses) {
        // Check distance from base (approximate workspace limit)
        double distance_from_base = sqrt(camera_pose.position.x * camera_pose.position.x +
                                         camera_pose.position.y * camera_pose.position.y +
                                         camera_pose.position.z * camera_pose.position.z);
        if (distance_from_base > 0.891) {
            RCLCPP_DEBUG(this->get_logger(), "Pose at (%.2f, %.2f, %.2f) exceeds distance limit (%.3f > 0.891)",
                         camera_pose.position.x, camera_pose.position.y, camera_pose.position.z, distance_from_base);
            continue;
        }

        // Compute end-effector pose
        geometry_msgs::msg::Pose end_effector_pose;
        if (!computeEndEffectorPose(camera_pose, end_effector_pose)) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to compute end-effector pose for camera pose at (%.2f, %.2f, %.2f)",
                         camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);
            continue;
        }

        // Check IK solution
        if (robot_state->setFromIK(jmg, end_effector_pose, end_effector_link, 1.0)) {
            // Add both camera pose and end-effector pose to the response
            response->reachable_camera_poses.push_back(camera_pose);
            response->corresponding_end_effector_poses.push_back(end_effector_pose);
            RCLCPP_DEBUG(this->get_logger(), "Pose at (%.2f, %.2f, %.2f) is reachable",
                         camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No IK solution for pose at (%.2f, %.2f, %.2f)",
                         camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Found %zu reachable poses out of %zu",
                response->reachable_camera_poses.size(), request->camera_poses.size());
}

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto service_node = std::make_shared<ReachablePosesService>();
    service_node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(service_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}