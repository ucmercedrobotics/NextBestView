#ifndef REACHABLITIY_CHECK_HPP
#define REACHABILITY_CHECK_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "kinova_action_interfaces/srv/reachability_check.hpp"

class ReachablePosesService : public rclcpp::Node {
public:
    // Constructor with default NodeOptions
    ReachablePosesService(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Public method to initialize the service
    void initialize();

private:
    // Private method to compute the end-effector pose from a desired camera pose
    bool computeEndEffectorPose(const geometry_msgs::msg::Pose& desired_camera_pose,
                                geometry_msgs::msg::Pose& end_effector_pose);

    // Private method to handle service requests
    void handle_request(const std::shared_ptr<kinova_action_interfaces::srv::ReachabilityCheck::Request> request,
                        std::shared_ptr<kinova_action_interfaces::srv::ReachabilityCheck::Response> response);

    // Member variables
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Service<kinova_action_interfaces::srv::ReachabilityCheck>::SharedPtr service_;
};

#endif  // REACHABLE_POSES_SERVICE_HPP