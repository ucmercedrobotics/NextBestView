#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <kinova_action_interfaces/srv/current_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class CurrentPoseService : public rclcpp::Node {
public:
    CurrentPoseService() : Node("current_pose_service") {
        // Initialize MoveIt MoveGroupInterface for the "manipulator" group
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "manipulator");

        // Create the service
        service_ = this->create_service<kinova_action_interfaces::srv::CurrentPose>(
            "current_pose",
            std::bind(&CurrentPoseService::handlePoseRequest, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "CurrentPose service is ready");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<kinova_action_interfaces::srv::CurrentPose>::SharedPtr service_;

    void handlePoseRequest(
        const std::shared_ptr<kinova_action_interfaces::srv::CurrentPose::Request> request,
        std::shared_ptr<kinova_action_interfaces::srv::CurrentPose::Response> response) {
        (void)request; // Ignore the empty request

        // Get the current pose of the end effector
        geometry_msgs::msg::PoseStamped current_pose =
            move_group_->getCurrentPose(move_group_->getEndEffectorLink());

        // Fill the response with the current pose
        response->pose = current_pose;

        RCLCPP_INFO(this->get_logger(), "Sent end effector pose: x=%.2f, y=%.2f, z=%.2f",
                    current_pose.pose.position.x, current_pose.pose.position.y,
                    current_pose.pose.position.z);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CurrentPoseService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}