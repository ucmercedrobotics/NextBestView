#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <kinova_action_interfaces/srv/current_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CurrentPoseService : public rclcpp::Node {
public:
    CurrentPoseService() : Node("current_pose_service") {
        // Initialize MoveIt MoveGroupInterface for the "manipulator" group
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "manipulator");

        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void handlePoseRequest(
        const std::shared_ptr<kinova_action_interfaces::srv::CurrentPose::Request> request,
        std::shared_ptr<kinova_action_interfaces::srv::CurrentPose::Response> response) {
        (void)request; // Ignore the empty request

        // Get the end effector link name
        std::string end_effector_link = move_group_->getEndEffectorLink();

        // Get the current end effector pose
        geometry_msgs::msg::PoseStamped current_end_effector_pose =
            move_group_->getCurrentPose(end_effector_link);

        // Set the end effector pose in the response
        response->end_effector_pose = current_end_effector_pose;

        // Calculate the camera pose
        geometry_msgs::msg::TransformStamped T_ec;
        try {
            // Look up the transform from end effector to camera_color_frame
            T_ec = tf_buffer_->lookupTransform(end_effector_link, "camera_color_frame", tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform from %s to camera_color_frame: %s",
                         end_effector_link.c_str(), ex.what());
            // Set a default camera pose with the same header but zero pose
            response->camera_pose.header = current_end_effector_pose.header;
            response->camera_pose.pose = geometry_msgs::msg::Pose();
            return;
        }

        // Convert poses and transforms to TF2 types
        tf2::Transform T_we, T_ec_tf, T_wc;
        tf2::fromMsg(current_end_effector_pose.pose, T_we);
        tf2::fromMsg(T_ec.transform, T_ec_tf);

        // Compute the camera pose in the world frame: T_wc = T_we * T_ec
        T_wc = T_we * T_ec_tf;

        // Convert the result back to a geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose camera_pose;
        tf2::toMsg(T_wc, camera_pose);

        // Set the camera pose in the response with the same header as the end effector pose
        response->camera_pose.header = current_end_effector_pose.header;
        response->camera_pose.pose = camera_pose;

        // Log the sent poses for debugging
        RCLCPP_INFO(this->get_logger(),
                    "Sent poses: EE (x=%.2f, y=%.2f, z=%.2f), Camera (x=%.2f, y=%.2f, z=%.2f)",
                    current_end_effector_pose.pose.position.x, current_end_effector_pose.pose.position.y,
                    current_end_effector_pose.pose.position.z,
                    camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CurrentPoseService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}