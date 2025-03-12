#ifndef POSE_GENERATOR_HPP
#define POSE_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <kinova_action_interfaces/srv/pose_generator.hpp>
#include <Eigen/Dense>

class PoseGenerator : public rclcpp::Node {
public:
    PoseGenerator();

private:
    // Service server
    rclcpp::Service<kinova_action_interfaces::srv::PoseGenerator>::SharedPtr service_;

    // Shape parameters
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

    // Methods
    void loadShapeParameters();
    void handleRequest(const std::shared_ptr<kinova_action_interfaces::srv::PoseGenerator::Request> request,
                       std::shared_ptr<kinova_action_interfaces::srv::PoseGenerator::Response> response);
    std::vector<geometry_msgs::msg::Pose> generatePoses(const std::string& shape,
                                                        const geometry_msgs::msg::Point& object_position);
    std::vector<geometry_msgs::msg::Pose> generateCylinderPoints(const geometry_msgs::msg::Point& object_position);
    std::vector<geometry_msgs::msg::Pose> generateHemispherePoints(const geometry_msgs::msg::Point& object_position);
    std::vector<geometry_msgs::msg::Pose> generateSpherePoints(const geometry_msgs::msg::Point& object_position);
    std::vector<geometry_msgs::msg::Pose> generateRectanglePoints(const geometry_msgs::msg::Point& object_position);
    std::vector<geometry_msgs::msg::Pose> generatePlanePoints(const geometry_msgs::msg::Point& object_position);
    void setOrientation(geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Point& object_position);
};

#endif // POSE_GENERATOR_HPP