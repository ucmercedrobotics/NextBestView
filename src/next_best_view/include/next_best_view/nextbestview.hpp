#ifndef NEXT_BEST_VIEW_HPP
#define NEXT_BEST_VIEW_HPP

#include <memory>
#include <thread>
#include <random>
#include <limits>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "kinova_action_interfaces/action/next_best_view.hpp"
#include "kinova_action_interfaces/action/save_point_cloud.hpp"

class NextBestView : public rclcpp::Node {
public:
    using Nbv = kinova_action_interfaces::action::NextBestView;
    using GoalHandleNbv = rclcpp_action::ServerGoalHandle<Nbv>;

    explicit NextBestView(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void initialize();

private:
    rclcpp_action::Server<Nbv>::SharedPtr action_server_;
    rclcpp_action::Client<kinova_action_interfaces::action::SavePointCloud>::SharedPtr save_point_cloud_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

    double distanceBetweenPoses(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2);

    std::vector<geometry_msgs::msg::Pose> solveTSP(const std::vector<geometry_msgs::msg::Pose>& poses,
                                                   const geometry_msgs::msg::Pose& start_pose);

    void loadShapeParameters(const std::string& shape);

    std::vector<geometry_msgs::msg::Pose> generatePoses(const std::string& shape,
                                                        const geometry_msgs::msg::Point& object_position,
                                                        float distance);

    std::vector<geometry_msgs::msg::Pose> generateCylinderPoints(const geometry_msgs::msg::Point& object_position,
                                                                 float radius);

    std::vector<geometry_msgs::msg::Pose> generateHemispherePoints(const geometry_msgs::msg::Point& object_position,
                                                                   float radius);

    std::vector<geometry_msgs::msg::Pose> generateSpherePoints(const geometry_msgs::msg::Point& object_position,
                                                               float radius);

    std::vector<geometry_msgs::msg::Pose> generateRectanglePoints(const geometry_msgs::msg::Point& object_position,
                                                                  float distance);

    std::vector<geometry_msgs::msg::Pose> generatePlanePoints(const geometry_msgs::msg::Point& object_position,
                                                              float distance);

    bool computeEndEffectorPose(const geometry_msgs::msg::Pose& desired_camera_pose,
                                geometry_msgs::msg::Pose& end_effector_pose);

    geometry_msgs::msg::Pose getCurrentCameraPose();

    void setOrientation(geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Point& object_position);

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const Nbv::Goal> goal);

    void execute(const std::shared_ptr<GoalHandleNbv> goal_handle);

    bool moveToPose(const geometry_msgs::msg::Pose& desired_camera_pose,
                    bool constrain_joints = false,
                    const std::vector<double>& fixed_joint_values = {});

    bool savePointCloud();

    geometry_msgs::msg::Pose rotatePose(const geometry_msgs::msg::Pose& original_pose, double angle_degrees);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNbv> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleNbv> goal_handle);

    std::vector<geometry_msgs::msg::Pose> selectRandomPoses(const std::vector<geometry_msgs::msg::Pose>& poses,
                                                            int count);

    bool moveToHome();
};

#endif  // NEXT_BEST_VIEW_SERVER_HPP