#ifndef COLLISION_OBJECT_HPP
#define COLLISION_OBJECT_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "kinova_action_interfaces/srv/add_collision_object.hpp"

class CollisionObjectService : public rclcpp::Node {
public:
    explicit CollisionObjectService(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void addCollisionObjectCallback(
        const std::shared_ptr<kinova_action_interfaces::srv::AddCollisionObject::Request> request,
        std::shared_ptr<kinova_action_interfaces::srv::AddCollisionObject::Response> response);

    rclcpp::Service<kinova_action_interfaces::srv::AddCollisionObject>::SharedPtr service_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Parameters for collision object dimensions
    double cylinder_height_;
    double cylinder_radius_;
    double box_length_;
    double box_width_;
    double box_height_;
};

#endif // COLLISION_OBJECT_SERVICE_HPP