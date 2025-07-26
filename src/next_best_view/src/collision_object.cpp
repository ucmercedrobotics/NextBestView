#include "next_best_view/collision_object.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

CollisionObjectService::CollisionObjectService(const rclcpp::NodeOptions& options)
    : Node("collision_object_service", options) {
    // Declare parameters with default values
    this->declare_parameter("cylinder_height", 2.0);
    this->declare_parameter("cylinder_diameter", 0.5);
    this->declare_parameter("box_length", 0.1);
    this->declare_parameter("box_width", 0.1);
    this->declare_parameter("box_height", 0.1);

    // Retrieve parameters
    cylinder_height_ = this->get_parameter("cylinder_height").as_double();
    cylinder_radius_ = this->get_parameter("cylinder_diameter").as_double() / 2.0;
    box_length_ = this->get_parameter("box_length").as_double();
    box_width_ = this->get_parameter("box_width").as_double();
    box_height_ = this->get_parameter("box_height").as_double();

    // Create the service
    service_ = this->create_service<kinova_action_interfaces::srv::AddCollisionObject>(
        "add_collision_object",
        std::bind(&CollisionObjectService::addCollisionObjectCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize static collision objects (rectangle_object and control_panel_object)
    moveit_msgs::msg::CollisionObject rectangle_collision_object;
    rectangle_collision_object.header.frame_id = "base_link";
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
    control_panel_object.header.frame_id = "base_link";
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

    std::vector<moveit_msgs::msg::CollisionObject> static_objects = {rectangle_collision_object, control_panel_object};
    planning_scene_interface_.applyCollisionObjects(static_objects);

    RCLCPP_INFO(this->get_logger(), "Static collision objects added and service initialized");
}

void CollisionObjectService::addCollisionObjectCallback(
    const std::shared_ptr<kinova_action_interfaces::srv::AddCollisionObject::Request> request,
    std::shared_ptr<kinova_action_interfaces::srv::AddCollisionObject::Response> response) {
    // Log the request
    RCLCPP_INFO(this->get_logger(), "Adding collision object for shape: %s at position (%.2f, %.2f, %.2f)",
                request->shape_type.c_str(), request->object_position.x, 
                request->object_position.y, request->object_position.z);

    // Remove existing "object" collision object
    planning_scene_interface_.removeCollisionObjects({"object"});

    // Create the dynamic collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "object";
    shape_msgs::msg::SolidPrimitive primitive;
    geometry_msgs::msg::Pose object_pose;
    object_pose.orientation.w = 1.0;
    object_pose.position = request->object_position;

    if (request->shape_type == "cylinder") {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = cylinder_height_;  // Height
        primitive.dimensions[1] = cylinder_radius_;  // Radius
    } else {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = box_length_;
        primitive.dimensions[1] = box_width_;
        primitive.dimensions[2] = box_height_;
    }

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;

    // Apply the collision object
    planning_scene_interface_.applyCollisionObject(collision_object);

    // Set response
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Collision object added successfully");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionObjectService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}