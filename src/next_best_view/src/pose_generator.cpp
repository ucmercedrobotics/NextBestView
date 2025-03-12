#include "next_best_view/pose_generator.hpp"
#include <cmath>
#include <Eigen/Dense>  // Required for vector operations in setOrientation

PoseGenerator::PoseGenerator() : Node("pose_generator") {
    // Declare non-dimensional parameters with default values
    this->declare_parameter("cylinder_total_poses", 180);
    this->declare_parameter("cylinder_z_increments", 10);
    this->declare_parameter("hemisphere_total_poses", 100);
    this->declare_parameter("sphere_total_poses", 200);
    this->declare_parameter("rectangle_total_poses", 100);
    this->declare_parameter("plane_total_poses", 100);

    // Declare dimensional parameter for cylinder height
    this->declare_parameter("cylinder_height", 1.0);  // Default height, adjustable via YAML

    // Create the service
    service_ = this->create_service<kinova_action_interfaces::srv::PoseGenerator>(
        "generate_poses",
        std::bind(&PoseGenerator::handleRequest, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Load initial parameters
    loadShapeParameters();

    RCLCPP_INFO(this->get_logger(), "Pose Generator Service is ready.");
}

void PoseGenerator::loadShapeParameters() {
    // Load non-dimensional parameters from YAML
    cylinder_total_poses_ = this->get_parameter("cylinder_total_poses").as_int();
    cylinder_z_increments_ = this->get_parameter("cylinder_z_increments").as_int();
    hemisphere_total_poses_ = this->get_parameter("hemisphere_total_poses").as_int();
    sphere_total_poses_ = this->get_parameter("sphere_total_poses").as_int();
    rectangle_total_poses_ = this->get_parameter("rectangle_total_poses").as_int();
    plane_total_poses_ = this->get_parameter("plane_total_poses").as_int();
    cylinder_height_ = this->get_parameter("cylinder_height").as_double();

    // Compute derived parameters with safety check
    cylinder_n_theta_ = (cylinder_z_increments_ > 0) ? cylinder_total_poses_ / cylinder_z_increments_ : 1;
}

void PoseGenerator::handleRequest(const std::shared_ptr<kinova_action_interfaces::srv::PoseGenerator::Request> request,
                                  std::shared_ptr<kinova_action_interfaces::srv::PoseGenerator::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request - shape: %s, position: (%.2f, %.2f, %.2f), distance: %.2f",
                request->shape.c_str(), request->object_position.x, request->object_position.y,
                request->object_position.z, request->distance);

    // Set shape dimensions dynamically using request->distance
    if (request->shape == "cylinder") {
        cylinder_radius_ = request->distance / 2.0;  // Radius set to distance / 2
        // cylinder_height_ is loaded from parameters, not overwritten
    } else if (request->shape == "hemisphere") {
        hemisphere_radius_ = request->distance;  // Radius set to distance
    } else if (request->shape == "sphere") {
        sphere_radius_ = request->distance;  // Radius set to distance
    } else if (request->shape == "rectangle") {
        rectangle_length_ = request->distance;  // All dimensions set to distance
        rectangle_width_ = request->distance;
        rectangle_height_ = request->distance;
    } else if (request->shape == "plane") {
        plane_width_ = request->distance;  // Both dimensions set to distance
        plane_height_ = request->distance;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown shape: %s", request->shape.c_str());
        response->candidate_poses = {};
        return;
    }

    // Generate poses using the dynamically set dimensions
    std::vector<geometry_msgs::msg::Pose> poses = generatePoses(request->shape, request->object_position);
    response->candidate_poses = poses;

    RCLCPP_INFO(this->get_logger(), "Returning %zu candidate poses", poses.size());
}

std::vector<geometry_msgs::msg::Pose> PoseGenerator::generatePoses(const std::string& shape,
                                                                   const geometry_msgs::msg::Point& object_position) {
    if (shape == "cylinder") {
        return generateCylinderPoints(object_position);
    } else if (shape == "hemisphere") {
        return generateHemispherePoints(object_position);
    } else if (shape == "sphere") {
        return generateSpherePoints(object_position);
    } else if (shape == "rectangle") {
        return generateRectanglePoints(object_position);
    } else if (shape == "plane") {
        return generatePlanePoints(object_position);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown shape: %s", shape.c_str());
        return {};
    }
}

std::vector<geometry_msgs::msg::Pose> PoseGenerator::generateCylinderPoints(const geometry_msgs::msg::Point& object_position) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    if (cylinder_z_increments_ <= 1 || cylinder_n_theta_ <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid cylinder parameters: z_increments=%d, n_theta=%d",
                     cylinder_z_increments_, cylinder_n_theta_);
        return nbv_poses;
    }

    for (int k = 0; k < cylinder_total_poses_; ++k) {
        int j = k % cylinder_n_theta_;
        int i = k / cylinder_n_theta_;
        double theta = 2.0 * M_PI * j / cylinder_n_theta_;
        double z_offset = -cylinder_height_ / 2.0 + cylinder_height_ * i / (cylinder_z_increments_ - 1.0);

        geometry_msgs::msg::Pose pose;
        pose.position.x = object_position.x + cylinder_radius_ * cos(theta);
        pose.position.y = object_position.y + cylinder_radius_ * sin(theta);
        pose.position.z = object_position.z + z_offset;

        setOrientation(pose, object_position);
        nbv_poses.push_back(pose);
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu cylinder poses", nbv_poses.size());
    return nbv_poses;
}

std::vector<geometry_msgs::msg::Pose> PoseGenerator::generateHemispherePoints(const geometry_msgs::msg::Point& object_position) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_phi = static_cast<int>(sqrt(hemisphere_total_poses_ / 2));
    int n_theta = 2 * n_phi;

    if (n_phi <= 0 || n_theta <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid hemisphere parameters: n_phi=%d, n_theta=%d", n_phi, n_theta);
        return nbv_poses;
    }

    for (int i = 0; i < n_phi; ++i) {
        double phi = M_PI / 2.0 * i / (n_phi - 1.0);
        for (int j = 0; j < n_theta; ++j) {
            double theta = 2.0 * M_PI * j / n_theta;
            geometry_msgs::msg::Pose pose;
            pose.position.x = object_position.x + hemisphere_radius_ * sin(phi) * cos(theta);
            pose.position.y = object_position.y + hemisphere_radius_ * sin(phi) * sin(theta);
            pose.position.z = object_position.z + hemisphere_radius_ * cos(phi);

            setOrientation(pose, object_position);
            nbv_poses.push_back(pose);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu hemisphere poses", nbv_poses.size());
    return nbv_poses;
}

std::vector<geometry_msgs::msg::Pose> PoseGenerator::generateSpherePoints(const geometry_msgs::msg::Point& object_position) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_phi = static_cast<int>(sqrt(sphere_total_poses_ / 2));
    int n_theta = 2 * n_phi;

    if (n_phi <= 0 || n_theta <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid sphere parameters: n_phi=%d, n_theta=%d", n_phi, n_theta);
        return nbv_poses;
    }

    for (int i = 0; i < n_phi; ++i) {
        double phi = M_PI * i / (n_phi - 1.0);
        for (int j = 0; j < n_theta; ++j) {
            double theta = 2.0 * M_PI * j / n_theta;
            geometry_msgs::msg::Pose pose;
            pose.position.x = object_position.x + sphere_radius_ * sin(phi) * cos(theta);
            pose.position.y = object_position.y + sphere_radius_ * sin(phi) * sin(theta);
            pose.position.z = object_position.z + sphere_radius_ * cos(phi);

            setOrientation(pose, object_position);
            nbv_poses.push_back(pose);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu sphere poses", nbv_poses.size());
    return nbv_poses;
}

std::vector<geometry_msgs::msg::Pose> PoseGenerator::generateRectanglePoints(const geometry_msgs::msg::Point& object_position) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_x = static_cast<int>(cbrt(rectangle_total_poses_));
    int n_y = n_x;
    int n_z = rectangle_total_poses_ / (n_x * n_y);

    if (n_x <= 0 || n_y <= 0 || n_z <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid rectangle parameters: n_x=%d, n_y=%d, n_z=%d", n_x, n_y, n_z);
        return nbv_poses;
    }

    for (int i = 0; i < n_x; ++i) {
        double x = -rectangle_length_ / 2.0 + rectangle_length_ * i / (n_x - 1.0);
        for (int j = 0; j < n_y; ++j) {
            double y = -rectangle_width_ / 2.0 + rectangle_width_ * j / (n_y - 1.0);
            for (int k = 0; k < n_z; ++k) {
                double z = -rectangle_height_ / 2.0 + rectangle_height_ * k / (n_z - 1.0);
                geometry_msgs::msg::Pose pose;
                pose.position.x = object_position.x + x;
                pose.position.y = object_position.y + y;
                pose.position.z = object_position.z + z;

                setOrientation(pose, object_position);
                nbv_poses.push_back(pose);
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Generated %zu rectangle poses", nbv_poses.size());
    return nbv_poses;
}

std::vector<geometry_msgs::msg::Pose> PoseGenerator::generatePlanePoints(const geometry_msgs::msg::Point& object_position) {
    std::vector<geometry_msgs::msg::Pose> nbv_poses;
    int n_width = static_cast<int>(std::sqrt(plane_total_poses_));
    int n_height = plane_total_poses_ / n_width;

    if (n_width <= 0 || n_height <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid plane parameters: n_width=%d, n_height=%d", n_width, n_height);
        return nbv_poses;
    }

    double norm = std::sqrt(object_position.x * object_position.x +
                            object_position.y * object_position.y +
                            object_position.z * object_position.z);
    if (norm < 1e-6) {
        RCLCPP_ERROR(this->get_logger(), "Object position is too close to origin");
        return nbv_poses;
    }

    Eigen::Vector3d unit_vec = Eigen::Vector3d(object_position.x / norm,
                                               object_position.y / norm,
                                               object_position.z / norm);
    Eigen::Vector3d plane_center = Eigen::Vector3d(object_position.x, object_position.y, object_position.z) -
                                   (plane_width_ / 2.0) * unit_vec;  // Offset by half width for centering

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

void PoseGenerator::setOrientation(geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Point& object_position) {
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}