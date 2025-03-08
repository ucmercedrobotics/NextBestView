#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

namespace fs = std::filesystem;

#include "kinova_action_interfaces/action/filter_point_cloud.hpp"

using FilterPointCloud = kinova_action_interfaces::action::FilterPointCloud;

class PointCloudFilterNode : public rclcpp::Node {
public:
  PointCloudFilterNode() : Node("pointcloud_filter_node") {
    // Input directory containing .pcd files
    filter_input_directory_ = "src/next_best_view/point_clouds/";
    // Output directory for filtered .pcd files
    filter_output_directory_ = "src/next_best_view/filtered_pointclouds/";

    // Create output directory if it doesn't exist
    if (!fs::exists(filter_output_directory_)) {
      if (fs::create_directory(filter_output_directory_)) {
        RCLCPP_INFO(this->get_logger(), "Created filtered point clouds directory: %s",
                    filter_output_directory_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create filtered point clouds directory: %s",
                     filter_output_directory_.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Filtered point clouds directory already exists: %s",
                  filter_output_directory_.c_str());
    }

    // Initialize the action server
    action_server_ = rclcpp_action::create_server<FilterPointCloud>(
        this,
        "filter_pointclouds",
        std::bind(&PointCloudFilterNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PointCloudFilterNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&PointCloudFilterNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PointCloudFilterNode action server started");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const FilterPointCloud::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal with start: %s, cylinder_center: (%f, %f, %f), diameter: %f, height: %f",
                goal->start_filtering ? "true" : "false",
                goal->cylinder_center_x, goal->cylinder_center_y, goal->cylinder_center_z,
                goal->cylinder_diameter, goal->cylinder_height);
    if (!goal->start_filtering) {
      RCLCPP_INFO(this->get_logger(), "Goal rejected: start is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<FilterPointCloud>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Cancellation request received and accepted");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FilterPointCloud>> goal_handle) {
    std::thread{std::bind(&PointCloudFilterNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FilterPointCloud>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Starting point cloud filtering");
    auto feedback = std::make_shared<FilterPointCloud::Feedback>();
    auto result = std::make_shared<FilterPointCloud::Result>();
    auto goal = goal_handle->get_goal();

    // Extract cylinder parameters
    double cx = goal->cylinder_center_x;
    double cy = goal->cylinder_center_y;
    double cz = goal->cylinder_center_z;
    double radius = goal->cylinder_diameter / 2.0;
    double z_min = cz - goal->cylinder_height / 2.0;
    double z_max = cz + goal->cylinder_height / 2.0;

    // Check input directory
    if (!fs::exists(filter_input_directory_) || !fs::is_directory(filter_input_directory_)) {
      feedback->status_message = "Input directory invalid";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      result->num_files_processed = 0;
      goal_handle->succeed(result);
      return;
    }

    // Collect .pcd files
    std::vector<fs::path> pcd_files;
    for (const auto& entry : fs::directory_iterator(filter_input_directory_)) {
      if (entry.path().extension() == ".pcd") {
        pcd_files.push_back(entry.path());
      }
    }

    if (pcd_files.empty()) {
      feedback->status_message = "No .pcd files found";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      result->num_files_processed = 0;
      goal_handle->succeed(result);
      return;
    }

    int num_files_processed = 0;
    for (const auto& pcd_file : pcd_files) {
      if (goal_handle->is_canceling()) {
        feedback->status_message = "Filtering canceled";
        goal_handle->publish_feedback(feedback);
        result->success = false;
        result->num_files_processed = num_files_processed;
        goal_handle->canceled(result);
        return;
      }

      // Load point cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file.string(), *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load %s", pcd_file.string().c_str());
        continue;
      }

      // Filter points inside the cylinder and matching color criteria
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto& point : *cloud) {
        double dx = point.x - cx;
        double dy = point.y - cy;
        bool inside_cylinder = (dx * dx + dy * dy <= radius * radius) && (point.z >= z_min && point.z <= z_max);
        bool is_white = point.r > 200 && point.g > 200 && point.b > 200;
        bool is_blue = point.b > 100 && point.b > point.r + 50 && point.b > point.g + 50;
        if (inside_cylinder && (is_white || is_blue)) {
          filtered_cloud->points.push_back(point);
        }
      }
      filtered_cloud->width = filtered_cloud->points.size();
      filtered_cloud->height = 1;
      filtered_cloud->is_dense = true;

      // Save filtered point cloud
      std::string original_filename = pcd_file.filename().string();
      std::string filtered_filename = "filtered_" + original_filename;
      fs::path filtered_path = fs::path(filter_output_directory_) / filtered_filename;
      if (pcl::io::savePCDFileASCII(filtered_path.string(), *filtered_cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save %s", filtered_path.string().c_str());
        continue;
      }

      feedback->status_message = "Saved: " + filtered_filename;
      goal_handle->publish_feedback(feedback);
      num_files_processed++;
    }

    result->success = true;
    result->num_files_processed = num_files_processed;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Filtering completed: %d files processed", num_files_processed);
  }

  std::string filter_input_directory_;
  std::string filter_output_directory_;
  rclcpp_action::Server<FilterPointCloud>::SharedPtr action_server_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}