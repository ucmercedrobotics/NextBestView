#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <chrono>
#include <mutex>

#include "kinova_action_interfaces/action/save_point_cloud.hpp"

// Define the action type alias in the global scope
using SavePointCloud = kinova_action_interfaces::action::SavePointCloud;

namespace fs = std::filesystem;

class PointCloudSaverNode : public rclcpp::Node {
 public:
  PointCloudSaverNode() : Node("pointcloud_saver_node"), file_counter_(0) {
    // Declare parameters with default values
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("save_retries", 3);
    this->declare_parameter("distance_limit", 1.5);
    this->declare_parameter("distance_below", 0.0);

    // Retrieve parameter values
    target_frame_ = this->get_parameter("target_frame").as_string();
    save_retries_ = this->get_parameter("save_retries").as_int();
    distance_limit_ = this->get_parameter("distance_limit").as_double();
    distance_below_ = this->get_parameter("distance_below").as_double();

    // Log parameter values for debugging
    RCLCPP_INFO(this->get_logger(), "Initialized with distance_limit: %.2f meters, distance_below: %.2f meters",
                distance_limit_, distance_below_);

    // Determine the save directory: one level above the source file's directory
    fs::path source_file_path = fs::path(__FILE__);
    fs::path parent_dir = source_file_path.parent_path().parent_path();
    fs::path save_dir = parent_dir / "point_clouds";

    // Create directory if it doesn’t exist
    if (!fs::exists(save_dir)) {
      if (fs::create_directories(save_dir)) {
        RCLCPP_INFO(this->get_logger(), "Created point_clouds directory: %s", save_dir.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create point_clouds directory: %s", save_dir.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Using existing point_clouds directory: %s", save_dir.c_str());
    }
    save_directory_ = save_dir.string() + "/";

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Define QoS profile: Best Effort, Keep Last with depth 1
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Subscription for point cloud data
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", qos,
        std::bind(&PointCloudSaverNode::pointcloudCallback, this, std::placeholders::_1));

    // Action server for saving point clouds
    action_server_ = rclcpp_action::create_server<SavePointCloud>(
        this,
        "save_pointcloud",
        std::bind(&PointCloudSaverNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PointCloudSaverNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&PointCloudSaverNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PointCloudSaverNode initialized.");
  }

 private:
  // Handle new goal requests
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const SavePointCloud::Goal> goal) {
    (void)uuid; // Unused
    if (!goal->start_saving) {
      RCLCPP_INFO(this->get_logger(), "Received goal with start_saving=false, rejecting");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (current_goal_handle_ != nullptr) {
      RCLCPP_INFO(this->get_logger(), "Another goal is being processed, rejecting new goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepted goal to save point cloud");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle goal cancellation
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SavePointCloud>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancellation request, accepting");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle accepted goals
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SavePointCloud>> goal_handle) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_handle_ = goal_handle;
  }

  // Process incoming point clouds
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<SavePointCloud>> goal_handle;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (current_goal_handle_ == nullptr) {
        return; // No active goal, skip processing
      }
      goal_handle = current_goal_handle_;
    }

    // Feedback: Point cloud received
    auto feedback = std::make_shared<SavePointCloud::Feedback>();
    feedback->status = "received point cloud";
    goal_handle->publish_feedback(feedback);

    // Step 1: Convert ROS PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Step 2: Filter points based on distance from sensor
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    filtered_cloud->header = cloud.header;
    double distance_limit_sq = distance_limit_ * distance_limit_;
    for (const auto& point : cloud.points) {
      double dist_sq = point.x * point.x + point.y * point.y + point.z * point.z;
      if (dist_sq <= distance_limit_sq) {
        filtered_cloud->points.push_back(point);
      }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    // Feedback: Filtered point cloud
    feedback->status = "filtered point cloud";
    goal_handle->publish_feedback(feedback);

    // Step 3: Convert filtered cloud back to ROS message for transformation
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header = msg->header;

    // Step 4: Transform to target frame
    sensor_msgs::msg::PointCloud2 transformed_msg;
    try {
      tf_buffer_->transform(filtered_msg, transformed_msg, target_frame_, tf2::Duration(std::chrono::seconds(1)));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
      auto result = std::make_shared<SavePointCloud::Result>();
      result->success = false;
      goal_handle->succeed(result);
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = nullptr;
      }
      return;
    }

    // Feedback: Transformed point cloud
    feedback->status = "transformed point cloud";
    goal_handle->publish_feedback(feedback);

    // Step 5: Convert transformed message back to PCL format
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::fromROSMsg(transformed_msg, transformed_cloud);

    // Step 6: Filter points below base_link’s z-axis threshold
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    final_cloud->header = transformed_cloud.header;
    for (const auto& point : transformed_cloud.points) {
      if (point.z >= -distance_below_) {
        final_cloud->points.push_back(point);
      }
    }
    final_cloud->width = final_cloud->points.size();
    final_cloud->height = 1;
    final_cloud->is_dense = true;

    // Feedback: Filtered below threshold
    feedback->status = "filtered below threshold";
    goal_handle->publish_feedback(feedback);

    // Step 7: Save the final filtered point cloud
    std::string filename = save_directory_ + "pointcloud_" + std::to_string(file_counter_++) + ".pcd";
    bool save_success = false;
    for (int i = 0; i <= save_retries_; ++i) {
      if (pcl::io::savePCDFileBinary(filename, *final_cloud) == 0) {
        RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.c_str());
        save_success = true;
        break;
      } else if (i == save_retries_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s after %d retries",
                     filename.c_str(), save_retries_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Retry %d/%d: Failed to save point cloud to %s",
                    i + 1, save_retries_, filename.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    // Feedback: Saved or failed to save
    feedback->status = save_success ? "saved point cloud" : "failed to save point cloud";
    goal_handle->publish_feedback(feedback);

    // Set result and complete the action
    auto result = std::make_shared<SavePointCloud::Result>();
    result->success = save_success;
    goal_handle->succeed(result);

    // Reset the goal handle
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_goal_handle_ = nullptr;
    }
  }

  // Parameters
  std::string target_frame_;
  int save_retries_;
  double distance_limit_;
  double distance_below_;
  std::string save_directory_;

  // State variables
  int file_counter_;

  // ROS and TF2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Action server components
  rclcpp_action::Server<SavePointCloud>::SharedPtr action_server_;
  std::mutex goal_mutex_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<SavePointCloud>> current_goal_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}