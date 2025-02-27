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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>  // For std::chrono literals

namespace fs = std::filesystem;

class PointCloudSaverNode : public rclcpp::Node {
 public:
  PointCloudSaverNode() : Node("pointcloud_saver_node"), save_data_(false), file_counter_(0) {
    // Declare parameters with default values
    this->declare_parameter("save_interval", 1.0);  // Time in seconds between saves
    this->declare_parameter("target_frame", "base_link");  // Target frame for transformation
    this->declare_parameter("save_retries", 3);  // Number of retries for saving

    // Retrieve parameter values
    save_interval_ = this->get_parameter("save_interval").as_double();
    target_frame_ = this->get_parameter("target_frame").as_string();
    save_retries_ = this->get_parameter("save_retries").as_int();

    // Determine the save directory: one level above the source file's directory
    fs::path source_file_path = fs::path(__FILE__);
    fs::path parent_dir = source_file_path.parent_path().parent_path();  // One level up
    fs::path save_dir = parent_dir / "point_clouds";

    // Check if the directory exists, create it if it doesn't
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

    // Define QoS profile: Best Effort, Keep Last with depth 5
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    // Subscriptions
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", qos, std::bind(&PointCloudSaverNode::pointcloudCallback, this, std::placeholders::_1));
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/save_trigger", 10, std::bind(&PointCloudSaverNode::triggerCallback, this, std::placeholders::_1));

    // Initialize last_save_time_ with the node's clock time
    last_save_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "PointCloudSaverNode initialized.");
  }

 private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received point cloud message");
    if (!save_data_) {
      return; // Skip if not triggered to save
    }

    // Check if enough time has passed since the last save
    rclcpp::Time current_time = this->now();
    if ((current_time - last_save_time_).seconds() < save_interval_) {
      return; // Skip if interval hasn't elapsed
    }

    // Convert ROS PointCloud2 to PCL PointCloud with RGB
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Transform the point cloud to the target frame
    sensor_msgs::msg::PointCloud2 transformed_msg;
    try {
      tf_buffer_->transform(*msg, transformed_msg, target_frame_, tf2::Duration(std::chrono::seconds(1)));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
      return;
    }

    // Convert transformed message back to PCL format with RGB
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::fromROSMsg(transformed_msg, transformed_cloud);

    // Generate filename
    std::string filename = save_directory_ + "pointcloud_" + std::to_string(file_counter_++) + ".pcd";

    // Save with retries
    for (int i = 0; i <= save_retries_; ++i) {
      if (pcl::io::savePCDFileBinary(filename, transformed_cloud) == 0) {
        RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.c_str());
        last_save_time_ = this->now(); // Update with node's clock time
        break;
      } else if (i == save_retries_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s after %d retries", filename.c_str(), save_retries_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Retry %d/%d: Failed to save point cloud to %s", i + 1, save_retries_, filename.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Brief delay before retry
      }
    }
  }

  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    save_data_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Save trigger set to %s", save_data_ ? "true" : "false");
  }

  // Parameters
  double save_interval_;            // Minimum time between saves (seconds)
  std::string target_frame_;        // Frame to transform point cloud to
  int save_retries_;                // Number of retries for saving
  std::string save_directory_;      // Fixed directory to save point cloud files

  // State variables
  bool save_data_;                  // Whether to save point clouds
  rclcpp::Time last_save_time_;     // Timestamp of the last save
  int file_counter_;                // Counter for unique filenames

  // ROS and TF2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}