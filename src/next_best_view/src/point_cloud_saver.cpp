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
#include <chrono>

namespace fs = std::filesystem;

class PointCloudSaverNode : public rclcpp::Node {
 public:
  PointCloudSaverNode() : Node("pointcloud_saver_node"), save_data_(false), file_counter_(0) {
    // Declare parameters with default values
    this->declare_parameter("save_interval", 3.0);    // Time in seconds between saves
    this->declare_parameter("target_frame", "base_link");  // Target frame for transformation
    this->declare_parameter("save_retries", 3);       // Number of retries for saving
    this->declare_parameter("distance_limit", 1.5);  // Max distance from sensor in meters
    this->declare_parameter("distance_below", 0.0);   // Threshold below base_link z-axis in meters

    // Retrieve parameter values
    save_interval_ = this->get_parameter("save_interval").as_double();
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

    // Create directory if it doesn't exist
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

    // Subscriptions
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", qos,
        std::bind(&PointCloudSaverNode::pointcloudCallback, this, std::placeholders::_1));
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/save_trigger", 10,
        std::bind(&PointCloudSaverNode::triggerCallback, this, std::placeholders::_1));

    last_save_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "PointCloudSaverNode initialized.");
  }

 private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!save_data_) {
      return; // Skip if not triggered to save
    }

    rclcpp::Time current_time = this->now();
    if ((current_time - last_save_time_).seconds() < save_interval_) {
      return; // Skip if interval hasn't elapsed
    }

    // Step 1: Convert ROS PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Step 2: Filter points based on distance from sensor (camera_color_frame)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    filtered_cloud->header = cloud.header;
    double distance_limit_sq = distance_limit_ * distance_limit_; // Compare squared distances for efficiency
    for (const auto& point : cloud.points) {
      double dist_sq = point.x * point.x + point.y * point.y + point.z * point.z;
      if (dist_sq <= distance_limit_sq) { // Keep points within distance_limit
        filtered_cloud->points.push_back(point);
      }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    // Step 3: Convert filtered cloud back to ROS message for transformation
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header = msg->header; // Preserve original header for transformation

    // Step 4: Transform to target frame (base_link)
    sensor_msgs::msg::PointCloud2 transformed_msg;
    try {
      tf_buffer_->transform(filtered_msg, transformed_msg, target_frame_, tf2::Duration(std::chrono::seconds(1)));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
      return;
    }

    // Step 5: Convert transformed message back to PCL format
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::fromROSMsg(transformed_msg, transformed_cloud);

    // Step 6: Filter points below base_link's z-axis threshold
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    final_cloud->header = transformed_cloud.header;
    for (const auto& point : transformed_cloud.points) {
      if (point.z >= -distance_below_) { // Keep points above -distance_below
        final_cloud->points.push_back(point);
      }
    }
    final_cloud->width = final_cloud->points.size();
    final_cloud->height = 1;
    final_cloud->is_dense = true;

    // Step 7: Save the final filtered point cloud
    std::string filename = save_directory_ + "pointcloud_" + std::to_string(file_counter_++) + ".pcd";
    for (int i = 0; i <= save_retries_; ++i) {
      if (pcl::io::savePCDFileBinary(filename, *final_cloud) == 0) {
        RCLCPP_INFO(this->get_logger(), "Saved point cloud to %s", filename.c_str());
        last_save_time_ = this->now();
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
  }

  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    save_data_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Save trigger set to %s", save_data_ ? "true" : "false");
  }

  // Parameters
  double save_interval_;        // Minimum time between saves (seconds)
  std::string target_frame_;    // Frame to transform point cloud to
  int save_retries_;            // Number of retries for saving
  double distance_limit_;       // Max distance from sensor (meters)
  double distance_below_;       // Threshold below base_link z-axis (meters)
  std::string save_directory_;  // Directory to save point cloud files

  // State variables
  bool save_data_;              // Whether to save point clouds
  rclcpp::Time last_save_time_; // Timestamp of the last save
  int file_counter_;            // Counter for unique filenames

  // ROS and TF2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}