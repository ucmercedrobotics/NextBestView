#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// New PCL headers for plane segmentation and voxel grid filtering
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <chrono>
#include <mutex>

#include "kinova_action_interfaces/action/save_point_cloud.hpp"

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

    RCLCPP_INFO(this->get_logger(), "Initialized with distance_limit: %.2f meters, distance_below: %.2f meters",
                distance_limit_, distance_below_);

    // Set up save directory
    fs::path source_file_path = fs::path(__FILE__);
    fs::path parent_dir = source_file_path.parent_path().parent_path();
    fs::path save_dir = parent_dir / "point_clouds";
    if (!fs::exists(save_dir)) {
      fs::create_directories(save_dir);
      RCLCPP_INFO(this->get_logger(), "Created point_clouds directory: %s", save_dir.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Using existing point_clouds directory: %s", save_dir.c_str());
    }
    save_directory_ = save_dir.string() + "/";

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Subscription
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", qos,
        std::bind(&PointCloudSaverNode::pointcloudCallback, this, std::placeholders::_1));

    // Action server
    action_server_ = rclcpp_action::create_server<SavePointCloud>(
        this, "save_pointcloud",
        std::bind(&PointCloudSaverNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PointCloudSaverNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&PointCloudSaverNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PointCloudSaverNode initialized.");
  }

 private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const SavePointCloud::Goal> goal) {
    (void)uuid;
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

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SavePointCloud>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancellation request, accepting");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SavePointCloud>> goal_handle) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    current_goal_handle_ = goal_handle;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::shared_ptr<rclcpp_action::ServerGoalHandle<SavePointCloud>> goal_handle;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (current_goal_handle_ == nullptr) return;
      goal_handle = current_goal_handle_;
    }

    auto feedback = std::make_shared<SavePointCloud::Feedback>();
    feedback->status = "received point cloud";
    goal_handle->publish_feedback(feedback);

    // Step 1: Convert to PCL
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);
    RCLCPP_INFO(this->get_logger(), "Initial points: %zu", cloud.points.size());

    // Step 2: Filter by distance from sensor
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
    RCLCPP_INFO(this->get_logger(), "After distance filter: %zu points", filtered_cloud->points.size());

    feedback->status = "filtered point cloud";
    goal_handle->publish_feedback(feedback);

    // Step 3: Transform to target frame
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header = msg->header;
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

    feedback->status = "transformed point cloud";
    goal_handle->publish_feedback(feedback);

    // Step 4: Convert back to PCL and filter by z-threshold
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::fromROSMsg(transformed_msg, transformed_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    z_filtered_cloud->header = transformed_cloud.header;
    for (const auto& point : transformed_cloud.points) {
      if (point.z >= -distance_below_) {
        z_filtered_cloud->points.push_back(point);
      }
    }
    z_filtered_cloud->width = z_filtered_cloud->points.size();
    z_filtered_cloud->height = 1;
    z_filtered_cloud->is_dense = true;
    RCLCPP_INFO(this->get_logger(), "After z-filter: %zu points", z_filtered_cloud->points.size());

    feedback->status = "filtered below threshold";
    goal_handle->publish_feedback(feedback);

    // Step 5: Plane segmentation to remove largest plane (e.g., table or floor)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005); // 1cm, tune this based on noise level
    seg.setInputCloud(z_filtered_cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "No plane found, keeping original cloud");
      *cloud_no_plane = *z_filtered_cloud;
    } else {
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(z_filtered_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true); // Keep points not on the plane
      extract.filter(*cloud_no_plane);
      RCLCPP_INFO(this->get_logger(), "After plane removal: %zu points", cloud_no_plane->points.size());
    }

    feedback->status = "plane removed";
    goal_handle->publish_feedback(feedback);

    // Step 6: Voxel grid filtering to downsample and reduce noise
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud_no_plane);
    voxel_grid.setLeafSize(0.001f, 0.001f, 0.001f); // 1cm voxels, adjust as needed
    voxel_grid.filter(*final_cloud);
    RCLCPP_INFO(this->get_logger(), "After voxel grid: %zu points", final_cloud->points.size());

    feedback->status = "downsampled point cloud";
    goal_handle->publish_feedback(feedback);

    // Step 7: Check if final cloud is empty
    if (final_cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Final point cloud is empty after filtering");
      auto result = std::make_shared<SavePointCloud::Result>();
      result->success = false;
      goal_handle->succeed(result);
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = nullptr;
      }
      return;
    }

    // Step 8: Save the final point cloud
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

    feedback->status = save_success ? "saved point cloud" : "failed to save point cloud";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<SavePointCloud::Result>();
    result->success = save_success;
    goal_handle->succeed(result);

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_goal_handle_ = nullptr;
    }
  }

  std::string target_frame_;
  int save_retries_;
  double distance_limit_;
  double distance_below_;
  std::string save_directory_;
  int file_counter_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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