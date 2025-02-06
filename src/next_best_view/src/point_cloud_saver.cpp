#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For transforming individual points

class PointCloudSaverNode : public rclcpp::Node {
 public:
  PointCloudSaverNode() : Node("pointcloud_saver_node"), save_data_(false) {
    // Define QoS settings to match the publisher (best-effort)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // Subscribe to the PointCloud2 topic with the defined QoS
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", qos,
        std::bind(&PointCloudSaverNode::pointcloudCallback, this,
                  std::placeholders::_1));

    // Subscribe to the boolean trigger topic
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/save_trigger", 10,
        std::bind(&PointCloudSaverNode::triggerCallback, this,
                  std::placeholders::_1));

    // Set the save directory (change this to your desired path)
    save_directory_ =
        "src/next_best_view/"
        "point_clouds/";

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

 private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (save_data_) {
      // Convert ROS PointCloud2 to PCL PointCloud (with color)
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::fromROSMsg(*msg, cloud);

      // Transform each point to the base frame
      pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
      try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(
                "base_link",           // Target frame (base frame)
                msg->header.frame_id,  // Source frame (camera frame)
                msg->header.stamp);

        for (const auto& point : cloud.points) {
          geometry_msgs::msg::PointStamped point_in, point_out;
          point_in.point.x = point.x;
          point_in.point.y = point.y;
          point_in.point.z = point.z;
          point_in.header = msg->header;

          // Transform the point
          tf2::doTransform(point_in, point_out, transform);

          // Add the transformed point to the new cloud (with color)
          pcl::PointXYZRGB transformed_point;
          transformed_point.x = point_out.point.x;
          transformed_point.y = point_out.point.y;
          transformed_point.z = point_out.point.z;
          transformed_point.r = point.r;  // Copy the color
          transformed_point.g = point.g;
          transformed_point.b = point.b;
          transformed_cloud.push_back(transformed_point);
        }
      } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "TF2 transform error: %s", ex.what());
        return;
      }

      // Filter points that are more than 1 meter away from the camera
      pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
      for (const auto& point : transformed_cloud.points) {
        if (std::sqrt(point.x * point.x + point.y * point.y +
                      point.z * point.z) <= 1.0) {
          filtered_cloud.push_back(point);
        }
      }

      // Save the filtered point cloud to a file in the specified directory
      std::string filename = save_directory_ + "pointcloud_" +
                             std::to_string(file_counter_++) + ".pcd";
      pcl::io::savePCDFileASCII(filename, filtered_cloud);
      RCLCPP_INFO(this->get_logger(), "Saved filtered point cloud to %s",
                  filename.c_str());
    }
  }

  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    save_data_ = msg->data;
    if (!save_data_) {
      RCLCPP_INFO(this->get_logger(), "Stopped saving point clouds.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Started saving point clouds.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool save_data_;
  int file_counter_ = 0;
  std::string save_directory_;  // Directory to save the point cloud files
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSaverNode>());
  rclcpp::shutdown();
  return 0;
}
