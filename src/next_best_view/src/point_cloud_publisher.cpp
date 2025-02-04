#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudPublisher : public rclcpp::Node {
 public:
  PointCloudPublisher() : Node("pcd_publisher") {
    // Create publisher for point cloud
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);

    // Timer to publish point cloud periodically
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PointCloudPublisher::publishPointCloud, this));
  }

 private:
  void publishPointCloud() {
    // Load PCD file with RGB information
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // Replace with your actual .pcd file path
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(
            "/home/mmelihtoslak/NextBestView_final/NextBestView/src/"
            "next_best_view/point_clouds/merged_pointcloud.pcd",
            *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file");
      return;
    }

    // Convert PCL point cloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);

    // Set frame ID (important for RViz visualization)
    ros_cloud.header.frame_id = "base_link";
    ros_cloud.header.stamp = this->get_clock()->now();

    // Publish the point cloud
    publisher_->publish(ros_cloud);
    RCLCPP_INFO(this->get_logger(), "Published point cloud");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and spin the node
  auto node = std::make_shared<PointCloudPublisher>();
  rclcpp::spin(node);

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
