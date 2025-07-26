#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <memory>

class OctomapPublisher : public rclcpp::Node {
public:
  OctomapPublisher() 
  : Node("octomap_publisher"), 
    octree_(std::make_shared<octomap::OcTree>(0.05)),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare parameters
    declare_parameter<double>("resolution", 0.05);
    declare_parameter<double>("max_range", 5.0);
    declare_parameter<double>("occupancy_threshold", 0.7);
    declare_parameter<double>("hit_probability", 0.7);
    declare_parameter<double>("miss_probability", 0.4);
    declare_parameter<std::string>("sensor_frame", "camera_color_frame");

    // Set Octomap parameters
    octree_->setResolution(get_parameter("resolution").as_double());
    octree_->setOccupancyThres(get_parameter("occupancy_threshold").as_double());
    octree_->setProbHit(get_parameter("hit_probability").as_double());
    octree_->setProbMiss(get_parameter("miss_probability").as_double());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    // Point cloud subscription
    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", qos,
      std::bind(&OctomapPublisher::pointcloud_callback, this, std::placeholders::_1));

    // Octomap points publisher
    octomap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "octomap_points", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "OctoMap publisher initialized");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      const std::string sensor_frame = get_parameter("sensor_frame").as_string();
      geometry_msgs::msg::TransformStamped transform;

      // Get transform from sensor frame to base_link
      try {
        transform = tf_buffer_.lookupTransform("base_link", sensor_frame, msg->header.stamp);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Transform unavailable: %s", ex.what());
        return;
      }

      // Convert to Eigen transform
      Eigen::Affine3d sensor_to_base = tf2::transformToEigen(transform);
      octomap::point3d sensor_origin(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
      );

      // Process point cloud
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);

      const auto max_range = get_parameter("max_range").as_double();
      size_t added_points = 0;

      // Insert points into OctoMap
      for (const auto& pt : cloud) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        Eigen::Vector3d point_in_sensor(pt.x, pt.y, pt.z);
        Eigen::Vector3d point_in_base = sensor_to_base * point_in_sensor;
        
        octomap::point3d endpoint(point_in_base.x(), point_in_base.y(), point_in_base.z());
        octree_->insertRay(sensor_origin, endpoint, max_range);
        added_points++;
      }

      // Update and prune octree
      octree_->updateInnerOccupancy();
      octree_->prune();

      // Publish occupied voxels as point cloud
      pcl::PointCloud<pcl::PointXYZ> occupied_voxels;
      for (octomap::OcTree::iterator it = octree_->begin(); it != octree_->end(); ++it) {
        if (octree_->isNodeOccupied(*it)) {
          occupied_voxels.push_back(pcl::PointXYZ(
            it.getX(),
            it.getY(),
            it.getZ()
          ));
        }
      }

      // Convert to ROS message and publish
      sensor_msgs::msg::PointCloud2 pc_msg;
      pcl::toROSMsg(occupied_voxels, pc_msg);
      pc_msg.header.stamp = now();
      pc_msg.header.frame_id = "base_link";
      octomap_pub_->publish(pc_msg);

      RCLCPP_DEBUG(get_logger(), "Processed %zu points, published %zu occupied voxels",
                 added_points, occupied_voxels.size());
    } 
    catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Error processing point cloud: %s", e.what());
    }
  }

  std::shared_ptr<octomap::OcTree> octree_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapPublisher>());
  rclcpp::shutdown();
  return 0;
}