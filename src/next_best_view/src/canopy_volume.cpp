#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

class CanopyVolumeNode : public rclcpp::Node {
 public:
  CanopyVolumeNode() : Node("canopy_volume_node") {
    // Define the path to the PCD file
    std::string pcd_file =
        "/home/mmelihtoslak/NextBestView_final/NextBestView/src/next_best_view/"
        "point_clouds/merged_pointcloud.pcd";  // Update with your file name

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s",
                   pcd_file.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded PCD file with %ld points.",
                cloud->size());

    // Extract the canopy
    auto canopy_cloud = extractCanopy(cloud);

    // Calculate the canopy volume
    double volume = calculateConvexHullVolume(canopy_cloud);
    RCLCPP_INFO(this->get_logger(), "Canopy Volume: %.4f cubic meters", volume);
  }

 private:
  // Function to extract the canopy
  pcl::PointCloud<pcl::PointXYZ>::Ptr extractCanopy(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // Step 1: Isolate the upper region
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");     // Filter along the Z-axis (height)
    pass.setFilterLimits(0.01, 0.4);  // Adjust these limits based on your data
    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*upper_cloud);

    // Step 2: Extract the canopy boundary using Convex Hull
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(upper_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr canopy_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*canopy_cloud);

    return canopy_cloud;
  }

  // Function to calculate the volume of the convex hull
  double calculateConvexHullVolume(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->size() < 4) {
      RCLCPP_ERROR(this->get_logger(),
                   "Not enough points to form a convex hull.");
      return 0.0;
    }

    // Compute the centroid of the point cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    // Calculate the volume of the convex hull
    double volume = 0.0;
    for (size_t i = 0; i < cloud->size(); ++i) {
      Eigen::Vector3f point = cloud->points[i].getVector3fMap();
      Eigen::Vector3f vec = point - centroid.head<3>();
      volume += vec.dot(vec);  // Sum of squared distances from the centroid
    }

    // Normalize the volume
    volume /= cloud->size();
    return volume;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanopyVolumeNode>());
  rclcpp::shutdown();
  return 0;
}
