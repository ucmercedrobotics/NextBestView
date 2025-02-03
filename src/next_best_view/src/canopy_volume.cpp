#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

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

    // Calculate tree height
    float tree_height = calculateTreeHeight(cloud);
    RCLCPP_INFO(this->get_logger(), "Tree Height: %.4f meters", tree_height);

    // Calculate canopy diameter
    float canopy_diameter = calculateCanopyDiameter(canopy_cloud);
    RCLCPP_INFO(this->get_logger(), "Canopy Diameter: %.4f meters",
                canopy_diameter);

    // Calculate canopy area
    float canopy_area = calculateCanopyArea(canopy_cloud);
    RCLCPP_INFO(this->get_logger(), "Canopy Area: %.4f square meters",
                canopy_area);

    // Calculate canopy density
    float canopy_density = calculateCanopyDensity(canopy_cloud, volume);
    RCLCPP_INFO(this->get_logger(),
                "Canopy Density: %.4f points per cubic meter", canopy_density);
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

  // Function to calculate tree height
  float calculateTreeHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min();

    for (const auto& point : *cloud) {
      if (point.z < min_z) min_z = point.z;
      if (point.z > max_z) max_z = point.z;
    }

    return max_z - min_z;
  }

  // Function to calculate canopy diameter
  float calculateCanopyDiameter(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& canopy_cloud) {
    float max_diameter = 0.0;

    for (size_t i = 0; i < canopy_cloud->size(); ++i) {
      for (size_t j = i + 1; j < canopy_cloud->size(); ++j) {
        float dx = canopy_cloud->points[i].x - canopy_cloud->points[j].x;
        float dy = canopy_cloud->points[i].y - canopy_cloud->points[j].y;
        float distance = std::sqrt(dx * dx + dy * dy);
        if (distance > max_diameter) {
          max_diameter = distance;
        }
      }
    }

    return max_diameter;
  }

  // Function to calculate canopy area
  float calculateCanopyArea(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& canopy_cloud) {
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(canopy_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*hull_cloud);

    float area = 0.0;
    for (size_t i = 1; i < hull_cloud->size() - 1; ++i) {
      float x1 = hull_cloud->points[i].x - hull_cloud->points[0].x;
      float y1 = hull_cloud->points[i].y - hull_cloud->points[0].y;
      float x2 = hull_cloud->points[i + 1].x - hull_cloud->points[0].x;
      float y2 = hull_cloud->points[i + 1].y - hull_cloud->points[0].y;
      area += (x1 * y2 - x2 * y1);
    }
    area = std::abs(area) / 2.0;

    return area;
  }

  // Function to calculate canopy density
  float calculateCanopyDensity(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& canopy_cloud, double volume) {
    if (volume <= 0.0) return 0.0;
    return canopy_cloud->size() / volume;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanopyVolumeNode>());
  rclcpp::shutdown();
  return 0;
}
