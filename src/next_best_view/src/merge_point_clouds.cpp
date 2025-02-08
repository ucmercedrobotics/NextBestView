#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

// Create an alias for the filesystem namespace for brevity.
namespace fs = std::filesystem;

class PointCloudMergerNode : public rclcpp::Node {
 public:
  PointCloudMergerNode() : Node("pointcloud_merger_node") {
    // Set the directory containing the individual saved point clouds
    save_directory_ = "src/next_best_view/point_clouds/";

    // Set the directory where the merged point cloud will be saved (one level
    // up)
    merged_directory_ = "src/next_best_view/merged_pointclouds/";

    // Check if the merged directory exists; if not, create it.
    if (!fs::exists(merged_directory_)) {
      if (fs::create_directory(merged_directory_)) {
        RCLCPP_INFO(this->get_logger(),
                    "Created merged point clouds directory: %s",
                    merged_directory_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create merged point clouds directory: %s",
                     merged_directory_.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Merged point clouds directory already exists: %s",
                  merged_directory_.c_str());
    }

    // Set the output file name for the merged point cloud in the merged
    // directory
    output_filename_ = merged_directory_ + "merged_pointcloud.pcd";

    // Merge all point clouds from the source directory
    mergePointClouds();
  }

 private:
  void mergePointClouds() {
    // Create a container for the merged point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // Iterate over all files in the source directory (../point_clouds/)
    for (const auto& entry : fs::directory_iterator(save_directory_)) {
      if (entry.path().extension() == ".pcd") {
        // Load the point cloud from the file
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(entry.path().string(),
                                                   *cloud) == -1) {
          RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s",
                       entry.path().string().c_str());
          continue;
        }

        // Add the loaded point cloud to the merged cloud
        *merged_cloud += *cloud;
        RCLCPP_INFO(this->get_logger(), "Loaded and added point cloud from: %s",
                    entry.path().string().c_str());
      }
    }

    // Check if any point clouds were loaded
    if (merged_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No point clouds found in the directory: %s",
                   save_directory_.c_str());
      return;
    }

    // Save the merged point cloud to a file in the merged directory
    if (pcl::io::savePCDFileASCII(output_filename_, *merged_cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to save merged point cloud to: %s",
                   output_filename_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Merged point cloud saved to: %s",
                  output_filename_.c_str());
    }
  }

  std::string
      save_directory_;  // Directory containing individual saved point clouds
  std::string merged_directory_;  // Directory to store the merged point cloud
  std::string output_filename_;   // Output file name for the merged point cloud
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
