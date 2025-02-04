#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class PointCloudMergerNode : public rclcpp::Node {
 public:
  PointCloudMergerNode() : Node("pointcloud_merger_node") {
    // Set the directory containing the saved point clouds
    save_directory_ =
        "/home/mmelihtoslak/NextBestView_final/NextBestView/src/next_best_view/"
        "point_clouds/";

    // Set the output file name for the merged point cloud
    output_filename_ = save_directory_ + "merged_pointcloud.pcd";

    // Merge all point clouds in the directory
    mergePointClouds();
  }

 private:
  void mergePointClouds() {
    // Create a container for the merged point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // Iterate over all files in the directory
    for (const auto& entry :
         std::filesystem::directory_iterator(save_directory_)) {
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

    // Save the merged point cloud to a file
    if (pcl::io::savePCDFileASCII(output_filename_, *merged_cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to save merged point cloud to: %s",
                   output_filename_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Merged point cloud saved to: %s",
                  output_filename_.c_str());
    }
  }

  std::string save_directory_;   // Directory containing the saved point clouds
  std::string output_filename_;  // Output file name for the merged point cloud
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
