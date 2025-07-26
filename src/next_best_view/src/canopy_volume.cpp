#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <regex>

// Define the action (replace "kinova_action_interfaces" with your package name)
#include "kinova_action_interfaces/action/process_tree_data.hpp"

using ProcessTreeData = kinova_action_interfaces::action::ProcessTreeData;
namespace fs = std::filesystem;

class CanopyVolumeNode : public rclcpp::Node {
 public:
  CanopyVolumeNode() : Node("canopy_volume_node") {
    // Declare parameters with default values
    this->declare_parameter<double>("filter_min_z", 0.0);
    this->declare_parameter<double>("filter_max_z", 0.6);

    // Retrieve parameter values and store them in member variables
    filter_min_z_ = this->get_parameter("filter_min_z").as_double();
    filter_max_z_ = this->get_parameter("filter_max_z").as_double();

    // Log the loaded values for debugging
    RCLCPP_INFO(this->get_logger(), "Using filter limits: min_z=%.2f, max_z=%.2f", 
                filter_min_z_, filter_max_z_);

    // Set directories
    merged_directory_ = "src/next_best_view/merged_pointclouds/";
    info_directory_ = "src/next_best_view/tree_informations/";

    // Create tree_informations directory if it doesn’t exist
    if (!fs::exists(info_directory_)) {
      if (fs::create_directory(info_directory_)) {
        RCLCPP_INFO(this->get_logger(), "Created directory: %s", info_directory_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", info_directory_.c_str());
      }
    }

    // Initialize action server
    action_server_ = rclcpp_action::create_server<ProcessTreeData>(
        this,
        "process_tree_data",
        std::bind(&CanopyVolumeNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanopyVolumeNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&CanopyVolumeNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "CanopyVolumeNode action server started");
  }

 private:
  /** Handle incoming goals */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ProcessTreeData::Goal> goal) {
    (void)uuid;
    if (!goal->start_processing) {
      RCLCPP_INFO(this->get_logger(), "Goal rejected: start_processing is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /** Handle cancellation */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessTreeData>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /** Execute accepted goal */
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessTreeData>> goal_handle) {
    std::thread{std::bind(&CanopyVolumeNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  /** Process tree data */
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ProcessTreeData>> goal_handle) {
    auto feedback = std::make_shared<ProcessTreeData::Feedback>();
    auto result = std::make_shared<ProcessTreeData::Result>();

    // Check merged directory
    if (!fs::exists(merged_directory_) || !fs::is_directory(merged_directory_)) {
      feedback->status_message = "Merged directory not found";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    // Find all tree_*.pcd files
    std::vector<fs::path> pcd_files;
    std::regex pattern(R"(tree_\d+\.pcd)");
    for (const auto& entry : fs::directory_iterator(merged_directory_)) {
      if (entry.is_regular_file() && std::regex_match(entry.path().filename().string(), pattern)) {
        pcd_files.push_back(entry.path());
      }
    }

    if (pcd_files.empty()) {
      feedback->status_message = "No tree_*.pcd files found";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    // Create CSV file
    std::string csv_filename = info_directory_ + "tree_data_" + std::to_string(getNextFileIndex()) + ".csv";
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
      feedback->status_message = "Failed to create CSV file";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    // Write CSV header
    csv_file << "Tree File,Canopy Volume (m³),Tree Height (m),Canopy Diameter (m),Canopy Area (m²),Canopy Density (points/m³)\n";

    // Process each file
    for (const auto& pcd_file : pcd_files) {
      if (goal_handle->is_canceling()) {
        feedback->status_message = "Processing canceled";
        goal_handle->publish_feedback(feedback);
        csv_file.close();
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      std::string filename = pcd_file.filename().string();
      feedback->status_message = "Processing " + filename;
      goal_handle->publish_feedback(feedback);

      // Load point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file.string(), *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load %s", pcd_file.string().c_str());
        continue;
      }

      // Calculate metrics
      auto canopy_cloud = extractCanopy(cloud);
      double volume = calculateConvexHullVolume(canopy_cloud);
      float height = calculateTreeHeight(cloud);
      float diameter = calculateCanopyDiameter(canopy_cloud);
      float area = calculateCanopyArea(canopy_cloud);
      float density = calculateCanopyDensity(canopy_cloud, volume);

      // Write to CSV
      csv_file << filename << "," << volume << "," << height << "," << diameter << "," << area << "," << density << "\n";
    }

    csv_file.close();
    feedback->status_message = "Processing completed, saved to " + csv_filename;
    goal_handle->publish_feedback(feedback);
    result->success = true;
    goal_handle->succeed(result);
  }

  /** Get next file index for CSV */
  int getNextFileIndex() {
    int index = 0;
    std::regex pattern(R"(tree_data_(\d+)\.csv)");
    for (const auto& entry : fs::directory_iterator(info_directory_)) {
      std::string filename = entry.path().filename().string();
      std::smatch match;
      if (std::regex_match(filename, match, pattern)) {
        int file_index = std::stoi(match[1].str());
        index = std::max(index, file_index + 1);
      }
    }
    return index;
  }

  /** Extract canopy */
  pcl::PointCloud<pcl::PointXYZ>::Ptr extractCanopy(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_min_z_, filter_max_z_);  // Use configurable member variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*upper_cloud);

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(upper_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr canopy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*canopy_cloud);
    return canopy_cloud;
  }

  /** Calculate convex hull volume */
  double calculateConvexHullVolume(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "Not enough points for convex hull");
      return 0.0;
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    double volume = 0.0;
    for (const auto& point : *cloud) {
      Eigen::Vector3f vec = point.getVector3fMap() - centroid.head<3>();
      volume += vec.dot(vec);
    }
    volume /= cloud->size();
    return volume;
  }

  /** Calculate tree height */
  float calculateTreeHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min();
    for (const auto& point : *cloud) {
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
    }
    return max_z - min_z;
  }

  /** Calculate canopy diameter */
  float calculateCanopyDiameter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& canopy_cloud) {
    float max_diameter = 0.0;
    for (size_t i = 0; i < canopy_cloud->size(); ++i) {
      for (size_t j = i + 1; j < canopy_cloud->size(); ++j) {
        float dx = canopy_cloud->points[i].x - canopy_cloud->points[j].x;
        float dy = canopy_cloud->points[i].y - canopy_cloud->points[j].y;
        float distance = std::sqrt(dx * dx + dy * dy);
        max_diameter = std::max(max_diameter, distance);
      }
    }
    return max_diameter;
  }

  /** Calculate canopy area */
  float calculateCanopyArea(const pcl::PointCloud<pcl::PointXYZ>::Ptr& canopy_cloud) {
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(canopy_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*hull_cloud);
    
    float area = 0.0;
    for (size_t i = 1; i < hull_cloud->size() - 1; ++i) {
      float x1 = hull_cloud->points[i].x - hull_cloud->points[0].x;
      float y1 = hull_cloud->points[i].y - hull_cloud->points[0].y;
      float x2 = hull_cloud->points[i + 1].x - hull_cloud->points[0].x;
      float y2 = hull_cloud->points[i + 1].y - hull_cloud->points[0].y;
      area += (x1 * y2 - x2 * y1);
    }
    return std::abs(area) / 2.0;
  }

  /** Calculate canopy density */
  float calculateCanopyDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& canopy_cloud, double volume) {
    return (volume > 0.0) ? canopy_cloud->size() / volume : 0.0;
  }

  // Member variables for directories and action server
  std::string merged_directory_;
  std::string info_directory_;
  rclcpp_action::Server<ProcessTreeData>::SharedPtr action_server_;

  // New member variables for configurable filter limits
  double filter_min_z_;
  double filter_max_z_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanopyVolumeNode>());
  rclcpp::shutdown();
  return 0;
}
