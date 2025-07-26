#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>
#include <regex> // Added for regex matching

// Alias for the filesystem namespace
namespace fs = std::filesystem;

// Include the action definition (replace "pointcloud_merger" with your actual package name)
#include "kinova_action_interfaces/action/merge_point_clouds.hpp"

using MergePointClouds = kinova_action_interfaces::action::MergePointClouds;

class PointCloudMergerNode : public rclcpp::Node {
 public:
  PointCloudMergerNode() : Node("pointcloud_merger_node") {
    // Set the directory containing the filtered point clouds
    save_directory_ = "src/next_best_view/filtered_pointclouds/";

    // Set the directory where merged point clouds will be saved
    merged_directory_ = "src/next_best_view/merged_pointclouds/";

    // Ensure the merged directory exists
    if (!fs::exists(merged_directory_)) {
      if (fs::create_directory(merged_directory_)) {
        RCLCPP_INFO(this->get_logger(), "Created merged point clouds directory: %s",
                    merged_directory_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create merged point clouds directory: %s",
                     merged_directory_.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Merged point clouds directory already exists: %s",
                  merged_directory_.c_str());
    }

    // Initialize the action server
    action_server_ = rclcpp_action::create_server<MergePointClouds>(
        this,
        "merge_pointclouds",
        std::bind(&PointCloudMergerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PointCloudMergerNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&PointCloudMergerNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PointCloudMergerNode action server started");
  }

 private:
  /** Handle incoming goals */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MergePointClouds::Goal> goal) {
    (void)uuid;
    if (!goal->start_merging) {
      RCLCPP_INFO(this->get_logger(), "Received goal with start_merging=false, rejecting");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepted goal to merge point clouds");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /** Handle cancellation requests */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MergePointClouds>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancellation request, accepting");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /** Handle accepted goals by starting execution in a new thread */
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MergePointClouds>> goal_handle) {
    std::thread{std::bind(&PointCloudMergerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  /** Execute the merging process */
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MergePointClouds>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal to merge point clouds");
    auto feedback = std::make_shared<MergePointClouds::Feedback>();
    auto result = std::make_shared<MergePointClouds::Result>();

    // Validate the source directory
    if (!fs::exists(save_directory_) || !fs::is_directory(save_directory_)) {
      feedback->status = "Source directory does not exist or is not a directory";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    // Collect all .pcd files
    std::vector<fs::path> pcd_files;
    for (const auto& entry : fs::directory_iterator(save_directory_)) {
      if (entry.path().extension() == ".pcd") {
        pcd_files.push_back(entry.path());
      }
    }

    if (pcd_files.empty()) {
      feedback->status = "No point clouds found in the directory";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    // Initialize the merged point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Merge each point cloud with feedback
    for (const auto& pcd_file : pcd_files) {
      if (goal_handle->is_canceling()) {
        feedback->status = "Goal canceled";
        goal_handle->publish_feedback(feedback);
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file.string(), *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file.string().c_str());
        continue;
      }

      *merged_cloud += *cloud;
      feedback->status = "Loaded point cloud from " + pcd_file.filename().string();
      goal_handle->publish_feedback(feedback);
    }

    if (merged_cloud->empty()) {
      feedback->status = "No points loaded from any point clouds";
      goal_handle->publish_feedback(feedback);
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    // Find the next available tree_<index>.pcd filename
    int next_index = 0;
    std::regex pattern(R"(tree_(\d+)\.pcd)");
    for (const auto& entry : fs::directory_iterator(merged_directory_)) {
      std::string filename = entry.path().filename().string();
      std::smatch match;
      if (std::regex_match(filename, match, pattern)) {
        int index = std::stoi(match[1].str());
        if (index >= next_index) {
          next_index = index + 1;
        }
      }
    }

    // Generate the output filename
    std::string output_filename = merged_directory_ + "tree_" + std::to_string(next_index) + ".pcd";

    // Save the merged point cloud
    if (pcl::io::savePCDFileASCII(output_filename, *merged_cloud) == -1) {
      feedback->status = "Failed to save merged point cloud";
      goal_handle->publish_feedback(feedback);
      result->success = false;
    } else {
      feedback->status = "Saved merged point cloud to " + output_filename;
      goal_handle->publish_feedback(feedback);
      result->success = true;
    }

    // Mark the goal as succeeded and return the result
    goal_handle->succeed(result);
  }

  std::string save_directory_;    // Directory with filtered point clouds
  std::string merged_directory_;  // Directory for merged point clouds
  rclcpp_action::Server<MergePointClouds>::SharedPtr action_server_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}
