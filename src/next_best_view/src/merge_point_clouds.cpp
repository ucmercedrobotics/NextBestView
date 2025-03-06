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
#include <chrono>
#include <iomanip>
#include <sstream>

// Alias for the filesystem namespace
namespace fs = std::filesystem;

// Include the action definition (replace "pointcloud_merger" with your actual package name)
#include "kinova_action_interfaces/action/merge_point_clouds.hpp"

using MergePointClouds = kinova_action_interfaces::action::MergePointClouds;

class PointCloudMergerNode : public rclcpp::Node {
 public:
  PointCloudMergerNode() : Node("pointcloud_merger_node") {
    // Set the directory containing the individual saved point clouds
    save_directory_ = "src/next_best_view/point_clouds/";

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
  /** Handle incoming goals
   * @param uuid Unique identifier for the goal
   * @param goal The goal message containing start_merging
   * @return ACCEPT_AND_EXECUTE if goal is valid, REJECT otherwise
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MergePointClouds::Goal> goal) {
    (void)uuid;  // Unused parameter
    if (!goal->start_merging) {
      RCLCPP_INFO(this->get_logger(), "Received goal with start_merging=false, rejecting");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepted goal to merge point clouds");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /** Handle cancellation requests
   * @param goal_handle Handle to the goal being canceled
   * @return ACCEPT to allow cancellation
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MergePointClouds>> goal_handle) {
    (void)goal_handle;  // Unused for now
    RCLCPP_INFO(this->get_logger(), "Received cancellation request, accepting");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /** Handle accepted goals by starting execution in a new thread
   * @param goal_handle Handle to the accepted goal
   */
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MergePointClouds>> goal_handle) {
    // Detach a new thread to execute the goal
    std::thread{std::bind(&PointCloudMergerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  /** Execute the merging process
   * @param goal_handle Handle to the current goal
   */
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
        continue;  // Skip failed files
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

    // Generate a unique output filename with a timestamp
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = ss.str();
    std::string output_filename = merged_directory_ + "merged_pointcloud_" + timestamp + ".pcd";

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

  std::string save_directory_;    // Directory with individual point clouds
  std::string merged_directory_;  // Directory for merged point clouds
  rclcpp_action::Server<MergePointClouds>::SharedPtr action_server_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMergerNode>());
  rclcpp::shutdown();
  return 0;
}