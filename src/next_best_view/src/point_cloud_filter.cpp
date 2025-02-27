#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_action_interfaces/action/filter_point_cloud.hpp"  // Correct package name
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

class FilterPointCloudActionServer : public rclcpp::Node {
public:
    // Define action types with the correct package name
    using FilterPointCloud = kinova_action_interfaces::action::FilterPointCloud;
    using GoalHandleFilterPointCloud = rclcpp_action::ServerGoalHandle<FilterPointCloud>;

    FilterPointCloudActionServer() : Node("filter_point_cloud_action_server") {
        using namespace std::placeholders;

        // Create the action server with the correct action type
        this->action_server_ = rclcpp_action::create_server<FilterPointCloud>(
            this,
            "filter_point_cloud",
            std::bind(&FilterPointCloudActionServer::handle_goal, this, _1, _2),
            std::bind(&FilterPointCloudActionServer::handle_cancel, this, _1),
            std::bind(&FilterPointCloudActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Filter Point Cloud Action Server started.");
    }

private:
    rclcpp_action::Server<FilterPointCloud>::SharedPtr action_server_;

    // Handle incoming goal requests
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FilterPointCloud::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for file: %s", goal->pointcloud_file_path.c_str());
        (void)uuid;  // Unused parameter
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancellation requests
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFilterPointCloud> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;  // Unused parameter
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle accepted goals by launching execution in a separate thread
    void handle_accepted(const std::shared_ptr<GoalHandleFilterPointCloud> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&FilterPointCloudActionServer::execute, this, _1), goal_handle}.detach();
    }

    // Execute the filtering operation
    void execute(const std::shared_ptr<GoalHandleFilterPointCloud> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FilterPointCloud::Result>();

        // Step 1: Load the point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(goal->pointcloud_file_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", goal->pointcloud_file_path.c_str());
            result->success = false;
            goal_handle->succeed(result);
            return;
        }

        // Step 2: Apply passthrough filter along z-axis
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(goal->center_z - goal->height / 2.0, goal->center_z + goal->height / 2.0);
        pass.filter(*cloud_filtered_z);

        // Step 3: Filter points within the radius in xy-plane
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (size_t i = 0; i < cloud_filtered_z->points.size(); ++i) {
            const auto& point = cloud_filtered_z->points[i];
            float dx = point.x - goal->center_x;
            float dy = point.y - goal->center_y;
            float distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= goal->radius) {
                inliers->indices.push_back(i);
            }
        }

        // Step 4: Extract the points inside the cylinder
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_filtered_z);
        extract.setIndices(inliers);
        extract.setNegative(false);  // Keep points inside (not outside)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*cloud_filtered);

        // Step 5: Save the filtered point cloud
        std::string filtered_file_path = "filtered_" + goal->pointcloud_file_path;
        if (pcl::io::savePCDFileBinary(filtered_file_path, *cloud_filtered) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved filtered point cloud to %s", filtered_file_path.c_str());
            result->success = true;
            result->filtered_file_path = filtered_file_path;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save filtered point cloud to %s", filtered_file_path.c_str());
            result->success = false;
        }

        // Step 6: Send the result
        goal_handle->succeed(result);
    }
};

// Main function to run the node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterPointCloudActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}