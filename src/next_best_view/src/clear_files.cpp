#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <filesystem>
#include <kinova_action_interfaces/action/clear_files.hpp> // Adjust based on your package

class ClearFilesServer : public rclcpp::Node {
public:
  using ClearFiles = kinova_action_interfaces::action::ClearFiles;
  using GoalHandleClearFiles = rclcpp_action::ServerGoalHandle<ClearFiles>;

  ClearFilesServer() : Node("clear_files_server") {
    // Define directories using the same format as in the query
    point_clouds_dir_ = "src/next_best_view/point_clouds/";
    filtered_pointclouds_dir_ = "src/next_best_view/filtered_pointclouds/";

    // Create the action server
    action_server_ = rclcpp_action::create_server<ClearFiles>(
        this,
        "clear_files",
        std::bind(&ClearFilesServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ClearFilesServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ClearFilesServer::handle_accepted, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "ClearFiles action server started.");
  }

private:
  rclcpp_action::Server<ClearFiles>::SharedPtr action_server_;
  std::string point_clouds_dir_;          // Directory for point cloud files
  std::string filtered_pointclouds_dir_;  // Directory for filtered point cloud files

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ClearFiles::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal with confirm_deletion: %s",
                goal->confirm_deletion ? "true" : "false");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleClearFiles> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleClearFiles> goal_handle) {
    std::thread{std::bind(&ClearFilesServer::execute, this, std::placeholders::_1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleClearFiles> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ClearFiles::Feedback>();
    auto result = std::make_shared<ClearFiles::Result>();

    if (!goal->confirm_deletion) {
      RCLCPP_INFO(this->get_logger(), "Deletion not confirmed, no action taken.");
      result->success = true;
      goal_handle->succeed(result);
      return;
    }

    int files_deleted = 0;
    bool success = true;

    // List of directories to clear, using the format from the query
    std::vector<std::string> directories = {point_clouds_dir_, filtered_pointclouds_dir_};

    for (const auto& dir : directories) {
      // Check if the directory exists
      if (!std::filesystem::exists(dir)) {
        RCLCPP_WARN(this->get_logger(), "Directory %s does not exist, skipping.", dir.c_str());
        continue;
      }

      // Verify it's a directory
      if (!std::filesystem::is_directory(dir)) {
        RCLCPP_ERROR(this->get_logger(), "%s is not a directory.", dir.c_str());
        success = false;
        continue;
      }

      // Iterate over files in the directory
      for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (goal_handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Goal canceled during execution.");
          result->success = false;
          goal_handle->canceled(result);
          return;
        }

        if (entry.is_regular_file()) {
          try {
            std::filesystem::remove(entry.path());
            files_deleted++;
            feedback->status_message = std::to_string(files_deleted) + " files are deleted";
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Deleted file: %s", entry.path().c_str());
          } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to delete %s: %s",
                         entry.path().c_str(), e.what());
            success = false;
          }
        }
      }
    }

    result->success = success;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Completed action: deleted %d files.", files_deleted);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClearFilesServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}