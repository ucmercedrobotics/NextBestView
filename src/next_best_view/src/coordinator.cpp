#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/client.hpp>  // Added for rclcpp::ClientBase
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kinova_action_interfaces/action/coordinator.hpp>
#include <kinova_action_interfaces/srv/add_collision_object.hpp>
#include <kinova_action_interfaces/srv/pose_generator.hpp>
#include <kinova_action_interfaces/srv/reachability_check.hpp>
#include <kinova_action_interfaces/srv/current_pose.hpp>
#include <kinova_action_interfaces/action/move_to_pose.hpp>
#include <kinova_action_interfaces/action/save_point_cloud.hpp>
#include <memory>
#include <thread>
#include <algorithm>
#include <random>
#include <string>  // Added for std::string

class CoordinatorActionServer : public rclcpp::Node {
public:
    using CoordinatorAction = kinova_action_interfaces::action::Coordinator;
    using GoalHandleCoordinator = rclcpp_action::ServerGoalHandle<CoordinatorAction>;
    using MoveToPose = kinova_action_interfaces::action::MoveToPose;
    using SavePointCloud = kinova_action_interfaces::action::SavePointCloud;

    CoordinatorActionServer() : Node("coordinator_action_server") {
        // Initialize service clients
        add_collision_client_ = create_client<kinova_action_interfaces::srv::AddCollisionObject>("add_collision_object");
        generate_poses_client_ = create_client<kinova_action_interfaces::srv::PoseGenerator>("generate_poses");
        reachability_client_ = create_client<kinova_action_interfaces::srv::ReachabilityCheck>("check_reachable_poses");
        current_pose_client_ = create_client<kinova_action_interfaces::srv::CurrentPose>("current_pose_service");

        // Initialize action clients
        move_client_ = rclcpp_action::create_client<MoveToPose>(this, move_action_name_);
        save_pc_client_ = rclcpp_action::create_client<SavePointCloud>(this, save_pc_action_name_);

        // Initialize action server
        action_server_ = rclcpp_action::create_server<CoordinatorAction>(
            this,
            "coordinator_action",
            std::bind(&CoordinatorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CoordinatorActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&CoordinatorActionServer::handle_accepted, this, std::placeholders::_1)
        );

        // Wait for services to be available (timeout of 10 seconds)
        for (auto client : {std::static_pointer_cast<rclcpp::ClientBase>(add_collision_client_),
                           std::static_pointer_cast<rclcpp::ClientBase>(generate_poses_client_),
                           std::static_pointer_cast<rclcpp::ClientBase>(reachability_client_),
                           std::static_pointer_cast<rclcpp::ClientBase>(current_pose_client_)}) {
            if (!client->wait_for_service(std::chrono::seconds(10))) {
                RCLCPP_ERROR(this->get_logger(), "Service %s not available after waiting.", client->get_service_name());
                throw std::runtime_error("Service unavailable");
            }
        }

        // Wait for action servers to be available (timeout of 10 seconds)
        if (!move_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server %s not available after waiting.", move_action_name_.c_str());
            throw std::runtime_error("Action server unavailable");
        }
        if (!save_pc_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server %s not available after waiting.", save_pc_action_name_.c_str());
            throw std::runtime_error("Action server unavailable");
        }

        RCLCPP_INFO(this->get_logger(), "Coordinator action server started.");
    }

private:
    rclcpp_action::Server<CoordinatorAction>::SharedPtr action_server_;
    rclcpp::Client<kinova_action_interfaces::srv::AddCollisionObject>::SharedPtr add_collision_client_;
    rclcpp::Client<kinova_action_interfaces::srv::PoseGenerator>::SharedPtr generate_poses_client_;
    rclcpp::Client<kinova_action_interfaces::srv::ReachabilityCheck>::SharedPtr reachability_client_;
    rclcpp::Client<kinova_action_interfaces::srv::CurrentPose>::SharedPtr current_pose_client_;
    rclcpp_action::Client<MoveToPose>::SharedPtr move_client_;
    rclcpp_action::Client<SavePointCloud>::SharedPtr save_pc_client_;

    // Action names as member variables
    std::string move_action_name_ = "move_to_pose_server";
    std::string save_pc_action_name_ = "save_point_cloud_server";

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const CoordinatorAction::Goal> goal) {
        if (goal->algorithm != "random" && goal->algorithm != "visit_all" && goal->algorithm != "next_best_view") {
            RCLCPP_ERROR(this->get_logger(), "Invalid algorithm: %s", goal->algorithm.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->distance <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Distance must be positive: %f", goal->distance);
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCoordinator> /*goal_handle*/) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCoordinator> goal_handle) {
        std::thread{std::bind(&CoordinatorActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleCoordinator> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CoordinatorAction::Feedback>();
        auto result = std::make_shared<CoordinatorAction::Result>();
        result->success = false;  // Default to failure unless all steps succeed

        // Step 1-4: Common initialization
        feedback->status = "Initializing candidate poses";
        goal_handle->publish_feedback(feedback);
        auto [reachable_camera_poses, corresponding_end_effector_poses] = initializeCandidatePoses(goal->object_position, goal->shape_type, goal->distance);
        if (corresponding_end_effector_poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No reachable poses found.");
            goal_handle->succeed(result);
            return;
        }

        if (goal->algorithm == "random") {
            // Step 5: Select a given number of poses randomly
            int num_to_visit = goal->num_locations;
            if (num_to_visit <= 0 || static_cast<size_t>(num_to_visit) > corresponding_end_effector_poses.size()) {
                RCLCPP_WARN(this->get_logger(), "Invalid num_locations (%d), using all %zu poses.", 
                            num_to_visit, corresponding_end_effector_poses.size());
                num_to_visit = corresponding_end_effector_poses.size();
            }
            std::vector<geometry_msgs::msg::Pose> selected_poses = corresponding_end_effector_poses;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::shuffle(selected_poses.begin(), selected_poses.end(), gen);
            selected_poses.resize(num_to_visit);

            // Step 6: Solve TSP
            feedback->status = "Solving TSP";
            goal_handle->publish_feedback(feedback);
            auto current_pose = getCurrentEndEffectorPose();
            auto tour = solveTSP(selected_poses, current_pose);

            // Step 7 & 8: Move and save point clouds
            if (!executeTour(goal_handle, tour, feedback, result)) {
                goal_handle->succeed(result);
                return;
            }
            result->success = true;
            goal_handle->succeed(result);
        } else if (goal->algorithm == "visit_all") {
            // Step 5: Use all reachable poses
            feedback->status = "Solving TSP for all poses";
            goal_handle->publish_feedback(feedback);
            auto current_pose = getCurrentEndEffectorPose();
            auto tour = solveTSP(corresponding_end_effector_poses, current_pose);

            // Step 7 & 8: Move and save point clouds
            if (!executeTour(goal_handle, tour, feedback, result)) {
                goal_handle->succeed(result);
                return;
            }
            result->success = true;
            goal_handle->succeed(result);
        } else if (goal->algorithm == "next_best_view") {
            // For next_best_view, perform steps 1-4 and print message
            RCLCPP_INFO(this->get_logger(), "I will continue to next best view implementation");
            result->success = true;
            goal_handle->succeed(result);
        }
    }

    std::pair<std::vector<geometry_msgs::msg::Pose>, std::vector<geometry_msgs::msg::Pose>> initializeCandidatePoses(
        const geometry_msgs::msg::Point& object_position, const std::string& shape_type, float distance) {
        // Step 1 & 2: Add collision object
        auto add_request = std::make_shared<kinova_action_interfaces::srv::AddCollisionObject::Request>();
        add_request->object_position = object_position;
        add_request->shape_type = shape_type;
        auto add_future = add_collision_client_->async_send_request(add_request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), add_future) != rclcpp::FutureReturnCode::SUCCESS ||
            !add_future.get()->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to add collision object.");
            return {{}, {}};
        }

        // Step 3: Generate candidate poses
        auto gen_request = std::make_shared<kinova_action_interfaces::srv::PoseGenerator::Request>();
        gen_request->object_position = object_position;
        gen_request->distance = distance;  // Use distance from action request
        gen_request->shape = shape_type;
        auto gen_future = generate_poses_client_->async_send_request(gen_request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), gen_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate poses.");
            return {{}, {}};
        }
        auto candidate_poses = gen_future.get()->candidate_poses;

        // Step 4: Check reachable poses
        auto reach_request = std::make_shared<kinova_action_interfaces::srv::ReachabilityCheck::Request>();
        reach_request->camera_poses = candidate_poses;
        auto reach_future = reachability_client_->async_send_request(reach_request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), reach_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to check reachable poses.");
            return {{}, {}};
        }
        return {reach_future.get()->reachable_camera_poses, reach_future.get()->corresponding_end_effector_poses};
    }

    geometry_msgs::msg::Pose getCurrentEndEffectorPose() {
        auto request = std::make_shared<kinova_action_interfaces::srv::CurrentPose::Request>();
        auto future = current_pose_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            return future.get()->end_effector_pose.pose;
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to get current end-effector pose.");
        return geometry_msgs::msg::Pose();  // Return default pose on failure
    }

    std::vector<geometry_msgs::msg::Pose> solveTSP(const std::vector<geometry_msgs::msg::Pose>& poses,
                                                  const geometry_msgs::msg::Pose& start_pose) {
        std::vector<geometry_msgs::msg::Pose> tour;
        std::vector<bool> visited(poses.size(), false);
        geometry_msgs::msg::Pose current = start_pose;

        while (tour.size() < poses.size()) {
            double min_dist = std::numeric_limits<double>::max();
            int next_index = -1;
            for (size_t i = 0; i < poses.size(); ++i) {
                if (!visited[i]) {
                    double dist = distanceBetweenPoses(current, poses[i]);
                    if (dist < min_dist) {
                        min_dist = dist;
                        next_index = i;
                    }
                }
            }
            if (next_index != -1) {
                current = poses[next_index];
                tour.push_back(current);
                visited[next_index] = true;
            } else {
                break;
            }
        }
        return tour;
    }

    double distanceBetweenPoses(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2) {
        double dx = p1.position.x - p2.position.x;
        double dy = p1.position.y - p2.position.y;
        double dz = p1.position.z - p2.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool moveToPose(const geometry_msgs::msg::Pose& pose) {
        auto goal_msg = MoveToPose::Goal();
        goal_msg.target_pose = pose;
        auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
        auto goal_handle_future = move_client_->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send move goal.");
            return false;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Move goal was rejected.");
            return false;
        }
        auto result_future = move_client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get move result.");
            return false;
        }
        return result_future.get().result->success;
    }

    bool savePointCloud() {
        auto goal_msg = SavePointCloud::Goal();
        goal_msg.start_saving = true;
        auto send_goal_options = rclcpp_action::Client<SavePointCloud>::SendGoalOptions();
        auto goal_handle_future = save_pc_client_->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send save point cloud goal.");
            return false;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Save point cloud goal was rejected.");
            return false;
        }
        auto result_future = save_pc_client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get save point cloud result.");
            return false;
        }
        return result_future.get().result->success;
    }

    bool executeTour(const std::shared_ptr<GoalHandleCoordinator> goal_handle,
                     const std::vector<geometry_msgs::msg::Pose>& tour,
                     std::shared_ptr<CoordinatorAction::Feedback> feedback,
                     std::shared_ptr<CoordinatorAction::Result> result) {
        for (const auto& pose : tour) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->success = false;
                goal_handle->canceled(result);
                return false;
            }
            feedback->status = "Moving to pose";
            goal_handle->publish_feedback(feedback);
            if (!moveToPose(pose)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to pose.");
                return false;
            }
            feedback->status = "Saving point cloud";
            goal_handle->publish_feedback(feedback);
            if (!savePointCloud()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud.");
                return false;
            }
        }
        return true;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CoordinatorActionServer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}