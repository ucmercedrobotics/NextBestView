#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <glog/logging.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <cfloat>
// Added includes for thread safety and futures
#include <queue>
#include <mutex>
#include <future>

#include <kinova_action_interfaces/action/voxel_map.hpp>

// Global voxel map
octomap::ColorOcTree _voxel_map(1.0);

class VoxelStruct : public rclcpp::Node {
private:
    // Point cloud parameters
    std::vector<Eigen::Vector3d> occupied_voxels_;

    // Voxel map parameters
    double voxel_resolution_;
    double ray_trace_step_;
    int surrounding_voxels_radius_;

    // Camera parameters
    std::vector<Eigen::Vector3d> frustum_points_;
    double camera_focal_length_;

public:
    VoxelStruct(const rclcpp::NodeOptions& options) : Node("voxel_struct", options) {
        this->declare_parameter("voxel_resolution", 1.0);
        this->declare_parameter("ray_trace_step", 0.1);
        this->declare_parameter("surrounding_voxels_radius", 1);
        this->declare_parameter("camera_focal_length_factor", 1.0);
        this->declare_parameter("camera_intrinsic", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        initialize();
    }

    void initialize() {
        voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
        _voxel_map.setResolution(voxel_resolution_);
        _voxel_map.clear();

        std::vector<double> camera_intrinsic_vec = this->get_parameter("camera_intrinsic").as_double_array();
        if (camera_intrinsic_vec.size() != 9) {
            LOG(ERROR) << "Invalid camera intrinsic matrix size";
            throw std::runtime_error("Invalid camera intrinsic matrix size");
        }
        Eigen::Matrix3d camera_intrinsic;
        camera_intrinsic << camera_intrinsic_vec[0], camera_intrinsic_vec[1], camera_intrinsic_vec[2],
                            camera_intrinsic_vec[3], camera_intrinsic_vec[4], camera_intrinsic_vec[5],
                            camera_intrinsic_vec[6], camera_intrinsic_vec[7], camera_intrinsic_vec[8];
        double camera_focal_length_factor = this->get_parameter("camera_focal_length_factor").as_double();
        std::pair<int, int> image_size;
        analyzeCameraIntrinsic(camera_intrinsic, camera_focal_length_factor, frustum_points_, camera_focal_length_, image_size);

        // Load ray tracing parameters
        ray_trace_step_ = this->get_parameter("ray_trace_step").as_double();
        surrounding_voxels_radius_ = this->get_parameter("surrounding_voxels_radius").as_int();

        LOG(INFO) << "VoxelStruct initialized successfully";
    }

    ~VoxelStruct() = default;

    octomap::ColorOcTree update_voxel_map(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const Eigen::Matrix4d &camera_pose,
        const Eigen::Vector3d &bbx_unknown_min,
        const Eigen::Vector3d &bbx_unknown_max,
        std::vector<Eigen::Vector3d> &output_frontier_voxels,
        std::vector<Eigen::Vector3d> &output_occupied_voxels) {
        
        output_frontier_voxels.clear();
        output_occupied_voxels.clear();
        std::vector<Eigen::Vector3d> frontier_voxels;
        octomap::ColorOcTree voxel_map(voxel_resolution_);

        if (cloud->size() == 0) {
            LOG(WARNING) << "Input point cloud is empty";
            return voxel_map;
        }

        // Set static bounding box from provided values
        voxel_map.setBBXMin(to_oct3d(bbx_unknown_min));
        voxel_map.setBBXMax(to_oct3d(bbx_unknown_max));

        // Add past occupied voxels within the bounding box
        for (const auto& voxel : occupied_voxels_) {
            if (is_within_bounds(voxel, bbx_unknown_min, bbx_unknown_max)) {
                octomap::point3d point(voxel[0], voxel[1], voxel[2]);
                voxel_map.updateNode(point, true);
                voxel_map.search(point)->setColor(0, 0, 255);
            }
        }
        LOG(INFO) << "Past occupied nodes added, leaf nodes: " << voxel_map.getNumLeafNodes();

        // Add current frame's occupied voxels within the bounding box
        for (const auto& point : cloud->points) {
            Eigen::Vector3d pt_vec(point.x, point.y, point.z);
            if (is_within_bounds(pt_vec, bbx_unknown_min, bbx_unknown_max)) {
                octomap::point3d pt(point.x, point.y, point.z);
                voxel_map.updateNode(pt, true);
                voxel_map.search(pt)->setColor(0, 0, 255);
                occupied_voxels_.push_back(pt_vec); // Accumulate within bounds
            }
        }
        LOG(INFO) << "Current frame nodes added, leaf nodes: " << voxel_map.getNumLeafNodes();

        // Ray tracing to classify voxels
        Eigen::Vector3d p1 = camera_pose.block<3,3>(0,0) * frustum_points_[0] + camera_pose.block<3,1>(0,3);
        Eigen::Vector3d p2 = camera_pose.block<3,3>(0,0) * frustum_points_[1] + camera_pose.block<3,1>(0,3);
        Eigen::Vector3d p3 = camera_pose.block<3,3>(0,0) * frustum_points_[2] + camera_pose.block<3,1>(0,3);
        Eigen::Vector3d p4 = camera_pose.block<3,3>(0,0) * frustum_points_[3] + camera_pose.block<3,1>(0,3);
        p1 = camera_pose.block<3,3>(0,0).transpose() * (p1 - camera_pose.block<3,1>(0,3));
        p2 = camera_pose.block<3,3>(0,0).transpose() * (p2 - camera_pose.block<3,1>(0,3));
        p3 = camera_pose.block<3,3>(0,0).transpose() * (p3 - camera_pose.block<3,1>(0,3));
        p4 = camera_pose.block<3,3>(0,0).transpose() * (p4 - camera_pose.block<3,1>(0,3));
        int num_horizontal_rays = ceil((p2 - p1)[0] / ray_trace_step_);
        int num_vertical_rays = ceil((p3 - p2)[1] / ray_trace_step_);
        int total_rays = num_horizontal_rays * num_vertical_rays;
        std::vector<std::vector<Eigen::Vector3d>> ray_hit_voxels(total_rays);
        double ray_trace_length = 1.0;

        std::vector<octomap::point3d*> ray_orign_vec(total_rays, nullptr);
        std::vector<octomap::point3d*> ray_end_vec(total_rays, nullptr);
        for (int k = 0; k < total_rays; k++) {
            int i = k / num_vertical_rays;
            int j = k % num_vertical_rays;
            Eigen::Vector3d ray_visual_end(p1[0] + i * ray_trace_step_, p1[1] + j * ray_trace_step_, p1[2]);
            ray_visual_end = camera_pose.block<3,3>(0,0) * ray_visual_end + camera_pose.block<3,1>(0,3);
            Eigen::Vector3d ray_origin = camera_pose.block<3,1>(0,3);
            Eigen::Vector3d ray_direction = (ray_visual_end - ray_origin).normalized();
            Eigen::Vector3d box_intersection;
            if (computeRayBoxIntersection(ray_origin, ray_direction, bbx_unknown_min, bbx_unknown_max, box_intersection)) {
                ray_origin = box_intersection;
                Eigen::Vector3d ray_end = ray_origin + ray_trace_length * ray_direction;
                ray_orign_vec[k] = new octomap::point3d(ray_origin[0], ray_origin[1], ray_origin[2]);
                ray_end_vec[k] = new octomap::point3d(ray_end[0], ray_end[1], ray_end[2]);
            }
        }

        std::vector<octomap::KeyRay*> ray_key_vec(total_rays, nullptr);
        #pragma omp parallel for
        for (int k = 0; k < total_rays; k++) {
            if (!ray_orign_vec[k] || !ray_end_vec[k]) continue;
            octomap::KeyRay* ray_ptr = new octomap::KeyRay;
            if (voxel_map.computeRayKeys(*ray_orign_vec[k], *ray_end_vec[k], *ray_ptr)) {
                ray_key_vec[k] = ray_ptr;
            } else {
                delete ray_ptr;
            }
            delete ray_orign_vec[k];
            delete ray_end_vec[k];
        }

        #pragma omp parallel for
        for (int k = 0; k < total_rays; ++k) {
            if (!ray_key_vec[k]) continue;
            std::vector<Eigen::Vector3d> ray_trace_shot_line(ray_key_vec[k]->size());
            auto it = ray_key_vec[k]->begin();
            for (size_t l = 0; l < ray_key_vec[k]->size(); ++l, ++it) {
                if (voxel_map.search(*it)) {
                    ray_trace_shot_line[l] = to_eigen3d(voxel_map.keyToCoord(*it));
                }
            }
            ray_hit_voxels[k] = ray_trace_shot_line;
            delete ray_key_vec[k];
        }

        // Classify voxels along rays
        for (const auto& ray : ray_hit_voxels) {
            bool start_occupied = false;
            for (const auto& voxel : ray) {
                auto voxel_map_it = voxel_map.search(voxel[0], voxel[1], voxel[2]);
                if (!voxel_map_it) continue;
                auto color = voxel_map_it->getColor();
                if (color.r == 0 && color.g == 0 && color.b == 255) {
                    start_occupied = true;
                } else if (start_occupied) {
                    auto _voxel_map_it = _voxel_map.search(voxel[0], voxel[1], voxel[2]);
                    if (_voxel_map_it) {
                        auto _color = _voxel_map_it->getColor();
                        if ((_color.r == 255 && _color.g == 255 && _color.b == 255) || (_color.r == 0 && _color.g == 0 && _color.b == 255)) {
                            voxel_map_it->setColor(_color.r, _color.g, _color.b);
                        } else {
                            voxel_map_it->setColor(188, 188, 188);
                        }
                    } else {
                        voxel_map_it->setColor(188, 188, 188);
                    }
                } else {
                    voxel_map_it->setColor(255, 255, 255);
                }
            }
        }

        // Identify frontier voxels
        for (auto it = voxel_map.begin_leafs(), end = voxel_map.end_leafs(); it != end; ++it) {
            auto color = it->getColor();
            if (color.r != 188 || color.g != 188 || color.b != 188) continue;
            auto neighbors = find_neighbors(it.getX(), it.getY(), it.getZ());
            bool has_empty = false, has_occupied = false;
            for (const auto& n : neighbors) {
                auto nb_it = voxel_map.search(n[0], n[1], n[2]);
                if (!nb_it) continue;
                auto nb_color = nb_it->getColor();
                if (nb_color.r == 188 && nb_color.g == 188 && nb_color.b == 188) has_empty = true;
                if (nb_color.r == 0 && nb_color.g == 0 && nb_color.b == 255) has_occupied = true;
                if (has_empty && has_occupied) {
                    it->setColor(255, 0, 0);
                    frontier_voxels.push_back(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
                    break;
                }
            }
        }
        LOG(INFO) << "Voxel classification completed, leaf nodes: " << voxel_map.getNumLeafNodes();

        // Update global map
        for (auto it = voxel_map.begin_leafs(), end = voxel_map.end_leafs(); it != end; ++it) {
            auto color = it->getColor();
            _voxel_map.updateNode(it.getX(), it.getY(), it.getZ(), true);
            _voxel_map.search(it.getX(), it.getY(), it.getZ())->setColor(color.r, color.g, color.b);
        }
        _voxel_map.updateInnerOccupancy();

        output_frontier_voxels = frontier_voxels;
        output_occupied_voxels = occupied_voxels_;

        return voxel_map;
    }

private:
    bool computeRayBoxIntersection(
        const Eigen::Vector3d& ray_origin,
        const Eigen::Vector3d& ray_direction,
        const Eigen::Vector3d& box_min,
        const Eigen::Vector3d& box_max,
        Eigen::Vector3d& intersection) {
        double tmin = (box_min.x() - ray_origin.x()) / ray_direction.x();
        double tmax = (box_max.x() - ray_origin.x()) / ray_direction.x();
        if (tmin > tmax) std::swap(tmin, tmax);

        double tymin = (box_min.y() - ray_origin.y()) / ray_direction.y();
        double tymax = (box_max.y() - ray_origin.y()) / ray_direction.y();
        if (tymin > tymax) std::swap(tymin, tymax);

        if (tmin > tymax || tymin > tmax) return false;
        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        double tzmin = (box_min.z() - ray_origin.z()) / ray_direction.z();
        double tzmax = (box_max.z() - ray_origin.z()) / ray_direction.z();
        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if (tmin > tzmax || tzmin > tmax) return false;
        if (tzmin > tmin) tmin = tzmin;
        if (tzmax < tmax) tmax = tzmax;

        intersection = ray_origin + ray_direction * tmin;
        return true;
    }

    bool is_within_bounds(const Eigen::Vector3d& point, const Eigen::Vector3d& min, const Eigen::Vector3d& max) {
        return (point[0] >= min[0] && point[0] <= max[0] &&
                point[1] >= min[1] && point[1] <= max[1] &&
                point[2] >= min[2] && point[2] <= max[2]);
    }

    std::vector<Eigen::Vector3d> find_neighbors(double x, double y, double z) {
        std::vector<Eigen::Vector3d> neighbors;
        double step = voxel_resolution_;
        for (double i = x - step; i <= x + step; i += step) {
            for (double j = y - step; j <= y + step; j += step) {
                for (double k = z - step; k <= z + step; k += step) {
                    if (i == x && j == y && k == z) continue;
                    neighbors.push_back(Eigen::Vector3d(i, j, k));
                }
            }
        }
        return neighbors;
    }

    octomap::point3d to_oct3d(const Eigen::Vector3d& v) {
        return octomap::point3d(v.x(), v.y(), v.z());
    }

    Eigen::Vector3d to_eigen3d(const octomap::point3d& p) {
        return Eigen::Vector3d(p.x(), p.y(), p.z());
    }

    std::vector<Eigen::Vector3d> getSurroundingVoxels(const Eigen::Vector3d& voxel, int r) {
        std::vector<Eigen::Vector3d> surrounding_voxels;
        double step = r * voxel_resolution_;
        for (double x = voxel[0] - step; x <= voxel[0] + step; x += voxel_resolution_) {
            for (double y = voxel[1] - step; y <= voxel[1] + step; y += voxel_resolution_) {
                for (double z = voxel[2] - step; z <= voxel[2] + step; z += voxel_resolution_) {
                    surrounding_voxels.push_back(Eigen::Vector3d(x, y, z));
                }
            }
        }
        return surrounding_voxels;
    }

    void analyzeCameraIntrinsic(
        const Eigen::Matrix3d& camera_intrinsic,
        const double &camera_focal_length_factor,
        std::vector<Eigen::Vector3d>& frustum_points,
        double &camera_focal_length,
        std::pair<int, int>& image_size) {
        frustum_points.resize(4);
        auto fx = camera_intrinsic(0, 0);
        auto fy = camera_intrinsic(1, 1);
        auto cx = camera_intrinsic(0, 2);
        auto cy = camera_intrinsic(1, 2);
        double width = 2 * cx;
        double height = 2 * cy;
        frustum_points[0] = Eigen::Vector3d((-width / 2) / fx * 1.0, (-height / 2) / fy * 1.0, 1.0);
        frustum_points[1] = Eigen::Vector3d((width / 2) / fx * 1.0, (-height / 2) / fy * 1.0, 1.0);
        frustum_points[2] = Eigen::Vector3d((width / 2) / fx * 1.0, (height / 2) / fy * 1.0, 1.0);
        frustum_points[3] = Eigen::Vector3d((-width / 2) / fx * 1.0, (height / 2) / fy * 1.0, 1.0);
        camera_focal_length = ((fx + fy) / 2.0 / 1000.0) * camera_focal_length_factor;
        image_size = {static_cast<int>(width), static_cast<int>(height)};
    }
};

class VoxelMapActionServer : public rclcpp::Node {
public:
    VoxelMapActionServer(const rclcpp::NodeOptions& options) : Node("voxel_map_action_server", options) {
        voxel_struct_ = std::make_shared<VoxelStruct>(options);
        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<kinova_action_interfaces::action::VoxelMap>(
            this,
            "update_voxel_map",
            std::bind(&VoxelMapActionServer::handle_goal, this, _1, _2),
            std::bind(&VoxelMapActionServer::handle_cancel, this, _1),
            std::bind(&VoxelMapActionServer::handle_accepted, this, _1));
        
        // Set up subscription with QoS: KeepLast(1) and best_effort
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", qos,
            std::bind(&VoxelMapActionServer::point_cloud_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "VoxelMapActionServer initialized");
    }

private:
    std::shared_ptr<VoxelStruct> voxel_struct_;
    rclcpp_action::Server<kinova_action_interfaces::action::VoxelMap>::SharedPtr action_server_;
    // Added members for subscription and point cloud handling
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    std::queue<std::promise<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>> promise_queue_;
    std::mutex promise_mutex_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const kinova_action_interfaces::action::VoxelMap::Goal> /*goal*/) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<kinova_action_interfaces::action::VoxelMap>> /*goal_handle*/) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kinova_action_interfaces::action::VoxelMap>> goal_handle) {
        std::thread(&VoxelMapActionServer::execute, this, goal_handle).detach();
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *cloud);
        {
            std::lock_guard<std::mutex> lock(promise_mutex_);
            if (!promise_queue_.empty()) {
                auto& promise = promise_queue_.front();
                promise.set_value(cloud);
                promise_queue_.pop();
            }
            // If no promises are waiting, discard the point cloud
        }
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<kinova_action_interfaces::action::VoxelMap>> goal_handle) {
        const auto goal = goal_handle->get_goal();

        // Create a promise and future to wait for the next point cloud
        std::promise<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> promise;
        auto future = promise.get_future();
        {
            std::lock_guard<std::mutex> lock(promise_mutex_);
            promise_queue_.push(std::move(promise));
        }
        // Wait for the next point cloud after the request
        auto cloud = future.get();

        // Convert camera pose
        Eigen::Affine3d affine;
        tf2::fromMsg(goal->camera_pose, affine);
        Eigen::Matrix4d camera_pose = affine.matrix();

        // Extract bounding box from goal
        Eigen::Vector3d bbx_min(goal->bbx_unknown_min.x, goal->bbx_unknown_min.y, goal->bbx_unknown_min.z);
        Eigen::Vector3d bbx_max(goal->bbx_unknown_max.x, goal->bbx_unknown_max.y, goal->bbx_unknown_max.z);

        // Execute the voxel map update with the received point cloud
        std::vector<Eigen::Vector3d> frontier_voxels, occupied_voxels;
        octomap::ColorOcTree voxel_map = voxel_struct_->update_voxel_map(
            cloud, camera_pose, bbx_min, bbx_max, frontier_voxels, occupied_voxels);

        // Prepare result
        auto result = std::make_shared<kinova_action_interfaces::action::VoxelMap::Result>();
        octomap_msgs::fullMapToMsg(voxel_map, result->octomap);
        for (const auto& v : frontier_voxels) {
            geometry_msgs::msg::Point p;
            p.x = v.x(); p.y = v.y(); p.z = v.z();
            result->frontier_voxels.push_back(p);
        }
        for (const auto& v : occupied_voxels) {
            geometry_msgs::msg::Point p;
            p.x = v.x(); p.y = v.y(); p.z = v.z();
            result->occupied_voxels.push_back(p);
        }

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action completed successfully");
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<VoxelMapActionServer>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}