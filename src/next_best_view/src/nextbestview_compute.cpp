#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kinova_action_interfaces/action/compute_nextbestview.hpp> // Replace with your package name

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <CGAL/Cartesian_d.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Approximate_min_ellipsoid_d.h>
#include <CGAL/Approximate_min_ellipsoid_d_traits_d.h>

#include <memory>
#include <thread>
#include <vector>

// Structure to hold ellipsoid parameters
struct EllipsoidParam {
    std::string type;           // "frontier" or "occupied"
    Eigen::Matrix4d pose;       // Pose of the ellipsoid
    Eigen::Vector3d radii;      // Radii of the ellipsoid
};

class NBVActionServer : public rclcpp::Node {
public:
    using ComputeNextbestview = kinova_action_interfaces::action::ComputeNextbestview;
    using GoalHandleComputeNextbestview = rclcpp_action::ServerGoalHandle<ComputeNextbestview>;

    NBVActionServer() : Node("nbv_action_server") {
        // Declare parameters to be loaded from YAML
        this->declare_parameter("camera_intrinsic", std::vector<double>{});
        this->declare_parameter("camera_focal_length_factor", 1.0);
        this->declare_parameter("max_gmm_cluster_num", 10);

        // Load parameters
        auto camera_intrinsic_vec = this->get_parameter("camera_intrinsic").as_double_array();
        if (camera_intrinsic_vec.size() != 9) {
            RCLCPP_ERROR(this->get_logger(), "Camera intrinsic must be a 3x3 matrix (9 elements)");
            throw std::runtime_error("Invalid camera intrinsic matrix size");
        }
        camera_intrinsic_ << camera_intrinsic_vec[0], camera_intrinsic_vec[1], camera_intrinsic_vec[2],
                            camera_intrinsic_vec[3], camera_intrinsic_vec[4], camera_intrinsic_vec[5],
                            camera_intrinsic_vec[6], camera_intrinsic_vec[7], camera_intrinsic_vec[8];
        
        camera_focal_length_factor_ = this->get_parameter("camera_focal_length_factor").as_double();
        max_gmm_cluster_num_ = this->get_parameter("max_gmm_cluster_num").as_int();
        min_gmm_cluster_num_ = 5; // Hardcoded minimum as in original code

        // Compute image size from camera intrinsics
        analyzeCameraIntrinsic();

        // Initialize action server
        action_server_ = rclcpp_action::create_server<ComputeNextbestview>(
            this,
            "compute_nbv",
            std::bind(&NBVActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NBVActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&NBVActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "NBV Action Server started");
    }

private:
    rclcpp_action::Server<ComputeNextbestview>::SharedPtr action_server_;
    Eigen::Matrix3d camera_intrinsic_;
    double camera_focal_length_factor_;
    int max_gmm_cluster_num_;
    int min_gmm_cluster_num_;
    std::pair<int, int> image_size_;

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const ComputeNextbestview::Goal> /*goal*/) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleComputeNextbestview> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleComputeNextbestview> goal_handle) {
        std::thread{std::bind(&NBVActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleComputeNextbestview> goal_handle) {
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "Processing NBV goal");

        // Convert ROS messages to internal types
        std::vector<Eigen::Vector3d> frontier_voxels = pointsToVectors(goal->frontier_voxels);
        std::vector<Eigen::Vector3d> occupied_voxels = pointsToVectors(goal->occupied_voxels);
        std::vector<Eigen::Matrix4d> candidate_poses;
        for (const auto& pose : goal->candidate_poses) {
            candidate_poses.push_back(poseToMatrix(pose));
        }

        // Cluster voxels into ellipsoids
        std::vector<EllipsoidParam> ellipsoids = clusterEllipsoids(frontier_voxels, occupied_voxels);
        if (ellipsoids.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No ellipsoids computed, aborting");
            goal_handle->abort(std::make_shared<ComputeNextbestview::Result>());
            return;
        }

        // Evaluate each candidate pose
        std::vector<double> scores(candidate_poses.size());
        for (size_t i = 0; i < candidate_poses.size(); ++i) {
            cv::Mat projection_img;
            scores[i] = computeProjectionScore(candidate_poses[i], ellipsoids, projection_img);
        }

        // Select the best pose
        size_t best_index = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
        Eigen::Matrix4d nbv = candidate_poses[best_index];
        geometry_msgs::msg::Pose nbv_pose = matrixToPose(nbv);

        // Send result
        auto result = std::make_shared<ComputeNextbestview::Result>();
        result->nbv_pose = nbv_pose;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "NBV computed and returned");
    }

    // Helper functions
    void analyzeCameraIntrinsic() {
        double cx = camera_intrinsic_(0, 2);
        double cy = camera_intrinsic_(1, 2);
        image_size_.first = static_cast<int>(2 * cx);  // Width
        image_size_.second = static_cast<int>(2 * cy); // Height
    }

    Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose& pose) {
        Eigen::Affine3d affine;
        tf2::fromMsg(pose, affine);
        return affine.matrix();
    }

    std::vector<Eigen::Vector3d> pointsToVectors(const std::vector<geometry_msgs::msg::Point>& points) {
        std::vector<Eigen::Vector3d> vectors;
        for (const auto& point : points) {
            vectors.emplace_back(point.x, point.y, point.z);
        }
        return vectors;
    }

    geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& matrix) {
        Eigen::Affine3d affine(matrix);
        return tf2::toMsg(affine);
    }

    std::vector<EllipsoidParam> clusterEllipsoids(
        const std::vector<Eigen::Vector3d>& frontier_voxels,
        const std::vector<Eigen::Vector3d>& occupied_voxels) {
        std::vector<EllipsoidParam> ellipsoids;

        // Cluster frontier voxels
        auto frontier_clusters = gmmClustering(frontier_voxels);
        ellipsoids = fitEllipsoids(frontier_clusters, "frontier");

        // Cluster occupied voxels
        auto occupied_clusters = gmmClustering(occupied_voxels);
        auto occupied_ellipsoids = fitEllipsoids(occupied_clusters, "occupied");
        ellipsoids.insert(ellipsoids.end(), occupied_ellipsoids.begin(), occupied_ellipsoids.end());

        return ellipsoids;
    }

    std::vector<std::vector<Eigen::Vector3d>> gmmClustering(const std::vector<Eigen::Vector3d>& voxels) {
        if (voxels.empty()) return {};

        cv::Mat samples(voxels.size(), 3, CV_32FC1);
        for (size_t i = 0; i < voxels.size(); ++i) {
            samples.at<float>(i, 0) = static_cast<float>(voxels[i].x());
            samples.at<float>(i, 1) = static_cast<float>(voxels[i].y());
            samples.at<float>(i, 2) = static_cast<float>(voxels[i].z());
        }

        std::vector<std::vector<Eigen::Vector3d>> clusters;
        int cluster_num = std::min(max_gmm_cluster_num_, static_cast<int>(voxels.size() / 2));
        cluster_num = std::max(cluster_num, min_gmm_cluster_num_);

        if (voxels.size() < static_cast<size_t>(min_gmm_cluster_num_)) {
            clusters.resize(1);
            clusters[0] = voxels;
            return clusters;
        }

        cv::Ptr<cv::ml::EM> em = cv::ml::EM::create();
        em->setClustersNumber(cluster_num);
        em->setCovarianceMatrixType(cv::ml::EM::COV_MAT_SPHERICAL);
        em->trainEM(samples);

        cv::Mat labels;
        em->predict(samples, labels);
        clusters.resize(cluster_num);

        for (int i = 0; i < samples.rows; ++i) {
            int cluster_idx = static_cast<int>(labels.at<double>(i, 0));
            clusters[cluster_idx].push_back(voxels[i]);
        }

        // Remove empty clusters
        clusters.erase(
            std::remove_if(clusters.begin(), clusters.end(),
                [](const auto& cluster) { return cluster.empty(); }),
            clusters.end());

        return clusters;
    }

    std::vector<EllipsoidParam> fitEllipsoids(
        const std::vector<std::vector<Eigen::Vector3d>>& clusters,
        const std::string& type) {
        typedef CGAL::Cartesian_d<double> Kernel;
        typedef CGAL::MP_Float ET;
        typedef CGAL::Approximate_min_ellipsoid_d_traits_d<Kernel, ET> Traits;
        typedef Traits::Point Point;
        typedef std::vector<Point> PointList;
        typedef CGAL::Approximate_min_ellipsoid_d<Traits> AME;

        std::vector<EllipsoidParam> ellipsoids;
        const double eps = 0.01;

        for (const auto& cluster : clusters) {
            if (cluster.size() < 4) continue; // Need at least 4 points in 3D

            PointList points;
            for (const auto& v : cluster) {
                std::vector<double> coords{v.x(), v.y(), v.z()};
                points.emplace_back(3, coords.begin(), coords.end());
            }

            Traits traits;
            AME ame(eps, points.begin(), points.end(), traits);

            if (!ame.is_full_dimensional() || ame.dimension() != 3) continue;

            EllipsoidParam ep;
            ep.type = type;
            ep.pose = Eigen::Matrix4d::Identity();
            ep.pose.block<3, 1>(0, 3) = Eigen::Vector3d(
                *ame.center_cartesian_begin(),
                *(ame.center_cartesian_begin() + 1),
                *(ame.center_cartesian_begin() + 2));
            ep.pose.block<3, 3>(0, 0) = Eigen::Matrix3d{
                {*ame.axis_direction_cartesian_begin(0), *ame.axis_direction_cartesian_begin(1), *ame.axis_direction_cartesian_begin(2)},
                {*(ame.axis_direction_cartesian_begin(0) + 1), *(ame.axis_direction_cartesian_begin(1) + 1), *(ame.axis_direction_cartesian_begin(2) + 1)},
                {*(ame.axis_direction_cartesian_begin(0) + 2), *(ame.axis_direction_cartesian_begin(1) + 2), *(ame.axis_direction_cartesian_begin(2) + 2)}
            };
            ep.radii = Eigen::Vector3d(
                *ame.axes_lengths_begin(),
                *(ame.axes_lengths_begin() + 1),
                *(ame.axes_lengths_begin() + 2));
            ellipsoids.push_back(ep);
        }

        return ellipsoids;
    }

    double computeProjectionScore(
        const Eigen::Matrix4d& camera_pose,
        const std::vector<EllipsoidParam>& ellipsoids,
        cv::Mat& projection_img) {
        // Compute camera projection matrix
        Eigen::Matrix<double, 3, 4> tmp;
        Eigen::Matrix4d inv_pose = camera_pose.inverse();
        tmp.block<3, 3>(0, 0) = inv_pose.block<3, 3>(0, 0);
        tmp.block<3, 1>(0, 3) = inv_pose.block<3, 1>(0, 3);
        Eigen::Matrix<double, 3, 4> camera_matrix = camera_intrinsic_ * tmp;

        // Compute z-depths for sorting
        std::vector<double> z_depths(ellipsoids.size());
        for (size_t i = 0; i < ellipsoids.size(); ++i) {
            Eigen::Vector3d center = ellipsoids[i].pose.block<3, 1>(0, 3);
            center = camera_pose.block<3, 3>(0, 0).transpose() * (center - camera_pose.block<3, 1>(0, 3));
            z_depths[i] = center.z();
        }

        // Sort indices by z-depth (farthest to nearest)
        std::vector<size_t> indices(ellipsoids.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
                  [&z_depths](size_t i1, size_t i2) { return z_depths[i1] < z_depths[i2]; });

        // Compute weights based on depth
        std::vector<double> weights(ellipsoids.size());
        for (size_t i = 0; i < weights.size(); ++i) {
            weights[indices[i]] = pow(0.5, i); // Exponential decay with depth
        }

        // Project ellipsoids
        std::vector<cv::Mat> img_vec(ellipsoids.size());
        for (size_t i = 0; i < ellipsoids.size(); ++i) {
            img_vec[i] = cv::Mat(cv::Size(image_size_.first, image_size_.second), CV_8UC1, cv::Scalar(0));
            Eigen::Matrix4d dual = createEllipsoidDualMatrix(ellipsoids[i]);
            if (dual.isZero()) continue;

            Eigen::Matrix3d ellipse_matrix = computeEllipsoidProjection(camera_matrix, dual);
            if (ellipse_matrix.isZero()) continue;

            double A = ellipse_matrix(0, 0);
            double B = ellipse_matrix(0, 1) + ellipse_matrix(1, 0);
            double C = ellipse_matrix(1, 1);
            double D = ellipse_matrix(0, 2) + ellipse_matrix(2, 0);
            double E = ellipse_matrix(1, 2) + ellipse_matrix(2, 1);
            double F = ellipse_matrix(2, 2);

            double x0 = (2 * C * D - B * E) / (B * B - 4 * A * C);
            double y0 = (2 * A * E - B * D) / (B * B - 4 * A * C);
            double theta = -0.5 * atan2(B, C - A) * 180 / CV_PI;

            double denom1 = A * cos(theta * CV_PI / 180) * cos(theta * CV_PI / 180) +
                           B * sin(theta * CV_PI / 180) * cos(theta * CV_PI / 180) +
                           C * sin(theta * CV_PI / 180) * sin(theta * CV_PI / 180);
            double denom2 = C * cos(theta * CV_PI / 180) * cos(theta * CV_PI / 180) -
                           B * sin(theta * CV_PI / 180) * cos(theta * CV_PI / 180) +
                           A * sin(theta * CV_PI / 180) * sin(theta * CV_PI / 180);
            if (denom1 == 0 || denom2 == 0) continue;

            double a_length = sqrt((A * x0 * x0 + B * x0 * y0 + C * y0 * y0 - F) / denom1);
            double b_length = sqrt((A * x0 * x0 + B * x0 * y0 + C * y0 * y0 - F) / denom2);

            if (a_length <= 0 || b_length <= 0) continue;

            try {
                cv::ellipse(img_vec[i], cv::Point(static_cast<int>(x0), static_cast<int>(y0)),
                            cv::Size(static_cast<int>(a_length), static_cast<int>(b_length)),
                            theta, 0, 360, 255, -1);
            } catch (const cv::Exception& e) {
                RCLCPP_WARN(this->get_logger(), "OpenCV exception in ellipse drawing: %s", e.what());
                continue;
            }
        }

        // Combine projections and compute score
        cv::Mat img(cv::Size(image_size_.first, image_size_.second), CV_8UC3, cv::Scalar(255, 255, 255));
        double frontier_score = 0.0;
        double occupied_score = 0.0;

        for (int y = 0; y < img.rows; ++y) {
            for (int x = 0; x < img.cols; ++x) {
                for (size_t i = 0; i < ellipsoids.size(); ++i) {
                    if (img_vec[i].at<uchar>(y, x) != 255) continue;

                    cv::Vec3b& color = img.at<cv::Vec3b>(y, x);
                    if (ellipsoids[i].type == "frontier") {
                        frontier_score += 255 * weights[i];
                        color = (color[1] == 255) ? cv::Vec3b(0, 0, 255) : cv::Vec3b(255, 0, 255);
                    } else if (ellipsoids[i].type == "occupied") {
                        occupied_score += 255 * weights[i];
                        color = (color[1] == 255) ? cv::Vec3b(255, 0, 0) : cv::Vec3b(255, 0, 255);
                    }
                }
            }
        }

        projection_img = img.clone();
        return frontier_score - occupied_score;
    }

    Eigen::Matrix4d createEllipsoidDualMatrix(const EllipsoidParam& param) {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
        Eigen::Vector3d radii_inv = param.radii.array().square().inverse();
        matrix.block<3, 3>(0, 0) = radii_inv.asDiagonal();
        matrix(3, 3) = -1;

        if (matrix.determinant() == 0) return Eigen::Matrix4d::Zero();

        Eigen::Matrix4d dual_origin = matrix.inverse();
        return param.pose * dual_origin * param.pose.transpose();
    }

    Eigen::Matrix3d computeEllipsoidProjection(
        const Eigen::Matrix<double, 3, 4>& camera_matrix,
        const Eigen::Matrix4d& dual_matrix) {
        Eigen::Matrix3d ellipse_dual = camera_matrix * dual_matrix * camera_matrix.transpose();
        if (ellipse_dual.determinant() == 0) return Eigen::Matrix3d::Zero();
        return ellipse_dual.inverse();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NBVActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}