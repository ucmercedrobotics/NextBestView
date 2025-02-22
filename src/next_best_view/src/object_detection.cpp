#include <memory>
#include <mutex>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "kinova_action_interfaces/action/detect_object.hpp"
#include "next_best_view/yolo_detector.hpp"

using namespace std::placeholders;
using DetectObject = kinova_action_interfaces::action::DetectObject;
using GoalHandleDetectObject = rclcpp_action::ServerGoalHandle<DetectObject>;

const float METER_TO_MILLIMETER = 1000.0f;

class ObjectDetectionNode : public rclcpp::Node {
public:
    ObjectDetectionNode()
        : Node("object_detection_node"),
          tf_buffer_(this->get_clock()),  // Use node's clock
          tf_listener_(tf_buffer_) {      // Pass buffer directly
        debug_ = true;

        // Initialize YOLO model (placeholder)
        yolo_model_ = std::make_unique<YoloDetector>("yolo11n.pt");

        // Action server
        action_server_ = rclcpp_action::create_server<DetectObject>(
            this,
            "/next_best_view/detect_object",
            std::bind(&ObjectDetectionNode::handle_goal, this, _1, _2),
            std::bind(&ObjectDetectionNode::handle_cancel, this, _1),
            std::bind(&ObjectDetectionNode::handle_accepted, this, _1)
        );

        // Image subscribers with synchronization
        color_sub_.subscribe(this, "/camera/color/image_raw");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(30), color_sub_, depth_sub_);  // Only queue size
        synchronizer_->registerCallback(
            std::bind(&ObjectDetectionNode::sync_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Object Detection Node has been started");
    }

private:
    bool debug_;
    std::unique_ptr<YoloDetector> yolo_model_;
    rclcpp_action::Server<DetectObject>::SharedPtr action_server_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> synchronizer_;

    struct Frames {
        cv::Mat bgr;
        cv::Mat depth;
    };
    std::optional<Frames> latest_frames_;
    std::mutex frames_mutex_;

    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
        try {
            cv::Mat bgr_image = imgmsg_to_cv2(*color, "bgr8");
            cv::Mat depth_image = imgmsg_to_cv2(*depth, "16UC1");

            std::lock_guard<std::mutex> lock(frames_mutex_);
            latest_frames_ = Frames{bgr_image.clone(), depth_image.clone()};
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing frames: %s", e.what());
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const DetectObject::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received detection request for %s",
                    goal->target_class.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDetectObject> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDetectObject> goal_handle) {
        std::thread{[this, goal_handle]() {
            this->detect_object_callback(goal_handle);
        }}.detach();
    }

    void detect_object_callback(const std::shared_ptr<GoalHandleDetectObject> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DetectObject::Feedback>();
        auto result = std::make_shared<DetectObject::Result>();

        std::string target_class = goal->target_class;
        float target_view_point_distance = goal->target_view_point_distance;  // Direct use as float

        Frames frames;
        {
            std::lock_guard<std::mutex> lock(frames_mutex_);
            if (!latest_frames_) {
                RCLCPP_ERROR(this->get_logger(), "No synchronized frame data available");
                goal_handle->abort(result);
                return;
            }
            frames = *latest_frames_;
        }

        try {
            auto detections = yolo_object_detection(frames, target_class);
            std::sort(detections.begin(), detections.end(),
                      [](const auto& a, const auto& b) { return a.confidence > b.confidence; });

            if (!detections.empty()) {
                const auto& best_detection = detections[0];
                auto [X, Y, Z] = compute_3d_position(best_detection);

                try {
                    *result = camera_to_base_link_transform(X, Y, Z, target_view_point_distance);
                    result->confidence = best_detection.confidence;
                    result->success = true;

                    std::stringstream ss;
                    ss << "Object detected at base_link coordinates ("
                       << result->object_position.x << ", " << result->object_position.y << ", "
                       << result->object_position.z << ") ("
                       << result->view_position.position.x << ", " << result->view_position.position.y << ", "
                       << result->view_position.position.z << ") ("
                       << result->view_position.orientation.x << ", " << result->view_position.orientation.y << ", "
                       << result->view_position.orientation.z << ", " << result->view_position.orientation.w << ") "
                       << "with confidence: " << result->confidence;
                    feedback->processing_status = ss.str();
                    goal_handle->publish_feedback(feedback);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "TF Transform failed: %s", e.what());
                    result->success = false;
                    result->confidence = 0.0;
                    feedback->processing_status = "Failed to transform object position to base_link";
                    goal_handle->publish_feedback(feedback);
                }
            } else {
                result->success = false;
                result->confidence = 0.0;
                feedback->processing_status = "No " + target_class + " found in image";
                goal_handle->publish_feedback(feedback);
            }

            if (debug_) {
                cv::Mat annotated = draw_detections(frames.bgr.clone(), detections, target_class);
                cv::imwrite(target_class + "_detected.jpg", annotated);
            }

            goal_handle->succeed(result);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Detection failed: %s", e.what());
            result->success = false;
            result->confidence = 0.0;
            goal_handle->succeed(result);
        }
    }

    DetectObject::Result camera_to_base_link_transform(float X, float Y, float Z,
                                                       float target_view_point_distance) {
        DetectObject::Result result;

        geometry_msgs::msg::PointStamped object_in_camera_frame;
        object_in_camera_frame.point.x = X;
        object_in_camera_frame.point.y = Y;
        object_in_camera_frame.point.z = Z;

        geometry_msgs::msg::PointStamped object_view_point;
        object_view_point.point.x = X;
        object_view_point.point.y = Y;
        object_view_point.point.z = Z - target_view_point_distance;

        auto transform = tf_buffer_.lookupTransform("base_link", "camera_link",
                                                    rclcpp::Time(0));
        geometry_msgs::msg::PointStamped object_in_base_frame;
        geometry_msgs::msg::PointStamped object_view_point_in_base_frame;
        tf2::doTransform(object_in_camera_frame, object_in_base_frame, transform);
        tf2::doTransform(object_view_point, object_view_point_in_base_frame, transform);

        result.object_position = object_in_base_frame.point;
        result.view_position.position = object_view_point_in_base_frame.point;

        Eigen::Vector3f object_pos(
            object_in_base_frame.point.x,
            object_in_base_frame.point.y,
            object_in_base_frame.point.z
        );
        Eigen::Vector3f view_pos(
            object_view_point_in_base_frame.point.x,
            object_view_point_in_base_frame.point.y,
            object_view_point_in_base_frame.point.z
        );

        auto quaternion = compute_orientation(object_pos, view_pos);
        result.view_position.orientation.x = quaternion.x();
        result.view_position.orientation.y = quaternion.y();
        result.view_position.orientation.z = quaternion.z();
        result.view_position.orientation.w = quaternion.w();

        return result;
    }

    Eigen::Quaternionf compute_orientation(const Eigen::Vector3f& object_pos,
                                           const Eigen::Vector3f& view_pos) {
        Eigen::Vector3f Z = (object_pos - view_pos).normalized();
        Eigen::Vector3f Y = Eigen::Vector3f::UnitZ();

        Y = Y - Z * Z.dot(Y);
        float Y_norm = Y.norm();
        if (Y_norm < 1e-6f) {
            if (std::abs(Z.z()) > 0.99f) {
                Y = Eigen::Vector3f::UnitY();
            } else {
                Y = Eigen::Vector3f::UnitX();
            }
            Y = Y - Z * Z.dot(Y);
            Y.normalize();
        } else {
            Y.normalize();
        }

        Eigen::Vector3f X = Y.cross(Z).normalized();

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << X, Y, Z;

        return Eigen::Quaternionf(rotation_matrix);
    }

    struct Detection {
        float confidence;
        std::vector<float> bbox; // [x1, y1, x2, y2]
        std::vector<int> center; // [x, y]
        float depth;
        std::vector<int> depth_center; // [x, y]
    };

    std::vector<Detection> yolo_object_detection(const Frames& frames,
                                                const std::string& target_class) {
        std::vector<Detection> detections;
        auto results = yolo_model_->detect(frames.bgr);

        int bgr_h = frames.bgr.rows, bgr_w = frames.bgr.cols;
        int depth_h = frames.depth.rows, depth_w = frames.depth.cols;
        float scale_x = static_cast<float>(depth_w) / bgr_w;
        float scale_y = static_cast<float>(depth_h) / bgr_h;

        for (const auto& result : results) {
            if (result.class_name.find(target_class) != std::string::npos) {
                Detection det;
                det.confidence = result.confidence;
                det.bbox = result.bbox;
                det.center = {(int)(result.bbox[0] + result.bbox[2]) / 2,
                              (int)(result.bbox[1] + result.bbox[3]) / 2};
                det.depth_center = {(int)(det.center[0] * scale_x),
                                    (int)(det.center[1] * scale_y)};
                det.depth_center[0] = std::clamp(det.depth_center[0], 0, depth_w - 1);
                det.depth_center[1] = std::clamp(det.depth_center[1], 0, depth_h - 1);
                det.depth = frames.depth.at<uint16_t>(det.depth_center[1], det.depth_center[0]) / METER_TO_MILLIMETER;
                detections.push_back(det);
                RCLCPP_INFO(this->get_logger(), "%s detected at depth %f m",
                            target_class.c_str(), det.depth);
            }
        }
        return detections;
    }

    std::tuple<float, float, float> compute_3d_position(const Detection& object) {
        const float fx = 360.01333f, fy = 360.01333f;
        const float cx = 243.87228f, cy = 137.9218444f;

        float X = (object.depth_center[0] - cx) * object.depth / fx;
        float Y = (object.depth_center[1] - cy) * object.depth / fy;
        float Z = object.depth;

        return {X, Y, Z};
    }

    cv::Mat draw_detections(cv::Mat image, const std::vector<Detection>& detections,
                            const std::string& target_class) {
        for (const auto& det : detections) {
            cv::rectangle(image,
                          cv::Point(det.bbox[0], det.bbox[1]),
                          cv::Point(det.bbox[2], det.bbox[3]),
                          cv::Scalar(0, 255, 0), 2);
            std::string label = target_class + ": " + std::to_string(det.confidence);
            cv::putText(image, label, cv::Point(det.bbox[0], det.bbox[1] - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::circle(image, cv::Point(det.center[0], det.center[1]), 4,
                       cv::Scalar(255, 0, 0), -1);
        }
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        return image;
    }

    cv::Mat imgmsg_to_cv2(const sensor_msgs::msg::Image& img_msg, const std::string& encoding) {
        if (encoding == "bgr8") {
            return cv::Mat(img_msg.height, img_msg.width, CV_8UC3,
                           const_cast<unsigned char*>(img_msg.data.data())).clone();
        } else { // "16UC1"
            cv::Mat raw(img_msg.height, img_msg.width * 2, CV_8U,
                        const_cast<unsigned char*>(img_msg.data.data()));
            cv::Mat depth(img_msg.height, img_msg.width, CV_16U);
            for (size_t i = 0; i < img_msg.height; ++i) {
                for (size_t j = 0; j < img_msg.width; ++j) {
                    depth.at<uint16_t>(i, j) = (raw.at<uint8_t>(i, j * 2 + 1) << 8) +
                                               raw.at<uint8_t>(i, j * 2);
                }
            }
            return depth.clone();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}