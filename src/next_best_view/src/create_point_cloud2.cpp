#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudCreator : public rclcpp::Node {
 public:
  PointCloudCreator() : Node("point_cloud_creator") {
    // Subscribers for color and depth images
    color_sub_.subscribe(this, "/camera/color/image_raw");
    depth_sub_.subscribe(this, "/camera/depth/image_raw");

    // Synchronize color and depth messages
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), color_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&PointCloudCreator::callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    // Publisher for the point cloud
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/point_cloud", 10);
  }

 private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    // Convert ROS images to OpenCV images
    cv::Mat color_image = cv_bridge::toCvCopy(color_msg, "bgr8")->image;
    cv::Mat depth_image =
        cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)
            ->image;

    // Check if images are valid
    if (color_image.empty() || depth_image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty color or depth image received.");
      return;
    }

    // Resize depth image to match color image size
    cv::resize(depth_image, depth_image, color_image.size());

    // Create point cloud from color and depth images
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    createPointCloud(color_image, depth_image, cloud);

    // Convert PCL point cloud to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header = color_msg->header;  // Use the color image header

    // Publish the point cloud
    pointcloud_pub_->publish(cloud_msg);
    RCLCPP_INFO(this->get_logger(), "Published point cloud.");
  }

  void createPointCloud(const cv::Mat& color_image, const cv::Mat& depth_image,
                        pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    // TODO: Camera intrinsic parameters ask Marcos how did he get them
    double fx = 360.01333;    // Focal length in x
    double fy = 360.01333;    // Focal length in y
    double cx = 243.87228;    // Principal point in x
    double cy = 137.9218444;  // Principal point in y

    // fx = 360.01333
    // fy = 360.01333
    // cx = 243.87228
    // cy = 137.9218444

    cloud.width = color_image.cols;
    cloud.height = color_image.rows;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int v = 0; v < color_image.rows; v++) {
      for (int u = 0; u < color_image.cols; u++) {
        float depth = depth_image.at<float>(v, u);

        if (depth > 0) {  // Ignore invalid depth values
          pcl::PointXYZRGB point;
          point.x = (u - cx) * depth / fx;
          point.y = (v - cy) * depth / fy;
          point.z = depth;

          // Set the color from the color image
          cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
          point.r = color[2];
          point.g = color[1];
          point.b = color[0];

          cloud.points[v * color_image.cols + u] = point;
        }
      }
    }
  }

  // Subscribers for color and depth images
  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;

  // Synchronizer for color and depth messages
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publisher for the point cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudCreator>());
  rclcpp::shutdown();
  return 0;
}
