#!/usr/bin/env python3
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

from kinova_action_interfaces.action import DetectObject


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")

        self.debug: bool = True

        # Initialize CV Bridge
        self.bridge: CvBridge = CvBridge()
        # Load YOLO model
        self.model: YOLO = YOLO("yolo11n.pt")

        # Create action server
        self._action_server = ActionServer(
            self,
            DetectObject,
            "detect_object",
            self.detect_object_callback,
        )

        # Latest synchronized images storage with thread safety
        self.latest_frames = None
        self.frames_lock = Lock()

        self._color_sub = Subscriber(self, Image, "/color/image")
        self._depth_sub = Subscriber(self, Image, "/stereo/depth")
        self._info_sub = Subscriber(self, CameraInfo, "/stereo/camera_info")
        self._time_sync = ApproximateTimeSynchronizer(
            [
                self._color_sub,
                self._depth_sub,
                self._info_sub,
            ],  # Synchronized subscribers
            30,
            0.5,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self._time_sync.registerCallback(self.sync_callback)

        self.get_logger().info("Object Detection Node has been started")

    def sync_callback(self, color: Image, depth: Image, info: CameraInfo):
        """Callback for synchronized RGB and depth images"""
        try:
            # Convert ROS Image messages to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth, desired_encoding="passthrough"
            )

            # Store synchronized frames with thread safety
            with self.frames_lock:
                self.latest_frames = {
                    "rgb": rgb_image,
                    "depth": depth_image,
                    "header": color.header,
                }

        except Exception as e:
            self.get_logger().error(f"Error processing synchronized frames: {str(e)}")

    async def detect_object_callback(self, goal_handle):
        """Action server callback to process detection requests"""
        self.get_logger().info("Received detection request")

        feedback_msg = DetectObject.Feedback()
        result = DetectObject.Result()

        # Get the target class from the goal
        target_class = goal_handle.request.target_class
        # TODO: add colors here

        try:
            # Get latest synchronized frames with thread safety
            with self.frames_lock:
                if self.latest_frames is None:
                    raise RuntimeError("No synchronized frame data available")
                frames = self.latest_frames.copy()

            # Run YOLOv8 detection on RGB image
            results = self.model(frames["rgb"])

            # Get detections for requested object class
            detections = []

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    cls_name = self.model.names[cls]

                    if cls_name == target_class:
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = box.xyxy[0].tolist()

                        # Calculate center point
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        # Get depth at center point (convert to meters)
                        depth = float(frames["depth"][center_y, center_x]) / 1000.0

                        detections.append(
                            {
                                "confidence": conf,
                                "bbox": [x1, y1, x2, y2],
                                "center": [center_x, center_y],
                                "depth": depth,
                            }
                        )

            # Sort detections by confidence
            detections.sort(key=lambda x: x["confidence"], reverse=True)

            if detections:
                best_detection = detections[0]

                # Calculate 3D position
                # Note: Adjust these values based on your camera calibration
                fx = 525.0  # focal length x
                fy = 525.0  # focal length y
                cx = 319.5  # optical center x
                cy = 239.5  # optical center y

                X = (best_detection["center"][0] - cx) * best_detection["depth"] / fx
                Y = (best_detection["center"][1] - cy) * best_detection["depth"] / fy
                Z = best_detection["depth"]

                # Set result
                result.success = True
                result.position.x = float(X)
                result.position.y = float(Y)
                result.position.z = float(Z)
                result.confidence = best_detection["confidence"]

                # Send feedback
                feedback_msg.processing_status = (
                    f"Object detected at ({X:.2f}, {Y:.2f}, {Z:.2f}) "
                    f"with confidence: {result.confidence:.2f}"
                )
                goal_handle.publish_feedback(feedback_msg)

            else:
                result.success = False
                result.confidence = 0.0
                feedback_msg.processing_status = f"No {target_class} found in image"
                goal_handle.publish_feedback(feedback_msg)

            goal_handle.succeed()

            if self.debug == True:  # Print debug images
                annotated_image = self._draw_detections(
                    frames["rgb"].copy(), detections, target_class
                )

                cv2.imshow("image_with_object", annotated_image)

        except Exception as e:
            self.get_logger().error(f"Detection failed: {str(e)}")
            result.success = False
            result.confidence = 0.0
            goal_handle.succeed()

        return result

    def _draw_detections(self, image, detections, target_class):
        """Draw bounding boxes and labels on the image"""
        image_with_boxes = image.copy()

        for det in detections:
            # Get bbox coordinates
            x1, y1, x2, y2 = map(int, det["bbox"])
            conf = det["confidence"]

            # Draw bounding box
            cv2.rectangle(image_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label with confidence
            label = f"{target_class}: {conf:.2f}"
            cv2.putText(
                image_with_boxes,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

            # Draw center point
            center_x, center_y = det["center"]
            cv2.circle(
                image_with_boxes, (int(center_x), int(center_y)), 4, (255, 0, 0), -1
            )

        return image_with_boxes


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
