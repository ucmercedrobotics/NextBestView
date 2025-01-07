#!/usr/bin/env python3
from threading import Lock
import sys

import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image

from kinova_action_interfaces.action import DetectObject


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")

        self.debug: bool = True

        # TODO: replace with cv_bridge once it's fixed
        # Initialize CV Bridge
        # self.bridge: CvBridge = CvBridge()
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

        self._color_sub = Subscriber(self, Image, "/camera/color/image_raw")
        self._depth_sub = Subscriber(self, Image, "/camera/depth/image_raw")
        # self._color_sub = Subscriber(self, Image, "/wrist_mounted_camera/image")
        # self._depth_sub = Subscriber(self, Image, "/wrist_mounted_camera/depth_image")
        self._time_sync = ApproximateTimeSynchronizer(
            [
                self._color_sub,
                self._depth_sub,
            ],  # Synchronized subscribers
            30,
            0.5,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self._time_sync.registerCallback(self.sync_callback)

        self.get_logger().info("Object Detection Node has been started")

    def sync_callback(self, color: Image, depth: Image):
        """Callback for synchronized RGB and depth images"""
        try:
            # Convert ROS Image messages to OpenCV format
            rgb_image = self._imgmsg_to_cv2(color, desired_encoding="rgb8")
            depth_image = self._imgmsg_to_cv2(depth, desired_encoding="16UC1")

            # Store synchronized frames with thread safety
            with self.frames_lock:
                self.latest_frames = {
                    "rgb": rgb_image,
                    "depth": depth_image,
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

        # Get latest synchronized frames with thread safety
        with self.frames_lock:
            if self.latest_frames is None:
                raise RuntimeError("No synchronized frame data available")
            frames = self.latest_frames.copy()

        # try:

        # Run YOLOv8 detection on RGB image
        results = self.model(frames["rgb"])

        # Get image dimensions
        rgb_h, rgb_w = frames["rgb"].shape[:2]
        depth_h, depth_w = frames["depth"].shape[:2]

        # Calculate scaling factors
        scale_x = depth_w / rgb_w
        scale_y = depth_h / rgb_h

        # Get detections for requested object class
        detections = []

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                cls_name = self.model.names[cls]

                if target_class in cls_name:
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    # Calculate center point
                    rgb_center_x = int((x1 + x2) / 2)
                    rgb_center_y = int((y1 + y2) / 2)

                    center_x = int(rgb_center_x * scale_x)
                    center_y = int(rgb_center_y * scale_y)

                    # Ensure we're within depth image bounds
                    center_x = min(max(0, center_x), depth_w - 1)
                    center_y = min(max(0, center_y), depth_h - 1)

                    # Get depth at center point (convert to meters)
                    depth = (
                        float(np.average(frames["depth"][center_y, center_x])) / 1000.0
                    )
                    self.get_logger().info(f"{target_class} object depth {depth}m")

                    detections.append(
                        {
                            "confidence": conf,
                            "bbox": [x1, y1, x2, y2],
                            "center": [rgb_center_x, rgb_center_y],
                            "depth": depth,
                            "depth_center": [center_x, center_y],
                        }
                    )

        # Sort detections by confidence
        detections.sort(key=lambda x: x["confidence"], reverse=True)

        if detections:
            best_detection = detections[0]

            # Calculate 3D position
            # TODO: figure out if we have and adjust these
            fx = 360.01333
            fy = 360.01333
            cx = 243.87228
            cy = 137.9218444

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

        # except Exception as e:
        #     self.get_logger().error(f"Detection failed: {str(e)}")
        #     result.success = False
        #     result.confidence = 0.0
        #     goal_handle.succeed()

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

            image_with_boxes = cv2.cvtColor(image_with_boxes, cv2.COLOR_BGR2RGB)
            cv2.imwrite(f"{target_class}_detected.jpg", image_with_boxes)

        return image_with_boxes

    def _imgmsg_to_cv2(self, img_msg, desired_encoding):
        """
        This function is required to replace cv_bridge since it's not built with numpy 2.
        There is an error loading the module since it isn't compatible with the pyenv.
        Instead, we rewrite the function we use from that library to get rid of it until they fix.
        NOTE: I'm not sure if this actually does what it's supposed to do.

        "bgr8" from https://robotics.stackexchange.com/questions/95630/cv-bridge-throws-boost-import-error-in-python-3-and-ros-melodic
        "passthrough" from https://gist.github.com/Merwanski/39580a5fc276583b0921eb44dd91a61e
        """
        if desired_encoding == "rgb8":
            dtype = np.dtype("uint8")  # Hardcode to 8 bits...
            image_opencv = np.ndarray(
                shape=(
                    img_msg.height,
                    img_msg.width,
                    3,
                ),  # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                dtype=dtype,
                buffer=img_msg.data,
            )
            return image_opencv
        else:
            # Get the image dimensions
            height = img_msg.height
            width = img_msg.width

            # Get raw data from message
            raw_data = np.frombuffer(img_msg.data, dtype=np.uint8)

            # Since the data is 16-bit but stored in 8-bit bytes,
            # we need to reshape and reinterpret it
            raw_data = raw_data.reshape(-1, 2)  # Pair bytes together
            depth_data = raw_data[:, 0].astype(np.uint16) + (
                raw_data[:, 1].astype(np.uint16) << 8
            )

            # Reshape to 2D array with correct dimensions
            image_opencv = depth_data.reshape(height, width)

            return image_opencv


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
