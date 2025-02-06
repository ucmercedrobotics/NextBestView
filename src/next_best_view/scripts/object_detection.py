#!/usr/bin/env python3
from typing import Tuple
from threading import Lock

import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from scipy.spatial.transform import Rotation

from kinova_action_interfaces.action import DetectObject

METER_TO_MILLIMETER: float = 1000.0


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_node")

        self._debug: bool = True

        # Load YOLO model
        self._model: YOLO = YOLO("yolo11n.pt")

        # Create action server
        self._action_server: ActionServer = ActionServer(
            self,
            DetectObject,
            "/next_best_view/detect_object",
            self.detect_object_callback,
        )

        # Latest synchronized images storage with thread safety
        self._latest_frames: dict = None
        self._frames_lock: Lock = Lock()

        # image topics
        self._color_sub: Subscriber = Subscriber(self, Image, "/camera/color/image_raw")
        self._depth_sub: Subscriber = Subscriber(self, Image, "/camera/depth/image_raw")
        # Synchronized subscribers
        self._time_sync: ApproximateTimeSynchronizer = ApproximateTimeSynchronizer(
            [
                self._color_sub,
                self._depth_sub,
            ],
            30,
            0.5,
        )
        # sync this function
        self._time_sync.registerCallback(self.sync_callback)

        # Initialize tf2 buffer and listener for transforms
        self._tf_buffer: tf2_ros.Buffer = tf2_ros.Buffer()
        self._tf_listener: tf2_ros.TransformListener = tf2_ros.TransformListener(
            self._tf_buffer, self
        )

        self.get_logger().info("Object Detection Node has been started")

    def sync_callback(self, color: Image, depth: Image) -> None:
        """Callback for synchronized BGR and depth images

        Args:
            color (Image): BGR image
            depth (Image): depth image
        """
        try:
            # Convert ROS Image messages to OpenCV format
            bgr_image = self._imgmsg_to_cv2(color, desired_encoding="bgr8")
            depth_image = self._imgmsg_to_cv2(depth, desired_encoding="16UC1")

            # Store synchronized frames with thread safety
            with self._frames_lock:
                self._latest_frames = {
                    "bgr": bgr_image,
                    "depth": depth_image,
                }

        except Exception as e:
            self.get_logger().error(f"Error processing synchronized frames: {str(e)}")

    async def detect_object_callback(self, goal_handle) -> DetectObject.Result:
        """Action server callback to process detection requests"""
        self.get_logger().info("Received detection request")

        # create DetectObject action type messages
        feedback_msg: DetectObject.Feedback = DetectObject.Feedback()
        result: DetectObject.Result = DetectObject.Result()

        # Get the target class from the goal
        target_class: str = goal_handle.request.target_class
        target_view_point_distance: str = goal_handle.request.target_view_point_distance
        # TODO: add colors here

        # Get latest synchronized frames with thread safety
        with self._frames_lock:
            if self._latest_frames is None:
                raise RuntimeError("No synchronized frame data available")
            frames = self._latest_frames.copy()

        try:
            detections = self._yolo_object_detection(frames, target_class)

            # Sort detections by confidence
            detections.sort(key=lambda x: x["confidence"], reverse=True)

            if detections:
                best_detection = detections[0]

                X, Y, Z = self._compute_3d_position(best_detection)

                try:
                    result = self._camera_to_base_link_transform(
                        X, Y, Z, target_view_point_distance
                    )

                    result.confidence = best_detection["confidence"]
                    result.success = True

                    feedback_msg.processing_status = (
                        f"Object detected at base_link coordinates "
                        f"({result.object_position.x:.2f}, {result.object_position.y:.2f}, {result.object_position.z:.2f}) "
                        f"({result.view_position.position.x:.2f}, {result.view_position.position.y:.2f}, {result.view_position.position.z:.2f}) "
                        f"({result.view_position.orientation.x:.2f}, {result.view_position.orientation.y:.2f}, {result.view_position.orientation.z:.2f}), {result.view_position.orientation.w:.2f})"
                        f"with confidence: {result.confidence:.2f}"
                    )
                    goal_handle.publish_feedback(feedback_msg)
                except Exception as e:
                    self.get_logger().error(f"TF Transform failed: {str(e)}")
                    result.success = False
                    result.confidence = 0.0
                    feedback_msg.processing_status = (
                        "Failed to transform object position to base_link"
                    )
                    goal_handle.publish_feedback(feedback_msg)

            else:
                result.success = False
                result.confidence = 0.0
                feedback_msg.processing_status = f"No {target_class} found in image"
                goal_handle.publish_feedback(feedback_msg)

            goal_handle.succeed()

            if self._debug == True:  # Print debug images
                annotated_image = self._draw_detections(
                    frames["bgr"].copy(), detections, target_class
                )
                cv2.imwrite(f"{target_class}_detected.jpg", annotated_image)

        except Exception as e:
            self.get_logger().error(f"Detection failed: {str(e)}")
            result.success = False
            result.confidence = 0.0
            goal_handle.succeed()

        return result

    def _camera_to_base_link_transform(
        self, X: float, Y: float, Z: float, target_view_point_distance: float
    ) -> DetectObject.Result:
        """This function converts an incoming 3D point with respect to camera_link to base_link

        Args:
            X (float): X coordinate
            Y (float): Y coordinate
            Z (float): Z coordinate
            target_view_point_distance (float): how far the camera is desired from object

        Returns:
            DetectObject.Result: ROS2 message result type for DetectObject
        """
        result: DetectObject.Result = DetectObject.Result()

        # Transform the object position from camera_link to base_link
        object_in_camera_frame = PointStamped()
        object_in_camera_frame.point.x = X
        object_in_camera_frame.point.y = Y
        object_in_camera_frame.point.z = Z

        object_view_point = PointStamped()
        object_view_point.point.x = X
        object_view_point.point.y = Y
        object_view_point.point.z = Z - target_view_point_distance

        transform = self._tf_buffer.lookup_transform(
            "base_link", "camera_link", rclpy.time.Time()
        )
        object_in_base_frame = do_transform_point(object_in_camera_frame, transform)

        result.object_position.x = object_in_base_frame.point.x
        result.object_position.y = object_in_base_frame.point.y
        result.object_position.z = object_in_base_frame.point.z

        object_view_point_in_base_frame = do_transform_point(
            object_view_point, transform
        )

        result.view_position.position.x = object_view_point_in_base_frame.point.x
        result.view_position.position.y = object_view_point_in_base_frame.point.y
        result.view_position.position.z = object_view_point_in_base_frame.point.z

        transform_ee = self._tf_buffer.lookup_transform(
            "base_link", "end_effector_link", rclpy.time.Time()
        )

        quaternion: np.array = self._compute_orientation(
            transform_ee, object_in_base_frame, object_view_point
        )

        result.view_position.orientation.x = quaternion[0]
        result.view_position.orientation.y = quaternion[1]
        result.view_position.orientation.z = quaternion[2]
        result.view_position.orientation.w = quaternion[3]

        return result

    def _compute_orientation(
        self,
        base_to_ee_transform: TransformStamped,
        object_in_base_frame: PointStamped,
        object_view_point: PointStamped,
    ) -> np.array:
        # Extract rotation (quaternion)
        end_effector_quat = [
            base_to_ee_transform.transform.rotation.x,
            base_to_ee_transform.transform.rotation.y,
            base_to_ee_transform.transform.rotation.z,
            base_to_ee_transform.transform.rotation.w,
        ]

        # print(f"end_effector_quat= x:{end_effector_quat[0]}, y:{end_effector_quat[1]}, z:{end_effector_quat[2]}, w:{end_effector_quat[3]} ")

        end_effector_rotation: Rotation = Rotation.from_quat(end_effector_quat)

        # print(f"end_effector_rotation= {end_effector_rotation.as_matrix()}")

        # Calculate direction vector
        direction = np.array(
            [
                object_view_point.point.x - object_in_base_frame.point.x,
                object_view_point.point.y - object_in_base_frame.point.y,
                object_view_point.point.z - object_in_base_frame.point.z,
            ]
        )

        z_axis_target = direction / np.linalg.norm(direction)

        z_axis_current = end_effector_rotation.as_matrix()[
            :, 2
        ]  # Third column is the current Z-axis

        # Compute the axis of rotation (cross product)
        axis = np.cross(z_axis_current, z_axis_target)
        axis = axis / np.linalg.norm(axis)  # Normalize the rotation axis

        # Compute the angle of rotation (dot product)
        cos_theta = np.dot(z_axis_current, z_axis_target)
        theta = np.arccos(cos_theta)

        # Rodrigues' rotation formula
        K = np.array(
            [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
        )

        I = np.eye(3)
        R_align = I + np.sin(theta) * K + np.dot(K, K) * (1 - np.cos(theta))

        rotation: Rotation = Rotation.from_matrix(R_align)
        quaternion: np.array = rotation.as_quat()

        self.get_logger().debug(f"quaternion of alligned end_effector = {quaternion}")

        return quaternion

    def _yolo_object_detection(self, frames: dict, target_class: str) -> list[dict]:
        """Detect object using YOLOv11 from BGR image frame and compute depth

        Args:
            frames (dict): incoming BGR and depth frames
            target_class (str): object name

        Returns:
            list[dict]: confidence, bounding box, BGR center, depth center, depth
        """
        # Run YOLOv11 detection on BGR image
        results = self._model(frames["bgr"])

        # Get image dimensions
        bgr_h, bgr_w = frames["bgr"].shape[:2]
        depth_h, depth_w = frames["depth"].shape[:2]

        # Calculate scaling factors
        scale_x = depth_w / bgr_w
        scale_y = depth_h / bgr_h

        # Get detections for requested object class
        detections = []

        # iterate through all object results
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                cls_name = self._model.names[cls]

                # if one of the objects is what you were looking for
                if target_class.lower() in cls_name.lower():
                    conf = float(box.conf[0])
                    # bounding box (bgr)
                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    # Calculate center point (bgr)
                    bgr_center_x = int((x1 + x2) / 2)
                    bgr_center_y = int((y1 + y2) / 2)

                    # Calculate center point (depth)
                    center_x = int(bgr_center_x * scale_x)
                    center_y = int(bgr_center_y * scale_y)

                    # Ensure we're within depth image bounds (depth)
                    center_x = min(max(0, center_x), depth_w - 1)
                    center_y = min(max(0, center_y), depth_h - 1)

                    # Get depth at center point (convert to meters)
                    depth = (
                        float(np.average(frames["depth"][center_y, center_x]))
                        / METER_TO_MILLIMETER
                    )
                    self.get_logger().info(
                        f"{target_class} object detected at depth of {depth}m"
                    )

                    detections.append(
                        {
                            "confidence": conf,
                            "bbox": [x1, y1, x2, y2],
                            "center": [bgr_center_x, bgr_center_y],
                            "depth": depth,
                            "depth_center": [center_x, center_y],
                        }
                    )

        return detections

    def _compute_3d_position(self, object: dict) -> Tuple[float, float, float]:
        # Calculate 3D position
        # TODO: move these into some configuration for the camera or parse them from the /camera_info topic
        fx = 360.01333
        fy = 360.01333
        cx = 243.87228
        cy = 137.9218444

        # this represents the object's left-right position (left is negative, right is positive).
        X = (object["depth_center"][0] - cx) * object["depth"] / fx
        # this represents the object's up-down position (up is negative, down is positive).
        Y = (object["depth_center"][1] - cy) * object["depth"] / fy
        # depth
        Z = object["depth"]

        return X, Y, Z

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

            # the kinova camera natively is BGR. Convert to RGB.
            image_with_boxes = cv2.cvtColor(image_with_boxes, cv2.COLOR_BGR2RGB)

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
        if desired_encoding == "bgr8":
            dtype = np.dtype("uint8")  # Hardcode to 8 bits...
            image_opencv = np.ndarray(
                shape=(
                    img_msg.height,
                    img_msg.width,
                    3,
                ),
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
