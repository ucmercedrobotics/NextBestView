import os
import sys

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from .network_interface import NetworkInterface
from .mission_decoder import MissionDecoder
from .behavior_tree import BehaviorTree
from .tasking import TaskLeaf, IdentifyObjectLeaf, NextBestViewLeaf
from kinova_action_interfaces.action import DetectObject


NODE_NAME: str = "kinova_mission_interface"


class MissionInterface(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("config", "src/mission_interface/config/kinova.yaml"),
            ],
        )

        self.config: str = (
            self.get_parameter("config").get_parameter_value().string_value
        )

        with open(self.config, "r") as file:
            config_yaml: yaml.Node = yaml.safe_load(file)

        # logging GPT output folder
        self.log_directory: str = config_yaml["log_directory"]
        rclpy.logging.set_logger_level(
            NODE_NAME, rclpy.logging.LoggingSeverity(config_yaml["logging"])
        )
        # decoder object
        self.decoder: MissionDecoder = MissionDecoder(
            config_yaml["schema"], self.get_logger()
        )
        # behavior tree object
        self.behavior_tree: BehaviorTree | None = None

        self._configure_network(config_yaml["host"], config_yaml["port"])

        self.detect_object_client = ActionClient(self, DetectObject, "detect_object")

        self.action_callback_map: dict = {
            "identifyObject": self._send_detect_object_goal,
            "nextBestView": self._send_nbv_goal,
        }

        # while True:
        self.nic.init_socket()
        ret: bool = self.run()
        self.nic.close_socket()
        #     if ret is True:
        #         break

        # raise SystemExit

    def run(self) -> bool:
        bytes_received, temp_xml_path = self.nic.receive_file()
        if bytes_received == 0:
            self.get_logger().warn("No mission plan was received over TCP...")
            return False

        ret, e = self.decoder.validate_output(temp_xml_path)
        if ret:
            self.behavior_tree = self.decoder.decode_xml_mp(temp_xml_path)
        else:
            self.get_logger().error(e)

        # if there is at least one task in the tree
        if self.behavior_tree.task_root is not None:
            # TODO: update this with a custom message that has a list of waypoints and robot actions
            self.get_logger().debug("Mission plan received successfully...")
            # self.behavior_tree.__repr__()
            ret = self._execute_tasks(self.behavior_tree.task_root)
        # if you actually didn't receive anything
        else:
            os.remove(temp_xml_path)
            self.get_logger().error("Mission plan not received...")
            return False

        return ret

    def _configure_network(self, host: str, port: int) -> None:
        self.nic: NetworkInterface = NetworkInterface(
            self.get_logger(), self.log_directory, host, port
        )

    def _execute_tasks(self, root: TaskLeaf) -> bool:
        curr: TaskLeaf = root
        while curr is not None:
            # if you have a conditional statment, this will always come first
            if curr.has_conditionals:
                # start with the first side of the branch
                # TODO: execute task through action/service
                # TODO: check outcome
                # TODO: if true, next true child
                # TODO: if false, check if there is a FALSE_BRANCH_IDX child
                # TODO: if not, you've failed
                # TODO: iterate recursively
                pass

            # otherwise, you would only have linear tasks, but you can have both.
            self.action_callback_map[curr.action_type](curr)

            curr = curr.next

        return True

    def _send_detect_object_goal(self, task: IdentifyObjectLeaf):
        goal: DetectObject.Goal = DetectObject.Goal()
        goal.target_class = task.object_name
        goal.colors = task.colors

        self.detect_object_client.wait_for_server()
        self.get_logger().info(
            f"Detect Object Action server available. Sending goal to find {goal.target_class}..."
        )
        self.detect_object_client.send_goal_async(goal).add_done_callback(
            self.goal_response_callback
        )

        # TODO: add centering inverse kinematic action call (moveit)

    def _send_nbv_goal(self, task: NextBestViewLeaf):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Generic Action Result callback"""
        result = future.result().result
        self.get_logger().info(f"Result: Success = {result.success}")
        # self.get_logger().info(
        #     f"Detect Object Action Result: Success = {result.success} \n\
        # X = {result.position.x} \n\
        # Y = {result.position.y} \n\
        # Z = {result.position.z} \n\
        # confidence = {result.confidence}"
        # )


def main(args=None) -> None:
    try:
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init(args=args)
        mp: MissionInterface = MissionInterface()

        try:
            rclpy.spin(mp)
        except SystemExit:
            mp.get_logger().info("Graceful exit and receipt of MissionPlan...")

    except KeyboardInterrupt:
        mp.get_logger().info("Ctrl+C received - exiting...")
        sys.exit(0)
    finally:
        mp.get_logger().info("ROS MissionInterface node shutdown...")
        if mp is not None:
            mp.destroy_node()
