import os
import sys

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from .network_interface import NetworkInterface
from .mission_decoder import MissionDecoder
from .behavior_tree import BehaviorTree
from .tasking import (
    TaskLeaf,
    IdentifyObjectLeaf,
    NextBestViewLeaf,
    GoToPositionLeaf,
    MovementLink,
    ActionType,
)
from kinova_action_interfaces.action import DetectObject, MoveTo


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

        self.detect_object_client: ActionClient = ActionClient(
            self, DetectObject, "/next_best_view/detect_object"
        )
        self.kinova_move_to_client: ActionClient = ActionClient(
            self, MoveTo, "/next_best_view/move_to_action"
        )

        self.action_callback_map: dict = {
            "identifyObject": self._send_detect_object_goal,
            "nextBestView": self._send_nbv_goal,
            "goToPosition": self._send_kinova_go_to_goal,
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

        if self.behavior_tree is None:
            self.get_logger().error(f"Empty behavior tree...")
        else:
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

    def _execute_tasks(self, root: TaskLeaf) -> None:
        curr: TaskLeaf = root
        while curr is not None:
            # TODO: decide if returning the entire action message result is a better design
            # TODO: feedback can be parsed by function from within tasking.py for each respective TaskLeaf subclass
            success: bool = self.action_callback_map[curr.action_type](curr)
            # if you have a conditional statment, this will always come first
            if curr.has_conditionals:
                for b, c in zip(curr.branches, curr.conditionals):
                    if isinstance(c.value, bool):
                        if c.value == success:
                            self.get_logger().info(
                                f"Executing {success} {c.comparator} {c.value} conditional branch of {curr.name}"
                            )
                            self._execute_tasks(b)
                    else:
                        self.get_logger().error(
                            f"Currently, {type(c.value)} comparisons are not supported. See {curr.name} and fix."
                        )

            curr = curr.next

    def _send_detect_object_goal(self, task: IdentifyObjectLeaf) -> bool:
        goal: DetectObject.Goal = DetectObject.Goal()
        goal.target_class = task.object_name
        goal.colors = task.colors

        self.detect_object_client.wait_for_server()
        self.get_logger().info(
            f"Detect Object Action server available. Sending goal to find {goal.target_class}..."
        )

        future = self.detect_object_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)

        result_future = self._goal_response_callback(future)

        rclpy.spin_until_future_complete(self, result_future)

        result: DetectObject.Result = self._result_callback(result_future)

        self.get_logger().debug(
            f"Detect Object Action Result: \n\
        X = {result.position.x} \n\
        Y = {result.position.y} \n\
        Z = {result.position.z} \n\
        confidence = {result.confidence}"
        )

        # TODO: we can convert this from X,Y movement to rotational movement (harder)
        task: GoToPositionLeaf = GoToPositionLeaf(
            "detect_object", ActionType.GOTOPOSITION, 0, "", MovementLink.BASE_LINK
        )
        task.set_pose(result.view_position)
        if not self._send_kinova_go_to_goal(task):
            result.success = False

        return result.success

    def _send_nbv_goal(self, task: NextBestViewLeaf):
        pass

    def _send_kinova_go_to_goal(self, task: GoToPositionLeaf) -> bool:
        goal: MoveTo.Goal = MoveTo.Goal()
        goal.task_name = task.name
        goal.movement_link = task.movement_link
        goal.pose = task.pose

        self.kinova_move_to_client.wait_for_server()
        self.get_logger().info(
            f"Kinova Move To server available. Sending move task {goal.task_name}..."
        )

        future = self.kinova_move_to_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)

        result_future = self._goal_response_callback(future)

        rclpy.spin_until_future_complete(self, result_future)

        result: MoveTo.Result = self._result_callback(result_future)

        self.get_logger().debug(
            f"Move To Result: \n\
        X = {goal.pose.x} \n\
        Y = {goal.pose.y} \n\
        Z = {goal.pose.z} ->\n\
        {result.success}"
        )

        return result.success

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        return goal_handle.get_result_async()

    def _result_callback(self, future):
        """Generic Action Result callback"""
        result = future.result().result
        self.get_logger().info(f"Result: Success = {result.success}")

        return result


def main(args=None) -> None:
    mp: MissionInterface = None

    try:
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init(args=args)
        mp = MissionInterface()

        try:
            rclpy.spin(mp)
        except SystemExit:
            mp.get_logger().info("Graceful exit and receipt of MissionPlan...")

    except KeyboardInterrupt:
        sys.exit(0)
    finally:
        if mp is not None:
            mp.destroy_node()
