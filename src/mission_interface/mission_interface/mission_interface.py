import os
import sys

import yaml
import rclpy
from rclpy.node import Node

from .network_interface import NetworkInterface
from .mission_decoder import MissionDecoder
from .behavior_tree import BehaviorTree

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

        # TODO: replace this with behavior tree object
        # TODO: parse tasks into behavior tree w/ feedback

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
            ret = self.behavior_tree.execute_tasks()
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
