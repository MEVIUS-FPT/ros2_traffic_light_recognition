import sys
import rclpy
from rclpy.node import Node

from traffic_light_recognition_interface.msg import *


class ListenerNode(Node):
    def __init__(self):
        super().__init__("listener")
        self.declare_parameter(
            "fpga_out_topic", "fpga_out_topic_traffic_light_recogn_0"
        )
        self.declare_parameter("msg_type_suffix", "1")
        msg_module_str = "traffic_light_recognition_interface.msg"
        msg_type_suffix = self.get_parameter("msg_type_suffix").value
        FpgaOut = getattr(
            sys.modules[msg_module_str], "FpgaOut" + msg_type_suffix
        )
        fpga_out_topic = self.get_parameter("fpga_out_topic").value
        self.create_subscription(
            FpgaOut, fpga_out_topic, self.read_node_callback, 10
        )

    def read_node_callback(self, msg):
        self.get_logger().info("Received: {}".format(msg))


def main(args=None):
    rclpy.init(args=args)

    listener = ListenerNode()

    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
