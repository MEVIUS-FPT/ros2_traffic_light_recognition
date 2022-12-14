import sys

# Import ROS python and Node functionalities
import rclpy
from rclpy.node import Node

# Import the messages defined for the ROS FPGA Node interface
from traffic_light_recognition_interface.msg import *

# ros_fpga_lib has the functions to interact with the FPGA HW
from .ros_fpga_lib import *


# Define the FPGA Node class (has publisher and subscriber methods)
class FpgaNode(Node):
    def __init__(self):
        super().__init__("fpga_node")
        param_list = [
            ("fpga_out_topic", "fpga_out_topic_traffic_light_recogn_0"),
            ("fpga_in_topic", "fpga_in_topic_traffic_light_recogn_0"),
            ("program_fpga", True),
            ("user_ip", "traffic_light_recogn_0"),
        ]
        params = self.declare_parameters(namespace="", parameters=param_list)
        fpga_out_topic, fpga_in_topic, program_fpga, user_ip = self.get_parameters(
            [param.name for param in params]
        )
        # Instantiate the FpgaDriver, program and setup the FPGA
        self.fpga = FpgaDriver(user_ip.value)
        if program_fpga.value:
            self.fpga.program_fpga()
        self.fpga.setup_fpga()

        msg_module_str = "traffic_light_recognition_interface.msg"
        self.FpgaIn = getattr(
            sys.modules[msg_module_str], "FpgaIn" + self.fpga.map_num_str
        )
        self.FpgaOut = getattr(
            sys.modules[msg_module_str], "FpgaOut" + self.fpga.map_num_str
        )
        # Publish to the fpga_out_topic topic, message type is self.FpgaOut
        self.publisher_ = self.create_publisher(
            self.FpgaOut, fpga_out_topic.value, 10
        )
        # Subscribe to the fpga_in_topic topic, message type is self.FpgaIn
        self.create_subscription(
            self.FpgaIn, fpga_in_topic.value, self.fpga_sub_callback, 10
        )
        self.msg = self.FpgaOut()

    def fpga_pub_callback(self):
        # Read output from HW
        for out_signal in self.fpga.out_map.keys():
            setattr(self.msg, out_signal, self.fpga.process_output(out_signal))
        # Publish the result and print to the command line
        self.publisher_.publish(self.msg)

    def fpga_sub_callback(self, msg):
        # Set input values
        for in_signal in self.fpga.in_map.keys():
            self.fpga.process_input(in_signal, getattr(msg, in_signal))
        if self.fpga.has_axis_in or self.fpga.has_axis_out:
            # Setup the input and/or output AXI-Stream buffers
            self.fpga.setup_dma_buffers()
        # Calculate the results
        self.fpga.do_calc()
        # Call the publisher callback
        self.fpga_pub_callback()


def main(args=None):
    rclpy.init(args=args)

    fpga_node = FpgaNode()

    rclpy.spin(fpga_node)

    fpga_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
