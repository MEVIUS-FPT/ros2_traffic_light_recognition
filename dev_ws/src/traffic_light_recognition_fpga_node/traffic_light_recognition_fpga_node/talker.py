import sys
import random
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_interface_path
from rosidl_adapter.parser import parse_message_file
from sensor_msgs.msg import Image

from traffic_light_recognition_interface.msg import *
import cv2
from cv_bridge import CvBridge
import numpy as np


class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker")
        self.declare_parameter(
            "fpga_in_topic", "fpga_in_topic_traffic_light_recogn_0"
        )
        self.declare_parameter("msg_type_suffix", "1")
        fpga_in_topic = self.get_parameter("fpga_in_topic").value
        msg_type_suffix = self.get_parameter("msg_type_suffix").value
        self.FpgaIn = getattr(
            sys.modules["traffic_light_recognition_interface.msg"], "FpgaIn" + msg_type_suffix
        )
        self.message_spec = parse_message_file(
            "traffic_light_recognition_interface",
            get_interface_path(
                "traffic_light_recognition_interface/msg/FpgaIn" + msg_type_suffix
            ),
        )
        self.publisher_ = self.create_publisher(
            self.FpgaIn, fpga_in_topic, 10
        )
        self.image_publisher_for_check  = self.create_publisher(
            Image, "traffic_light_image_type_publisher", 10
        )


        self.declare_parameters(
            namespace="",
            parameters=[("crop_h_min", 0), ("crop_h_max", 128), ("crop_w_min", 192), ("crop_w_max", 448), ("radius", 15), ("cond_green", [150,200,150]), ("cond_yellow", [200, 150, 150]), ("cond_red", [200,150,150]), ("cam_brightness", 0), ("cam_saturation", 80), ("cam_device", 0), ("cam_contrast", 10), ("cam_wb", 3000)]
        )
        self.radius = self.get_parameter("radius").value
        self.cond_green = self.get_parameter("cond_green").value
        self.cond_yellow = self.get_parameter("cond_yellow").value
        self.cond_red = self.get_parameter("cond_red").value
        self.crop_w_min = self.get_parameter("crop_w_min").value
        self.crop_w_max = self.get_parameter("crop_w_max").value
        self.crop_h_min = self.get_parameter("crop_h_min").value
        self.crop_h_max = self.get_parameter("crop_h_max").value

        self.br = CvBridge()
        self.cap = cv2.VideoCapture(self.get_parameter("cam_device").value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.get_parameter("cam_brightness").value)
        self.cap.set(cv2.CAP_PROP_SATURATION, self.get_parameter("cam_saturation").value)
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.get_parameter("cam_contrast").value)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.get_parameter("cam_wb").value)
        self.get_logger().info("width: {}".format(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self.get_logger().info("height: {} ".format(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.get_logger().info("brightness: {}".format(self.cap.get(cv2.CAP_PROP_BRIGHTNESS)))
        self.get_logger().info("saturation: {}".format(self.cap.get(cv2.CAP_PROP_SATURATION)))
        self.get_logger().info("contrast: {}".format(self.cap.get(cv2.CAP_PROP_CONTRAST)))
        self.get_logger().info("white balance temp: {}".format(self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE)))
        while True:
            self.timer_callback()

        self.get_logger().info("width: {}".format(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self.get_logger().info("height: {} ".format(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.get_logger().info("brightness: {}".format(self.cap.get(cv2.CAP_PROP_BRIGHTNESS)))
        self.get_logger().info("saturation: {}".format(self.cap.get(cv2.CAP_PROP_SATURATION)))
        while True:
            self.timer_callback()

    def timer_callback(self):

        ret, frame = self.cap.read()
        if frame is None:
            return
        frame = frame[self.crop_h_min:self.crop_h_max, self.crop_w_min:self.crop_w_max, :]
        self.image_publisher_for_check.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

        b_channel, g_channel, r_channel = cv2.split(frame)
        alpha_channel = np.zeros(b_channel.shape, dtype=b_channel.dtype)

        msg = self.FpgaIn()
        msg.image_in = cv2.merge((b_channel, g_channel, r_channel, alpha_channel)).flatten()

        msg.rows = 128
        msg.cols = 256
        msg.radius = self.radius

        msg.cond_green = self.cond_green
        msg.cond_yellow = self.cond_yellow
        msg.cond_red = self.cond_red

        self.publisher_.publish(msg)
        #self.get_logger().info("Publishing: {}".format(msg))


def main(args=None):
    rclpy.init(args=args)

    talker = TalkerNode()

    rclpy.spin(talker)

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
