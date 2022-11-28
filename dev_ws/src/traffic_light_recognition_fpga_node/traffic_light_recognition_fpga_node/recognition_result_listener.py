import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime

from traffic_light_recognition_interface.msg import FpgaOut1

class DrawCircle(Node):
    def __init__(self):

        super().__init__("tsr_draw_circle")
        self.subscription_img = self.create_subscription(
            Image, "traffic_light_image_type_publisher", self.listener_callback, 10
        )
        self.subscription_tsr = self.create_subscription(
            FpgaOut1, "fpga_out_topic_traffic_light_recogn_0", self.update_circle_coordinate, 10
        )
        self.circle_img_publisher = self.create_publisher(Image, "traffic_light_image_with_circle", 10)

        self.br = CvBridge()
        self.center_x = 0
        self.center_y = 0
        self.radius = 0
        self.color = 0

    def update_circle_coordinate(self, msg):
        self.get_logger().info("update circle coordinate: {}".format(msg))
        if msg.color != 0:
            self.center_x = msg.circle_center_x
            self.center_y = msg.circle_center_y
            self.radius = msg.circle_radius
            self.color = msg.color
            return
        self.center_x = 0
        self.center_y = 0
        self.radius = 0

    def listener_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg)
        center = (int(self.center_x), int(self.center_y))
        circle_color = (0,0,0)
        if self.color == 1:
            circle_color = (0,0,255)
        elif self.color == 2:
            circle_color = (0,255,255)
        elif self.color == 3:
            circle_color = (0,255,0)
        cv2.circle(frame, center=center, radius=int(self.radius), color=circle_color, thickness=3, lineType=cv2.LINE_8, shift=0)
        self.circle_img_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))


def main(args=None):

    rclpy.init(args=args)
    draw_circle = DrawCircle()
    rclpy.spin(draw_circle)
    img_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
