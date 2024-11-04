

# External imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from example_interfaces.msg import Float32  # std_msgs.msg.Float32 is deprecated
from cv_bridge import CvBridge
import cv2
import numpy as np

# Local imports
from vanishing_point import my_line_library as mll



class VPNode(Node):
    def __init__(self):
        super().__init__("vp_node")

        self.debug_pub = self.create_publisher(
            CompressedImage, "/debug/vpimg/image_raw/compressed", 1
        )

        #TODO: Define here your additional publishers
        # The publishers must always be defined before the subscribers
        # using them
        self.img_sub = self.create_subscription(
            CompressedImage, "camera", self.on_image, 1
        )
        self.bridge = CvBridge()

    def cb_params(self, data):
        self.get_logger().info(f"{data}")
        for p in data:
            name = p.name
            value = p.value
            setattr(self.vp_processor, name, value)
        return SetParametersResult(successful=True)

    def on_image(self, msg):
        # Extract and decode the frame from the ROS2 message
        # frame is of type ndarray
        # and of shape H , W , 3
        # encoded in Blue Green Red (BGR)
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info(
            f"Receiving video frame {frame.shape}, of type : {type(frame)}"
        )
        # TODO
        # Let us do some dummy operations on the image just to illustrate
        # the manipulation of the image and a publisher
        # You obvisouly have to adapt/remove this dummy code
        # Our dummy operation is inverting the colors
        if self.debug_pub.get_subscription_count() > 0:
            frame = 255 - frame
            # And drawing a line
            cv2.line(frame, (0, 0), (100, 100), (255, 0, 0), 5)
            outmsg = self.bridge.cv2_to_compressed_imgmsg(frame.copy())
            self.debug_pub.publish(outmsg)


def main(args=None):
    rclpy.init(args=args)

    vp_node = VPNode()
    rclpy.spin(vp_node)

    vp_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
