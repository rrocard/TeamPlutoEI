import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from example_interfaces.msg import Float32  # std_msgs.msg.Float32 is deprecated
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from scipy.ndimage import median_filter

from vanishing import my_line_library as mll

class ShiftNode(Node):

    def __init__(self):

        super().__init__("shift_node")

        #We will write a node that subscribes to an image flow and drone speed
        #and publish to a topic if the drone has free space or not
        #we will thus first determine the shift of a given row (the center)

        self.free_space = self.create_publisher(Bool,"free_space",10)
        self.debug_pub = self.create_publisher(
            CompressedImage, "/debug/vpimg/image_raw/compressed", 1
        )




        self.img_sub = self.create_subscription(
    CompressedImage, "/bebop/image_raw/compressed", self.on_image, 1
        )
        self.speed_sub = self.create_subscription(Float32, "/bebop/odom",self.on_speed,1)
        self.bridge = CvBridge()
        self.previousimg=None

    def on_image(self,msg):
                  

        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info(
            f"Receiving video frame {frame.shape}, of type : {type(frame)}"
        )


        width=frame.shape[1]
        height=frame.shape[0]
        h=height//2

        previousimg=frame.copy()

        if self.previousimg is not None  :

  
            icurrent=mll.intensity_mesure(frame,h)
            ipast=mll.intensity_mesure(self.previousimg,h)

            icurrent=median_filter(icurrent,size=10)
            ipast=median_filter(ipast,size=10)

            mll.draw_function(frame,ipast,0,h,0,255,(0,0,255),2) #passé
            mll.draw_function(frame,icurrent,0,h,0,255,(0,255,255),2) #actuel

            # self.get_logger().info("past : "+str(ipast))
            # self.get_logger().info("current : "+str(icurrent))

            sigma=50
            shift=mll.local_shift(ipast,icurrent,sigma,25)

            shift=median_filter(shift,size=25)

            mll.draw_function(frame,shift,h,7*height//8,0,sigma,(255,255,0),1)

            door=mll.door(shift,2*sigma//5,width//4)

            mll.draw_function(frame,door,h,7*height//8,0,1,(255,255,255),2)

            outmsg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.debug_pub.publish(outmsg)
        
        else:
            outmsg = self.bridge.cv2_to_compressed_imgmsg(frame)
            print("no parent")
            self.debug_pub.publish(outmsg)
        
        self.previousimg=previousimg
    

    def on_speed(self,msg):
        return None


def main(args=None):
    rclpy.init(args=args)

    shift_node = ShiftNode()
    rclpy.spin(shift_node)

    shift_node.destroy_node()
    rclpy.shutdown()