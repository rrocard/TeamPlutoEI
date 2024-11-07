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
        outmsg = self.bridge.cv2_to_compressed_imgmsg(frame)


        width=frame.shape[1]
        height=frame.shape[0]
        h=height//2

        
        if self.previousimg is not None  :

            
            start_time=time.time()
            icurrent=mll.intensity_mesure(frame,h)
            ipast=mll.intensity_mesure(self.previousimg,h)
            end_time=time.time()

            print(end_time-start_time)

            self.previousimg=frame 

            start_time=time.time()
            shift=mll.local_shift(ipast,icurrent)
            end_time=time.time()

            print(end_time-start_time)

            mll.draw_function(frame,ipast,0,height,0,ipast[0],(0,0,255),2) #passé
            mll.draw_function(frame,icurrent,0,height,0,ipast[0],(0,255,255),2) #actuel

            diff= h-shift
            mll.draw_function(frame,diff,0,height,0,height,(255,255,0),1)

            outmsg = self.bridge.cv2_to_compressed_imgmsg(frame)

            self.debug_pub.publish(outmsg)
        
        else:
            outmsg = self.bridge.cv2_to_compressed_imgmsg(frame)

            self.previousimg=frame
            print("no parent")
            self.debug_pub.publish(outmsg)
        

        



   

    def on_speed(self,msg):
        return None


def main(args=None):
    rclpy.init(args=args)

    shift_node = ShiftNode()
    rclpy.spin(shift_node)

    shift_node.destroy_node()
    rclpy.shutdown()