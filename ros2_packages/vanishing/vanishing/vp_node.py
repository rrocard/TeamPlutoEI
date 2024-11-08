#!/usr/bin/env python
#
# Software License Agreement (BSD)
#
# \file      vp_node.py
# \authors   Jeremy Fix <jeremy.fix@centralesupelec.fr>
# \copyright Copyright (c) 2022, CentraleSupélec, All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation and/or
#    other materials provided with the distribution.
#  * Neither the name of Autonomy Lab nor the names of its contributors may be
#    used to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WAR- RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, IN- DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

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

from vanishing import my_line_library as mll


class VPNode(Node):

    def __init__(self):
        super().__init__("vp_node")

        self.debug_pub = self.create_publisher(
            CompressedImage, "/debug/vpimg/image_raw/compressed", 1
        )

        self.horizontal_displacement = self.create_publisher(Float32, "vp_offset",10)

        self.angle_ratio = self.create_publisher(Float32,"vp_angle", 10)

        #TODO: Define here your additional publishers
        # The publishers must always be defined before the subscribers
        # using them

        self.img_sub = self.create_subscription(
            CompressedImage, "/bebop/camera/image_raw/compressed", self.on_image, 1
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

        #if self.debug_pub.get_subscription_count() > 0:

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lines = cv2.createLineSegmentDetector(0).detect(gray)[0]
        lines = mll.from_lsd(lines)


        #if self.horizontal_displacement.get_subscription_count() > 0 or self.angle_ratio.get_subscription_count() > 0 :
                
        lines=mll.length_filtering(lines,80) #On utilise les arguments par défaut des fonctions 

        #On affiche les lignes à filtrer
        mll.draw_lines(frame, lines, (0, 0, 255), 1)

        lines=mll.ceiling_filtering(lines,20) 
        lines=mll.angle_filtering(lines,85,20)
        lines=mll.cluster_filtering(frame,lines)

        #On affiche les lines gardées après filtrage
        mll.draw_lines(frame, lines, (50, 255, 255), 1)

        ylim=20
        xlim=200
        vanishing_point=mll.vanishing_point(frame,lines,xlim,ylim)

        cv2.circle(frame, (int(vanishing_point[0]),int(vanishing_point[1])), 5, (0,0,255),-1)

        width=frame.shape[1]
        height=frame.shape[0]
        

        horizontal_displacement=vanishing_point[0]-width//2

        #Remapping from [-width/2,width/2]
        if int(vanishing_point[0])==0:
            normalized_horizontal_displacement=500
        else :
           normalized_horizontal_displacement=horizontal_displacement/width/2



        msg_horizontal=Float32()
        msg_horizontal.data=float(normalized_horizontal_displacement)

        print("horizdes",horizontal_displacement)
        print("normhori",normalized_horizontal_displacement)

        self.horizontal_displacement.publish(msg_horizontal)

        if len(lines)!=2:
            ratio = 0
        else :
            tan = lines[..., 4]/lines[...,5]
            angle=np.arctan(tan)
            print("angle",angle)
            a,b=angle
            if np.sign(a)!=np.sign(b):
                ratio=angle[0]+angle[1] #pb des valeurs positives et comment déterminer droite gauche ?
            else :
                ratio=0

            
        normalized_ratio=ratio/np.pi #remaping from [-pi/2,pi/2] to [-1/2,1/2]

        print("ratio",ratio)
        print("normalized ratio", normalized_ratio)

        msg_ratio=Float32()
        msg_ratio.data=float(normalized_ratio)

        #print("msg_ratio",msg_ratio.data)

        self.angle_ratio.publish(msg_ratio)

        outmsg = self.bridge.cv2_to_compressed_imgmsg(frame.copy())

        self.debug_pub.publish(outmsg)


def main(args=None):
    rclpy.init(args=args)

    vp_node = VPNode()
    rclpy.spin(vp_node)

    vp_node.destroy_node()
    rclpy.shutdown()


