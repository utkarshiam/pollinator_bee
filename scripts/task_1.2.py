#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32

class WayPoint:
	
	def __init__(self):

		rospy.init_node('ros_bridge')
		
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('whycon/image_out', Image, self.image_callback)
		
	def image_callback(self,msg):

		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


		
if __name__ == '__main__':
	test = WayPoint()
	rospy.spin()

		