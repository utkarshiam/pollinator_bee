#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from plutodrone.msg import *
from pid_tune.msg import *
#from geometry_msgs.msg import PoseArray
from std_msgs.msg import *
import rospy
import time
class WayPoint():
	
	def __init__(self):

		rospy.init_node('ros_bridge')
		
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('whycon/image_out', Image, self.image_callback)
	# 	rospy.init_node('pluto_fly', disable_signals=True)

 #        self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

 #        rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

 #        #function that subscribes to the topic /drone_yaw
 #        rospy.Subscriber('/drone_yaw', Float64, self.get_yaw)

 #        # To tune the drone during runtime
 #        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
 #        rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
 #        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
 #        rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)

 #        self.cmd = PlutoMsg()

 #        # Position to hold.
 #        self.wp_x = 0.00
 #        self.wp_y = 0.00
 #        self.wp_z = 0.00

 #        self.cmd.rcRoll = 1500
 #        self.cmd.rcPitch = 1500
 #        self.cmd.rcYaw = 1500
 #        self.cmd.rcThrottle = 1500
 #        self.cmd.rcAUX1 = 1500
 #        self.cmd.rcAUX2 = 1500
 #        self.cmd.rcAUX3 = 1500
 #        self.cmd.rcAUX4 = 1000
 #        self.cmd.plutoIndex = 0

 #        self.drone_x = 0.0
 #        self.drone_y = 0.0
 #        self.drone_z = 0.0
 #        self.drone_yaw = 0.0

 #        # PID constants for Roll
 #        self.kp_roll = 10.0
 #        self.ki_roll = 0.0
 #        self.kd_roll = 1.0

 #        # PID constants for Pitch
 #        self.kp_pitch = 6.0
 #        self.ki_pitch = 0.0
 #        self.kd_pitch = 0.0

 #        # PID constants for Yaw
 #        self.kp_yaw = 15.0
 #        self.ki_yaw = 0.0
 #        self.kd_yaw = 4.0

 #        # PID constants for Throttle
 #        self.kp_throt = 21.0
 #        self.ki_throt = 2.0
 #        self.kd_throt = 3.02

 #        # Correction values after PID is computed
 #        self.correct_roll = 0.0
 #        self.correct_pitch = 0.0
 #        self.correct_yaw = 0.0
 #        self.correct_throt = 0.0

 #        # Loop time for PID computation. You are free to experiment with this
 #        self.last_time = 0.0
 #        self.loop_time = 0.020

 #        # self Defined variables

 #        self.errorsumX = 0
 #        self.errorsumY = 0
 #        self.errorsumZ = 0
 #        self.errorsumYaw = 0

 #        self.prevErrorY = 0
 #        self.prevErrorX = 0
 #        self.prevErrorZ = 0
 #        self.prevErrorYaw = 0

 #        self.errx=0
 #        self.erry=0
 #        self.errz=0
 #        rospy.sleep(.1)
		
	# def arm(self):
	# 	self.cmd.rcAUX4 = 1500
	# 	self.cmd.rcThrottle = 1000
	# 	self.pluto_cmd.publish(self.cmd)
 # 		rospy.sleep(.1)

	# def disarm(self):
	#     self.cmd.rcAUX4 = 1100
	#     self.pluto_cmd.publish(self.cmd)
	#     rospy.sleep(.1)

	# def position_hold(self,s_p, t_p):

	#     self.drone_x=s_p[0]
	#     self.drone_y=s_p[1]
	#     self.drone_z=s_p[2]
	#     self.wp_x=t_p[0]
	#     self.wp_y=t_p[1]
	#     self.wp_z=t_p[2]

	#     rospy.sleep(2)

	#     print
	#     "disarm"
	#     self.disarm()
	#     rospy.sleep(.2)
	#     print
	#     "arm"
	#     self.arm()
	#     rospy.sleep(.1)

	#     #variable that stores initial value(setpoint) of yaw

	#     self.init_yaw = self.drone_yaw

	#     while True:
	#         self.calc_pid()

	#         # Check your X and Y axis. You MAY have to change the + and the -.
	#         # We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
	#         pitch_value = int(1500 - self.correct_pitch)
	#         self.cmd.rcPitch = self.limit(pitch_value, 1600, 1400)

	#         roll_value = int(1500 - self.correct_roll)
	#         self.cmd.rcRoll = self.limit(roll_value, 1600, 1400)

	#         throt_value = int(1400 - self.correct_throt)
	#         self.cmd.rcThrottle = self.limit(throt_value, 1550, 1250)

	#         yaw_value = int(1500 + self.correct_yaw)
	#         self.cmd.rcYaw = self.limit(yaw_value, 1570, 1420)

	#         self.pluto_cmd.publish(self.cmd)
	#         if ((((self.errx)<=0.15) and ((self.errx)>=-0.15)) and (self.wp_x!=0.0)) and ((((self.erry)<=0.15) and ((self.erry)>=-0.15)) and (self.wp_y!=0.0)):
	            
	#             self.cmd.rcRoll = 1500
	#             self.cmd.rcPitch = 1500
	#             self.cmd.rcYaw = 1500
	#             self.cmd.rcThrottle = 1500
	#             self.cmd.rcAUX1 = 1500
	#             self.cmd.rcAUX2 = 1500
	#             self.cmd.rcAUX3 = 1500
	#             self.cmd.rcAUX4 = 1000
	#             self.cmd.plutoIndex = 0
	#             print self.errx, self.erry
	#             self.errx=100.0
	#             self.erry=1000.0
	#             self.errz=10000.0
	#             break
	#         if (self.wp_x==0.0):
	#             continue

	# def calc_pid(self):
	#     self.seconds = time.time()
	#     current_time = self.seconds - self.last_time
	#     if (current_time >= self.loop_time):
	        
	#         self.pid_roll()
	#         self.pid_pitch()
	#         self.pid_throt()
	#         self.pid_yaw()
	#         self.last_time = self.seconds




	# def pid_roll(self):
	#     error = self.wp_y - self.drone_y
	#     self.errorsumY += error
	#     dErr = error - self.prevErrorY
	#     PID = self.kp_roll * error + self.kd_roll * dErr / self.loop_time + self.ki_roll * self.errorsumY * self.loop_time
	#     self.prevErrorY = error
	#     self.correct_roll = PID
	#     self.erry=error


	# def pid_pitch(self):
	#     # Compute Pitch PID here
	#     error = self.wp_x - self.drone_x
	#     self.errorsumX += error
	#     dErr = error - self.prevErrorX
	#     PID = self.kp_pitch * error + self.kd_pitch * dErr / self.loop_time + self.ki_pitch * self.errorsumX * self.loop_time
	#     self.correct_pitch = PID
	#     self.errx=error
	    

	# def pid_throt(self):
	#     # Compute Throttle PID here
	#     error = self.wp_z - self.drone_z
	#     self.errorsumZ += error
	#     dErr = error - self.prevErrorZ
	#     PID = self.kp_throt * error + self.kd_throt * dErr / self.loop_time + self.ki_throt * self.errorsumZ * self.loop_time
	#     self.prevErrorZ = error
	#     self.correct_throt = PID
	#     self.errz=error


	# def pid_yaw(self):
	#     # yaw tuning
	#     error = self.init_yaw - self.drone_yaw
	#     self.errorsumYaw += error
	#     dErr = error - self.prevErrorYaw
	#     PID = self.kp_yaw * error + self.kd_yaw * dErr / self.loop_time + self.ki_yaw * self.errorsumYaw * self.loop_time
	#     self.prevErrorYaw = error
	#     self.correct_yaw = PID


	# def limit(self, input_value, max_value, min_value):

	#     # Use this function to limit the maximum and minimum values you send to your drone

	#     if input_value > max_value:
	#         return max_value
	#     if input_value < min_value:
	#         return min_value
	#     else:
	#         return input_value

	# # You can use this function to publish different information for your plots
	# #\ def publish_plot_data(self):

	# def set_pid_alt(self, pid_val):

	#     # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

	#     self.kp_throt = pid_val.Kp
	#     self.ki_throt = pid_val.Ki
	#     self.kd_throt = pid_val.Kd

	# def set_pid_roll(self, pid_val):

	#     # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

	#     self.kp_roll = pid_val.Kp
	#     self.ki_roll = pid_val.Ki
	#     self.kd_roll = pid_val.Kd

	# def set_pid_pitch(self, pid_val):

	#     # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

	#     self.kp_pitch = pid_val.Kp
	#     self.ki_pitch = pid_val.Ki
	#     self.kd_pitch = pid_val.Kd

	# def set_pid_yaw(self, pid_val):

	#     # This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

	#     self.kp_yaw = pid_val.Kp
	#     self.ki_yaw = pid_val.Ki
	#     self.kd_yaw = pid_val.Kd

	# def get_pose(self, pose):

	#     # This is the subscriber function to get the whycon poses
	#     # The x, y and z values are stored within the drone_x, drone_y and the drone_z variables

	#     self.drone_x = pose.poses[0].position.x
	#     self.drone_y = pose.poses[0].position.y
	#     self.drone_z = pose.poses[0].position.z

	# #subscriber function to get data from the topic /drone_yaw
	# def get_yaw(self, data):
	#     self.drone_yaw = data.data


	def image_callback(self,msg):

		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		#separating into blue, green and red channels
		param1r= [0,0,208]
		param2r= [0,0,255]
		param1g=[0,218,0]
		param2g=[0,218,0]
		param1b=[208,0,0]
		param2b=[255,0,0]
		count=0

		#hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		#ret, thresh = cv2.threshold(hsv, 0,255,0)
		lowerR=np.array(param1r, dtype="uint8")
		upperR=np.array(param2r, dtype="uint8")
		shapeMaskR= cv2.inRange(image,lowerR,upperR)
		#res=cv2.bitwise_and(image, image, mask=shapeMask)
		#cv2.imshow('image', image)
		#cv2.imshow('mask', mask)
		#cv2.imshow('res',res)
		im2R, contoursR, hierarchyR = cv2.findContours(shapeMaskR,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		print len(contoursR)
		cv2.imshow("Maskr", shapeMaskR)
 
		# loop over the contours
		for c in contoursR:
			# draw the contour and show it
			count=count+1
			cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
			#cv2.imshow("Image", image)
			#cv2.waitKey(0)

		lowerB=np.array(param1b, dtype="uint8")
		upperB=np.array(param2b, dtype="uint8")
		shapeMaskB= cv2.inRange(image,lowerB,upperB)
		#res=cv2.bitwise_and(image, image, mask=shapeMask)
		#cv2.imshow('image', image)
		#cv2.imshow('mask', mask)
		#cv2.imshow('res',res)
		im2B, contoursB, hierarchyB = cv2.findContours(shapeMaskB,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		print len(contoursB)
		cv2.imshow("Maskb", shapeMaskB)
 
		# loop over the contours
		for c in contoursB:
			# draw the contour and show it
			count=count+1
			cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
			#cv2.imshow("Image", image)
			#cv2.waitKey(0)	

		lowerG=np.array(param1g, dtype="uint8")
		upperG=np.array(param2g, dtype="uint8")
		shapeMaskG= cv2.inRange(image,lowerG,upperG)
		#res=cv2.bitwise_and(image, image, mask=shapeMask)
		#cv2.imshow('image', image)
		#cv2.imshow('mask', mask)
		#cv2.imshow('res',res)
		im2G, contoursG, hierarchyG = cv2.findContours(shapeMaskG,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		print len(contoursG)
		cv2.imshow("Maskg", shapeMaskG)
 
		# loop over the contours
		for c in contoursG:
			# draw the contour and show it
			count=count+1
			cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
			cv2.imshow("Image", image)
			cv2.waitKey(0)	


if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = WayPoint()
        # list_lol_start=[[0,0,0],[-5.63,-5.63,30],[5.57,-5.63,30],[5.55,5.54,30],[-5.60,5.54,30]]
        # list_lol_togo=[[-5.63,-5.63,30],[5.57,-5.63,30],[5.55,5.54,30],[-5.60,5.54,30],[0.0,0.0,30]]
        # for i in range(0,5):
        #     temp.position_hold(list_lol_start[i], list_lol_togo[i])
        rospy.spin()