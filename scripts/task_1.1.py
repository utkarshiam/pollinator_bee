#!/usr/bin/env python
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
import rospy
import time


class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self):
		
		rospy.init_node('pluto_fly', disable_signals = True)

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
		
		self.cmd = PlutoMsg()

		# Position to hold.
		self.wp_x = -1.0
		self.wp_y = -1.0
		self.wp_z = 2.0
		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0

		#PID constants for Roll
		self.kp_roll = 0.0
		self.ki_roll = 0.0
		self.kd_roll = 0.0

		#PID constants for Pitch
		self.kp_pitch = 0.0
		self.ki_pitch = 0.0
		self.kd_pitch = 0.0
		
		#PID constants for Yaw
		# self.kp_yaw = 0.2
		# self.ki_yaw = 0.1
		# self.kd_yaw = 0.2

		#PID constants for Throttle
		self.kp_throt = 0.2
		self.ki_throt = 0.0
		self.kd_throt = 0.1

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		# Loop time for PID computation. You are free to experiment with this
		
		self.current_time=time.time()
		self.last_time = self.current_time
		self.sample_time = 0.050
		self.Pterm_R=0.0
		self.Pterm_P=0.0
		self.Pterm_T=0.0

		self.Iterm_R=0.0
		self.Iterm_P=0.0
		self.Iterm_T=0.0

		self.Dterm_R=0.0
		self.Dterm_P=0.0
		self.Dterm_T=0.0
		
		self.int_error=0.0
		self.windup_guard=20.0
		# self.output_roll=0.0
		# self.output_pitch=0.0
		# self.output_throt=0.0

		self.Le_r=0.0
		self.Le_p=0.0
		self.Le_t=0.0

		rospy.sleep(.1)


	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)

		while True:

			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	print self.correct_throt

		 	pitch_value = int(1500 + self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 + self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
															
			throt_value = int(1500 + self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
															
			self.pluto_cmd.publish(self.cmd)


	def pid_roll(self):

		#Compute Roll PID here

		error=self.wp_x - self.drone_x
		self.current_time= time.time() / 1000
		delta_time=self.current_time-self.last_time
		delta_error= error - self.Le_r
		if (delta_time >= self.sample_time):
			self.Pterm_R=self.kp_roll * error
			self.Iterm_R += error * delta_time

		if (self.Iterm_R< -self.windup_guard):
			self.Iterm_R= -self.windup_guard

		elif(self.Iterm_R > self.windup_guard):
			self.Iterm_R= self.windup_guard

		self.Dterm_R=0.0
		if(delta_time > 0.0):
			self.Dterm_R=delta_error / delta_time

		self.last_time= self.current_time
		self.Le_r=error
		self.correct_roll= self.Pterm_R + (self.ki_roll * self.Iterm_R) + (self.kd_roll * self.Dterm_R)






	def pid_pitch(self):

		#Compute Pitch PID here
		error=self.wp_y - self.drone_y
		self.current_time= time.time() / 1000
		delta_time=self.current_time-self.last_time
		delta_error= error - self.Le_p
		if (delta_time >= self.sample_time):
			self.Pterm_P=self.kp_pitch * error
			self.Iterm_P += error * delta_time

		if (self.Iterm_P< -self.windup_guard):
			self.Iterm_P= -self.windup_guard

		elif(self.Iterm_P > self.windup_guard):
			self.Iterm_P= self.windup_guard

		self.Dterm_P=0.0
		if(delta_time > 0.0):
			self.Dterm_P=delta_error / delta_time

		self.last_time= self.current_time
		self.Le_p=error
		self.correct_pitch= self.Pterm_P + (self.ki_pitch * self.Iterm_P) + (self.kd_pitch * self.Dterm_P)

	def pid_throt(self):

		#Compute Throttle PID here
		error=self.wp_z - self.drone_z
		self.current_time= time.time() / 1000
		delta_time=self.current_time-self.last_time
		delta_error= error - self.Le_t
		if (delta_time >= self.sample_time):
			self.Pterm_T=self.kp_throt * error
			self.Iterm_T += error * delta_time

		if (self.Iterm_T< -self.windup_guard):
			self.Iterm_T= -self.windup_guard

		elif(self.Iterm_T > self.windup_guard):
			self.Iterm_T= self.windup_guard

		self.Dterm_T=0.0
		if(delta_time > 0.0):
			self.Dterm_T=delta_error / delta_time

		self.last_time= self.current_time
		self.Le_t=error
		self.correct_throt= self.Pterm_T + (self.ki_throt * self.Iterm_T) + (self.kd_throt * self.Dterm_T)

	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

	#You can use this function to publish different information for your plots
	# def publish_plot_data(self):


	def set_pid_alt(self,pid_val):
		
		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd
		
	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd
		
	def set_pid_yaw(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

		self.kp_yaw = pid_val.Kp
		self.ki_yaw = pid_val.Ki
		self.kd_yaw = pid_val.Kd
		
	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z


if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()