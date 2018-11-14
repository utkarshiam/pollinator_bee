#!/usr/bin/env python

#The required packages are imported here
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
		self.wp_x = -5.63
		self.wp_y = -5.63
		self.wp_z = 10.0
		
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
		self.kp_yaw = 0.0
		self.ki_yaw = 0.0
		self.kd_yaw = 0.0

		#PID constants for Throttle
		self.kp_throt = 0.0
		self.ki_throt = 0.0
		self.kd_throt = 0.0

		# Correction values after PID is computed
		self.correct_roll = 0.0 
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		self.last_err_roll = 0
		self.last_err_pitch = 0
		self.last_err_throt = 0

		self.dt = 0 # current_time ~ time elapsed from the last iteration 

		self.iterm_pitch = 0
		self.iterm_roll = 0
		self.iterm_throt = 0

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
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
															
			throt_value = int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
															
			self.pluto_cmd.publish(self.cmd)
		
		publish_plot_data(self, self.output)
		publish_plot_data(self, self.error_roll)
		publish_plot_data(self, self.error_pitch)
		publish_plot_data(self, self.error_throt)
	

	def calc_pid(self):
		self.seconds = time.time()
		self.dt = self.seconds - self.last_time
		if(self.dt >= self.loop_time):
			# publish_plot_data()

			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			
			self.last_time = self.seconds


	def pid_roll(self):

		self.error_roll = self.wp_x - self.drone_x

		# Integral control
		self.iterm_roll += (self.error_roll * self.dt) 

		# differential control
		dErr = (self.error_roll - self.last_err_roll) / self.dt
		
		self.output = (self.kp_roll * self.error_roll) + (self.ki_roll * self.iterm_roll) + (self.kd_roll * dErr)

		self.last_err_roll = self.error_roll
		self.last_time = time.time()


	def pid_pitch(self):

		self.error_pitch = self.wp_x - self.drone_x
		
		# Integral control
		self.iterm_pitch += (self.error_pitch * self.dt) 

		# differential control
		dErr = (self.error_pitch - self.last_err_pitch) / self.dt
		
		self.output = (self.kp_pitch * self.error_pitch) + (self.ki_pitch * self.iterm_pitch) + (self.kd_pitch * dErr)
		
		self.last_err_pitch = self.error_pitch
		self.last_time = time.time()
		
		#self.error_pitch = self.wp_y - self.drone_y 
		#iterm_pitch += (self.error_pitch * self.current_time)
		#self.dErr = self.error_pitch - self.correct_pitch
		#self.output = self.kp_pitch * self.error_pitch + self.ki_pitch * self.errSum + self.kd_pitch * self.dErr
		#self.correct_pitch = self.error_pitch
		#self.last_time = self.current_time


	def pid_throt(self):
		self.error_throt = self.wp_x - self.drone_x
		
		# Integral control
		self.iterm_throt += (self.error_throt * self.dt) 

		# differential control
		dErr = (self.error_throt - self.last_err_throt) / self.dt
		
		self.output = (self.kp_throt * self.error_throt) + (self.ki_throt * self.iterm_throt) + (self.kd_throt * dErr)

		self.last_err_throt = self.error_throt
		self.last_time = time.time()


		#self.error_throt = self.wp_z - self.drone_z 
		#iterm_throt += (self.error_throt * self.current_time)
		#self.dErr = (self.error_throt - self.correct_throt)
		#self.output = self.kp_throt * self.error_throt + self.ki_throt * self.errSum + self.kd_throt * self.dErr
		#self.correct_throt = self.error_throt
		#self.last_time = self.current_time

	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

	#You can use this function to publish different information for your plots
	def publish_plot_data(self, data):
		rospy.Publisher('/error', String, data)

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
