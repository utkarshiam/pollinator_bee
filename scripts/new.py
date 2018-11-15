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

		# To publish the drone errors\
		self.pub_roll = rospy.Publisher('error_roll', Float64, queue_size=5)
		self.pub_throt = rospy.Publisher('error_throt', Float64, queue_size=5)
		self.pub_pitch = rospy.Publisher('error_pitch', Float64, queue_size=5)
		
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

            roll_pid, err = self.pid_control(wp_y, self.drone_y, self.last_error_roll, self.sigma_error_roll, self.kp_roll, self.kd_roll, self.ki_roll)
            self.sigma_error_roll += err
            self.last_error_roll = err
            self.cmd.rcRoll += int(self.limit(1500 + roll_pid, 1750, 1350))

            throt_pid, err = self.pid_control(wp_z, self.drone_z, self.last_error_throt, self.sigma_error_throt, self.kp_throt, self.kd_throt, self.ki_throt)
            self.sigma_error_throt += err
            self.last_error_throt = err
            self.cmd.rcPitch += int(self.limit(1500 + pitch_pid, 1600, 1400))

            pitch_pid, err = self.pid_control(wp_x, self.drone_x, self.last_error_pitch, self.sigma_error_pitch, self.kp_pitch, self.kd_pitch, self.ki_pitch)
            self.sigma_error_pitch += err
            self.last_error_pitch = err
            self.cmd.rcThrottle += int(self.limit(1500 + throt_pid, 1600, 1400))
															
			self.pluto_cmd.publish(self.cmd)

	
	def pid_control(self, target, current, last_error, sigma_error, kp, kd, ki):
		dt = time.time() - self.last_time
		if(dt >= self.sample_time):
			error = target - currents
			sigma_error += error * dt
			diff_err = (error - last_error) / dt

		out = (kp * error) + (kd * diff_err) + (ki * sigma_error)
		last_error = error
        self.last_time = time.time()

		return out, error


	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

			
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