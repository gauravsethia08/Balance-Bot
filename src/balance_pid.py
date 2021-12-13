#! /usr/bin/env python

# Importing Libraries
import os
import time
import rospy
from sensor_msgs.msg import Imu as IMU
from std_msgs.msg import Float32, Float64
from tf.transformations import euler_from_quaternion


class Balance:
	# Initializing all variables
	def __init__(self, tune  = False):
		# Defining variables
		self.prev_error = 0
		self.position = [0, 0 ,0]
		self.error_sum = 0

		# Initializing PID with tuned values
		self.Kp = 16.5
		self.Ki = 1e-5
		self.Kd = 350#345.3

		# Initial value for pitch to avoid errors during 1st iteration
		self.p = 1e-4

		# Subscribing to IMU data
		rospy.Subscriber('/imu', IMU, self.imu_callback)

		# Subscribing 

		# Defining cmd_vel publisher
		self.left_cmd_pub = rospy.Publisher('/balance_bot/left_wheel_controller/command', Float64, queue_size = 10)
		self.right_cmd_pub = rospy.Publisher('/balance_bot/right_wheel_controller/command', Float64, queue_size = 10)
		
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float32, queue_size=1)


	# IMU Callback function
	def imu_callback(self, data):
		# Converting Quaternion to euler angles
		orientation = data.orientation
		orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		self.r, self.p, self.y = euler_from_quaternion(orientation_list)


	# PID Control Algorithm
	def pid(self):
		# Computing the change in time
		self.now = time.time()
		self.last_time = 0
		self.time_change = self.now - self.last_time

		# To not update over a very small period of time
		if (self.time_change > 0.005):
			# Computing Error (for Proportional Term)
			self.pitch_error = self.position[1] - self.p
			# Computing Change in Error (For Derivative Term)
			self.change_in_error = self.pitch_error - self.prev_error

			# Computing the output of controller
			# output = (P*error) + (I*sum_of_error) + (D*change_in_error)
			self.out_vel = (self.Kp * self.pitch_error) + (self.Ki * self.error_sum) + (self.Kd * self.change_in_error)

			# Computing Sum of errors (For Integral Term)
			self.error_sum += self.pitch_error
			# Updating pervious erros
			self.prev_error = self.pitch_error

			# Making cmd message and publishing
			self.left_cmd_pub.publish(self.out_vel)
			self.right_cmd_pub.publish(self.out_vel)

			# Publishing for visualization
			self.pitch_error_pub.publish(self.pitch_error)

		

# Main Thread
if __name__ == "__main__":
	# Initializing the node
	rospy.init_node('balance_bot_pid')

	# Initializing the class
	balancer = Balance()
	while not rospy.is_shutdown():
		balancer.pid()
	rospy.spin()

	# srv = Server(paramsConfig, callback)
	# rospy.spin()
