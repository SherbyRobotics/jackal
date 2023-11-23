#! /usr/bin/env python

"""
Creator: Charles-Alexis Lavoie
Date:    30-june-2021

needs to replace husky_control/config/twist_mux.yaml ->

topics:
- name    : con_vel
  topic   : con_vel
  timeout : 0.5
  priority: 10
locks:
- name    : e_stop
  topic   : e_stop
  timeout : 0.0
  priority: 255

launch gazebo for jackal with this command: roslaunch jackal_gazebo jackal_world.launch config:=front_laser

"""

import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ControlTest:
	def __init__(self, front_scan_point,rate):
		self.front_scan_point      = front_scan_point
		self.front_scan            = np.zeros(front_scan_point)
		self.safe                  = True
		self.safe_rotate           = True
		self.linear_cmd            = np.nan
		self.angular_cmd           = np.nan
		self.last_linear_cmd       = np.nan
		self.last_angular_cmd      = np.nan
		self.min_distance          = np.nan
		self.safe_distance         = 1.5

		#decceleration
		self.reaction_time         = 0.3 #Depends on user
		self.decceleration_cmd     = 0.
		self.saturation            = False
		self.max_vehicle_speed     = 1.0	
		self.vehicle_speed         = 0.
		self.saturation_distance   = 2. * self.reaction_time * self.max_vehicle_speed

		#publishing
		self.pub                   = rospy.Publisher('con_vel', Twist, queue_size=10)
		rospy.init_node('control_test')
		self.rate                  = rospy.Rate(rate)
		self.pub_msg               = Twist()
		self.lock_input            = False

		self.control_test()
	
	#main loop
	def control_test(self):
		rospy.Subscriber("scan", LaserScan, self.callback_scan)
		rospy.Subscriber("front/scan", LaserScan, self.callback_scan)
		rospy.Subscriber("cmd_vel", Twist, self.callback_cmd)
		rospy.Subscriber("twist_marker_server/cmd_vel", Twist, self.callback_cmd)
		rospy.Subscriber("bluetooth_teleop/cmd_vel", Twist, self.callback_cmd)
		rospy.Subscriber("joy_teleop/cmd_vel", Twist, self.callback_cmd)
		rospy.Subscriber("husky_velocity_controller/odom", Odometry, self.callback_odom)
		rospy.Subscriber("jackal_velocity_controller/odom", Odometry, self.callback_odom)
		worker = threading.Thread(target=self.publish_thread)
		worker.start()		
		rospy.spin()
	
	#check front scanner
	def callback_scan(self, data):
		self.front_scan = np.asarray(data.ranges)[(len(data.ranges)/2)-(self.front_scan_point/2):(len(data.ranges)/2)+(self.front_scan_point/2)]
		self.min_distance = np.min(self.front_scan)
		
	def callback_cmd(self,data):
		if self.lock_input is False:
			self.last_linear_cmd = data.linear.x
		if self.lock_input is True and  data.linear.x < 0.0:
			self.last_linear_cmd = data.linear.x
		self.last_angular_cmd = data.angular.z

	#constantly check husky speed
	def callback_odom(self, data):
		self.vehicle_speed = data.twist.twist.linear.x
		if self.vehicle_speed > self.max_vehicle_speed:
			self.max_vehicle_speed = self.vehicle_speed		
		self.saturation_distance = 2 * self.reaction_time * self.vehicle_speed
		if self.saturation_distance < self.safe_distance:
			self.saturation_distance = self.safe_distance

	#Stop the robot if he is too close to something
	def publish_thread(self):
		while not rospy.is_shutdown():
			self.is_safe()
			self.is_safe_rotate()
			if self.safe is False and self.saturation is False and self.safe_rotate is False:
				print('rouge')
				self.linear_cmd = 0
				self.pub_msg.linear.x = self.linear_cmd 
				self.angular_cmd = self.last_angular_cmd
			elif self.saturation is True and self.safe is True:
				print('jaune')
				dec_fac1 = (1.0/ (   (2.0*self.reaction_time*self.vehicle_speed)-self.safe_distance)) * self.min_distance
				dec_fac2 = self.safe_distance/((2.*self.reaction_time*self.vehicle_speed) -self.safe_distance)   
				self.linear_cmd = self.last_linear_cmd * (dec_fac1-dec_fac2)
				self.last_linear_cmd = self.linear_cmd
				self.angular_cmd = self.last_angular_cmd 
			else:
				print('vert')
				self.linear_cmd = self.last_linear_cmd
				self.angular_cmd = self.last_angular_cmd

			self.pub_msg.linear.x = self.linear_cmd 
			self.pub_msg.angular.z = self.angular_cmd 
			self.pub.publish(self.pub_msg)				
			self.rate.sleep()

	#check if something is in front of the robot
	def is_safe(self):
		#zone rouge stop
		if self.min_distance < self.safe_distance:
			self.lock_input = True
			self.safe = False
			self.safe_rotate = False
			self.saturation = False 
		#zone jaune decceleration
		elif self.min_distance > self.safe_distance and self.min_distance < self.saturation_distance:	
			self.lock_input = True 
			self.safe = True
			self.saturation = True 
		#zone verte libre
		else:	
			self.lock_input = False
			self.safe = True
			self.saturation = False		

	#check if the user is trying to rotate and can rotate
	def is_safe_rotate(self):
		if self.safe is False and self.safe_rotate is False and (self.last_angular_cmd is not 0. or self.last_linear_cmd < 0.) and self.last_linear_cmd < 0.20:
			self.safe_rotate = True
	
			
#rate              = publishing rate of the new cmd
#front_scan_point  = Number of point for the front scanner
if __name__ == '__main__':                                               
	controler = ControlTest(front_scan_point=30, rate=100)
	

