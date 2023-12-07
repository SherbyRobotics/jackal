#! /usr/bin/env python

"""
Creator: Ian Lalonde, Charles-Alexis Lavoie
Date:    24-11-2023


launch gazebo for jackal with this command: roslaunch jackal_gazebo jackal_world.launch config:=front_laser

"""

import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ControlTest:
	def __init__(self, front_scan_point):
		self.front_scan_point      	= front_scan_point
		self.front_scan            	= np.zeros(front_scan_point)
		self.safe                  	= True
		self.safe_rotate           	= True
		self.linear_cmd            	= np.nan
		self.angular_cmd           	= np.nan
		self.last_linear_cmd       	= np.nan
		self.last_angular_cmd      	= np.nan
		self.min_distance          	= np.nan
		self.safe_distance         	= 1.5

		#decceleration
		self.reaction_time         	= 0.3 #Depends on user
		self.decceleration_cmd     	= 0.
		self.saturation            	= False
		self.max_vehicle_speed     	= 1.0	
		self.vehicle_speed         	= 0.
		self.saturation_distance   	= 2. * self.reaction_time * self.max_vehicle_speed

		#publishing
		self.pub                   	= rospy.Publisher('con_vel', Twist, queue_size=10) #con_vel, change priority in twist_mux.yaml to overwrite other com_vels
		self.pub_msg               	= Twist()
		
		self.subScan			= rospy.Subscriber("scan", LaserScan, self.callback_scan)
		self.subFrontScan		= rospy.Subscriber("front/scan", LaserScan, self.callback_scan)
		#self.subCmd_vel		= rospy.Subscriber("cmd_vel", Twist, self.callback_cmd)
		self.subTwistCmd_vel	= rospy.Subscriber("twist_marker_server/cmd_vel", Twist, self.callback_cmd)
		self.subBtCmd_vel		= rospy.Subscriber("bluetooth_teleop/cmd_vel", Twist, self.callback_cmd)
		self.subJoyCmd_vel		= rospy.Subscriber("joy_teleop/cmd_vel", Twist, self.callback_cmd)
		self.subOdom 			= rospy.Subscriber("jackal_velocity_controller/odom", Odometry, self.callback_odom)
		
		self.dt				= 0.01
		self.timer			= rospy.Timer( rospy.Duration( self.dt ), self.publish_loop )
	
		self.lock_input			= False

	
	#check front scanner
	def callback_scan(self, data):
		self.front_scan = np.asarray(data.ranges)[(len(data.ranges)/2)-(self.front_scan_point/2):(len(data.ranges)/2)+(self.front_scan_point/2)]
		self.min_distance = np.min(self.front_scan)
		
	def callback_cmd(self,data):
		if not self.lock_input:
			self.last_linear_cmd = data.linear.x
		if self.lock_input and  data.linear.x < 0.0:
			self.last_linear_cmd = data.linear.x
		self.last_angular_cmd = data.angular.z

	#constantly check jackal speed
	def callback_odom(self, data):
		self.vehicle_speed = data.twist.twist.linear.x
		

	#Stop the robot if he is too close to something
	def publish_loop(self, timer):
		while not rospy.is_shutdown():
			self.is_safe()
			if self.safe:
				#print('green')
				self.linear_cmd = self.last_linear_cmd
				self.angular_cmd = self.last_angular_cmd

			else :
				#print('red')
				self.linear_cmd = min(0, self.last_linear_cmd)
				self.pub_msg.linear.x = self.linear_cmd 
				self.angular_cmd = self.last_angular_cmd

			self.pub_msg.linear.x = self.linear_cmd 
			self.pub_msg.angular.z = self.angular_cmd 
			self.pub.publish(self.pub_msg)			

	#check if something is in front of the robot
	def is_safe(self):
		#danger zone
		if self.min_distance < self.safe_distance:
			self.safe = False
		#safe zone
		else:	
			self.safe = True

	
			
#rate              = publishing rate of the new cmd
#front_scan_point  = Number of point for the front scanner
if __name__ == '__main__':           
	rospy.init_node('control_test')                 
	node = ControlTest(front_scan_point=30)
	rospy.spin()
	

