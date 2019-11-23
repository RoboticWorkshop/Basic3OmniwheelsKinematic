#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class Server:
	def __init__(self):
		self.pose = np.matrix([[0., 0., 0.]]).T
		self.pose_des = np.matrix([[0., 0., 0.]]).T
		self.cek_des = np.matrix([[2. ,-2. 0.], [4., -2., 0], [4., 2., 0], [2., 2., 0]])
		self.game_mode = 0
		self.counter = 0
		print "Program is running"
	
	def pose_callback(self, dat):
		self.pose[0,0] = dat.x
		self.pose[1,0] = dat.y
		self.pose[2,0] = dat.z
		self.compute()
		
	def pose_desired_callback(self, dat):
		self.pose_des[0,0] = dat.x
		self.pose_des[1,0] = dat.y
		self.pose_des[2,0] = dat.z
	
	def flag_callback(self, dat):
		self.counter = self.counter + 1
		if(self.counter > 3):
			self.counter = 0
		self.compute()
		
	def game_mode_callback(self, dat):
		self.game_mode = dat.data
	
	def compute(self):
		if(self.game_mode == 2):
			error = self.pose_des - self.pose
			cmd_vel.linear.x = error[0,0]
			cmd_vel.linear.y = error[1,0]
			cmd_vel.angular.z = error[2,0]
			cmd_vel_publisher.publish(cmd_vel)
		elif(self.game_mode == 3):
			#get pose desired from cek_desired
			self.pose_des[0,0] = self.cek_des[self.counter,0]
			self.pose_des[1,0] = self.cek_des[self.counter,1]
			self.pose_des[2,0] = self.cek_des[self.counter,2]
			#compute error
			error = self.pose_des - self.pose
			#send error as cmd vel
			cmd_vel.linear.x = error[0,0]
			cmd_vel.linear.y = error[1,0]
			cmd_vel.angular.z = error[2,0]
			cmd_vel_publisher.publish(cmd_vel)
		print cmd_vel

if __name__ == "__main__":
	rospy.init_node("robot_command_velocity_node")
	cmd_vel_publisher = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=2)
	cmd_vel = Twist()
	server = Server()
	try:
		rospy.Subscriber("/robot/pose", Vector3, server.pose_callback)
		rospy.Subscriber("/robot/target_pose", Vector3, server.pose_desired_callback)
		rospy.Subscriber("/robot/command_flag", Int32, server.flag_callback)
		rospy.Subscriber("/team/game_mode", Int32MultiArray, server.game_mode_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
