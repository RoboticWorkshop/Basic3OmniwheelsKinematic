#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class Server:
	def __init__(self):
		self.pose = np.matrix([[0., 0., 0.]]).T
		self.pose_des = np.matrix([[0., 0., 0.]]).T
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
	
	def compute(self):
		error = self.pose_des - self.pose
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
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
