#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32


class Server:
	def __init__(self):
		#==================================================================================================
		#variabel kinematik
		self.alp = np.array([[0*np.pi/180., 120.*np.pi/180., 240.*np.pi/180.]]).T #robot specification
		self.l = 0.20 #panjang posisi roda robot dari titik tengah robot
		self.r = 0.05 #besar jari - jari roda robot
		self.Jr = self.jacobianR()
		#self.lamda = np.matrix([[25., 0., 0.], [0., 30., 0.], [0., 0., 35.]])
		self.lamda = np.matrix([[15., 0., 0.], [0., 15., 0.], [0., 0., 15.]])
		self.ts = 0.1
		self.er = np.array([[0., 0., 0.]]).T
		self.pose = np.matrix([[0., 0., 0.]]).T
		self.pose_des = np.matrix([[0., 0., 0.]]).T
		#=======================================================================================
		self.game_mode = 0.
		self.attack = 0.
		self.treshold_error = 0.2
		self.goal_reached = 0.
		self.limit_pwm = 700
		self.min_pwm = 130
		print "robot kinematic is running"
	
	def jacobianR(self):
		tr = -90.*np.pi/180.
		R  = np.matrix([[np.cos(tr), -np.sin(tr), 0.], [np.sin(tr), np.cos(tr), 0.], [0., 0., 1]])
		Jr = np.matrix([[0.,0.,0.],[0.,0.,0.],[0.,0.,0.]])
		Jr[0,0]=np.cos(self.alp[0,0])
		Jr[0,1]=np.cos(self.alp[1,0])
		Jr[0,2]=np.cos(self.alp[2,0])
		Jr[1,0]=np.sin(self.alp[0,0])
		Jr[1,1]=np.sin(self.alp[1,0])
		Jr[1,2]=np.sin(self.alp[2,0])
		Jr[2,0]=1/self.l
		Jr[2,1]=1/self.l
		Jr[2,2]=1/self.l
		return self.r*(R*Jr)
		#return self.r*Jr
	
	def jacobianW(self,th,Jr):
		rotZ = np.matrix([[np.cos(th), -np.sin(th), 0.], [np.sin(th), np.cos(th), 0.], [0., 0., 1]])
		return rotZ*Jr	
	
	def game_mode_callback(self, dat): #callback untuk mode permainan (start, stop, positioning) dan status menyerang(menyerang/bertahan)
		self.game_mode = dat.data[0]
		self.attack = dat.data[1]
	
	def cmd_vel_callback(self, dat):
		vx = dat.linear.x
		vy = dat.linear.y
		vz = dat.angular.z
		self.er = np.matrix([[vx, vy, vz]]).T
		self.main()
	
	def pose_callback(self, dat):
		self.pose[0,0] = dat.x
		self.pose[1,0] = dat.y
		self.pose[2,0] = dat.z
		#self.main()
	
	def pose_des_callback(self, dat):
		self.pose_des[0,0] = dat.x
		self.pose_des[1,0] = dat.y
		self.pose_des[2,0] = dat.z
	
	def pwm_leveling(self, w):
		temp = np.array([abs(w[0,0]), abs(w[1,0]), abs(w[2,0])])
		out = np.matrix([[0., 0., 0.]]).T
		maximum = np.max(temp)
		if(maximum > self.limit_pwm):
			out[0,0] = (w[0,0]/maximum)*self.limit_pwm
			out[1,0] = (w[1,0]/maximum)*self.limit_pwm
			out[2,0] = (w[2,0]/maximum)*self.limit_pwm
		else:
			out[0,0] = w[0,0]
			out[1,0] = w[1,0]
			out[2,0] = w[2,0]
		#====================== W1 ====================
		if(out[0,0]>0.4) and (out[0,0]<self.min_pwm):
			out[0,0] = self.min_pwm
		elif (out[0,0]<-0.4) and (out[0,0]>-self.min_pwm):
			out[0,0] = -self.min_pwm
		#====================== W2 ====================
		if(out[1,0]>0.4) and (out[1,0]<self.min_pwm):
			out[1,0] = self.min_pwm
		elif (w[1,0]<-0.4) and (out[1,0]>-self.min_pwm):
			out[1,0] = -self.min_pwm
		#====================== W3 ====================
		if(out[2,0]>0.4) and (out[2,0]<self.min_pwm):
			out[2,0] = self.min_pwm
		elif (out[2,0]<-0.4) and (out[2,0]>-self.min_pwm):
			out[2,0] = -self.min_pwm
		#==============================================
		motor.x = out[0,0] #belakang
		motor.y = out[1,0] #kanan
		motor.z = out[2,0] #kiri	
		print motor
		pwm_publisher.publish(motor)
	
	def invers_kinematic(self):
		J = self.jacobianW(self.pose[2,0],self.Jr)
		Jinv = np.linalg.inv(J)
		#self.er = self.pose_des - self.pose
		e_norm = np.linalg.norm(self.er)
		#print e_norm
		if(e_norm < self.treshold_error):
			self.er = 0.
			if(self.goal_reached != 1):
				self.goal_reached = 1
				goal_reached_publisher.publish(1)
		else:
			self.goal_reached = 0		
		w = self.lamda*Jinv*self.er
		self.pwm_leveling(w)
	
	def stop_motor(self):
		motor.x = 0 #belakang
		motor.y = 0 #kanan
		motor.z = 0 #kiri		
		pwm_publisher.publish(motor)
	
	def main(self):
		if(self.game_mode == 0):
			self.stop_motor()
			print "STOP"
		
		elif(self.game_mode == 1):
			print "START"
		
		elif(self.game_mode == 2):
			self.invers_kinematic()

if __name__ == "__main__":
	rospy.init_node("robot_kinematics_node")
	pwm_publisher = rospy.Publisher("robot/pwm", Vector3, queue_size = 3)
	goal_reached_publisher = rospy.Publisher("robot/command_flag", Int32, queue_size = 1)
	server = Server()
	motor = Vector3()
	try:
		rospy.Subscriber("/robot/cmd_vel", Twist, server.cmd_vel_callback)
		rospy.Subscriber("/team/game_mode", Int32MultiArray, server.game_mode_callback)
		rospy.Subscriber("/robot/pose", Vector3, server.pose_callback)
		#rospy.Subscriber("/robot/target_pose", Vector3, server.pose_des_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
