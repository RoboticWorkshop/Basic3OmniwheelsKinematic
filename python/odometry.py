#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3

class Server:
    def __init__(self):
        self.alp = np.matrix([[45.*np.pi/180., 135.*np.pi/180.]]).T
        self.enc = np.matrix([[0., 0.]]).T
        self.last_enc = np.matrix([[0., 0.]]).T
        self.w = np.matrix([[0., 0., 0.]]).T
        self.pose = np.matrix([[0., 0., 0.]]).T
        self.pose_dot = np.matrix([[0., 0., 0.]]).T
        self.imu = 0.
        self.last_imu = 0.
        self.r = 0.05
        self.ppr = 360.
        self.ts = 0.1
        self.Jr = self.get_jacobianR()
        print "odometry is running"
    
    def encoder_callback(self, dat):
        k = 2. * np.pi * self.r / self.ppr
        self.enc[0,0] = k*dat.x
        self.enc[1,0] = k*dat.y
        self.compute_odometry()
    
    def imu_callback(self, dat):
        self.imu = dat.x*np.pi/180.
    
    def pose_callback(self, dat):
        self.pose[0,0] = dat.x
        self.pose[1,0] = dat.y
    
    def get_jacobianR(self):
        j = np.matrix([[0., 0., 0.], [0.,0.,0.], [0.,0.,0.]])
        j[0,0] = np.cos(self.alp[0,0])
        j[0,1] = np.cos(self.alp[1,0])
        j[0,2] = 0.
        j[1,0] = np.sin(self.alp[0,0])
        j[1,1] = np.sin(self.alp[1,0])
        j[1,2] = 0.
        j[2,0] = 0.
        j[2,1] = 0.
        j[2,2] = 1.
        return self.r*j
    
    def get_jacobianW(self, th, Jr):
        rotZ = np.matrix([[np.cos(th), -np.sin(th), 0.], [np.sin(th), np.cos(th), 0.], [0., 0., 1.]])
        J = rotZ * Jr
        return J
    
    def compute_odometry(self):
        self.w[0,0] = (self.enc[0,0] - self.last_enc[0,0])/self.ts
        self.w[1,0] = (self.enc[1,0] - self.last_enc[1,0])/self.ts
        
        J = self.get_jacobianW(self.imu, self.Jr)
        
        self.pose_dot = J * self.w
        self.pose_dot[0,0] = self.pose_dot[0,0] * 10.
        self.pose_dot[1,0] = self.pose_dot[1,0] * 10.
        self.pose_dot[2,0] = (self.imu - self.last_imu)/self.ts
        
        self.pose[0,0] = self.pose[0,0] + self.pose_dot[0,0] * self.ts
        self.pose[1,0] = self.pose[1,0] + self.pose_dot[1,0] * self.ts
        self.pose[2,0] = self.pose[2,0] + self.pose_dot[2,0] * self.ts
        self.last_enc[0,0] = self.enc[0,0]
        self.last_enc[1,0] = self.enc[1,0]
        self.last_imu = self.imu    
        pose_data.x = self.pose[0,0]
        pose_data.y = self.pose[1,0]
        pose_data.z = self.pose[2,0]
        print pose_data
        pose_publisher.publish(pose_data)

if __name__ == "__main__":
    rospy.init_node("robot_odometry_node")
    pose_publisher = rospy.Publisher("/robot/pose", Vector3, queue_size = 3)
    pose_data = Vector3()
    server = Server()
    try:
        rospy.Subscriber('/robot/encoder', Vector3, server.encoder_callback)
        rospy.Subscriber('/robot/bno055', Vector3, server.imu_callback)
        rospy.Subscriber('/robot/update_pose', Vector3, server.pose_callback)        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
