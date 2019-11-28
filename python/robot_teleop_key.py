#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import numpy as np

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
	--	ROBOT TELEOP KEY --
----------------------------------
===== robot2 general command =====
		q	w	e

		a	s	d

			x

w = increase linear speed x by 0.25
x = decrease linear speed x by 0.25
a = increase linear speed y by 0.25
d = decrease linear speed y by 0.25
q = increase angular speed z by 0.25
e = decrease angular speed z by 0.25
CTRL-C to quit
"""
#skill, position, info, goalkeeper command
movement = {
		'w':( 0.25, 0.0 , 0.0),
		'x':(-0.25, 0.0 , 0.0),
		'a':( 0.0 , 0.25, 0.0),
		'd':( 0.0 ,-0.25, 0.0),
		'q':( 0.0 , 0.0 , 0.25),
		'e':( 0.0 , 0.0 ,-0.25),
		}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('robot_teleop_node')
	cmd_vel_publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size = 3)
	cmd_vel = Twist()
	x = 0.; y = 0.; th = 0.
	try:
		print(msg)
		while(1):
			key = getKey()
			if key in movement.keys():
				x  = x  + movement[key][0]
				y  = y  + movement[key][1]
				th = th + movement[key][2]
			else:
				x  = 0.
				y  = 0.
				th = 0.
				if (key == '\x03'):
					break	
			cmd_vel.linear.x = x
			cmd_vel.linear.y = y
			cmd_vel.angular.z = th
			cmd_vel_publisher.publish(cmd_vel)
			print(cmd_vel)
		
	except Exception as e:
		print(e)

	finally:
		cmd_vel.linear.x = x
		cmd_vel.linear.y = y
		cmd_vel.angular.z = th
		cmd_vel_publisher.publish(cmd_vel)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
