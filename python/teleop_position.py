#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import numpy as np

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Vector3

import sys, select, termios, tty

msg = """
Welcome to -- ROBSONEMA BASE STATION TELEOP KEY --
---------------------------
===== robot2 general command =====
s = Stop

CTRL-C to quit
"""
#skill, position, info, goalkeeper command
position = {
		'S':(0.,0.,0.,0),
		'q':(0.,0.,0.,2),
		'w':(0.,0.,90.,2),
		'e':(0.,0.,-90.,2),
		'a':(2.,0.,0.,2),
		's':(0.,1.,0.,2),
		'd':(1.,1.,0.,2),
		'f':(2.,0.,90.,2),
		'z':(0.,2.,0.,2),
		'x':(5.,0.,0.,2),
		'r':(1.,-1.,0.,2),
		't':(-1.,1.,0.,2),
		'y':(1.,0.,0.,2),
		}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('position_teleop_node')
	mode_publisher = rospy.Publisher('/team/game_mode', Int32MultiArray, queue_size = 2)
	position_publisher = rospy.Publisher('/robot/target_pose', Vector3, queue_size = 2)
	command = Int32MultiArray()
	pos = Vector3()
	command.data = [0,0]
	try:
		print(msg)
		while(1):
			key = getKey()
			if key in position.keys():
				pos.x = position[key][0]
				pos.y = position[key][1]
				pos.z = position[key][2] * np.pi / 180.
				mod = position[key][3]
				att = 0
			else:
				cod = 0
				att = 0
				if (key == '\x03'):
					break	
			command.data[0] = mod
			command.data[1] = att 
			mode_publisher.publish(command)
			position_publisher.publish(pos)
			print(command, pos)
		
	except Exception as e:
		print(e)

	finally:
		head = 0
		cod = 0
		od = 0
		command.data[0] = mod 
		command.data[1] = att 
		mode_publisher.publish(command)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
