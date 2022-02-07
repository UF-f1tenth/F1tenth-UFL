#!/usr/bin/env python

import rospy
import pdb
import numpy as np
import tf
import os
import time
import sys, select, termios, tty
from race.msg import drive_param
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header

pub = rospy.Publisher("vesc/high_level/ackermann_cmd_mux/input/nav_0",AckermannDriveStamped,queue_size = 10)
freq = 36

def get_js_input_ssh():
	tty.setraw(sys.stdin.fileno())
	print("You have 1s for ip : ")
	select.select([sys.stdin], [], [], 1)
	line = sys.stdin.readline()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	print("Received : ", line)
	if line.find("ip["):
		line.replace("ip[","")
		line.replace("]","")
		vel = float(line.split(",")[0])
		angle = float(line.split(",")[1])
		if vel < 0.4:
			vel = 0.0
		return True, vel, angle

	else:
		return False, 0.0, 0.0 


def callback():
	rospy.init_node("dead_mans_switch")
	rate = rospy.Rate(freq)

	angle    = 0.0
	velocity = 0.0
	counter  = 0
	
	while not rospy.is_shutdown():
		received, velocity_ip, angle_ip = get_js_input_ssh()
		if received:
			velocity = velocity_ip
			angle    = angle_ip
			counter  = 0
		else:
			counter +=1
				
		if counter > 20:
			print("No input! Stopping.")
			angle    = 0.0
			velocity = 0.0
			counter  = 0

		drive_msg = AckermannDriveStamped()
		drive_msg.header.stamp = rospy.Time.now()
		drive_msg.header.frame_id = "base_link"
		drive_msg.drive.steering_angle = angle
		drive_msg.drive.speed = velocity
		pub.publish(drive_msg)
		#rate.sleep()

if __name__ == "__main__":
	settings = termios.tcgetattr(sys.stdin)
	callback()
	rospy.spin()
