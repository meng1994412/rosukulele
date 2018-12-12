#!/usr/bin/env python

import rospy
from rosukulele.msg import Pitch
from rosukulele.srv import GetPitch
total = 0
count = 0

def msgRec(msg):
	global total
	global count

	count += 1
	total += msg.note

def getPitch(something):
	global count
	global total

	count = 0
	while (count<3):
		continue
	count = 0
	total = 0.0
	while (count<3):
		continue
	return total/3

def listener():
	rospy.init_node('pitches')
	rospy.Subscriber("/pitch", Pitch, msgRec)
	s = rospy.Service('get_pitch', GetPitch, getPitch)
	print("Ready to call get_pitch")
	rospy.spin()

if __name__ == '__main__':
	listener()