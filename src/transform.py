#!/usr/bin/env python
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
from rosukulele.srv import Transform

global T

# def callback(msg):
# 	global trans
# 	global rot
# 	Marker = AlvarMarkers()
# 	x_o = msg.markers[0].pose.pose.orientation.x
# 	y_o = msg.markers[0].pose.pose.orientation.y
# 	z_o = msg.markers[0].pose.pose.orientation.z
# 	w_o = msg.markers[0].pose.pose.orientation.w
#
# 	x_p = msg.markers[0].pose.pose.position.x
# 	y_p = msg.markers[0].pose.pose.position.y
# 	z_p = msg.markers[0].pose.pose.position.z


def transform_matrix(myArgs):
	global T
	count = 0
	rate = rospy.Rate(10.0)
	listener = tf.TransformListener()
	# listener.waitForTransform("/base", "/ar_marker_2", rospy.Time(), rospy.Duration(4.0))
	while not rospy.is_shutdown() and count < 30000:
		try:
			now = rospy.Time.now()
			# listener.waitForTransform("/base", "/ar_marker_2", now, rospy.Duration(2.0))
			(trans,rot) = listener.lookupTransform('/base', '/ar_marker_2', rospy.Time(0))
			
			# rospy.set_param('~trans', trans)
			# rospy.set_param('~rot', rot)
			q1 = rot[0]
			q2 = rot[1]
			q3 = rot[2]
			q0 = rot[3]
			tx = trans[0]
			ty = trans[1]
			tz = trans[2]
			T = np.array([
						  [(1-2*(q2**2 + q3**2)), (2*(q1*q2 -q0*q3)), (2*(q0*q2 + q1*q3)), tx],
						  [(2*(q1*q2 +q0*q3)), (1-2*(q1**2 +q3**2)), (2*(q2*q3 - q0*q1)),ty],
						  [(2*(q1*q3-q0*q2)), (2*(q0*q1 +q2*q3)), (1-2*(q1**2+q2**2)), tz],
						  [0, 0, 0, 1]
						  ])
			
			new = myArgs.call.split(" ")
			x_n =eval(new[0])
			y_n =eval(new[1])
			z_n =eval(new[2])
			new = np.array([[x_n],[y_n],[z_n],[1]])


			new_point = np.matmul(T,new)
			new_point_str = str(new_point[0][0]) +" "+str(new_point[1][0]) +" "+str(new_point[2][0])
			print(new_point_str)
			
			return new_point_str
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			#print("tf error")
			count+=1
			print(count)
			continue
		rospy.sleep(rate)

	#if using old t
	print("using old T")
	new = myArgs.call.split(" ")
	x_n =eval(new[0])
	y_n =eval(new[1])
	z_n =eval(new[2])
	new = np.array([[x_n],[y_n],[z_n],[1]])
	new_point = np.matmul(T,new)
	new_point_str = str(new_point[0][0]) +" "+str(new_point[1][0]) +" "+str(new_point[2][0])
	print(new_point_str)
	return new_point_str


def main():
	rospy.init_node('uku_tf_listener')

	s = rospy.Service('transformation_matrix', Transform, transform_matrix)
	print("Ready to call Transform")
	rospy.spin()



if __name__ == '__main__':
	try:
   		main()
   	except KeyboardInterrupt:
   		rospy.loginfo("node killed")