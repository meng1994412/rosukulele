#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
# from ar_track_alvar_msgs.msg import AlvarMarkers
import geometry_msgs.msg
import std_msgs.msg
import numpy as np

global trans
global rot


# def callback(msg):
# 	global trans
# 	global rot
# 	Marker = AlvarMarkers()
# 	x_o = msg.markers[0].pose.pose.orientation.x
# 	y_o = msg.markers[0].pose.pose.orientation.y
# 	z_o = msg.markers[0].pose.pose.orientation.z
# 	w_o = msg.markers[0].pose.pose.orientation.w
	
# 	x_p = msg.markers[0].pose.pose.position.x
# 	y_p = msg.markers[0].pose.pose.position.y
# 	z_p = msg.markers[0].pose.pose.position.z

# 	q1 = rot[0]
# 	q2 = rot[1]
# 	q3 = rot[2]
# 	q0 = rot[3]
# 	tx = trans[0]
# 	ty = trans[1]
# 	tz = trans[2]

# 	T = np.array([
# 				  [(1-2*(q2**2 + q3**2)), (2*(q1*q2 -q0*q3)), (2*(q0*q2 + q1*q3)), tx],
# 				  [(2*(q1*q2 +q0*q3)), (1-2*(q1**2 +q3**2)), (2*(q2*q3 - q0*q1)),ty],
# 				  [(2*(q1*q3-q0*q2)), (2*(q0*q1 +q2*q3)), (1-2*(q1**2+q2**2)), tz],
# 				  [0, 0, 0, 1]
# 				  ])

# 	x_new 


# 	print(T)




def main():
	global rot
	global trans
	rospy.init_node('uku_tf_listener')

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
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
			print(T)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# print("tf error")
			continue
		# rospy.Subscriber("/ar_pose_marker",AlvarMarkers, callback)
		rate.sleep()
    


if __name__ == '__main__':
	try:
   		main()
   	except KeyboardInterrupt:
   		rospy.loginfo("node killed")