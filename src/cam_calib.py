#! /usr/bin/env python

import rospy
from sensor_msgs import CameraInfo

def main():
		rospy.init_node('cam_calib')
		calib_pub = rospy.Publisher("io/internal_camera/head_camera/camera_info/cmd_vel", Twist, queue_size = 10)
		YAML = CameraInfo()
		YAML



		loop_rate = rospy.Rate(10) #20Hz publishing
		calib_pub.publish(YAML)









if __name__ == '__main__':
   	try:
   		main()
   	except KeyboardInterrupt:
   		rospy.loginfo("node killed")