#!/usr/bin/env python  
# import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
# import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('uku_tf_listener')

    listener = tf.TransformListener()
    '''
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
    '''

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base', '/ar_marker_1', rospy.Time(0))
            rospy.set_param('~trans', trans)
            rospy.set_param('~rot', rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
