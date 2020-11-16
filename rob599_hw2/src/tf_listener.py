#!/usr/bin/env python

import rospy, geometry_msgs.msg
import tf


if __name__ == '__main__':
    rospy.init_node('listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform("/map", "/base_link" , rospy.Time(0))
            print(trans,rot)
            if trans:
                break
    # This will give you the coordinate of the child in the parent frame
        except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
            pass
    rate.sleep()
