#!/usr/bin/env python  
import roslib
roslib.load_manifest('ros_course')
import rospy
from math import atan2
import tf
import geometry_msgs.msg

rospy.init_node('tf_turtle')

listener = tf.TransformListener()

rate = rospy.Rate(1.0)

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/bob/turtle1/base_link', '/margaret/turtle1/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    print "Margaret relative to Bob"
    print trans
    print rot

    rate.sleep()
