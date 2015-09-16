#!/usr/bin/env python  
import roslib
roslib.load_manifest('ros_course')
import rospy
from math import atan2
import tf
from geometry_msgs.msg import Twist

rospy.init_node('turtle_stare')

listener = tf.TransformListener()

pub = rospy.Publisher('/bob/turtle1/cmd_vel', Twist, queue_size=3)

rate = rospy.Rate(10.0)

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/bob/turtle1/base_link', '/margaret/turtle1/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    ang_bob_sees_marg = atan2(trans[1],trans[0])

    msg = Twist()
    msg.angular.z = 0.3*ang_bob_sees_marg
    pub.publish(msg)

    rate.sleep()
