#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt

# start the node
rospy.init_node('listen')    

# callback for pose does all the work
def pose_callback(data):
  rospy.loginfo("x is now %f" % data.x)

# and the subscriber
rospy.Subscriber("turtle1/pose",Pose,pose_callback)
rospy.spin()
