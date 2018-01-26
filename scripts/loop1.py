#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt

# start the node
rospy.init_node('bounce')    
# set up a publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=3)

# callback for pose does all the work
def poseCallback(data):
  radius = sqrt((data.x-5.0)**2 + (data.y-5.0)**2)
  rospy.loginfo("radius is now %f" % radius)
  turn_rate = 0.3*(radius - 4.0)
  msg = Twist()
  msg.linear.x = 0.5
  msg.angular.z = turn_rate
  pub.publish(msg)

# and the subscriber
rospy.Subscriber("turtle1/pose",Pose,poseCallback)
rospy.spin()
