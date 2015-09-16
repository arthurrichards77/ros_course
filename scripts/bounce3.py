#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

# callback for pose does all the work
def poseCallback(data):
    # check for exit
    if data.x>10:
      turn=1.0
    elif data.y>10:
      turn=1.0
    elif data.x<1:
      turn=1.0
    elif data.y<1:
      turn=1.0
    else:
      turn = 5*(random.random()-0.5)
    # prepare command message
    msg = Twist()
    msg.linear.x = 1.0
    msg.angular.z = turn
    # send it
    pub.publish(msg)

# start the node
rospy.init_node('bounce')    
# set up a publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=3)
# and the subscriber
rospy.Subscriber("turtle1/pose",Pose,poseCallback)
rospy.spin()
