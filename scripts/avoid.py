#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

inside = False

# callback for command
def cmdCallback(data):
    # prepare command message
    msg = data
    if inside:
        rospy.logwarn("In obstacle - turning")
        msg.angular.z = 2        
    # send it
    pub.publish(msg)

# callback for pose does all the work
def poseCallback(data):
    global inside
    # check for exit
    if (data.x-8)*(data.x-8)+(data.y-5)*(data.y-5) < 4:
        rospy.logwarn("Entered obstacle")
        inside=True
    else:
        inside=False

# start the node
rospy.init_node('avoid')    
# set up a publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=3)
# and the subscribers
rospy.Subscriber("turtle1/pose",Pose,poseCallback)
rospy.Subscriber("turtle1/ref_vel",Twist,cmdCallback)
rospy.spin()
