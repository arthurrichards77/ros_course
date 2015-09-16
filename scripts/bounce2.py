#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

speed = 1.0
mode = 0

# callback for pose does all the work
def poseCallback(data):
    global speed
    global mode
    # check for exit
    if data.x>10:
        if mode==0:
            speed=-speed
            mode=1
    elif data.y>10:
        if mode==0:
            speed=-speed
            mode=1
    elif data.x<1:
        if mode==0:
            speed=-speed
            mode=1
    elif data.y<1:
        if mode==0:
            speed=-speed
            mode=1
    else:
        mode=0

# start the node
rospy.init_node('bounce')    
# set up a publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=3)
# and the subscriber
rospy.Subscriber("turtle1/pose",Pose,poseCallback)

# loop
r = rospy.Rate(2)
while not rospy.is_shutdown():
    turn = 5*(random.random()-0.5)
    # prepare command message
    msg = Twist()
    msg.linear.x = speed
    msg.angular.z = turn
    # send it
    pub.publish(msg)
    # sleep
    r.sleep()

