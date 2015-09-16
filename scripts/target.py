#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sin, cos, sqrt

seq=0

# callback for pose does all the work
def poseCallback(data):
    # only do every 5
    global seq
    seq=seq+1
    if (seq % 5)==0:
        actualCallback(data)

def actualCallback(data):
    # get speed
    speed = rospy.get_param('turtle1/turtle_speed',1.0)
    # get target
    target_x = rospy.get_param('turtle1/target/x',7.0)
    target_y = rospy.get_param('turtle1/target/y',7.0)
    # relative vector
    dx = target_x - data.x
    dy = target_y - data.y
    # velocity vector
    vx = speed*cos(data.theta)
    vy = speed*sin(data.theta)
    # get prop nav gain
    k = rospy.get_param('turtle1/k',-6.0)
    # set angular command
    turn = k*(dx*vy - dy*vx)/(dx*dx + dy*dy)
    # prepare command message
    msg = Twist()
    msg.linear.x = speed
    msg.angular.z = turn
    # send it
    pub.publish(msg)
    # log
    rospy.loginfo("Aiming for (%f,%f) turning %f" % (target_x,target_y,turn))

# start the node
rospy.init_node('target')    
# set up a publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=3)
# and the subscriber
rospy.Subscriber("turtle1/pose",Pose,poseCallback)
rospy.spin()
