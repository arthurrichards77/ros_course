#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
import random

# set up a publisher
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
# start the node
rospy.init_node('driver')
# will be updating at 2 Hz
r = rospy.Rate(2)

# get name for topic
cmd_vel_name = rospy.resolve_name('turtle1/cmd_vel')
# show it
rospy.loginfo("Publishing to topic %s" % cmd_vel_name)

# get speed from parameter
turtle_speed = rospy.get_param('turtle1/turtle_speed',1.0)

while not rospy.is_shutdown():
   # make a blank velocity message
   msg = Twist()
   # pick a random direction
   msg.angular.z = 2*(random.random() - 0.5)
   # constant speed
   msg.linear.x = turtle_speed
   # publish it
   pub.publish(msg)
   # show a message
   rospy.loginfo("New turn rate=%s"%msg.angular.z)
   # wait for next time
   r.sleep()

