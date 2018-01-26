#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt

class TurtleControlNode:

  def __init__(self):
    self.radius = 4.0
    # start the node
    rospy.init_node('loop_tidy')    
    # set up a publisher
    self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=3)
    # rate and control
    self.rate = rospy.Rate(10)
    self.msg = Twist()

  def poseCallback(self,data):
    self.radius = sqrt((data.x-5.0)**2 + (data.y-5.0)**2)
    rospy.loginfo("radius is now %f" % self.radius)

  def run(self):
    # start the subscriber
    rospy.Subscriber("turtle1/pose",Pose,self.poseCallback)
    # main control loop
    while not rospy.is_shutdown():
      turn_rate = 0.3*(self.radius - 4.0)
      self.msg.linear.x = 0.5
      self.msg.angular.z = turn_rate
      self.pub.publish(self.msg)
      self.rate.sleep()

if __name__=='__main__':
  t = TurtleControlNode()
  t.run()

