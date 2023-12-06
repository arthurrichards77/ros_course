#!/usr/bin/env python  
import roslib
roslib.load_manifest('ros_course')
import rospy
import tf
import turtlesim.msg

class TurtleTfPublisher:

    def __init__(self):
        rospy.init_node('turtle_tf_broadcaster', anonymous=True)
        self.turtle_name = rospy.resolve_name('turtle1')
        self.tf_broadcaster = tf.TransformBroadcaster()
    
    def pose_callback(self, msg):
        self.tf_broadcaster.sendTransform((msg.x, msg.y, 0),
                                          tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                                          rospy.Time.now(),
                                          self.turtle_name+'/base_link',
                                          "world")

    def run(self):
        rospy.Subscriber('turtle1/pose',
                         turtlesim.msg.Pose,
                         self.pose_callback)
        rospy.spin()

if __name__ == '__main__':
    tp = TurtleTfPublisher()
    tp.run()
