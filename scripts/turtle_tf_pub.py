#!/usr/bin/env python  
import roslib
roslib.load_manifest('ros_course')
import rospy
import tf
import turtlesim.msg
 
def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename+'/base_link',
                     "world")
 
if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster', anonymous=True)
    turtlename=rospy.resolve_name('turtle1')
    rospy.Subscriber('turtle1/pose',
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
