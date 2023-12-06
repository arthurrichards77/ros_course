# ros_course
# Part 2: URDF, TF, etc

So far, we have learnt how to use ROS for building and running systems of interconnected software _nodes_ that exchange information using _topics_.  In Part 2, we will build on this to show ROS' robot visualization techniques (URDF - the universal robot description format) and its geometric transformation utilities (TF - transform).

## Modeling the turtle in URDF

Create a `urdf` subfolder in your `ros_course` folder.  In it, make the following file and call it something like `turtle.xml` with these contents:
```xml
<?xml version="1.0"?>
<robot name="turtle">

  <link name="turtle1/base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.5"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0.11"/>
    </visual>
  </link>

  <link name="turtle1/head">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0.5 0 0.2"/>
    </visual>
  </link>

  <joint name="turtle1/head_to_body" type="fixed">
    <parent link="turtle1/base_link"/>
    <child link="turtle1/head"/>
    <origin xyz="0 0 0"/>
  </joint>

</robot>
```
This describes our model as two _links_ (solid shapes) connected by a single fixed _joint_.

> URDF can handle much more complicated models including CAD descriptions of links and various types of moving joints.  We're only just dipping in.

The URDF tutorial package in ROS comes with a helpful viewing utility for developing URDF models, which we shall use to inspect the model we have built.  Run the following from the URDF folder:
```
roslaunch urdf_tutorial display.launch model:=turtle.xml 
```
All being well you should see a turtle picture.  The viewer program is RVIZ, which we will see much more of.

![URDF model of a turtle robot](turtle_urdf.png)

## Animating the turtle

It would be good to connect our turtle model to the simulation, with the goal of seeing multiple turtles move together and interact.  We'll do this using TF, a ROS convention that enables common handling of _transforms_ between reference frames.  Any node can publish a transform message to the common topic `/tf` reporting where one named frame is relative to another.  Then a standard software client can subscribe to `/tf` and calculate the relation between any two known frames.  We'll use it to figure out where one turtle is relative to another.

> This illustrates a different way of using a topic, as a broadcast channel rather than point-to-point.

Some simulators and robots support TF themselves, but we need an extra node to report the turtle position as a transform using TF.  Make the following in your scripts folder:
```python
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
```
Note that TF is so common, they've wrapped the publishing in a little client helper for us.  This would work with the same `publish` method we used earlier too.

Finally we will need a launch file to start everything:
```xml
<launch>
    <param name="robot_description" command="cat $(find ros_course)/urdf/turtle.xml" />
    <node name="turtlesim" pkg="turtlesim" type="turtlesim_node" />
    <node name="driver" pkg="ros_course" type="drive.py" />
    <node name="tfpub" pkg="ros_course" type="turtle_tf_pub.py" />
    <node name="rviz" pkg="rviz" type="rviz" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
</launch>
```
It's worth talking this through:
```xml
<param name="robot_description" command="cat $(find ros_course)/urdf/turtle.xml" />
```
The line above reads our URDF model into a parameter named `robot_description`.  RViz and several other ROS tools look here by default for the URDF of the robot.
```xml
<node name="turtlesim" pkg="turtlesim" type="turtlesim_node" />
<node name="driver" pkg="ros_course" type="drive.py" />
```
The two lines above start the turtle simulator and our rdiver node from part 1, that just made it wander randomly.
```xml
<node name="tfpub" pkg="ros_course" type="turtle_tf_pub.py" />
```
Above starts the new node that publishes the turtle pose to `/tf`.
```xml
<node name="rviz" pkg="rviz" type="rviz" />
```
Above launches the `RViz` viewer tool.
```xml
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
```
Finally the line above starts a `robot_state_publisher` node which is a standard tool in ROS.  It reads the URDF from the `robot_description` model and calculates the transforms between the different links, publishing them to TF.  THis is the link between URDF and TF.  If our robot model including moving joints, it would read their values in and determine the forward kinematics for us. 
