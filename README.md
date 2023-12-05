# ros_course
Short ROS training course at the Bristol Robotics Lab, developed by the FARSCOPE Centre for Doctoral Training

## Background

ROS (in our context) stands for the [Robot Operating System](https://ros.org/).  It's not really an operating system as it runs on top of Ubuntu Linux, but it provides a blackboard-type message passing and standards for robots.  ROS is great for being supported by lots of off-the-shelf robots, making it easy to share and build on existing software, and taking care of repetitive housekeeping stuff like logging.  However, the learning curve is quite steep, it's quite limited to Ubuntu, the timing model is challenging, and it's hard to keep up with regular upgrades.

> ROS2 has been released that improves things like networking, security, and the timing model.  It also offers Windows support.  However, it is sometimes even more complicated.  We've chosen to stay with ROS "1" for now.

This version of tutorial is written and tested for ROS Noetic (having evolved over years from previous versions).  All coding will be done in Python, although C++ is also supported.

Key ROS topics are *nodes* (programmes that talk using ROS) and *topics* (channels over which messages are exchanged).  A ROS _node_ works by publishing and/or subscribing to different ROS _topics_.

> ROS topics are asynchronous, like emails.  You send one, and you may or may not get an answer.  ROS also supports *services* which are like phone calls.  A services is called and gives a response.  Services are gest avoided for various reasons, not least because they block the caller.  There are also *actions* which are special arrangements of _topics_ offering call, response, progress reports, cancellation or completion.  They're much more flexible and robust, but complicated, beyond the scope.

## Set up your Workspace

Having [installed ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment), open up a terminal and type the following to create the workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
If that fails, try `source /opt/ros/noetic/setup.bash` and then try again.

Packages in your workspace are compiled using the following, test this now.
```
cd ~/catkin_ws/
catkin_make
```
To let ROS know where the workspace is you should update your .bashrc with the following command.  This will save you a lot of typing over the course of this tutorial.
```
echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc
```
The double chevrons after an echo allow you to append to a file, we can check how this has changed the .bashrc by typing
```
less ~/.bashrc
```
Press Q to exit the file preview. Now reopen the terminal or type
```
source ~/.bashrc
```
We can check that this all worked by looking at the ROS package path with
```
echo $ROS_PACKAGE_PATH
```
which should provide the following output
```
/home/USERNAME/catkin_ws/src:/opt/ros/noetic/share
```

## Drive a Turtle

Start a ROS core (the hub of all ROS communications – you’ll get “cannot communicate with master” errors if you try and do anything ROS-based without a core running)
```
roscore
```
Now in a new terminal or tab (press Ctrl+Shift+T), run the turtle simulator:
```
rosrun turtlesim turtlesim_node
```
And now manually send a command to the turtle
```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist '{linear:  {x: 1.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.2}}'
```
And then use RQT for the same job
```
rqt
```

## Creating a ROS Package
Create a package called “ros_course” for use with all the exercises
```
cd ~/catkin_ws/src
catkin_create_pkg ros_course std_msgs rospy turtlesim
cd ..
catkin_make
```
This creates a package called ros_course, which depends on the packages std_msgs, rospy and turtlesim
Check ROS has found your new package by looking for it in the package list:
```
rospack list
```

## A Publisher Node
Put the following in a file `drive.py` in the `scripts` directory of your new ROS package. 
```
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
# constant speed for now
turtle_speed = 2.0

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
```
*Make it executable*
```
chmod +x drive.py
```
Make sure your `roscore` and turtle simulator are still running, and now type
```
rosrun ros_course drive.py
```
You should see your turtle wander aimlessly round its world, probably up against the wall before long.

> You would have got away with just executing the file, but the `rosrun` is the _right_ way and we'll need it later.

