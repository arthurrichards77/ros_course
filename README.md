# ros_course
Short ROS training course at the Bristol Robotics Lab, developed by the FARSCOPE Centre for Doctoral Training

## Background

ROS (in our context) stands for the [Robot Operating System](https://ros.org/).  It's not really an operating system as it runs on top of Ubuntu Linux, but it provides a blackboard-type message passing and standards for robots.  ROS is great for being supported by lots of off-the-shelf robots, making it easy to share and build on existing software, and taking care of repetitive housekeeping stuff like logging.  However, the learning curve is quite steep, it's quite limited to Ubuntu, the timing model is challenging, and it's hard to keep up with regular upgrades.

> ROS2 has been released that improves things like networking, security, and the timing model.  It also offers Windows support.  However, it is sometimes even more complicated.  We've chosen to stay with ROS "1" for now.

This version of tutorial is written and tested for ROS Noetic (having evolved over years from previous versions).  All coding will be done in Python, although C++ is also supported.

Key ROS topics are *nodes* (programmes that talk using ROS) and *topics* (channels over which messages are exchanged).  A ROS _node_ works by publishing and/or subscribing to different ROS _topics_.

> ROS topics are asynchronous, like emails.  You send one, and you may or may not get an answer.  ROS also supports *services* which are like phone calls.  A services is called and gives a response.  Services are gest avoided for various reasons, not least because they block the caller.  There are also *actions* which are special arrangements of _topics_ offering call, response, progress reports, cancellation or completion.  They're much more flexible and robust, but complicated, beyond the scope.

