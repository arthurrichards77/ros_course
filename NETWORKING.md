# ROS Networking Example

> This may not work!  Some networks will block ROS traffic.

Partner with someone next to you on a different computer.  We’ll call the two PCs “PC A” and “PC B”.  Make sure you get these clearly identified as you’ll need to set them up carefully.

1. Identify PC A’s IP address by running `ifconfig`.  Somewhere in the output, you’ll see an address like “164.11.73.XX”.  Write it down.  Do the same for PC B.
2. On PC A, run `ping <PC B’s IP addr>` and check you get reply messages.  Repeat vice versa on PC B.  **Give up if you can't get this to work.**
3. On PC A, make a file named “roscommA.sh” as follows:
```
export ROS_IP=<PC A’s IP addr>
```
4. Then on PC B, make another file named “roscommB.sh” as follows:
```
export ROS_IP=<PC B’s IP addr>
export ROS_MASTER_URI=http://<PC A’s IP addr>:11311
```
5. Back on PC A, open a new window or tab and run `source roscommA.sh` and then `roscore`.
4. Still on PC A, open a new window or tab and run `source roscommA.sh` and then `rosrun turtlesim turtlesim_node`.
5. And then over on PC B, open a new window or tab and run `source roscommB.sh` and then `rostopic list`.

If successful, you should be able to see the `/turtle1/...` topics from PC A's turtle simulator from PC B.  If you can, try driving the PC A turtle from PC B.
