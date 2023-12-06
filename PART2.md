# ros_course
# Part 2: URDF, TF, etc

So far, we have learnt how to use ROS for building and running systems of interconnected software _nodes_ that exchange information using _topics_.  In Part 2, we will build on this to show ROS' robot visualization techniques (URDF - the universal robot description format) and its geometric transformation utilities (TF - transform).

## Modeling the turtle in URDF

Create a `urdf` subfolder in your `ros_course` folder.  In it, make the following file and call it something like `turtle.xml` with these contents:
```xml
<?xml version="1.0"?>
<robot name="turtle">

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.5"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0.11"/>
    </visual>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0.5 0 0.2"/>
    </visual>
  </link>

  <joint name="head_to_body" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
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
