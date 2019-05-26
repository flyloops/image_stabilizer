# image_stabilizer 

ROS package which provides simple image stabilization. Designed to stabilize
the camera images on the Rockafly Arena.  Consists of a single node which
provides image stabilization. Can be enable/disabled via a proxy service.

## Installation

1. Install ROS. Instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
if you have not already done this.  Tested with ros kinetic, desktop install

2. Setup your catkin workspace.  Instructions can be found [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 
if you have not already done this. 

3. Then clone the git repository and run catkin make.

```bash
$cd ~/catkin_ws/src
$git clone https://github.com/willdickson/image_stabilizer.git
$cd ~/catkin_ws
$catkin_make

```


## Example launch file 

```xml
<launch>
    <node pkg="image_stabilizer" name="my_image_stabilizer" type="image_stabilizer_node.py">
        <param name="threshold" type="int" value="90"/>
        <param name="input_image" type="str" value="/camera/image_mono"/>
        <param name="output_image" type="str" value="/stabilized_image"/>
        <param name="show_contour_image" type="bool" value="true"/>
        <param name="show_stabilized_image" type="bool" value="true"/>
        <param name="enabled" type="bool" value="true"/>
    </node>
</launch>
```

