# sphero_ros2
ROS2 wrapper for accessing Sphero robots. Currently only Sphero Bolt is supported.

# Acknowledgments
This ROS2 wrapper builds upon the following projects:

- [pysphero](https://github.com/EnotYoyo/pysphero/tree/master/pysphero)
- [spherov2](https://github.com/artificial-intelligence-class/spherov2.py)
- [sphero_ros](https://github.com/mmwise/sphero_ros)

# Functionalities:

## TF
The driver provides the frame /base_link, /base_footprint, and /odom

## Provided Services
### Set stabilization:
- Data type: std_srvs/SetBool
- Topic: set_stabilization
- Description: Turns stabilization on/off

### Set text:
- Data type: sphero_interfaces/SetText
- Topic: set_text
- Description: Displays text on main LED (maximum is 6 characters)

## Provided Listeners
### Set back LED
- Data type: std_msgs/ColorRGBA
- Topic: set_back_led
- Description: Sets the color of the back LED of the robot

### Set front LED
- Data type: std_msgs/ColorRGBA
- Topic: set_front_led
- Description: Sets the color of the front LED of the robot
  
### Set main LED  
- Data type: std_msgs/ColorRGBA
- Topic: set_main_led
- Description: Sets the color of the entire LED array of the robot

### Set command velocity:
- Data type: geometry_msgs/Twist
- Topic: cmd_vel
- Description: Sets the direction (in odometry/global frame of reference) 
and speed of the robot given a velocity in x and y direction

### Set command velocity relative:
- Data type: geometry_msgs/Twist
- Topic: cmd_vel
- Description: Sets the direction (in local/robot frame of reference) 
and speed of the robot given a velocity in x and y direction

### Set heading:
- Data type: std_msgs/Float32 (defined in radians; from -pi to pi)
- Topic: set_heading
- Description: Sets the heading of the robot in the global/odom reference frame 

### Set raw motor command (behaves strangely) :
- Data type: sphero_interfaces/RawMotorCommand (defined from -255 to 255)
- Topic: raw_motor_command
- Description: Sets raw speed of left and right motor

# Installation
0. Edit .bashrc and add:
```
  export LC_NUMERIC="en_US.UTF-8"
``` 

1. Create ROS2 workspace
```
  $ mkdir -p sphero_ws/src
``` 

2. Download source
```
  $ cd sphero_ws/src
  $ git clone https://github.com/juehess/sphero_ros2.git
```
3. Update git submodules:
```
  $ cd sphero_ros2
  $ git submodule init
  $ git submodule update
```
4. Install pysphero driver
```
  $ pip3 install lib/pysphero
```
5. Install ROS2 dependencies
```
  $ cd sphero_ws
  $ rosdep install --from-paths src/ --ignore-src -y
```
6. Build workspace
```
  $ colcon build
```
7. Source workspace
```
  $ . install/setup.bash
```

# Running
0. Find and note down the mac address of your sphero robot
1. Running the ROS2 driver
```
  $ ros2 run sphero_node sphero -m <??:??:??:??:??:??>
```
2. Run TF publisher:
The TF publisher publishes the transformation from /odom to /base_link
```
  $ ros2 run sphero_node sphero_tf_pub
```
3. Run RVIZ and visualization
```
  $ ros2 launch sphero_description sphero.launch.py
```
