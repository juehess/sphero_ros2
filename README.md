# sphero_ros2
ROS2 wrapper for accessing Sphero robots. Currently only Sphero Bolt is supported.

# Acknowledgments
This ROS2 wrapper builds upon the following projects:

- [pysphero](https://github.com/EnotYoyo/pysphero/tree/master/pysphero)
- [spherov2](https://github.com/artificial-intelligence-class/spherov2.py)
- [sphero_ros](https://github.com/mmwise/sphero_ros)

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
  $ ros2 run sphero_node sphero -m <mac_address>
```
2. Run TF publisher:
The TF publisher publishes the transformation from /odom to /base_link
```
  $ ros2 run sphero_node sphero_tf_publisher
```
3. Run RVIZ and visualization
```
  $ ros2 launch sphero_description sphero.launch.py
```
