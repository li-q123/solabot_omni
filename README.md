# solabot_omni
ROS navigation stack implementing on omni (holonomic) vehicles.
> Gazebo simulation is scheduled to be created in summer 2019

## To Start
### Navigation
1. Run `roscore` in terminal of the master computer.
2. Launch `solabot_rpi.launch` on the Raspberry Pi. Then launch `solabot_movebase_teb.launch` on the master computer.
> local planner __teb__ is stable. __dwa__ should be tested further.

### Other Launch Files
* __teleop__: Keyboard teleop.
> keyboard teleop: 
> u i o
> j k l 
> m , .
* __gmapping__: Building maps manually. Please use keyboard to teleop.
* __explore__: Auto-mapping (greedy frontier-based exploration).
* __amcl__: Perform localization given a map. 
* __core__: Merge scans. Provide odometry (__rf2o_laser_odometry__ + __robot_localization__)

## Packages that we also used

### [rf2o_laser_odometry](http://wiki.ros.org/rf2o)
Estimation of 2D odometry based on planar laser scans.

### [ira_laser_tools (kinetic branch)](https://github.com/iralabdisco/ira_laser_tools/tree/kinetic)
Merge multiple laser scans.

### [explore-lite](http://wiki.ros.org/explore_lite)
Auto-mapping.

## Other references
### [node_example](https://github.com/tdenewiler/node_example)

### [husky_navigation](http://wiki.ros.org/husky_navigation)

## Blacklist
[frontier_exploration](http://wiki.ros.org/frontier_exploration): Not stable.
