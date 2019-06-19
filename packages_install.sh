#!/bin/bash
###-- This file mainly install packages for sensors or navigation packages --### 
###-- WARNING: Read the instruction of each package carefully --###

###-- basic ros command and package --###
# rosbash, map_server
sudo apt-get -y install ros-kinetic-rosbash ros-kinetic-map-server

###-- joy --###
sudo apt-get -y install ros-kinetic-joy

###-- rplidar --###
sudo apt-get -y install ros-kinetic-rplidar-ros
# INSTRUCTION: Remember to comment one of the two lidars in solabot/src/solabot_navigation/launch/solabot_core.launch

###-- urg_node (hokuyo lidar) --###
sudo apt-get -y install ros-kinetic-urg-node
# INSTRUCTION: Remember to comment one of the two lidars in solabot/src/solabot_navigation/launch/solabot_core.launch

# (Alternative for urg_node) From source:
# cd ~/catkin_ws/src
#git clone https://github.com/ros-drivers/urg_node.git
#cd urg_node
#git checkout indigo-devel # there is no kinetic-level yet
#cd ..
#git clone https://github.com/ros-perception/laser_proc.git
#cd laser_proc
#git checkout indigo-devel # there is no kinetic-level yet
#cd ..
#git clone https://github.com/ros-drivers/urg_c.git
#cd ~/catkin_ws/src
#catkin_make

###-- razor_imu_9DOF --###
sudo apt-get -y install ros-kinetic-razor-imu-9dof
# INSTRUCTION: Copy solabot/src/solabot_navigation/scripts/solabot_imu.py to /opt/ros/kinetic/share/razor_imu_9dof/nodes

###-- rf2o --###
sudo apt-get -y install ros-kinetic-mrpt-ekf-slam-2d

###-- move_base --###
### move_base 
sudo apt-get -y install ros-kinetic-move-base
### dwa_local_planner 
sudo apt-get -y install ros-kinetic-dwa-local-planner
### teb_local_planner 
sudo apt-get -y install ros-kinetic-teb-local-planner
### global_planner
sudo apt-get -y install ros-kinetic-global-planner

###-- amcl --###
sudo apt-get -y install ros-kinetic-amcl

###-- robot_localization --###
sudo apt-get -y install ros-kinetic-robot-localization

###-- rqt --###
sudo apt-get -y install ros-kinetic-rqt
sudo apt-get -y install ros-kinetic-rqt-common-plugins

###-- mrpt (rf2o is part of this package) --###
sudo apt-get -y install mrpt-apps libmrpt-dev
# see trouble shooting if something is wrong #

###-- effort_controllers (solabot_simulator in gazebo) --###
sudo apt-get -y install ros-kinetic-effort-controllers

### rosserial-python
sudo apt-get -y install ros-kinetic-rosserial-python

### rosserial-arduino
sudo apt-get -y install ros-kinetic-rosserial-arduino

### ros-kinetic-joy
sudo apt-get -y install ros-kinetic-joy

###-- joint_state_controller --###
sudo apt-get -y install ros-kinetic-joint-state-controller

###-- slam-gmapping --###
sudo apt-get -y install ros-kinetic-slam-gmapping

###-- openssh --###
apt-get -y install openssh-server
# both TX2 and the remote computer need to install

###-- ros_controll --###
sudo apt-get -y install ros-kinetic-ros-control ros-kinetic-ros-controllers

###-- python-wtool for scan-tools --##
sudo apt-get -y install python-wstool

###-- scan_tools --##
sudo apt-get -y install ros-kinetic-scan-tools
