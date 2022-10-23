# METR4202 REPO

## Dependencies
-fiducial_msgs

-geometry_msgs

-rospy

-sensor_msgs

-std_msgs

## Steps to run
Open terminal
'''
cd METR4202/catkin_ws

git checkout main

git pull

catkin build

source devel/setup.bash

usb
'''
### For stationary pickup
'''
roslaunch demo_pkg stationary.launch
'''
### For moving pickup
'''
roslaunch demo_pkg moving.launch
'''
### For throwing the block
'''
roslaunch demo_pkg yeet.launch
'''
