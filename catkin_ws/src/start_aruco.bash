#!/bin/bash
#
# source this file to start everything

PIDS=

cd ~/catkin_ws

source devel/setup.bash

echo "Fix USB memory sizing"
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb

echo "Run roscore"
roscore &
PIDS="$PIDS $!"
sleep 5

echo "Start camera"
rosrun ximea_ros ximea_demo &
PIDS="$PIDS $!"
sleep 5

echo "Aruco launch"
roslaunch ximea_ros ximea_aruco.launch serial:=32704451 &
PIDS="$PIDS $!"
sleep 5

echo "Aruco detect"
rosrun rqt_image_view rqt_image_view &
PIDS="$PIDS $!"

echo
echo
echo "Press ENTER to exit."
echo
echo
read t

kill $PIDS
wait






