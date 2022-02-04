#!/bin/bash

echo 'stopping any active ROS processes'

sudo kill $(ps aux | grep '[m]isc/priorities.py' | awk '{print $2}') &>/dev/null
killall roslaunch &>/dev/null
killall rqt &>/dev/null
killall rviz &>/dev/null

sleep 2

killall roscore &>/dev/null
lidar_off

sleep 2

ps aux | grep '[r]os]' 
ps aux | grep '[m]isc/priorities.py'

