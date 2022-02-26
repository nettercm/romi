#!/bin/bash

echo 'stopping any active ROS processes'

sudo kill $(ps aux | grep '[m]isc/priorities.py' | awk '{print $2}') &>/dev/null
killall roslaunch &>/dev/null
killall rqt &>/dev/null
killall rviz &>/dev/null

sleep 2

killall roscore &>/dev/null
lidar_off

sudo chrt -p -r 3 $$
sudo sysctl kernel.sched_rr_timeslice_ms=1
sudo cpufreq-set -g performance

sleep 2 

echo 'starting ROS'

echo 'adjusting kernel thread priorities'
sudo misc/priorities.py < /dev/null &>/dev/null &
# how to kill:
# sudo kill $(ps aux | grep '[m]isc/priorities.py' | awk '{print $2}')

echo 'starting roscore'
roscore &
#< /dev/null &>/dev/null &
# how to kill:
#killall roscore

lidar_on

lidar_speed 400

sleep 8

cd ros

echo 'starting robot state publisher'
roslaunch ./robot_state.launch &
#< /dev/null &>/dev/null &

echo 'starting map-to-odom transform publisher'
roslaunch ./tf_map_to_odom.launch &
#< /dev/null &>/dev/null &

roslaunch ./rplidar.launch &

cd ..

sleep 8

ps aux | grep '[r]os]' 
ps aux | grep '[m]isc/priorities.py'

