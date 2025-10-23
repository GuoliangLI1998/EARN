#!/bin/bash

source devel/setup.bash

sleep 5 
roslaunch earn_ros Town04_spawn_earn.launch &
sim_pid=$!

sleep 5 
roslaunch earn_ros Town04_traffic_earn.launch  &
traffic_pid=$!

sleep 5 
roslaunch earn_ros Town04_run_earn.launch  &
node_pid=$!

sleep 1000
kill -s 9 $sim_pid
kill -s 9 $traffic_pid
kill -s 9 $node_pid

rosnode kill -a
