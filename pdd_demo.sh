#!/bin/bash

source devel/setup.bash

for ((i=1; i<=1; i++))
do
sleep 5 
roslaunch pdd_ros Town04_spawn_car.launch &
sim_pid=$!

sleep 5 
roslaunch pdd_ros Town04_generate_traffic.launch  &
traffic_pid=$!

sleep 5 
roslaunch pdd_ros Town04_pdd_carla.launch  &
node_pid=$!

sleep 1000
kill -s 9 $sim_pid
kill -s 9 $traffic_pid
kill -s 9 $node_pid

rosnode kill -a
sleep 5

done
