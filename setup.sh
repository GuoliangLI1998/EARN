#!/bin/bash

cd ./src

# Clone rvo_ros repository
if [ ! -d "ros-bridge" ]; then
  git clone https://github.com/carla-simulator/ros-bridge
else
  echo "ros bridge already cloned."
fi

# Build the workspace
cd ..
catkin_make

echo "Build Carla ROS bridge completed successfully."


