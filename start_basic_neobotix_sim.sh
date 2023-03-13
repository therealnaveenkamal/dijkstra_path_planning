#! /bin/bash 

NEOBOTIX_SIM_PATH=/home/user/ros2_ws/install/setup.bash
source $NEOBOTIX_SIM_PATH
export GAZEBO_MODEL_PATH=/home/user/ros2_ws/src/neobotix_ros2/neo_simulation2/models:/home/user/ros2_ws/src:/home/user/ros2_ws/src/neobotix_ros2
ros2 launch neo_simulation2 simulation_basics.launch.py
