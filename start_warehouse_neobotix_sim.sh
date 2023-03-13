#! /bin/bash 

NEOBOTIX_SIM_PATH=/home/simulations/ros2_sims_ws/install/setup.bash
source $NEOBOTIX_SIM_PATH
export GAZEBO_MODEL_PATH=/home/simulations/ros2_sims_ws/src/neobotix_ros2/neo_simulation2/models:/home/simulations/ros2_sims_ws/src:/home/simulations/ros2_sims_ws/src/neobotix_ros2:/home/simulations/ros2_sims_ws/src/aws_simulations/aws-robotics/aws-robomaker-small-warehouse-world/models
ros2 launch neo_simulation2 simulation_warehouse.launch.py
