#!/bin/bash

PACKAGE_PATH=$(rospack find gen3_moveit)

# Spawn all models in parallel
rosrun gazebo_ros spawn_model -file $PACKAGE_PATH/models/table1/model.sdf -sdf -model table1 &
rosrun gazebo_ros spawn_model -file $PACKAGE_PATH/models/table2/model.sdf -sdf -model table &
sleep 1
rosrun gazebo_ros spawn_model -file $PACKAGE_PATH/models/object/model.sdf -sdf -model object &
rosrun gazebo_ros spawn_model -file $PACKAGE_PATH/models/hole/model.sdf -sdf -model hole &

# Wait for all background processes to finish
wait
