#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#! /bin/bash

if [[ -z "${ROS_ROVER_HOME_DIR}" ]]; then
  ROVER_HOME="${HOME}"
else
  ROVER_HOME="${ROS_ROVER_HOME_DIR}"
fi

[ -f "/opt/ros/melodic/setup.sh" ] && source "/opt/ros/melodic/setup.sh"
[ -f "$ROVER_HOME/rover/devel_isolated/setup.sh" ] && source "$ROVER_HOME/rover/devel_isolated/setup.sh"
[ -f "/etc/default/ros" ] && source "/etc/default/ros"

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$ROVER_HOME/rover/src/rover_ros/rover_gazebo/models/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$ROVER_HOME/rover/src/rover_ros/rover_gazebo/worlds/

roslaunch rover_gazebo rover_gazebo.launch
