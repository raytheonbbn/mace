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

#rosrun gps_driver node.py

#MAVROS_ARG=$(ls /dev/serial/* | grep ^usb.*[Cube][fmuv].*f00$)

#FCU_URL=/dev/serial/by-id/${MAVROS_ARG}

#sudo chmod 777 $FCU_URL

roslaunch rover_launcher mavros.launch fcu_url:="tcp://0.0.0.0:5761" #fcu_url:="${FCU_URL}:921600"
