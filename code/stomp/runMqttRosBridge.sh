#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#! /bin/bash

if [[ -z "${ROS_ROVER_HOME_DIR}" ]]; then
  ROVER_HOME="${HOME}"
else
  ROVER_HOME="${ROS_ROVER_HOME_DIR}"
fi

echo "Starting the MQTT ROS bridge on port $mqttRosBridgePort"

[ -f "/opt/ros/melodic/setup.sh" ] && source "/opt/ros/melodic/setup.sh"
[ -f "$ROVER_HOME/rover/devel_isolated/setup.sh" ] && source "$ROVER_HOME/rover/devel_isolated/setup.sh"
[ -f "/etc/default/ros" ] && source "/etc/default/ros"

export ROS_MASTER_URI=http://localhost:$mqttRosBridgePort
# export ROS_MASTER_URI=http://172.19.0.2:$mqttRosBridgePort
# export ROS_IP=172.19.0.2
# export ROS_HOSTNAME=172.19.0.2
roslaunch mqtt_to_ros bridge.launch &

