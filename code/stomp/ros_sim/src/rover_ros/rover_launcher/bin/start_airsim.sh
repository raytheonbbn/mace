#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash

if [[ -z "${ROS_ROVER_HOME_DIR}" ]]; then
  ROVER_HOME="${HOME}"
else
  ROVER_HOME="${ROS_ROVER_HOME_DIR}"
fi

[ -f "/opt/ros/melodic/setup.sh" ] && source "/opt/ros/melodic/setup.sh"
[ -f "$ROVER_HOME/rover/devel_isolated/setup.sh" ] && source "$ROVER_HOME/rover/devel_isolated/setup.sh"
[ -f "/etc/default/ros" ] && source "/etc/default/ros"

PLATFORM_TYPE=$1
BASE_ORDINAL=$2
PLATFORM_ORDINAL=$3
MODE=$4

ABSOLUTE_PLATFORM_ORDINAL=$(echo "$BASE_ORDINAL + $PLATFORM_ORDINAL" | bc)

echo "PLATFORM_TYPE: $1 ... BASE_ORDINAL: $2 ... PLATFORM_ORDINAL: $3 ... MODE = $MODE"

PLATFORM_NAME=${PLATFORM_TYPE}$PLATFORM_ORDINAL
BASE_ROS_PORT=11312
BASE_BRIDGE_PORT=9091
CCAST_ROS_PORT=$(echo "$BASE_ROS_PORT + $BASE_ORDINAL + $PLATFORM_ORDINAL" | bc)
CCAST_BRIDGE_PORT=$(echo "$BASE_BRIDGE_PORT + $BASE_ORDINAL + $PLATFORM_ORDINAL" | bc)

export ROS_MASTER_URI=http://localhost:$CCAST_ROS_PORT

echo "Launching start_airsim.py with: PLATFORM_NAME = $PLATFORM_NAME, CCAST_ROS_PORT = $CCAST_ROS_PORT, RosBridge port = $CCAST_BRIDGE_PORT, mode = heavyweight_stack"
echo "ROS_MASTER_URI = $ROS_MASTER_URI"

python2 start_airsim.py $PLATFORM_NAME $ABSOLUTE_PLATFORM_ORDINAL $CCAST_ROS_PORT $CCAST_BRIDGE_PORT $MODE
