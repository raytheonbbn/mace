#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash

pushd code/stomp/
source ccastConfig.properties
source localDevice.properties
popd
source /opt/ros/melodic/setup.bash
pushd code/stomp/
echo "Starting MQTT to ROS birdge ..."
source runMqttRosBridge.sh