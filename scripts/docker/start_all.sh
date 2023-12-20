#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash
echo "Starting ntp server"
ntpd
echo "Have args: $@"
screen -dmS mosquitto scripts/docker/start_mosquitto.sh
echo "Starting rover ros stacks."
scripts/docker/start_sim_ros.sh $2
echo "Starting payloads."
scripts/docker/start_payloads.sh $1 $2
echo "Starting targets."
scripts/docker/start_targets.sh $3
echo "Starting the MQTT to ROS bridge."
scripts/docker/start_mqtt_ros_bridge.sh
echo "Starting analytics web server."
scripts/docker/start_analytics_webserver.sh
echo "Starting analytics server."
scripts/docker/keep_analytics_server.sh
while true; do
  sleep 2;
done
