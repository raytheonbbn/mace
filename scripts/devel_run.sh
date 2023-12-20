#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash

# Development script to compile and run code locally

# build steps
pushd ../code/stomp
echo "Building STOMP..."
./gradlew jar shadowjar
popd

pushd ../src/analytics_server
echo "Building MACE"
gradle build -x test
popd

# run steps

# start mosquitto if not running
pushd ../config/mosquitto
mosquitto -d -c sim_mosquitto.conf
popd

# start sim (server + python + STOMP)
pushd ../code/stomp
./startSim.sh
popd

# cleanup broker
pkill mosquitto