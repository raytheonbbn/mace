#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


numRovers=$1
echo Launching $numRovers Lightweight ROS stacks...

DEFAULT_ROS_PORT=11311

pushd $HOME/rover/src/rover_ros/rover_launcher/bin

# IFOs (these will eventually get heavyweight stacks)
echo "Starting lightweight stack"
baseOrdinal=0
. run_multiple_airsim.sh "Rover" $baseOrdinal $numRovers "lightweight_stack"

#let baseOrdinal=$(($baseOrdinal+$numRovers))
#. run_multiple_airsim.sh "Solo" $baseOrdinal $numSolos "lightweight_stack"
#
#let baseOrdinal=$(($baseOrdinal+$numSolos))
#. run_multiple_airsim.sh "SoloDownFacing" $baseOrdinal $numDownFacingSolos "lightweight_stack"

popd
