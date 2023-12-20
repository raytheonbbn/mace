#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#TODO: parse numRovers from localDevice.properties to determine ports to forward.
source ../code/stomp/localDevice.properties

ROVER_PORTS=""
NEXT_BRIDGE_PORT=9091
NEXT_MASTER_PORT=11312
for i in $(seq 1 $numRovers); do
  ROVER_PORTS="${ROVER_PORTS}-p ${NEXT_BRIDGE_PORT}:${NEXT_BRIDGE_PORT} "
  ROVER_PORTS="${ROVER_PORTS}-p ${NEXT_MASTER_PORT}:${NEXT_MASTER_PORT} "
  ((NEXT_BRIDGE_PORT++))
  ((NEXT_MASTER_PORT++))
done

echo "Launching with ${numSolos} quadcopters, ${numRovers} rovers, and ${numTargets} targets."
docker run --rm -p 1883:1883 ${ROVER_PORTS} --name mace-docker -it mace ./scripts/docker/start_all.sh $numSolos $numRovers $numTargets
