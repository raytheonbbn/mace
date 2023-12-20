#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#! /bin/bash

PLATFORM_TYPE=$1
BASE_ORDINAL=$2
NUM_PLATFORMS=$3
MODE=$4

echo "Launching airsim"

for i in $(seq 0 $(($NUM_PLATFORMS-1))); do
	./start_airsim.sh $PLATFORM_TYPE $BASE_ORDINAL $i $MODE &
	
done

