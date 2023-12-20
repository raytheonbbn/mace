#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash
PID_FILE="./payload_process_pids.txt"

pushd ../../src/hardware/
PIDS=$(cat ${PID_FILE})
echo "Killing Payload Manager - ${PIDS}"
kill ${PIDS}
popd

exit
