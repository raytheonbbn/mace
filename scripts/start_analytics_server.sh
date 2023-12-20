#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash
PID_FILE="./analytics_server_pid.txt"
pushd ../src/analytics_server
nohup java -cp build/libs/analytics_server-0.1.jar:build/dependencies/* com.bbn.mace.server.Server >/dev/null 2>&1 &
PID=$!
echo "${PID}" > ${PID_FILE}
echo "Starting Analytics server on PID ${PID}"
