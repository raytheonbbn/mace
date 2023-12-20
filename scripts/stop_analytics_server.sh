#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash
PID_FILE="../src/analytics_server/analytics_server_pid.txt"

PID=$(cat ${PID_FILE})
echo "Killing Analytics Server - ${PID}"
kill ${PID}

exit
