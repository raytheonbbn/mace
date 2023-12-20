#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash
MANAGER_HOST="127.0.0.1"
TARGET_HOST="127.0.0.1"
PID_FILE="./payload_process_pids.txt"
source localDevice.properties


help()
{
   # Display Help
   echo "Purpose: Start a number of payloads"
   echo
   echo "Syntax: start_target [-h|v|] space separated list of payloads"
   echo "Options:"
   echo "h      Print this help message"
   echo "v      Enable verbose mode in which the payload will print all logging messages"
   echo
}


setup()
{
    # Navigate to the correct directory
    pushd ../../src/hardware/
}


run_in_background()
{
    echo "[INFO] Starting a new payload daemon for ${numSolos} Quadcopters and ${numRovers} Rovers."
    echo "" > ${PID_FILE}

   # Start the target as a background process 
   nohup python3 SimPayloadManager.py --num_quad_payloads=${numSolos} --num_rover_payloads=${numRovers} >/dev/null 2>&1 &
   echo "Starting SimPayloadManager on pid $!"
   echo "$!" >> ${PID_FILE}
}


run_with_logs()
{
    echo "[INFO] Starting a new payload process with logging enabled"
    echo "" > ${PID_FILE}

   # Start the target as a background process 
   nohup python3 SimPayloadManager.py --num_quad_payloads=${numSolos} --num_rover_payloads=${numRovers} --log >/dev/null 2>&1 &
   echo "Starting SimPayloadManager on pid $!"
   echo "$!" >> ${PID_FILE}
}


while getopts ":hvd" option; do
    case $option in
    h)
        help
        exit
        ;;
    v)
        shift
        setup
        run_with_logs "$@"
        exit
        ;;
    \?) # Invalid option
        echo "[ERROR] Invalid flag provided"
        help
        exit
        ;;
    esac
done

# Run the setup method
setup

# Start the target as a background process
run_in_background "$@"

exit
