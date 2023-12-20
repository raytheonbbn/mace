#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash
MANAGER_HOST="127.0.0.1"
TARGET_HOST="127.0.0.1"
PID_FILE="./target_process_pids.txt"
numTargets=$1

help()
{
   # Display Help
   echo "Purpose: Start a number of targets target"
   echo
   echo "Syntax: start_target [-h|v|] number_of_targets"
   echo "Options:"
   echo "h      Print this help message"
   echo "v      Enable verbose mode in which the target will print all logging messages"
   echo
}


setup()
{
    # Navigate to the correct directory
    pushd src/hardware/
}


run_in_background()
{
   echo "[INFO] Starting a new target daemon for ${numTargets} targets." 
   echo "" > ${PID_FILE}

   # Start the target as a background process
   python3 SimTargetManager.py --num_targets=${numTargets} >/dev/null & #2>&1 &
   echo "Starting SimTargetManager on pid $!"
   echo "$!" >> ${PID_FILE}
}


run_with_logs()
{
   echo "[INFO] Starting a new target daemon for ${numTargets} targets." 
   echo "" > ${PID_FILE}

   # Start the target as a background process
   python3 SimTargetManager.py --num_targets=${numTargets} --log >/dev/null & #2>&1 &
   echo "Starting SimTargetManager on pid $!"
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
        run_with_logs $1
        exit
        ;;
    \?) # Invalid option
        echo "[ERROR] Invalid flag provided"
        exit
        ;;
    esac
done

# Run the setup method
setup

# Start the target as a background process
run_in_background $1

exit
