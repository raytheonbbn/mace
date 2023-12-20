#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash

help()
{
   # Display Help
   echo "Purpose: Start a target"
   echo
   echo "Syntax: start_target [-h|g|d|v|r uid] server_host_ip server_port"
   echo "Options:"
   echo "h      Print this help message"
   echo "g      Configure GPSD on startup (note that this should only be done outside of a docker image)"
   echo "d      Run process in the background"
   echo "v      Enable verbose mode in which the target will print all logging messages"
   echo "r      Rename the UID of the target"
   echo
}


setup()
{
    # Navigate to the correct directory
    cd ../src/hardware/
}


run_in_background()
{
    echo "[INFO] Starting a new target daemon"

    # Perform setup
    setup

    if [[ $3 != "" ]]
    then
        nohup python3 execute_target.py --manager_host $1 --target_host $1 --manager_port $2 --target_port $2 --manager_uid "MANAGER-$3" --target_uid $3 &
    else
        nohup python3 execute_target.py --manager_host $1 --target_host $1 --manager_port $2 --target_port $2 &
    fi
}


default()
{
    echo "[INFO] Starting a new target process"

    # Perform setup
    setup

    if [[ $3 != "" ]]
    then
        python3 execute_target.py --manager_host $1 --target_host $1 --manager_port $2 --target_port $2 --manager_uid "MANAGER-$3" --target_uid $3
    else
        python3 execute_target.py --manager_host $1 --target_host $1 --manager_port $2 --target_port $2
    fi
}


run_with_logs()
{
    echo "[INFO] Starting a new target process with logging enabled"
     
    # Perform setup
    setup

    if [[ $3 != "" ]]
    then
        python3 execute_target.py --manager_host $1 --target_host $1 --manager_port $2 --target_port $2 --manager_uid "MANAGER-$3" --target_uid $3 --log
    else
        python3 execute_target.py --manager_host $1 --target_host $1 --manager_port $2 --target_port $2 --log
    fi
}

# Flags
RUN_IN_VERBOSE=false
RUN_AS_DAEMON=false
CONFIGURE_GPSD=false
NEW_UID=""

while getopts ":hvdgr:" opt; do
    case "${opt}" in
        h)
            help
            exit
            ;;
        v)
            RUN_IN_VERBOSE=true
            ;;
        d)
            RUN_AS_DAEMON=true
            ;;
        g)
            CONFIGURE_GPSD=true
            ;;
        r)
            NEW_UID=${OPTARG}
            ;;
        \?)
            echo "[ERROR] Invalid flag provided"
            exit
            ;;
    esac
done

# Ensure that GPSD is properly configured
if [ $CONFIGURE_GPSD == true ]
then
    ./configure_gpsd.sh
fi

# Ensure BLE has correct port set
./configure_ble.sh

ARG1=${@:$OPTIND:1}
ARG2=${@:$OPTIND+1:1}

# Determine the method to run the process
if [ $RUN_AS_DAEMON == true ]
then
    run_in_background $ARG1 $ARG2 $NEW_UID
elif [ $RUN_IN_VERBOSE == true ]
then
    run_with_logs $ARG1 $ARG2 $NEW_UID
else
    default $ARG1 $ARG2 $NEW_UID
fi

exit