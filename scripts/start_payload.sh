#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


help ()
{
   # Display Help
   echo "Purpose: Start a payload"
   echo
   echo "Syntax: start_payload [-h|g|d|v|r uid] server_host_ip server_port agent_host_ip agent_port"
   echo "Options:"
   echo "h      Print this help message"
   echo "g      Configure GPSD on startup (note that this should only be done outside of a docker image)"
   echo "d      Run payload process in the background"
   echo "v      Run payload in verbose mode in which the payload will print all logging messages"
   echo "r      Rename the UID of the payload"
   echo
}


setup ()
{
    # Navigate to the correct directory
    cd ../src/hardware/
}


run_in_background ()
{
    echo "[INFO] Starting a new payload daemon"

    # Perform setup
    setup

    if [[ $5 != "" ]]
    then
        nohup python3 execute_payload.py --server_host $1 --server_port $2 --agent_host $3 --agent_port $4 --payload_uid $5 &
    else
        nohup python3 execute_payload.py --server_host $1 --server_port $2 --agent_host $3 --agent_port $4 &
    fi
}


default ()
{
    echo "[INFO] Starting a new payload process"
    
    # Perform setup
    setup

    if [[ $5 != "" ]]
    then
        python3 execute_payload.py --server_host $1 --server_port $2 --agent_host $3 --agent_port $4 --payload_uid $5
    else
        python3 execute_payload.py --server_host $1 --server_port $2 --agent_host $3 --agent_port $4
    fi
}


run_with_logs ()
{
    echo "[INFO] Starting a new payload process with logging enabled"

    # Perform setup
    setup
    
    if [[ $5 != "" ]]
    then
        python3 execute_payload.py --server_host $1 --server_port $2 --agent_host $3 --agent_port $4 --payload_uid $5 --log
    else
        python3 execute_payload.py --server_host $1 --server_port $2 --agent_host $3 --agent_port $4 --log
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
ARG3=${@:$OPTIND+2:1}
ARG4=${@:$OPTIND+3:1}

# Determine the method to run the process
if [[ $RUN_AS_DAEMON == true ]]
then
    run_in_background $ARG1 $ARG2 $ARG3 $ARG4 $NEW_UID
elif [[ $RUN_IN_VERBOSE == true ]]
then
    run_with_logs $ARG1 $ARG2 $ARG3 $ARG4 $NEW_UID
else
    default $ARG1 $ARG2 $ARG3 $ARG4 $NEW_UID
fi

exit