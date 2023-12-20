#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



run#!/bin/bash



help ()
{
   # Display Help
   echo "Purpose: Start MACE analytics server and STOMP"
   echo
   echo "Syntax: ./run_mace -h|d|w|z"
   echo "Options:"
   echo "h      Print this help message"
   echo "d      Set number of sim objects to default (4 Quads, 2 Rovers, 6 Targets)"
   echo "z      Set number of sim objects to zero "
   echo "w      Run STOMP in white force mode"
   echo
}

set_num_sim_objects()
{
    # Get number of sim payloads/targets from user input
    echo How many sim QUADS:
    read NUM_QUADS
    echo How many sim ROVERS:
    read NUM_ROVERS
    echo How many sim TARGETS:
    read NUM_TARGETS

    # If user input is blank, set number to 0 
    if [ -z "$NUM_QUADS" ];
    then
        NUM_QUADS=0
    fi
    if [ -z "$NUM_ROVERS" ];
    then
        NUM_ROVERS=0
    fi
    if [ -z "$NUM_TARGETS" ];
    then
        NUM_TARGETS=0
    fi

    # Get line number of sim objects in docker-compose.yaml
    LINE_NUMBER=$(grep -n 'command:' ../docker-compose.yaml | cut -d ':' -f1)
    # Replace 
    sed -i ${LINE_NUMBER}'s/.*/    command: ["'${NUM_QUADS}'", "'${NUM_ROVERS}'", "'${NUM_TARGETS}'"]/' ../docker-compose.yaml
}

set_default_num_sim_objects()
{
    echo Default Number of Sim Target/Payloads Set
    # Get line number of sim objects in docker-compose.yaml
    LINE_NUMBER=$(grep -n 'command:' ../docker-compose.yaml | cut -d ':' -f1)
    # Replace 
    sed -i ${LINE_NUMBER}'s/.*/    command: ["4", "2", "6"]/' ../docker-compose.yaml 
}

set_zero_num_sim_objects()
{
    echo Zero Sim Target/Payloads Set
    # Get line number of sim objects in docker-compose.yaml
    LINE_NUMBER=$(grep -n 'command:' ../docker-compose.yaml | cut -d ':' -f1)
    # Replace 
    sed -i ${LINE_NUMBER}'s/.*/    command: ["0", "0", "0"]/' ../docker-compose.yaml  
}

# TODO: fix this to use a single line
set_blue_force_type()
{
    echo Blue Force Mode Set
    # Get line numbers of force
    FORCE_TYPE_LINE_NUMBER=$(grep -n 'forceType=' ../code/stomp/localDevice.properties | cut -d ':' -f1)

    # Change force type to blue 
    sed -i ${FORCE_TYPE_LINE_NUMBER}'s/.*/forceType=blue/' ../code/stomp/localDevice.properties
    }

# TODO: fix this to use a single line
set_white_force_type()
{
    echo White Force Mode Set
    # Get line numbers of force
    FORCE_TYPE_LINE_NUMBER=$(grep -n 'forceType=' ../code/stomp/localDevice.properties | cut -d ':' -f1)

    # Change force type to white     
    sed -i ${FORCE_TYPE_LINE_NUMBER}'s/.*/forceType=white/' ../code/stomp/localDevice.properties
}


start_mace_and_stomp()
{
    echo Starting MACE
    # Start up Analytics Server
    cd ..
    gnome-terminal --title='Analytics Server' --tab -- bash -c 'docker-compose up'

    # Start STOMP
    cd code/stomp
    gnome-terminal --title='STOMP' --tab -- bash -c './start.sh' 
}

# Flags
SIM_OBJECTS_SET=false
WHITE_FORCE_MODE=false
while getopts "hdzw" opt; do
    case "${opt}" in
        h)
            help
            exit
            ;;
        d)
            set_default_num_sim_objects
            SIM_OBJECTS_SET=true
            ;;
        
        z)
            set_zero_num_sim_objects
            SIM_OBJECTS_SET=true  
            ;;
        w)
            set_white_force_type
            WHITE_FORCE_MODE=true
            ;;
        \?)
            echo "[ERROR] Invalid flag provided"
            exit
            ;;
        *)
        ;;
    esac
done

# If not set in white force mode, default to blue
if ! $WHITE_FORCE_MODE;
then
    set_blue_force_type
fi

# If number of sim quads/rovers/targets not set, then prompt user to get amounts
if ! $SIM_OBJECTS_SET;
then
    set_num_sim_objects
fi

start_mace_and_stomp



