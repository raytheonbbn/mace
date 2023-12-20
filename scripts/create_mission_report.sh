#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

help()
{
   # Display Help
   echo "Purpose: Generate a complete mission report evaluating targets and payloads"
   echo
   echo "Syntax: create_mission_report [-h] full/path/to/target/state/log full/path/to/payload/state/log full/path/to/target/config/log full/path/to/payload/config/log"
   echo "Options:"
   echo "h     Print this help message"
   echo
}

while getopts ":h" option; do
   case $option in
      h)
         help
         exit
         ;;
   esac
done

# Generate the report folder name
folder_name="mission-report-"`date +"%Y-%m-%d-%H-%M-%S"`
 
 # Get the input files to use for evaluation
target_state_log=$1
payload_state_log=$2
target_config_log=$3
payload_config_log=$4


# Navigate to the correct directory
cd ../src/mission_analysis/

# Generate the reports
python3 generate_event_map.py --target_state_file $target_state_log --payload_state_file $payload_state_log --output_file_dir $folder_name
python3 generate_movement_map.py --target_state_file $target_state_log --payload_state_file $payload_state_log --output_file_dir $folder_name
python3 generate_event_timeline.py --target_state_file $target_state_log --payload_state_file $payload_state_log --target_config_file $target_config_log --payload_config_file $payload_config_log --output_file_dir $folder_name
python3 generate_mission_statistics.py --target_state_file $target_state_log --payload_state_file $payload_state_log --output_file_dir $folder_name