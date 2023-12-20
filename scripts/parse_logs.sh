#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash
OUTPUT_DIR="scenario-analytics-output"
CUR_DIR=$(pwd)
LOG_DIR=$(ls docker/logs | sort -V | tail -n 1)
LOG_DIR="${CUR_DIR}/docker/logs/${LOG_DIR}"
pushd ../src/mission_analysis
echo "Generating event timeline."
python3 generate_event_timeline.py --output_file_dir $OUTPUT_DIR --target_state_file ${LOG_DIR}/target_wf_output_state.csv --payload_state ${LOG_DIR}/payload_output_state.csv --payload_config_file ${LOG_DIR}/payload_output_config.csv --target_config_file ${LOG_DIR}/target_output_configs.csv
echo "Generating event map."
python3 generate_event_map.py --output_file_dir $OUTPUT_DIR --target_state_file ${LOG_DIR}/target_wf_output_state.csv --payload_state ${LOG_DIR}/payload_output_state.csv
echo "Generating movement map."
python3 generate_movement_map.py --output_file_dir $OUTPUT_DIR --target_state_file ${LOG_DIR}/target_wf_output_state.csv --payload_state ${LOG_DIR}/payload_output_state.csv
echo "Generating mission statistics."
python3 generate_mission_statistics.py --output_file_dir $OUTPUT_DIR --target_state_file ${LOG_DIR}/target_wf_output_state.csv --payload_state ${LOG_DIR}/payload_output_state.csv
popd
rm -r $OUTPUT_DIR
mv "../src/mission_analysis/$OUTPUT_DIR" .
