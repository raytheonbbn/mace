#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash

pushd ~/mace/config/mosquitto
sudo ./setup_payload_interface.sh
mosquitto -d -c payload_mosquitto.conf
popd
cd ~/mace/scripts
./start_payload.sh -g 192.168.1.100 1883 169.254.0.1 1884