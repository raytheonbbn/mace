#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash

echo "[INFO] Configuring GPSD to enable communication with the BU-353S4 GPS Module"

# Kill all existing gpsd processes
sudo killall gpsd

# Configure the baudrate to be 4800
stty -F /dev/ttyUSB0 4800

# Stop the gpsd socket
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket

# Start gpsd
sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock