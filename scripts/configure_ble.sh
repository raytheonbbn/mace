#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


echo "[INFO] Configuring pi to use consistent port for ble dongle"

# This sets the ble dongle to a specfic port called 'ble_tag'
echo 'ACTION=="add", ATTRS{idVendor}=="2458", ATTRS{idProduct}=="0001", SYMLINK+="ble_tag"' | sudo tee /etc/udev/rules.d/10-mace.rules