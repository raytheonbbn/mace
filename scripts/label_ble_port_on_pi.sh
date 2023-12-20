#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash

sudo echo 'ACTION=="add", ATTRS{idVendor}=="2458", ATTRS{idProduct}=="0001", SYMLINK+="ble_tag"' > /etc/udev/rules.d/10-mace.rules