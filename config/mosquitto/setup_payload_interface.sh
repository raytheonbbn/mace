#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash

# here we add a alias for the ethernet interface that agents can connect to for MQTT
# use the link local address space because not DHCP server
# the static IP selected is reserved for future use and will never conflict with random IP of agents
# NOTE: run this before starting the agent MQTT broker on a hardware device
sudo ifconfig eth0:0 169.254.0.1