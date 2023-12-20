#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


# This script will start init_payload/init_target (depending on arg) for an arbitrary username on pi

USER=$(ls /home)
DEVICE=$1 

/home/${USER}/mace/scripts/init_${DEVICE}.sh

