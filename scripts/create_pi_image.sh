#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash
sdCardDevice=/dev/mmcblk0
outputName="$1.img"
user=$USER
group='users'

echo "Copying..."

sudo dd bs=4M if=$sdCardDevice of=$outputName status=progress
sudo chown $USER:$group $outputName
echo "Zipping..."
gzip $outputName