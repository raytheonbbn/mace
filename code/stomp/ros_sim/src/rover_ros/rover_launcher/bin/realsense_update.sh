#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


read SERIALNUM <<< $(rs-fw-update -l | awk '/serial number/ { print $8 }' | awk -F',' '{ print $1 }')
rs-fw-update -f ~/rover/src/rover_ros/rover_launcher/config/Signed_Image_UVC_5_12_12_100.bin -s $SERIALNUM

#read SERIALNUM <<< $(rs-fw-update -l | awk '/serial number/ { print $8 }' | awk -F',' '{ print $0 }')
#read SERIALNUM_FINAL <<< $awk
#rs-fw-update -f ~/rover/src/rover_ros/rover_launcher/config/Signed_Image_UVC_5_12_3_0.bin -s $SERIALNUM
#read SERIALNUM <<< $(rs-fw-update -l | awk -F',' '/serial number/ { print $0 }' | awk -F':' '{ print $1 }')
#read SERIALNUM <<< $ awk -F':' {SERIALNUM}

#| awk -F':' '{ print $1 }'


