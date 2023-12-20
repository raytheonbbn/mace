#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

#!/bin/bash

MAVROS_ARG=$(ls /dev/serial/* | grep ^usb.*[Cube][fmuv].*f00$)

FCU_URL=/dev/serial/by-id/${MAVROS_ARG}

sudo chmod 777 $FCU_URL

/usr/bin/python /usr/local/bin/mavproxy.py --daemon --master=$FCU_URL,921600 --out=tcpin:0.0.0.0:5760 --out=tcpin:0.0.0.0:5761 --out=tcpin:0.0.0.0:5762 --logfile=/home/nvidia/mav.tlog

