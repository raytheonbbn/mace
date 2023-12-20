#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


killall -9 roscore

sleep 0.5

killall -9 rosmaster

sleep 0.5

kill -9 $(pgrep -f 'gazebo')

sleep 0.5

kill -9 $(pgrep -f 'gzserver')

sleep 0.5

screen -ls | grep Detached | cut -d. -f1 | awk '{print $1}' | xargs kill

sleep 0.5

screen -wipe

