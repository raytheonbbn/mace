#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


if [[ -z "${ROS_ROVER_HOME_DIR}" ]]; then
  ROVER_HOME="${HOME}"
else
  ROVER_HOME="${ROS_ROVER_HOME_DIR}"
fi

ROVER_LAUNCHER_BIN=$ROVER_HOME/rover/src/rover_ros/rover_launcher/bin/
ROVER_LOG=$ROVER_HOME/rover/logs/

cd $ROVER_HOME/rover/logs/ && rm *.log

echo 'Initializing rover simulator:'
echo '    - roscore'
screen -d -m -S roscore  -L -Logfile $ROVER_LOG/roscore.log $ROVER_LAUNCHER_BIN/start_roscore.sh

sleep 10
echo '    - gazebo'

screen -d -m -S gazebo  -L -Logfile $ROVER_LOG/gazebo.log  $ROVER_LAUNCHER_BIN/start_sim_gazebo.sh

sleep 5
echo '    - avoidance'

screen -d -m -S avoidance  -L -Logfile $ROVER_LOG/avoidance.log  $ROVER_LAUNCHER_BIN/start_avoidance_sitl.sh

sleep 5
echo '    - april tag detections'

screen -d -m -S april_tags  -L -Logfile $ROVER_LOG/april_tags.log  $ROVER_LAUNCHER_BIN/start_tag_detection_gazebo.sh

sleep 2
echo '    - gps driver'

screen -d -m -S gps_driver  -L -Logfile $ROVER_LOG/gps_driver.log  $ROVER_LAUNCHER_BIN/start_gps_driver_gazebo.sh

sleep 2
echo '    - kalman filter'

screen -d -m -S ekf  -L -Logfile $ROVER_LOG/ekf.log  $ROVER_LAUNCHER_BIN/start_ekf.sh

sleep 2
echo '    - utils'

screen -d -m -S utils -L -Logfile $ROVER_LOG/utils.log $ROVER_LAUNCHER_BIN/start_utilities.sh

sleep 2
echo '    - navigation'

screen -d -m -S navigation_global  -L -Logfile $ROVER_LOG/navigation_global.log  $ROVER_LAUNCHER_BIN/start_nav_global.sh

sleep 2
echo '    - state observer'

screen -d -m -S state_obs  -L -Logfile $ROVER_LOG/state_obs.log  $ROVER_LAUNCHER_BIN/start_state_observer_gazebo.sh

sleep 5
echo '    - rosbridge'

screen -d -m -S rosbridge  -L -Logfile $ROVER_LOG/rosbridge.log  $ROVER_LAUNCHER_BIN/start_rosbridge.sh

sleep 2
echo '    - rviz'

screen -d -m -S rviz  -L -Logfile $ROVER_LOG/rviz.log  $ROVER_LAUNCHER_BIN/start_rviz.sh

