#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


if [[ -z "${ROS_ROVER_HOME_DIR}" ]]; then
  ROVER_HOME="${HOME}"
else
  ROVER_HOME="${ROS_ROVER_HOME_DIR}"
fi

ROVER_LAUNCHER_BIN=$ROVER_HOME/rover/src/rover_ros/rover_launcher/bin/

ROVER_LOG=$ROVER_HOME/rover/logs
rm $ROVER_LOG/*.log

$ROVER_LAUNCHER_BIN/start_rosclean.sh

sleep 1

DEPTH_SN=$(rs-fw-update -l | awk '/D435, serial number/ { print $8 }' | awk -F',' '{ print $1 }')
TRACKING_SN=$(rs-fw-update -l | awk '/T265, serial number/ { print $8 }' | awk -F',' '{ print $1 }')

echo $DEPTH_SN >> depth.log
echo $TRACKING_SN >> tracking.log

echo 'Starting nodes:'
echo '    - roscore'

screen -d -m -S roscore -L -Logfile $ROVER_LOG/roscore.log $ROVER_LAUNCHER_BIN/start_roscore.sh

sleep 10
echo '    - robot description'

screen -d -m -S robot_description -L -Logfile $ROVER_LOG/robot_description.log $ROVER_LAUNCHER_BIN/start_robot_description.sh

sleep 5
echo '    - avoidance'

screen -d -m -S avoidance -L -Logfile $ROVER_LOG/avoidance.log $ROVER_LAUNCHER_BIN/start_avoidance.sh "$TRACKING_SN" "$DEPTH_SN"

sleep 2
echo '    - april tag detections'

screen -d -m -S april_tags -L -Logfile $ROVER_LOG/april_tags.log $ROVER_LAUNCHER_BIN/start_tag_detection.sh

sleep 2
echo '    - imu'

screen -d -m -S imu -L -Logfile $ROVER_LOG/imu.log $ROVER_LAUNCHER_BIN/start_imu.sh

sleep 2
echo '    - drive system'

screen -d -m -S drive -L -Logfile $ROVER_LOG/drive.log $ROVER_LAUNCHER_BIN/start_drive.sh

sleep 2
echo '    - mavproxy'

screen -d -m -S mavproxy -L -Logfile $ROVER_LOG/mavproxy.log $ROVER_LAUNCHER_BIN/start_mavproxy.sh

sleep 2
echo '    - gps driver'

screen -d -m -S gps_driver -L -Logfile $ROVER_LOG/gps_driver.log $ROVER_LAUNCHER_BIN/start_gps_driver.sh

sleep 2
echo '    - gps converter'

screen -d -m -S gps_conv -L -Logfile $ROVER_LOG/gps_conv.log $ROVER_LAUNCHER_BIN/start_gps_converter.sh

sleep 2
echo '    - lidar'

screen -d -m -S lidar -L -Logfile $ROVER_LOG/lidar.log $ROVER_LAUNCHER_BIN/start_lidar.sh

sleep 2
echo '    - kalman filter'

screen -d -m -S ekf -L -Logfile $ROVER_LOG/ekf.log $ROVER_LAUNCHER_BIN/start_ekf.sh

sleep 2
echo '    - utils'

screen -d -m -S utils -L -Logfile $ROVER_LOG/utils.log $ROVER_LAUNCHER_BIN/start_utilities.sh

sleep 2
echo '    - navigation'

screen -d -m -S navigation_global -L -Logfile $ROVER_LOG/navigation_global.log $ROVER_LAUNCHER_BIN/start_nav_global.sh

sleep 2
echo '    - rosbridge'

screen -d -m -S rosbridge -L -Logfile $ROVER_LOG/rosbridge.log $ROVER_LAUNCHER_BIN/start_rosbridge.sh

sleep 5
echo '    - state observer'

screen -d -m -S state_obs -L -Logfile $ROVER_LOG/state_obs.log $ROVER_LAUNCHER_BIN/start_state_observer.sh

