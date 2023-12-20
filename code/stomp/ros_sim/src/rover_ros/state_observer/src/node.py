#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/usr/bin/python

import time
import numpy as np
from datetime import datetime
from math import pi, cos, sin
from observer import Observer

import tf
import rospy
import rosnode

import actionlib
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from state_observer.srv import SetMode
from sensor_msgs.msg import BatteryState
from state_observer.msg import Diagnostics
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Quaternion, Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction

class Node:

    def __init__(self):
        """
        Node class constructor
        """

        rospy.init_node('STATE')

        # parameters
        self.is_sitl = rospy.get_param('~is_sitl', False)        
        self.is_airsim = rospy.get_param('~is_airsim', False)          
        self.home_dir = rospy.get_param('~home_dir', '$HOME')
        self.platform_ordinal = rospy.get_param('~platform_ordinal', 0)
        self.platform_name = rospy.get_param('~platform_name', 'rover_0')
        self.rate = rospy.get_param('~rate', 1.0)

        # node state setup
        self.observer = Observer(self.is_sitl, self.is_airsim, self.platform_name, self.platform_ordinal, self.home_dir)

        # ROS pub-sub setup
        self.pub_diag = rospy.Publisher('/system_diagnostics', Diagnostics, queue_size = 1)
        self.srv_cmd_state = rospy.Service('set_mode', SetMode, self.set_mode_callback)

        rospy.wait_for_service('/MOVE/clear_costmaps')        
        self.clear_costmaps_srv = rospy.ServiceProxy('/MOVE/clear_costmaps', Empty)

        

        rospy.loginfo('Starting state observer...')

        # variable initialization
        self.diag = Diagnostics()

        print("Platform name: " + self.platform_name + ", absolute platform ordinal: " + str(self.platform_ordinal))


    def run(self):
        """
        Main loop function executed by the node checking for system health and publishing diagnostics
        """

        rospy.loginfo('Starting state broadcast')

        while not rospy.is_shutdown():
            state, status = self.observer.get_system_info()
            self.diag.state = state
            self.diag.status = status
            self.pub_diag.publish(self.diag)
            rospy.sleep(1.0/self.rate)


    def set_mode_callback(self, msg):
        """
        Callback function for state machine mode switch requests

        Parameters
        ----------
        msg : `SetMode`
            fields:
                cmd: command to execute: `set` or `reset`
                target_mode: mode to switch into, supported modes: `global`, `transition`, `slam` or `amcl`
                entryId: unique id of entry point used
                dataDir: directory used to look for data
                regionId: id of current region
                regionDir: path to region directory folder
                initialPose: initial map pose utilized in AMCL mode
        """

        reply = 'set_mode did not receive response from system...'

        self.client_goal = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        if msg.cmd == 'set':

            self.client_goal.cancel_goal()
            self.client_goal.cancel_all_goals()
            self.clear_costmaps_srv(EmptyRequest())

            reply = self.observer.set_system_mode(msg.target_mode, msg.entryId, msg.dataDir, msg.regionDir, msg.regionId, msg.initialPose)            

            # time.sleep(10.0)

        elif msg.cmd == 'reset':
            self.observer.system_reset()
            reply = "system reset"

        return reply


if __name__ == '__main__':

    try:
        node = Node()
        node.run()

    except rospy.ROSInterruptException:
        pass

    rospy.loginfo('Exiting')
