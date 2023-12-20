#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/usr/bin/python

import os
import sys
import yaml
import time
import shutil
import os.path
from os import path
from websocket import create_connection
from system_manager import SystemManager

import rospy
import rosnode
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float64
from gps_common.msg import GPSFix
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu, Image
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from state_observer.srv import SetMode
from state_observer.msg import Diagnostics
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Quaternion, Twist, PoseStamped, PoseWithCovarianceStamped

import dynamic_reconfigure.client

class Observer:

    def adjust_keys_for_platform_suffix(self, dictionary):

        for k,v in dictionary.items():
            new_key = k + '_' + self.platform_name
            # print('Old key: ' + k + ', key with suffix: ' + new_key)
            del dictionary[k]
            dictionary[new_key] = v


    def adjust_strings_for_platform_suffix(self, string_array):

        result = []
        for name in string_array:
            result.append(name + '_' + self.platform_name)
        return result

    def check_node_defs(self, requested_node_names, runtime_nodes, node_set_name):

        for requested_node_name in requested_node_names:

            got_it = False
            for node in runtime_nodes:
                if node['name'] == requested_node_name:
                    got_it = True

            if not got_it:
                print("ERROR: undefined " + node_set_name + " node '" + requested_node_name + "'")

    def __init__(self, is_sitl, is_airsim, platform_name, platform_ordinal, home_dir):

        self.initialpose = PoseWithCovarianceStamped()
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)
        self.pub_control = rospy.Publisher('manage_controller', String, queue_size = 1)

        self.mode_switching = False

        if is_airsim:
            nodes_filename = 'nodes_sitl_airsim.yaml'

        elif is_sitl:
            nodes_filename = 'nodes_sitl.yaml'

        else:
            nodes_filename = 'nodes.yaml'

        print('nodes_filename: ' + nodes_filename)

        with open(sys.path[0] + '/../config/' + nodes_filename, 'r') as stream:
            self.NODES = yaml.load(stream)

        # Add any key suffix (for multi-platform configurations)
        self.is_airsim = is_airsim
        self.platform_name = platform_name

        if self.is_airsim:
            self.platform_ordinal = platform_ordinal
            self.platform_suffix = "_" + str(self.platform_ordinal)
            self.adjust_keys_for_platform_suffix(self.NODES)

        else:
            self.platform_suffix = ""

        self.manager = SystemManager(self.is_airsim, self.platform_name, home_dir)

        for k,v in self.NODES.items():
            # Let all nodes know their names - which are their keys in the self.NODES map
            v['name'] = k
            v['topic_type'] = eval(v['topic_type'])

        # Setting dynamic parameters for the dwa planner
        # self.global_dwa_params = {
        #     'acc_lim_x': 0.4,
        #     'max_vel_x': 0.4,
        #     'min_vel_x': -0.15,
        #     'max_vel_trans': 0.4,
        #     'min_vel_trans': -0.15,

        #     'max_vel_theta': 1.0,
        #     'min_vel_theta': 0.0,
        #     'acc_lim_theta': 0.7,

        #     'sim_time': 4.0,
        #     'vx_samples': 3,
        #     'vth_samples': 8,

        #     'xy_goal_tolerance': 1.25,
        #     'yaw_goal_tolerance': 0.2,

        #     'path_distance_bias': 10.0,
        #     'goal_distance_bias': 50.0,
        #     'occdist_scale' : 0.1

        #     }

        # self.global_locost_params = {
        #     'width': 5.0,
        #     'height': 5.0,
        #     'resolution': 0.1,
        #     'footprint_padding': 0.1
        # }

        # self.global_glocost_params = {
        #     'width': 12.0,
        #     'height': 12.0,
        #     'resolution': 0.2,
        #     'footprint_padding': 0.1
        # }

        # self.global_glocost_inflation_params = {
        #     'enabled': 'true'
        # }

        # self.transition_dwa_params = {
        #     'acc_lim_x': 0.35,
        #     'max_vel_x': 0.3,
        #     'min_vel_x': -0.1,
        #     'max_vel_trans': 0.35,
        #     'min_vel_trans': -0.1,

        #     'max_vel_theta': 0.75,
        #     'min_vel_theta': 0.0,
        #     'acc_lim_theta': 2.5,

        #     'sim_time': 4.0,
        #     'vx_samples': 3,
        #     'vth_samples': 8,

        #     'xy_goal_tolerance': 0.5,
        #     'yaw_goal_tolerance': 0.1,

        #     'path_distance_bias': 2.0,
        #     'goal_distance_bias': 20.0,
        #     'occdist_scale' : 0.03

        #     }

        # self.transition_locost_params = {
        #     'width': 8.0,
        #     'height': 8.0,
        #     'resolution': 0.015,
        #     'footprint_padding': 0.0
        # }

        # self.transition_glocost_params = {
        #     'width': 20.0,
        #     'height': 20.0,
        #     'resolution': 0.025,
        #     'footprint_padding': 0.0
        # }

        # self.transition_glocost_inflation_params = {
        #     'enabled': 'false'
        # }

        # self.slam_dwa_params = {
        #     'acc_lim_x': 0.25,
        #     'max_vel_x': 0.25,
        #     'min_vel_x': -0.1,
        #     'max_vel_trans': 0.25,
        #     'min_vel_trans': -0.1,

        #     'max_vel_theta': 0.5,
        #     'min_vel_theta': -0.5,
        #     'acc_lim_theta': 0.7,

        #     'sim_time': 3.5,
        #     'vx_samples': 3,
        #     'vth_samples': 8,

        #     'xy_goal_tolerance': 0.5,
        #     'yaw_goal_tolerance': 0.15,

        #     'path_distance_bias': 32.0,
        #     'goal_distance_bias': 20.0,
        #     'occdist_scale' : 0.03

        #     }

        # self.slam_locost_params = {
        #     'width': 5.0,
        #     'height': 5.0,
        #     'resolution': 0.025,
        #     'footprint_padding': 0.0
        # }

        # self.slam_glocost_params = {
        #     'width': 10.0,
        #     'height': 10.0,
        #     'resolution': 0.05,
        #     'footprint_padding': 0.0
        # }

        # self.slam_glocost_inflation_params = {
        #     'enabled': 'true'
        # }

        # self.amcl_dwa_params = {
        #     'acc_lim_x': 0.25,
        #     'max_vel_x': 0.35,
        #     'min_vel_x': -0.1,
        #     'max_vel_trans': 0.35,
        #     'min_vel_trans': -0.1,

        #     'max_vel_theta': 0.35,
        #     'min_vel_theta': -0.35,
        #     'acc_lim_theta': 0.7,

        #     'sim_time': 3.5,
        #     'vx_samples': 3,
        #     'vth_samples': 8,

        #     'xy_goal_tolerance': 0.5,
        #     'yaw_goal_tolerance': 0.15,

        #     'path_distance_bias': 32.0,
        #     'goal_distance_bias': 20.0,
        #     'occdist_scale' : 0.03

        #     }

        # self.amcl_locost_params = {
        #     'width': 5.,
        #     'height': 5.,
        #     'resolution': 0.04,
        #     'footprint_padding': 0.05
        # }

        # self.amcl_glocost_params = {
        #     'width': 10.,
        #     'height': 10.,
        #     'resolution': 0.05,
        #     'footprint_padding': 0.05
        # }

        # self.amcl_glocost_inflation_params = {
        #     'enabled': 'true'
        # }

        if is_sitl == False:
            common_node_names = ['roscore', 'robot_description', 'april_tags', 'imu', 'state_obs', 'mavproxy', 'gps_driver', 'utils', 'rosbridge', 'avoidance', 'drive', 'lidar']
            global_node_names = ['control_global', 'navigation_global', 'ekf']
            transition_node_names = ['control_transition', 'navigation_transition', 'ekf', 'door_entry']
            slam_node_names = ['map', 'explore', 'navigation', 'ekf_indoors']
            amcl_node_names = ['amcl', 'navigation', 'ekf_indoors']

        else:

            if is_airsim:
                common_node_names = ['roscore', 'state_obs', 'rosbridge', 'april_tags', 'sensor_transforms', 'gps_driver_airsim', 'utils', 'avoidance']
                common_node_names = self.adjust_strings_for_platform_suffix(common_node_names)

                global_node_names = ['control_global', 'navigation_global', 'ekf']
                global_node_names = self.adjust_strings_for_platform_suffix(global_node_names)

                print('Common node names: ')
                print(common_node_names)

                transition_node_names = ['control_transition', 'navigation_transition', 'ekf', 'door_entry']
                transition_node_names = self.adjust_strings_for_platform_suffix(transition_node_names)

                slam_node_names = ['map', 'explore', 'navigation', 'ekf_indoors']
                slam_node_names = self.adjust_strings_for_platform_suffix(slam_node_names)

                amcl_node_names = ['amcl', 'navigation', 'ekf_indoors']
                amcl_node_names = self.adjust_strings_for_platform_suffix(amcl_node_names)

            else:
                common_node_names = ['roscore', 'video', 'state_obs', 'april_tags', 'rosbridge', 'gazebo', 'utils', 'gps_driver',  'avoidance', 'rviz']
                global_node_names = ['control_global', 'navigation_global', 'ekf']
                transition_node_names = ['control_transition', 'navigation_transition', 'ekf', 'door_entry']
                slam_node_names = ['map', 'explore', 'navigation', 'ekf_indoors']
                amcl_node_names = ['amcl', 'navigation', 'ekf_indoors']

        self.common_nodes = {k:v for k,v in self.NODES.items() if k in common_node_names}.values()
        self.global_nodes = {k:v for k,v in self.NODES.items() if k in global_node_names}.values()
        self.transition_nodes = {k:v for k,v in self.NODES.items() if k in transition_node_names}.values()
        self.slam_nodes = {k:v for k,v in self.NODES.items() if k in slam_node_names}.values()
        self.amcl_nodes = {k:v for k,v in self.NODES.items() if k in amcl_node_names}.values()

        # Error checking: ensure that we don't ask for any undefined nodes
        self.check_node_defs(common_node_names, self.common_nodes, 'Common')
        self.check_node_defs(global_node_names, self.global_nodes, 'Global')
        self.check_node_defs(transition_node_names, self.transition_nodes, 'Transition')
        self.check_node_defs(slam_node_names, self.slam_nodes, 'Slam')
        self.check_node_defs(amcl_node_names, self.amcl_nodes, 'AMCL')

        self.system_states = ['idle', 'broadcasting', 'fault']
        self.system_modes = ['', 'slam', 'amcl','global', 'transition']
        self.system_nodes = {'': [], 'slam': self.slam_nodes, 'amcl': self.amcl_nodes, 'global': self.global_nodes, 'transition': self.transition_nodes}
        # self.system_dwa_params = {'': [], 'slam': self.slam_dwa_params, 'amcl': self.amcl_dwa_params, 'global': self.global_dwa_params, 'transition': self.transition_dwa_params}
        # self.system_locost_params = {'': [], 'slam': self.slam_locost_params, 'amcl': self.amcl_locost_params, 'global': self.global_locost_params, 'transition': self.transition_locost_params}
        # self.system_glocost_params = {'': [], 'slam': self.slam_glocost_params, 'amcl': self.amcl_glocost_params, 'global': self.global_glocost_params, 'transition': self.transition_glocost_params}
        # self.system_glocost_inflation_params = {'': [], 'slam': self.slam_glocost_inflation_params, 'amcl': self.amcl_glocost_inflation_params, 'global': self.global_glocost_inflation_params, 'transition': self.transition_glocost_inflation_params}

        self.current_system_mode = ''
        self.current_system_diagnostics = ''

        self.startup_mode = True
        self.update_system_on = False
        self.set_system_on = False
        self.failed_nodes = []
        self.to_be_healed = []
        self.count = 0
        self.failed_node_count = 0
        self.control_msg = String()

    def update_system_info(self, which_nodes = 'all'):

        self.failed_nodes = []

        if which_nodes == 'all':

            current_nodes = self.common_nodes + self.system_nodes[self.current_system_mode]

        elif which_nodes == 'healed':

            current_nodes = self.to_be_healed

        # checking node health
        for node in current_nodes:

            if node['method'] == 'topic':

                try:

                    rospy.wait_for_message(node['topic'], node['topic_type'], node['timeout'])

                except:

                    self.failed_nodes.append(node['name'])

            elif node['method'] == 'service':

                try:

                    rospy.wait_for_service(node['topic'], timeout=node['timeout'])

                except:

                    self.failed_nodes.append(node['name'])

            elif node['method'] == 'node':

                if self.manager.check_package(node['name']):

                    self.failed_nodes.append(node['name'])

            elif node['method'] == 'websocket':

                try:

                    if self.is_airsim:
                        port = + 9091 + self.platform_ordinal
                        connection_name = "ws://localhost:" + str(port)
                    else:
                        connection_name = "ws://localhost:9090"

                    self.ws = create_connection(connection_name)
                    self.ws.send("ping_websocket")

                except Exception as inst:

                    self.failed_nodes.append(node['name'])
                    print("Adding bridge to failed_nodes due to exception: ", inst)

            elif node['method'] == 'none':
                pass


    def heal_nodes(self):

        self.to_be_healed = {k:v for k,v in self.NODES.items() if k in self.failed_nodes}.values()
        self.manager.restart_stack(self.to_be_healed)


    def get_system_info(self):

        self.update_system_info()

        if self.failed_nodes != []:

            self.failed_node_count += 1

            self.current_system_diagnostics = 'faulty nodes: ' + str(self.failed_nodes)

        else:

            self.failed_node_count = 0

            self.current_system_diagnostics = 'system healthy'

        self.startup_mode = False

        if self.mode_switching:
            self.current_system_diagnostics = 'system switching modes'

        return (self.current_system_mode, self.current_system_diagnostics)


    def set_system_mode(self, new_mode, entryPoint, tempDir, regionDir, regionId, poseServ):

        self.mode_switching = True

        regionDir = regionDir + '/'

        tempDir = tempDir + '/'

        if new_mode == "amcl":

            if path.exists(regionDir + regionId + ".pgm") and \
            path.exists(regionDir + regionId + ".yaml") and \
            path.exists(regionDir + regionId + "_meta.yaml"):

                src_map_file = regionDir + regionId + ".pgm"

                src_map_info_file = regionDir + regionId + ".yaml"

                src_map_meta_file = regionDir + regionId + "_meta.yaml"

            elif path.exists(tempDir + regionId + ".pgm") and \
            path.exists(tempDir + regionId + ".yaml") and \
            path.exists(tempDir + regionId + "_meta.yaml"):

                src_map_file = tempDir + regionId + ".pgm"

                src_map_info_file = tempDir + regionId + ".yaml"

                src_map_meta_file = tempDir + regionId + "_meta.yaml"

            else:

                return "mode change failed: map files not found"

            # standard file location
            map_path = sys.path[0] + '/../../rover_launcher/maps/'

            if not path.exists(map_path):
                os.makedirs(map_path)

            src_map_file_new = map_path + 'map.pgm'
            src_map_info_file_new = map_path + 'map.yaml'
            src_map_meta_file_new = map_path + 'map_meta.yaml'

            shutil.copy(src_map_file, src_map_file_new)

            shutil.copy(src_map_info_file, src_map_info_file_new)

            shutil.copy(src_map_meta_file, src_map_meta_file_new)

        nodes  = []

        all_nodes = (self.common_nodes + self.system_nodes[new_mode])

        for node in all_nodes:

            nodes.append(node['name'])

        cur_nodes = self.manager.get_active_packages()

        to_be_stopped = [x for x in cur_nodes if x not in nodes]

        for stack in to_be_stopped:

            self.manager.stop_package(stack)

        self.current_system_mode = new_mode

        to_be_started_keys = [x for x in nodes if x not in cur_nodes]

        to_be_started = [k for k in all_nodes if k['name'] in to_be_started_keys]

        self.manager.start_stack(to_be_started)

        # if new_mode == "transition":
        #     self.reconf_dwa = dynamic_reconfigure.client.Client('/MOVE/DWAPlannerROS', timeout=30)
        #     self.reconf_locost = dynamic_reconfigure.client.Client('/MOVE/local_costmap', timeout=30)
        #     self.reconf_glocost = dynamic_reconfigure.client.Client('/MOVE/global_costmap', timeout=30)
        #     self.reconf_glocost_inflation = dynamic_reconfigure.client.Client('/MOVE/global_costmap/inflation', timeout=30)

        #     self.reconf_dwa.update_configuration(self.system_dwa_params[new_mode])
        #     self.reconf_locost.update_configuration(self.system_locost_params[new_mode])
        #     self.reconf_glocost.update_configuration(self.system_glocost_params[new_mode])
        #     self.reconf_glocost_inflation.update_configuration(self.system_glocost_inflation_params[new_mode])

        self.mode_switching = False

        return 'mode set to: ' + str(self.current_system_mode)


    def system_reset(self):

        self.count = 0

        self.startup_mode = True

        self.manager.restart_stack(self.common_nodes + self.system_nodes[self.current_system_mode])
