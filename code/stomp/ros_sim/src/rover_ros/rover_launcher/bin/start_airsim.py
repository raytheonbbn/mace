#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/usr/bin/env python
try:
    import os
    import re
    import sys
    import time
    import yaml
    import rospy
    import traceback
    import subprocess
    import collections
    from websocket import create_connection

    from sensor_msgs.msg import *
    from std_msgs.msg import *
    from gps_common.msg import *
    from nav_msgs.msg import *
    from geometry_msgs.msg import *
    from state_observer.msg import *
    from rosgraph_msgs.msg import *

    # -+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+- #
    #                                                                       #
    #                               ARGS                                    #
    #                                                                       #
    # -+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+- #

    print('Starting ROS stack with ' + str(len(sys.argv)) + ' arguments:')
    for i in range(len(sys.argv)):
        print('Argument ' + str(i) + ' = ' + sys.argv[i])

    PLATFORM_SUFFIX = sys.argv[1]
    ABSOLUTE_PLATFORM_ORDINAL = int(sys.argv[2])
    CCAST_ROS_PORT = int(sys.argv[3])
    CCAST_BRIDGE_PORT = int(sys.argv[4])
    MODE = sys.argv[5]

    # -+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+- #
    #                                                                       #
    #                               METHODS                                 #
    #                                                                       #
    # -+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+- #

    def get_process_list():

        try:

            list = subprocess.check_output('screen -ls', shell = True)

        except:

            list = ''

        return list


    def get_topics_from_shell():

        try:

            list = subprocess.check_output('rostopic list', shell = True)

        except:

            list = ''

        return list


    def get_active_packages():

        cmd = 'screen -wipe'

        os.system(cmd)

        active_pkgs = []

        for st in re.findall(r'\d+\.\w+', get_process_list()):

            to_append = re.findall(r'\w+', st)[1]

            active_pkgs.append(to_append)

        filtered_pkgs = []

        for pack in active_pkgs:

            if pack.endswith(platform_suffix):

                filtered_pkgs.append(pack)

        return filtered_pkgs


    def stop_package(package):

        cmd = 'screen -X -S '+ package + ' kill'

        os.system(cmd)


    def check_package(pkg):

        if (get_process_list()).find(pkg) != -1:

            return True

        else:

            return False


    def start_stack(pkg_list):

        for pkg in pkg_list:

            start_package(pkg['name'], pkg['script'])

            wait_for_package(pkg)


    def wait_for_package(node):

        print("    - " + str(node['name']))

        init_time = time.time()

        delta_time = 0.0

        while delta_time < node['timeout']:
        
            delta_time = time.time() - init_time

            if node['method'] == 'shell':

                if (get_topics_from_shell()).find(node['topic']) != -1:

                    break


            elif node['method'] == 'topic':

                if (str(rospy.get_published_topics()).find(node['topic']) != -1):

                    break


            elif node['method'] == 'node':

                if check_package(node['name']):

                    break

                else:

                    pass


            elif node['method'] == 'websocket':

                try: 
                    
                    port = CCAST_BRIDGE_PORT

                    connection_name = "ws://localhost:" + str(port)

                    ws = create_connection(connection_name)
                    
                    ws.send("ping_websocket")

                    break

                except Exception as inst:

                    # Yield to other processes a bit, to give them a chance 
                    time.sleep(0.2)
                    pass        

        if delta_time >= node['timeout']:

            print('      ... ' + str(node['name']) + ' failed at time ' + str(time.time() - session_start_time))

        else:

            print('      ... ' + str(node['name']) + ' succeeded at time ' + str(time.time() - session_start_time))


    def start_package(package, script):

        wait_time = 0.0

        if (package.find('roscore') != -1):

            suffix = ' ' + str(CCAST_ROS_PORT)

            wait_time = 2.0

        elif (package.find('rosbridge') != -1):

            suffix = ' ' + str(CCAST_BRIDGE_PORT)

        elif (package.find('state_obs') != -1):

            suffix = ' ' + PLATFORM_SUFFIX + ' ' + str(ABSOLUTE_PLATFORM_ORDINAL)

        else:

            suffix = ''

        log_name =  '$HOME/rover/logs/' + package + '.log '

        cmd = 'screen -d -m -S ' + package + ' -L -Logfile ' + log_name + ' $HOME/rover/src/rover_ros/rover_launcher/bin/' + script + suffix

        os.system(cmd)

        time.sleep(wait_time)


    def adjust_strings_for_platform_suffix(string_array, suffix):

        result = []

        for name in string_array:

            result.append(name + suffix)

        return result


    def adjust_keys_for_platform_suffix(dictionary, suffix):

        for k,v in dictionary.items():

            new_key = k + suffix

            del dictionary[k]

            dictionary[new_key] = v


    # -+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+- #
    #                                                                       #
    #                               SCRIPT                                  #
    #                                                                       #
    # -+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+- #

    platform_suffix = "_" + PLATFORM_SUFFIX
    session_start_time = time.time()

    cmd = 'cd $HOME/rover/logs/ && rm *.log'

    os.system(cmd)

    with open(sys.path[0] + '/../../state_observer/config/nodes_lw.yaml', 'r') as stream:

        NODES = yaml.load(stream)

    adjust_keys_for_platform_suffix(NODES, platform_suffix)

    for k,v in NODES.items():
                
        v['name'] = k

        v['topic_type'] = eval(v['topic_type'])

    # Lightweight for Solos, Heavyweight for rovers
    print("LIGHTWEIGHT mode; mode = " + MODE)
    common_node_names = ['roscore', 'rosbridge']

    common_node_names = adjust_strings_for_platform_suffix(common_node_names, platform_suffix)

    unordered_nodes = {k:v for k,v in NODES.items() if k in common_node_names}

    common_nodes = unordered_nodes.values()

    nodes = collections.OrderedDict()

    for key in common_node_names:

        nodes[key] = unordered_nodes[key]

    print('Starting nodes:')
    start_stack(nodes.values())
except Exception as e:
    print(e)
