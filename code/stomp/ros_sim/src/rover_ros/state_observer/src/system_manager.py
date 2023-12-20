#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/usr/bin/python

import os
import re
import time
import rospy
import subprocess

class SystemManager:
    """
    System manager class performs system-level tasks 
    """

    def __init__(self, is_airsim, platform_suffix, home_dir = '$HOME'):
        """
        Class constructor 
        """

        self.platform_suffix = platform_suffix
        self.is_airsim = is_airsim
        self.home_dir = home_dir


    def start_package(self, package, script):
        """
        Function starts nodes with a package name and run by a startup script

        Parameters
        ----------
        package : `string`
            name of the package to be started, which is the same as the screen session generated
        script : `string`
            name of the script utilized to sping up the session
        """

        log_name =  self.home_dir + '/rover/logs/' + package + '.log '
        cmd = 'screen -d -m -S ' + package + ' -L -Logfile ' + log_name + ' ' + self.home_dir + '/rover/src/rover_ros/rover_launcher/bin/' + script 
        os.system(cmd)


    def stop_package(self, package):
        """
        Kills any running screen sessions with name matching `package`

        Parameters
        ----------
        package : `string`
            name of the session to be stopped
        """

        cmd = 'screen -X -S '+ package + ' kill'
        os.system(cmd)


    def restart_package(self, package, script):
        """
        Restarts node with package name and executed by startup script

        Parameters
        ----------
        package : `string`
            name of the package to be started, which is the same as the screen session generated
        script : `string`
            name of the script utilized to sping up the session
        """

        self.stop_package(package)
        time.sleep(0.5)
        self.start_package(package, script)
        time.sleep(0.5)


    def check_package(self, pkg):
        """
        Verifies if a package named `pkg` is currently running

        Parameters
        ----------
        pkg : `string`
            name of the package to be verified
        """

        if (subprocess.check_output('screen -ls', shell = True)).find(pkg) == -1:    
            return True

        else:
            return False 

        
    def get_active_packages(self):
        """
        Retrieves a list of all active packages in the system
        """

        # first we clear out any stale sessions
        cmd = 'screen -wipe'        
        os.system(cmd)

        # now, we try to look for the active packages in the system
        active_pkgs = []

        # looking them up by name
        for st in re.findall(r'\d+\.\w+', subprocess.check_output('screen -ls', shell = True)):
            to_append = re.findall(r'\w+', st)[1]
            active_pkgs.append(to_append)

        # now, if we are running with AS backed, we need to check for the platform suffix
        if self.is_airsim:            
            filtered_pkgs = []

            for pack in active_pkgs:
                if pack.endswith(self.platform_suffix):
                    filtered_pkgs.append(pack)

            return filtered_pkgs

        else:            
            return active_pkgs

    def wait_for_package(self, node):
        """
        Polls for method using defined methods and waits until node is up or waiting times out

        Parameters
        ----------
        node : `dictionary`
            fields:
                timeout: longest wait until call a node faulty
                method: method utilized to check nodes health
                topic: name of topic to listen to if method is `topic`
                service: name of service to check if method is service

        """

        init_time = time.time()
        delta_time = 0.0

        # now we just need to check all methods and determine if node is healthy or node
        while delta_time < 2.0 * node['timeout']:        
            delta_time = time.time() - init_time

            if node['method'] == 'shell':
                if (get_topics_from_shell()).find(node['topic']) != -1:
                    break

            elif node['method'] == 'topic':
                try:
                    rospy.wait_for_message(node['topic'], node['topic_type'], node['timeout'])
                    break

                except:
                    pass
                # if (str(rospy.get_published_topics()).find(node['topic']) != -1):
                #     break

            elif node['method'] == 'node':
                if self.check_package(node['name']):
                    break

                else:
                    pass

            elif node['method'] == 'websocket':
                try:                     
                    port = + 9091 + PLATFORM_ORDINAL
                    connection_name = "ws://localhost:" + str(port)
                    ws = create_connection(connection_name)                    
                    ws.send("ping_websocket")
                    break

                except Exception as inst:
                    time.sleep(0.2)
                    pass        

        if delta_time >= node['timeout']:
            print('      ... ' + str(node['name']) + ' failed at time ' + str(time.time()))

        else:
            print('      ... ' + str(node['name']) + ' succeeded at time ' + str(time.time()))



    def start_stack(self, pkg_list):
        """
        Starts a list of nodes 

        Parameters
        ----------
        pkg_list : list [`node`]
        """

        for pkg in pkg_list:
            self.start_package(pkg['name'], pkg['script'])
            self.wait_for_package(pkg)


    def restart_stack(self, pkg_list):
        """
        Restarts a list of nodes 

        Parameters
        ----------
        pkg_list : list [`node`]
        """

        if pkg_list != []: 
            for pkg in pkg_list:
                self.restart_package(pkg['name'], pkg['script'])
                self.wait_for_package(pkg)


    def stop_stack(self, pkg_list):
        """
        Stops a list of nodes 

        Parameters
        ----------
        pkg_list : list [`node`]
        """

        if pkg_list != []:        
            for pkg in pkg_list:
                self.stop_package(pkg['name'])
                time.sleep(1) 

    def check_stack(self, pkg_list):
        """
        Checks health for a list of nodes

        Parameters
        ----------
        pkg_list : list [`node`]
        """

        failed = []
        if pkg_list != []:         
            for pkg in pkg_list:
                if self.check_package(pkg['name']):
                    failed.append(pkg['name'])

        return failed


 
