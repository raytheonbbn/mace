#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from os import rename
import paho.mqtt.client as mqtt
import json


class LinkedNetwork:
    def __init__(self, uid, detection_range, network):

        # Initialize the class variables
        self.uid = uid
        self.old_uid = uid
        self.network = network
        self.detection_range = detection_range
        self.captured = False
        self.client = None
        self.modify_capture_state = False
        self.delete = False
        self.rename = False


    def connect(self, host, uid):
        """
        Connect the client to the desired broker
        """
        # Create an MQTT Client
        self.client = mqtt.Client(uid)

        # Connect the client to the mqtt broker
        self.client.connect(host)

        return


    def disconnect_client(self):
        """
        Disconnect the client from the broker
        """
        self.client.disconnect()

        return


    def start_client(self):
        """
        Start the MQTT Client
        """
        # Start an mqtt client
        self.client.loop_start() 

        return
        

    def stop_client(self):
        """
        Stop the MQTT client
        """
        self.client.loop_stop()

        return


    def get_uid(self):
        """
        Get the network UID
        """
        return self.uid


    def _set_uid(self, uid):
        """
        Set the UID
        """
        self.uid = uid

        return


    def _get_detection_range(self):
        """
        Get the detection range
        """
        return self.detection_range


    def set_detection_range(self, detection_range):
        """
        Set the detection range
        """
        self.detection_range = detection_range

        return


    def _get_captured(self):
        """
        Get the capture state
        """
        return self.captured


    def set_captured(self, captured):
        """
        Set the capture state
        """
        self.captured = captured

        return


    def get_network(self):
        """
        Get the list of targets in the network
        """
        return self.network


    def add_target_to_network(self, target):
        """
        Add a target to the network
        """
        self.network.append(target)

        return


    def remove_target_from_network(self, target):
        """
        Remove a target from the network
        """
        self.network.remove(target)

        return


    def _get_modify_capture_state(self):
        """
        Get the modify_capture_state flage
        """
        return self.modify_capture_state


    def set_modify_capture_state(self, captured):
        """
        Set the modify_capture_state flag
        """
        self.modify_capture_state = captured

        return


    def _get_delete(self):
        """
        Get the delete flag
        """
        return self.delete


    def _set_delete(self, delete):
        """
        Set the delete flag
        """
        self.delete = delete

        return


    def _get_rename(self):
        """
        Get the rename flag
        """
        return self.rename


    def _set_rename(self, rename):
        """
        Set the rename flag
        """
        self.rename = rename

        return


    def _get_old_uid(self):
        """
        Get the old UID
        """
        return self.old_uid


    def _set_old_uid(self, uid):
        """
        Set the old UID
        """
        self.old_uid = uid

        return


    def print_state(self):
        """
        Helper method used to print the object state
        """
        print(f'UID: {self.get_uid()}')
        print(f'Detection Range: {self._get_detection_range()}')
        print('Network: ')
        for target in self.get_network():
            print('  - ' + target.get_uid())
        print(f'Captured: {self._get_captured()}')

        return


    def get_network_uids(self):
        """
        Get the network as a list of UIDs
        """
        # List to store the UIDs
        network = []

        # Add the UIDs to the list
        for target in self.get_network():
            network.append(target.get_uid())

        return network


    def rename(self, uid, topic):
        """
        Helper method used to rename a network
        """
        self._set_rename(True)
        self._set_old_uid(self.get_uid())
        self._set_uid(uid)
        self.publish_configs(topic)
        self._set_rename(False)

        return


    def delete_network(self, topic):
        """
        Helper method used to delete a network
        """
        self._set_delete(True)
        
        self.publish_configs(topic)

        self._set_delete(False)

        return


    def publish_configs(self, topic):
        """
        Helper method used to publish the configurations for a network
        """
        # Create a dictionary containing the network configs
        configs = {
            'uid': self.get_uid(),
            'type': 'LINK',
            'delete': self._get_delete(),
            'change_uid': self._get_rename(),
            'old_uid': self._get_old_uid(),
            'network': self.get_network_uids(),
            'detection_range': self._get_detection_range(),
            'modify_capture_state': self._get_modify_capture_state(),
            'captured': self._get_captured()
        }

        # Convert the network configs to JSON
        json_target_configs = json.dumps(configs)

        # Publish the configs to the broker
        self.client.publish(topic, json_target_configs)
        
        # Reset the modify capture state flag to be false
        self.set_modify_capture_state(False)

        return