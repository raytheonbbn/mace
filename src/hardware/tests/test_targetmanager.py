#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import unittest
from mock import Mock
from uuid import getnode as get_mac_address
import paho.mqtt.client as mqtt
import time
import json
import sys
sys.path.append('..')
from target.TargetManager import TargetManager


class TestTargetManager(unittest.TestCase):
    # Create a new mock object
    mock = Mock()

    # Configure the test target client data
    test_host = 'test.mosquitto.org'
    target_identifier = 'TARGET'
    configuration_topic = '/target_test/target_configs/'


    def set_up_test_client(self):
        """
        Create a test client to use for target MQTT client testing
        """
        self.test_client = mqtt.Client("Test Client")
        self.test_client.connect(self.test_host)

        return


    def test_change_target_type_to_mass(self):
        """
        Test whether the manager is able to successfully convert the target type to a mass configuration
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET', self.configuration_topic)

        # New configuration type of the target
        new_config = 'MASS'

        # Change the config type
        manager._change_target_type(new_config)

        # Ensure that the new target was changed correctly
        self.assertEqual(manager.target.get_target_type(), new_config)

        return


    def test_change_target_type_to_periodic(self):
        """
        Test whether the manager is able to successfully convert the target type to a periodic configuration
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET', self.configuration_topic)

        # New configuration type of the target
        new_config = 'PERI'

        # Change the config type
        manager._change_target_type(new_config)

        # Ensure that the new target was changed correctly
        self.assertEqual(manager.target.get_target_type(), new_config)

        return
        

    def test_change_target_type_to_linked(self):
        """
        Test whether the manager is able to successfully convert the target type to a linked configuration
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET'), self.configuration_topic)

        # New configuration type of the target
        new_config = 'LINK'

        # Change the config type
        manager._change_target_type(new_config)

        # Ensure that the new target was changed correctly
        self.assertEqual(manager.target.get_target_type(), new_config)

        return
        

    def test_change_target_type_to_idle(self):
        """
        Test whether the manager is able to successfully convert the target type to an idle configuration
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET'), self.configuration_topic)

        # New configuration type of the target to be mass
        new_config = 'MASS'

        # Change the config type
        manager._change_target_type(new_config)

        # New configuration type of the target to be idle
        new_config = 'IDLE'

        # Change the config type
        manager._change_target_type(new_config)

        # Ensure that the new target was changed correctly
        self.assertEqual(manager.target.get_target_type(), new_config)

        return
        

    def test_change_target_type_to_invalid(self):
        """
        Test whether the manager successfully raises an exception when an invalid configuration type is passed as a target type
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET'), self.configuration_topic)

        # New configuration type of the target
        new_config = 'invalid'

        # Ensure that the invalid change was handled properly
        self.assertRaises(AssertionError, manager._change_target_type, new_config)

        return
        

    def test_update_target_configurations_mass_config(self):
        """
        Test whether the manager is able to successfully update the mass target configurations
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET'), self.configuration_topic)

        # Config to change the target to
        new_config = 'MASS'

        # Change the target type
        manager._change_target_type(new_config)

        new_detection_range = 1.5
        new_required_payloads = 5

        # Configurations to set
        configs = {
            'uid': manager.target.get_uid(),
            'type': new_config,
            'modify_capture_state': False,
            'captured': False,
            'detection_range': new_detection_range,
            'required_payloads': new_required_payloads
        }

        # Update the configurations
        manager._update_target_configurations(configs)

        # Ensure the configs were updated properly
        self.assertEqual(manager.target._get_acceptable_detection_range(), new_detection_range)
        self.assertEqual(manager.target._get_required_payloads_for_capture(), new_required_payloads)

        return
        

    def test_update_target_configurations_periodic_config(self):
        """
        Test whether the manager is able to successfully update the periodic target configurations
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET'), self.configuration_topic)

        # Config to change the target to
        new_config = 'PERI'

        # Change the target type
        manager._change_target_type(new_config)

        new_detection_range = 1.5
        new_required_payloads = 5
        new_countdown = 60

        # Configurations to set
        configs = {
            'uid': manager.target.get_uid(),
            'type': new_config,
            'modify_capture_state': False,
            'captured': False,
            'detection_range': new_detection_range,
            'required_payloads': new_required_payloads,
            'capture_countdown': new_countdown
        }

        # Update the configurations
        manager._update_target_configurations(configs)

        # Ensure the configs were updated properly
        self.assertEqual(manager.target._get_acceptable_detection_range(), new_detection_range)
        self.assertEqual(manager.target._get_required_payloads_for_capture(), new_required_payloads)
        self.assertEqual(manager.target._get_countdown_required_for_capture(), new_countdown)

        return
        

    def test_update_target_configurations_linked_config(self):
        """
        Test whether the manager is able to successfully update the linked target configurations
        """
        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET'), self.configuration_topic)

        # Config to change the target to
        new_config = 'LINK'

        # Change the target type
        manager._change_target_type(new_config)

        new_detection_range = 1.5
        new_network_capture_state = True
        new_targets = ['target1', 'target2', 'target3']

        # Configurations to set
        configs = {
            'uid': manager.target.get_uid(),
            'type': new_config,
            'modify_capture_state': False,
            'captured': False,
            'detection_range': new_detection_range,
            'network_captured': new_network_capture_state,
            'network_targets': new_targets
        }

        # Update the configurations
        manager._update_target_configurations(configs)

        # Ensure the configs were updated properly
        self.assertEqual(manager.target._get_acceptable_detection_range(), new_detection_range)
        self.assertEqual(manager.target._get_network_capture_state(), new_network_capture_state)
        self.assertEqual(manager.target._get_network_targets(), new_targets)

        return
        

    def test_read_target_configurations(self):
        """
        Test whether the manager is able to successfully handle configurations published by a server and
        update the target according to those configurations
        """
        # Create a new test client
        self.set_up_test_client()

        manager = TargetManager(self.mock, self.mock, 'MANAGER', 'TARGET'), self.configuration_topic)

        # Connect the target and manager to the MQTT broker
        manager.connect_clients(self.test_host)

        # subscribe the manager to the configurations topic
        manager.subscribe_to_configurations(self.configuration_topic)

        # Start the manager and target
        manager.start_clients()

        # Start the test client
        self.test_client.loop_start()

        target_config = 'MASS'
        detection_range = 1.5
        required_payloads = 4

        # Create new set of configurations
        test_configurations = {
            'uid': manager.target.get_uid(),
            'type': target_config,
            'modify_capture_state': False,
            'captured': False,
            'detection_range': detection_range,
            'required_payloads': required_payloads
        }

        # Convert the configs to JSON
        json_configurations = json.dumps(test_configurations)

        # Publish the target configs to the broker
        for _ in range(100):
            self.test_client.publish(self.configuration_topic, json_configurations)

        # Delay to ensure that the system is able to publish the state
        time.sleep(2)

        # Stop and disconnect the manager
        manager.stop_clients()
        manager.disconnect_clients()

        # Stop and disconnect the test client
        self.test_client.loop_stop()
        self.test_client.disconnect()

        # Ensure the configs were updated properly
        self.assertEqual(manager.target.get_target_type(), target_config)
        self.assertEqual(manager.target._get_acceptable_detection_range(), detection_range)
        self.assertEqual(manager.target._get_required_payloads_for_capture(), required_payloads)

        return
        
