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
from target.LinkedTarget import LinkedTarget


class TestLinkedTarget(unittest.TestCase):
    # Create a new mock object
    mock = Mock()
    
    target_identifier = 'TARGET'
    

    def test_update_valid_capture_state(self):
        """
        Test whether the linked target correctly updates its capture state to captured
        """
        # Create a new target
        target = LinkedTarget(self.mock, self.mock, self.target_identifier)

        # Set the payloads in range to be larger than the required number of payloads
        payloads_in_range = 5

        # Update the target state
        target._set_payloads_in_range(payloads_in_range)

        # Update the target capture state
        target._update_individual_capture_state()

        self.assertTrue(target._get_individual_capture_state())

        return
        

    def test_update_invalid_capture_state(self):
        """
        Test whether the linked target correctly updates its capture state to not captured
        """
        # Create a new target
        target = LinkedTarget(self.mock, self.mock, self.target_identifier)

        # Set the payloads in range to be less than the required number of payloads
        payloads_in_range = 0

        # Update the target state
        target._set_payloads_in_range(payloads_in_range)

        # Update the target capture state
        target._update_individual_capture_state()

        self.assertFalse(target._get_individual_capture_state())

        return
        

if __name__ == 'main':
    unittest.main()