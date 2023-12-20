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
from target.PeriodicTarget import PeriodicTarget


class TestPeriodicTarget(unittest.TestCase):
        # Create a new mock object
    mock = Mock()

    target_identifier = 'TARGET'

    
    def test_update_entrance_timestamp_payload_entrance(self):
        """
        Test setting the entrance timestamp when a payload enters the detection range
        """
        # Create a new target
        target = PeriodicTarget(self.mock, self.mock, self.target_identifier)

        # Ensure that the start timestamp is 0
        self.assertEqual(target._get_entrance_timestamp(), 0)

        # Set one payload that just entered the detection range
        target._set_last_payload_count(0)
        target._set_payloads_in_range(1)

        # Update the entrance timestamp
        target._update_entrance_timestamp()

        # Ensure that the timestamp was set
        self.assertTrue(target._get_entrance_timestamp() != 0)

        return
        

    def test_update_entrance_timestamp_payload_exit(self):
        """
        Test setting the entrance timestamp after all payloads leave the detection range
        """
        # Create a new target
        target = PeriodicTarget(self.mock, self.mock, self.target_identifier)

        # Assert that the timestamp is starting at 0
        self.assertEqual(target._get_entrance_timestamp(), 0)

        # One payload just entered the detection range
        target._set_last_payload_count(0)
        target._set_payloads_in_range(1)

        # Start the timer
        target._update_entrance_timestamp()

        # Ensure that the timestamp was set
        self.assertTrue(target._get_entrance_timestamp() != 0)

        # The payload just left the detection range
        target._set_last_payload_count(1)
        target._set_payloads_in_range(0)

        # Update the timer
        target._update_entrance_timestamp()

        # Assert that the timestamp was reset
        self.assertEqual(target._get_entrance_timestamp(), 0)

        return
        

    def test_update_entrance_timestamp_existing_payloads(self):
        """
        Test setting the entrance timestamp when existing payloads are in range and have been in range: assert that the timestamp is unchanged
        """
        # Create a new target
        target = PeriodicTarget(self.mock, self.mock, self.target_identifier)

        # Assert that the timestamp is starting at 0
        self.assertEqual(target._get_entrance_timestamp(), 0)

        # One payload just entered the detection range
        target._set_last_payload_count(0)
        target._set_payloads_in_range(1)

        # Start the timer
        target._update_entrance_timestamp()

        # Ensure that the timestamp was set
        self.assertTrue(target._get_entrance_timestamp() != 0)

        last_timestamp = target._get_entrance_timestamp()

        # No payloads left the detection range
        target._set_last_payload_count(1)
        target._set_payloads_in_range(2)

        # Update the timer
        target._update_entrance_timestamp()

        # Assert that the timestamp was NOT reset
        self.assertEqual(target._get_entrance_timestamp(), last_timestamp)

        return
        

    def test_update_valid_capture_state(self):
        """
        Test whether the periodic target correctly updates its capture state to captured
        """
        # Create a new target
        target = PeriodicTarget(self.mock, self.mock, self.target_identifier)

        # Set the number of required payloads for capture to be 3 payloads
        target._set_required_payloads_for_capture(3)

        # Set the timeframe that the payloads must arrive within to be 15 seconds
        target._set_countdown_required_for_capture(15)

        # A payload just entered the detection range
        target._set_last_payload_count(0)
        target._set_payloads_in_range(1)

        # Get the entrance timestamp
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        self.assertFalse(target._get_capture_state())

        # Wait 2 seconds
        time.sleep(2)

        # Update the payloads in range
        target._set_last_payload_count(1)
        target._set_payloads_in_range(3)

        # Update the entrance timestamp again as would be done in practice
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        # Assert that the target is captured
        self.assertTrue(target._get_capture_state())

        return
        

    def test_update_invalid_capture_state(self):
        """
        Test whether the periodic target correctly updates its capture state to not captured
        """
       # Create a new target
        target = PeriodicTarget(self.mock, self.mock, self.target_identifier)

        # Set the number of required payloads for capture to be 3 payloads
        target._set_required_payloads_for_capture(3)

        # Set the timeframe that the payloads must arrive within to be 2s seconds
        target._set_countdown_required_for_capture(2)

        # A payload just entered the detection range
        target._set_last_payload_count(0)
        target._set_payloads_in_range(1)

        # Get the entrance timestamp
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        self.assertFalse(target._get_capture_state())

        # Wait 3 seconds
        time.sleep(3)

        # Update the payloads in range
        target._set_last_payload_count(1)
        target._set_payloads_in_range(3)

        # Update the entrance timestamp as would be done in practice
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        # Assert that the target is not captured
        self.assertFalse(target._get_capture_state())

        return
        

    def test_update_capture_state_after_previous_fail(self):
        """
        Test whether the target effectively handles a previous failed capture and a new success
        """
        # Create a new target
        target = PeriodicTarget(self.mock, self.mock, self.target_identifier)

        # Set the number of required payloads for capture to be 3 payloads
        target._set_required_payloads_for_capture(3)

        # Set the timeframe that the payloads must arrive within to be 2 seconds
        target._set_countdown_required_for_capture(2)

        # A payload just entered the detection range
        target._set_last_payload_count(0)
        target._set_payloads_in_range(1)

        # Get the entrance timestamp
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        # Ensure that the target is not captured yet
        self.assertFalse(target._get_capture_state())

        # Wait 3 seconds
        time.sleep(3)

        # Update the payloads in range
        target._set_last_payload_count(1)
        target._set_payloads_in_range(3)

        # Update the entrance timestamp as would be done in practice
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        # Assert that the payloads failed to capture the target in time
        self.assertFalse(target._get_capture_state())

        # The payloads just left the detection range
        target._set_last_payload_count(3)
        target._set_payloads_in_range(0)

        # Get the entrance timestamp
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        # Ensure that the target still isn't captured
        self.assertFalse(target._get_capture_state())

        # Wait 3 seconds again
        time.sleep(3)

        # Update the payloads in range
        target._set_last_payload_count(0)
        target._set_payloads_in_range(3)

        # Update the entrance timestamp as would be done in practice
        target._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        target._update_capture_duration()

        # Update the capture state
        target._update_capture_state()

        # Assert that the target is captured
        self.assertTrue(target._get_capture_state())

        return
        

if __name__ == 'main':
    unittest.main()