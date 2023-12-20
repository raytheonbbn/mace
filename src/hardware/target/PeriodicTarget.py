#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import json
import time
from .Target import Target
import logging
import sys


class PeriodicTarget(Target):
    def __init__(self, gps, ble, identifier, uid=None, log=False, is_sim=False):
        super().__init__('PERI', gps, ble, identifier, uid, log, is_sim=is_sim)

        self.logger = logging.getLogger(__name__)
        if log:
            logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

        # Initialize mass class variables
        # The acceptable detection range that a payload must be within to connect
        self.acceptable_detection_range = None
        # The last count of payloads in range
        self.last_payload_count = 0
        # The number of payloads that must be in the detection range for capture
        self.required_payloads_for_capture = float('inf')
        # The timeframe that the targets must enter the detection range within for capture
        self.countdown_required_for_capture = None
        # The current capture state of the target
        self.captured = False
        self.suppression = False
        # The tiemstampe that was recorded when a payload first entered the detection range
        self.entrance_timestamp = 0
        # The current duration that the payloads have been in the detection range for
        self.capture_duration = float('inf')

    def _get_acceptable_detection_range(self):
        """
        Getter to retrieve the acceptable detection range for the target
        """
        return self.acceptable_detection_range

    def _set_acceptable_detection_range(self, range):
        """
        Setter to set the acceptable detection range for the target
        """
        self.acceptable_detection_range = range

        return

    def _get_required_payloads_for_capture(self):
        """
        Getter used to retrieve the number of required payloads for capture for the mass target
        """
        return self.required_payloads_for_capture

    def _set_required_payloads_for_capture(self, payloads):
        """
        Setter used to set the required number of payloads for capture
        """
        assert float(payloads).is_integer(
        ), 'The number of payloads that are required to be in range for capture must be an integer'

        self.required_payloads_for_capture = payloads

        return

    def _get_countdown_required_for_capture(self):
        """
        Getter that retrieves the total time that the payloads must be in the area to capture the target
        """
        return self.countdown_required_for_capture

    def _set_countdown_required_for_capture(self, duration):
        """
        Setter that sets total time that the payloads must be in the area to capture the target
        """
        self.countdown_required_for_capture = duration

        return
    
    def _set_suppression(self, suppression):
        """
        Setter to set whether takedown needs to be persisted or not
        """
        self.suppression = suppression
        
    def _get_suppression(self):
        """
        Getter to get whether takedown needs to be persisted or not
        """
        return self.suppression

    def update_target_settings(self, detection_range, required_payloads, duration, suppression, captured=None):
        """
        Setter used to update the target settings when a new configuration setting for the target is received
        """
        self._set_acceptable_detection_range(detection_range)
        self._set_required_payloads_for_capture(required_payloads)
        self._set_countdown_required_for_capture(duration)
        self._set_suppression(suppression)

        # Reset the capture state if necessary
        if captured is not None:
            self._set_capture_state(captured)

        return

    def _get_last_payload_count(self):
        """
        Getter used to retrieve the number of payloads that were in range on the last measurement
        """
        return self.last_payload_count

    def _set_last_payload_count(self, payloads):
        """
        Setter used to set the number of payloads that were last in range
        """
        self.last_payload_count = payloads

        return

    def _update_entrance_timestamp(self):
        """
        Update the entrance timestamp according to the number of payloads within range
        """
        # If a payload just entered detection range, start the timer
        if self._get_last_payload_count() == 0 and self.ble_manager.get_total_in_range_payloads() > 0:
            self.entrance_timestamp = time.perf_counter()
        # If the payloads all left the detection range, reset the timer
        elif self.ble_manager.get_total_in_range_payloads() == 0:
            self.entrance_timestamp = 0

        return

    def _get_entrance_timestamp(self):
        """
        Getter used to retrieve the entrance timestamp
        """
        return self.entrance_timestamp

    def _get_capture_duration(self):
        """
        Getter used to retrieve the duration since the payload entrance
        """
        return self.capture_duration

    def _update_capture_duration(self):
        """
        Setter used to set the duration between the current time and the entrance time
        """
        self.capture_duration = time.perf_counter() - self._get_entrance_timestamp()

        return

    def _get_remaining_time(self):
        """get the remaining time able to capture target"""
        if self.ble_manager.get_total_in_range_payloads() == 0:
            return -1

        remaining: float = self._get_countdown_required_for_capture(
        ) - (time.perf_counter() - self._get_entrance_timestamp())
        if remaining < 0:
            return 0
        return remaining

    def _update_capture_state(self):
        """
        Determine whether the target has been captured
        """
        # Set to captured if the required number of payloads have arrived within the time restriction
        if self._get_payloads_in_range() >= self._get_required_payloads_for_capture() and self._get_capture_duration() <= self._get_countdown_required_for_capture():
            self.captured = True
        elif self._get_payloads_in_range() < self._get_required_payloads_for_capture() and self.suppression:
            self.captured = False
        
        self.ble_manager.set_capture_state_characteristic(self.captured)

        return

    def is_captured(self):
        return self._get_capture_state()

    def _get_capture_state(self):
        """
        Getter used to retrieve the mass target capture state
        """
        return self.captured

    def _set_capture_state(self, captured):
        """
        Setter used to enable the server to reset the capture state manually
        """
        self.captured = captured
        self.ble_manager.set_capture_state_characteristic(self.captured)

        return

    def _get_duration_to_send(self):
        """
        Getter used to retrieve the value that should be sent to the analytics server if no payloads are in range
        """
        if self._get_payloads_in_range() == 0:
            duration_to_send = -1
        else:
            duration_to_send = self._get_capture_duration()

        return duration_to_send

    def scan(self):
        """
        Detect the payloads within range and determine whether the target has been captured successfully
        """
        # Count the number of payloads in range
        self._count_payloads_in_range()

        # Update the timestamp based on the number of payloads
        self._update_entrance_timestamp()

        # Update the timer that is measuring how long the payloads have been in the detection range
        self._update_capture_duration()

        # Determine whether the target was captured or not
        self._update_capture_state()

        self._set_last_payload_count(self.ble_manager.get_total_in_range_payloads())

        return

    def publish_target_state(self):
        """
        Publish the target state to the mqtt broker
        """

        # Create a dictionary containing the target state information
        target_state = {
            'uid': self.get_uid(),
            'callsign': self.get_callsign(),
            'ipAddress': self.get_ip_address(),
            'type': self.get_target_type(),
            'latitude': self.gps.get_latitude(),
            'longitude': self.gps.get_longitude(),
            'altitude': self.gps.get_altitude(),
            'all_payloads': self.ble_manager.get_total_in_range_payloads(),
            'payloads': self._get_payloads_in_range(),
            'required_payloads': self._get_required_payloads_for_capture(),
            'countdown': self._get_countdown_required_for_capture(),
            'current_duration': self._get_remaining_time(),
            'captured': self._get_capture_state(),
            'suppression': self._get_suppression()
        }

        # Convert the payload state to JSON
        json_target_state = json.dumps(target_state)

        return json_target_state
