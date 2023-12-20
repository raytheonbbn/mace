#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import json
from .Target import Target
import logging


class MassTarget(Target):
    def __init__(self, gps, ble, identifier, uid=None, log=False, is_sim=False):
        super().__init__('MASS', gps, ble, identifier, uid, log, is_sim=is_sim)

        # Initialize mass class variables
        # The detection range that a payload must enter to be connected to
        self.acceptable_detection_range = None
        # The number of pyaloads that are required for capture
        self.required_payloads_for_capture = float('inf')
        # The current capture state of the target
        self.captured = False
        self.suppression = False
        self.ble = ble
        self.logger = logging.getLogger(__name__)

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

    def update_target_settings(self, detection_range, required_payloads, suppression, captured=None):
        """
        Setter used to update the target settings when a new configuration setting for the mass target is received
        """
        self._set_acceptable_detection_range(detection_range)
        self._set_required_payloads_for_capture(required_payloads)
        self._set_suppression(suppression)

        # Reset the capture state if necessary
        if captured is not None:
            self._set_capture_state(captured)

        return

    def _update_capture_state(self):
        """
        Set the capture state according to the number of payloads in range
        """
        # Update the capture status according to the results of the count
        if self.payloads_in_range >= self.required_payloads_for_capture:
            self.logger.debug(f'MASS capture state updated to True')
            self.captured = True
        elif self.suppression:
            self.logger.debug(f'MASS capture state updated to False')
            self.captured = False
        
        # This is where captures happen, Dylan
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
        Setter used to set the capture statte of the target
        """
        self.captured = captured

        return

    def scan(self):
        """
        Public method used to scan for payloads and interact with payloads and update the capture state based off of the results
        """
        self._count_payloads_in_range()
        self._update_capture_state()

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
            'payloads': self._get_payloads_in_range(),
            'required_payloads': self._get_required_payloads_for_capture(),
            'captured': self._get_capture_state(),
            'suppression': self._get_suppression()
        }

        # Convert the payload state to JSON
        json_target_state = json.dumps(target_state)

        return json_target_state
