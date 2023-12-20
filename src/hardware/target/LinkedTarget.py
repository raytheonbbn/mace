#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import json
from .Target import Target


class LinkedTarget(Target):
    def __init__(self, gps, ble, identifier, uid=None, log=False, is_sim=False):
        super().__init__('LINK', gps, ble, identifier, uid, log, is_sim=is_sim)

        # Initialize mass class variables
        # The current acceptable detection range of the target
        self.acceptable_detection_range = None
        # The capture state of the individual target, this will return to false if the full network is not captured
        self.individual_captured = False
        # The network that the target is a part of was captured
        self.networks_captured = dict()
        self.networks = dict()               # List of targets that exist in the network
        self.debug = "test"
        self.debug2 = "test2"
        self.suppression = False

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

    def _get_network_capture_state(self):
        """
        Getter used to retrieve the capture state of the linked target network
        """
        return self.networks_captured

    def _set_network_capture_state(self, captured):
        """
        Setter used to set the capture state of the linked target network
        """
        """
        if not bool(self.networks):
            self.networks = targets
            return
        
        names = list(self.networks)
        
        ii = 0
        for key, capture in captured.items():
            if ii < len(names):
                if key == names[ii]:
                    self.networks_captured[names[ii]] = capture
                ii += 1
            else:
                self.networks_captured[key] = capture
        """
        self.networks_captured = captured
        
        return

    def _get_networks(self):
        """
        Getter used to retrieve the list of targets in the network
        """
        return self.networks

    def _set_networks(self, targets):
        """
        Setter used to set the list of all targets that exist in the network
        """
        """
        if not bool(self.networks):
            self.networks = targets
            return
        
        names = list(self.networks)
        
        ii = 0
        for key, targ in targets.items():
            if ii < len(names):
                if key != names[ii]:
                    self.networks[names[ii]] = targ
                ii += 1
            else:
                self.networks[key] = targ
        """
        self.networks = targets
        
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

    def update_target_settings(self, detection_range, networks_captured, networks, suppression):
        """
        Setter used to update the target settings when a new configuration setting for the mass target is received
        """
        self.debug = networks
        self._set_acceptable_detection_range(detection_range)
        self._set_networks(networks)
        self._set_network_capture_state(networks_captured)
        self._set_suppression(suppression)
        
        return

    def _update_individual_capture_state(self):
        """
        Set the capture state according to the number of payloads in range
        """
        # Update the capture status according to the results of the count
        
        if self.suppression:
            if self.payloads_in_range > 0:
                self.individual_captured = True
            else:
                self.individual_captured = False
        else:
            if self.payloads_in_range > 0 or not False in self.networks_captured.values() and len(self.networks_captured) > 0:
                self.individual_captured = True
            else:
                self.individual_captured = False

        self.ble_manager.set_capture_state_characteristic(
            self.individual_captured)

        return

    def is_captured(self):
        return self._get_individual_capture_state()

    def _get_individual_capture_state(self):
        """
        Getter used to retrieve the mass target capture state
        """
        return self.individual_captured

    def scan(self):
        """
        Public method used to scan for payloads and interact with payloads and update the capture state based off of the results
        """
        self._count_payloads_in_range()
        self._update_individual_capture_state()

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
            'payloads': self._get_payloads_in_range(),
            'required_payloads': 1,
            'captured': self._get_individual_capture_state(),
            'networks_captured': self._get_network_capture_state(),
            'suppression': self._get_suppression()
        }

        # Convert the payload state to JSON
        json_target_state = json.dumps(target_state)

        return json_target_state
