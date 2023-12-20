#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import json
from .Target import Target


class MassTarget(Target):
    def __init__(self, uid):
        super().__init__('MASS', uid)

        # Initialize the class variables
        self.payloads_in_range = 0
        self.captured = False
        self.required_payloads = 0
        self.detection_range = 0.0
        self.modify_capture_state = False


    def _get_payloads_in_range(self):
        """
        Get the number of payloads in range
        """
        return self.payloads_in_range


    def _set_payloads_in_range(self, payloads):
        """
        Set the number of payloads in range
        """
        self.payloads_in_range = payloads

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


    def _get_required_payloads(self):
        """
        Get the required number of payloads
        """
        return self.required_payloads


    def set_required_payloads(self, payloads):
        """
        Set the required number of payloads
        """
        self.required_payloads = payloads

        return


    def _get_modify_capture_state(self):
        """
        Get the modify_capture_state flag
        """
        return self.modify_capture_state


    def set_modify_capture_state(self, modify):
        """
        Set the modify_capture_state flag
        """
        self.modify_capture_state = modify

        return


    def _get_detection_range(self):
        """
        Get the detection range
        """
        return self.detection_range


    def set_detection_range(self, distance):
        """
        Set the detection range
        """
        self.detection_range = distance

        return


    def publish_configs(self, topic):
        """
        Publish the current target configurations to the analytics server
        """
        # Create a dictionary containing the target config information
        configs = {
            'uid': self.get_uid(),
            'type': self.get_target_type(),
            'detection_range': self._get_detection_range(),
            'required_payloads': self._get_required_payloads(),
            'modify_capture_state': self._get_modify_capture_state(),
            'captured': self._get_captured()
        }

        # Convert the target configs to JSON
        json_target_configs = json.dumps(configs)

        # Publish the configs to the broker
        self.client.publish(topic, json_target_configs)

        self.set_modify_capture_state(False)

        return


    def update_state(self, latitude, longitude, altitude, payloads, captured):
        """
        Setter used to set all properties at once
        """
        self._set_latitude(latitude)
        self._set_longitude(longitude)
        self._set_altitude(altitude)
        self._set_payloads_in_range(payloads)
        self.set_captured(captured)

        return

    def print_state(self):
        """
        Helper method used to print the current target state
        """
        print(f'UID: {self.get_uid()}')
        print(f'Type: {self.get_target_type()}')
        print(f'Latitude: {self._get_latitude()}')
        print(f'Longitude: {self._get_longitude()}')
        print(f'Altitude: {self._get_altitude()}')
        print(f'Detection Range: {self._get_detection_range()}')
        print(f'Payloads in Range: {self._get_payloads_in_range()}')
        print(f'Required Payloads: {self._get_required_payloads()}')
        print(f'Captured: {self._get_captured()}') 

        return 
