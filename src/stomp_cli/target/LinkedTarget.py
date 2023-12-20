#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from .Target import Target


class LinkedTarget(Target):
    def __init__(self, uid):
        super().__init__('LINK', uid)

        # Initialize the class variables
        self.payloads_in_range = 0
        self.detection_range = 0.0
        self.individual_captured = False
        self.network_captured = False


    def _get_payloads_in_range(self):
        """
        Get the number of payloads in range of the target
        """
        return self.payloads_in_range


    def _set_payloads_in_range(self, payloads):
        """
        Set the number of payloads in range of the target
        """
        self.payloads_in_range = payloads

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


    def _get_individual_captured(self):
        """
        Get the individual_captured state
        """
        return self.individual_captured


    def _set_individual_captured(self, captured):
        """
        Set the individual_captured state
        """
        self.individual_captured = captured

        return


    def _get_network_captured(self):
        """
        Get the network_captured state
        """
        return self.network_captured


    def set_network_captured(self, captured):
        """
        Set the network_captured state
        """
        self.network_captured = captured

        return


    def update_state(self, latitude, longitude, altitude, payloads, individual_captured, network_captured):
        """
        Setter used to set all properties at once
        """
        self._set_latitude(latitude)
        self._set_longitude(longitude)
        self._set_altitude(altitude)
        self._set_payloads_in_range(payloads)
        self._set_individual_captured(individual_captured)
        self.set_network_captured(network_captured)

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
        print(f'Individual Captured: {self._get_individual_captured()}')
        print(f'Network Captured: {self._get_network_captured()}')

        return