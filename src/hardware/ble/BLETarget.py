#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import sys
import logging
import time


class BLETarget:
    def get_total_in_range_payloads(self) -> int:
        """
        Get a count of the payloads in range of the target
        Does not have to match intent
        """
        raise NotImplementedError(
            "A subclass should implement get_total_payloads_in_range!")

    def get_interacting_payloads(self) -> int:
        """
        Get a count of the payloads in range of the target
        This is equivalent to the total number of connections currently being maintained
        This method should be implemented by subclasses.
        """
        raise NotImplementedError(
            "A subclass should implement get_total_payloads_in_range!")

    def set_target_uid(self, uid):
        """
        Used to set the uid of the target for the sim BLE.
        """
        pass

    def set_target_configuration_type(self, config: str):
        pass


    def advertise(self, uid: str, param: str, param1: bool):
        """
        Used to advertise the state over BLE in hardware
        """
        pass

    def set_capture_state_characteristic(self, captured):
        pass
