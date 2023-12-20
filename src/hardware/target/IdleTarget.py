#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import json
from .Target import Target


class IdleTarget(Target):
    def __init__(self, gps, ble, identifier, uid=None, log=False, is_sim=False):
        super().__init__('IDLE', gps, ble, identifier, uid, log, is_sim=is_sim)

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
        }

        # Convert the payload state to JSON
        json_target_state = json.dumps(target_state)

        return json_target_state

    def scan(self):
        pass

    def is_captured(self):
        return False
