#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import json
from .Target import Target


class IdleTarget(Target):
    def __init__(self, uid):
        super().__init__('IDLE', uid)


    def publish_configs(self, topic):
        """
        Publish the target configurations to the analytics server
        """
        # Create a dictionary containing the target configs
        configs = {
            'uid': self.get_uid(),
            'type': self.get_target_type(),
        }

        # Convert the target configs to JSON
        json_target_configs = json.dumps(configs)

        # Publish the configs to the broker
        self.client.publish(topic, json_target_configs)

        return


    def update_state(self, latitude, longitude, altitude):
        """
        Setter used to set all properties at once
        """
        self._set_latitude(latitude)
        self._set_longitude(longitude)
        self._set_altitude(altitude)

        return


    def print_state(self):
        """
        Print the current state fo the object
        """
        print(f'UID: {self.get_uid()}')
        print(f'Type: {self.get_target_type()}')
        print(f'Latitude: {self._get_latitude()}')
        print(f'Longitude: {self._get_longitude()}')
        print(f'Altitude: {self._get_altitude()}')

        return