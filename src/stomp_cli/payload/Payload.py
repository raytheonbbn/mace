#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json


class Payload:
    def __init__(self, uid):

        # Initialize class variables
        self.uid = uid
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.in_range = None
        self.client = None
        self.intent = 'IDLE'
        self.type_in_range = None


    def connect(self, host, uid):
        """
        Connect the client to the desired broker
        """
        # Create an MQTT Client
        self.client = mqtt.Client(uid)

        # Connect the client to the mqtt broker
        self.client.connect(host)

        return


    def disconnect_client(self):
        """
        Disconnect the client from the broker
        """
        self.client.disconnect()

        return


    def start_client(self):
        """
        Start the MQTT Client
        """
        # Start an mqtt client
        self.client.loop_start()

        return


    def stop_client(self):
        """
        Stop the MQTT client
        """
        self.client.loop_stop()

        return


    def get_uid(self):
        """
        Get the UID
        """
        return self.uid


    def get_latitude(self):
        """
        Get the latitude
        """
        return self.latitude


    def set_latitude(self, latitude):
        """
        Set the latitude
        """
        self.latitude = latitude

        return


    def get_longitude(self):
        """
        Get the longitude
        """
        return self.longitude


    def set_longitude(self, longitude):
        """
        Set the longitude
        """
        self.longitude = longitude

        return


    def get_altitude(self):
        """
        Get the altitude
        """
        return self.altitude


    def set_altitude(self, altitude):
        """
        Set the altitude
        """
        self.altitude = altitude

        return


    def get_in_range(self):
        """
        Getter used to retrieve the in_range variable
        """
        return self.in_range


    def set_in_range(self, in_range):
        """
        Set the in_range variable
        """ 
        self.in_range = in_range

        return


    def get_type_in_range(self):
        """
        Getter used to retrieve the type of target in range of the payload
        """
        return self.type_in_range


    def set_type_in_range(self, target_type):
        """
        Set the type of target in range
        """
        self.type_in_range = target_type

        return


    def get_intent(self):
        """
        Get the intent
        """
        return self.intent


    def set_intent(self, intent):
        """
        Set the intent
        """
        self.intent = intent

        return


    def publish_configs(self, topic):
        """
        Publish the current configurations to the analytics server
        """
        # Create a dictionary containing the config information
        configs = {
            'uid': self.get_uid(),
            'intent': self.get_intent(),
        }

        # Convert the payload state to JSON
        json_payload_state = json.dumps(configs)

        # Publish the payload state to the broker
        self.client.publish(topic, json_payload_state)


    def update_state(self, latitude, longitude, altitude, in_range, target_type, modify_intent, intent):
        """
        Setter used to set all state properties at once
        """
        self.set_latitude(latitude)
        self.set_longitude(longitude)
        self.set_altitude(altitude)
        self.set_in_range(in_range)
        self.set_type_in_range(target_type)

        if modify_intent:
            self.set_intent(intent)

        return


    def print_state(self):
        """
        Helper method used to print the current state of the payload object
        """
        print(f'UID: {self.get_uid()}')
        print(f'Latitude: {self.get_latitude()}')
        print(f'Longitude: {self.get_longitude()}')
        print(f'Altitude: {self.get_altitude()}')
        print(f'Intent: {self.get_intent()}')
        print(f'In Range: {self.get_in_range()}')
        print(f'Target Type In Range: {self.get_type_in_range()}')

        return