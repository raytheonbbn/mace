#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt


class Target:
    def __init__(self, target_type, uid):

        # Initialize the class variables
        self.target_type = target_type
        self.client = None
        self.uid = uid
        self.latitude = None
        self.longitude = None
        self.altitude = None


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


    def publish_configs(self):
        """
        Publish the respective configurations to the analytics server
        """
        raise NotImplementedError


    def update_state(self):
        """
        Update the state of the object using the provided state values
        """
        raise NotImplementedError


    def get_target_type(self):
        """
        Get the target type
        """
        return self.target_type


    def get_uid(self):
        """
        Get the target UID
        """
        return self.uid


    def _get_latitude(self):
        """
        Get the latitude
        """
        return self.latitude


    def _set_latitude(self, latitude):
        """
        Set the latitude
        """
        self.latitude = latitude

        return


    def _get_longitude(self):
        """
        Get the longitude
        """
        return self.longitude


    def _set_longitude(self, longitude):
        """
        Set the longitude
        """
        self.longitude = longitude

        return


    def _get_altitude(self):
        """
        Get the altitude
        """
        return self.altitude


    def _set_altitude(self, altitude):
        """
        Set the altitude
        """
        self.altitude = altitude

        return