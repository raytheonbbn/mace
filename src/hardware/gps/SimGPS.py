#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from gps3.agps3threaded import AGPS3mechanism
from .GPS import GPS
import paho.mqtt.client as mqtt
import threading
import logging
import json
import ssl

class SimGPS(GPS):
    def __init__(self, host="localhost", port=1883, topic=None, mqtt_user='target', mqtt_password=None, ca_certs="ca_certificates/ca.crt"):

        # Invalid value indicating an 'n/a' reading from the gps
        self.INVALID = 15000

        # Class Variables
        self.latitude = self.INVALID       # Current latitude of the gps
        self.longitude = self.INVALID      # Current longitude of the gps
        self.altitude = self.INVALID       # Current altitude of the gps
        self.lock = threading.Lock()
        self.host = host
        self.port = port
        self.mqtt_user = mqtt_user
        self.mqtt_password = mqtt_password
        self.ca_certs = ca_certs
        self.gps_topic = topic
        self.client = None


    def set_client(self, client):
        self.client = client
        print(f'Subscribing to {self.gps_topic}')
        self.client.subscribe(self.gps_topic)
        self.client.message_callback_add(self.gps_topic, self.handle_gps_msg)


    def set_target_uid(self, target_uid):
        if target_uid is None:
            return
        self.uuid = target_uid
        self.gps_topic = "sim/hardware/position/" + str(self.uuid) if self.gps_topic is None else self.gps_topic


    def read_gps(self):
        """
        Responsible for reading the current gps measurements and populating the 
        object class values accordingly.  Since this is done on receipt of MQTT
        messages, this method does nothing in the SimGPS class.
        """
        return


    def get_latitude(self):
        """
        Getter used to retrieve the saved latitude
        """
        self.lock.acquire()
        lat = self.latitude
        self.lock.release()
        return lat


    def get_longitude(self):
        """
        Getter used to retrieve the saved longitude
        """
        self.lock.acquire()
        lon = self.longitude
        self.lock.release()
        return lon


    def get_altitude(self):
        """
        Getter used to retrieve the saved altitude
        """
        self.lock.acquire()
        alt = self.altitude
        self.lock.release()
        return alt


    def __on_connect(self, client, userdata, flags, rc):
        """
        Callback function used to log MQTT client connection status
        """
        if rc == 0:
            logging.debug('Successfully connected to the MQTT broker')
            print(f'Subscribing to {self.gps_topic}')
            self.client.subscribe(self.gps_topic)
            self.client.on_message = self.handle_gps_msg
        else:
            logging.debug('SimGPS was unable to successfully connect to the MQTT client')

        return


    def handle_gps_msg(self, client, userdata, message):
        """
        Callback function responsible for updating the simulated GPS position.
        """
        # Read the message to a JSON element
        json_gps_msg = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        gps_fix = json.loads(json_gps_msg)
        if self.uuid in self.gps_topic or self.uuid == gps_fix['uid']:
            self.lock.acquire()
            self.latitude = gps_fix['latitude']
            self.longitude = gps_fix['longitude']
            self.altitude = gps_fix['altitude']
            self.lock.release()
