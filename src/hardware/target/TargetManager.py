#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import logging
from uuid import getnode as get_mac_address
import os
import json
import sys
import time
import paho.mqtt.client as mqtt
from .IdleTarget import IdleTarget
from .MassTarget import MassTarget
from .PeriodicTarget import PeriodicTarget
from .LinkedTarget import LinkedTarget
from ble.BLEHardwareTarget import BLEHardwareTarget
import threading
import ssl



class TargetManager:
    def __init__(self, gps, ble, manager_identifier, target_identifier, configuration_topic, manager_uid=None, target_uid=None, log=False, is_sim=False, ca_certs="ca_certificates/ca.crt"):
        # The list of acceptable configurations that a target may exist as
        self.acceptable_target_types = ['IDLE', 'MASS', 'PERI', 'LINK']

        # Initialize class variables
        # Whether or not this is a simulated target
        self.is_sim = is_sim
        # The GPS object that will be used by the target
        self.gps = gps
        # The BluetoothManager object that will be used by the target
        self.ble = ble
        # The identifier that should be used by the target
        self.target_identifier = target_identifier
        # UID assigned to a target
        self.target_uid = target_uid
        # The current target being managed, this is set to an idle target by default
        self.target = IdleTarget(
            self.gps, self.ble, target_identifier, target_uid, is_sim=is_sim)
        # The MQTT client that the manager should used to read configurations
        self.client = None
        # The UID of the manager used by the MQTT client
        self.uid = manager_uid
        # The MQTT host that the target should connect to
        self.target_host = None
        # Flag indicating whether the targets should log outputs
        self.log = log
        # Logger for the class
        self.logger = logging.getLogger(__name__)
        self.configuration_topic = configuration_topic
        #CA cert for MQTT TLS
        self.ca_certs=ca_certs

        if self.uid is None:
            # Create a UID for the manager
            self.uid = self.__generate_uid(manager_identifier)

        self.ble.set_uid(self.target.get_uid())

        if log:
            # Set up the system logger
            self.__setup_logger()

    def __setup_logger(self):
        """
        Configure the logger to send system logs to the console
        """
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

        return

    def __generate_uid(self, identifier):
        """
        Generate a constant UID based off of the device MAC address and a constant identifier
        """
        # Get system MAC address
        mac_address = str(get_mac_address())

        uid = identifier + '-' + mac_address

        return uid

    def get_uid(self):
        """
        Getter to retrieve the target unique identification code
        """
        return self.uid

    def connect_clients(self, manager_host, target_host=None, manager_port=1883, target_port=1883, mqtt_user="target", mqtt_password=None):
        """
        Create a new client and connect it to the desired broker, connect the target client as well
        Note that two host parameters have been included: the manager host and the target host.
        If the target host is not provided, then the manager host will be set for both the current target
        and the manager. If a target host is provided, the target will connect to the other host 
        """
        logging.debug(
            f'Creating a new MQTT target configuration subscriber: {self.get_uid()}')

        # Create an MQTT Client
        self.client = mqtt.Client(self.get_uid())

        # Setting the client callback function for successful connections
        self.client.on_connect = self.__on_connect

        # Setup the target username/password
        self.client.username_pw_set(mqtt_user, mqtt_password)

        #Setup TLS
        self.client.tls_set(ca_certs=self.ca_certs, tls_version=ssl.PROTOCOL_TLSv1_2)
        self.client.tls_insecure_set(True)

        logging.debug(f'Attempting to connect to the host: {manager_host}')

        try:
            # Connect the payload client to the mqtt broker
            self.client.connect(manager_host)

            self.ble.set_client(self.client)
            self.gps.set_client(self.client)
            self.mqtt_user = mqtt_user
            self.mqtt_password = mqtt_password

            # Determine if the the target host is different than the manager host
            # if so, connect the target to it
            if target_host is not None:
                # Connect the target client
                self.target.connect_client(target_host, mqtt_user, mqtt_password)
                self.target_host = target_host
            else:
                # Connect the target client
                self.target.connect_client(manager_host, mqtt_user, mqtt_password)
                self.target_host = manager_host
        except ConnectionRefusedError as err:
               logging.warn("Failed to connect: " + str(err))
               return False
    
        return True

    def disconnect_clients(self):
        """
        Disconnect the manager client and the target client from the broker
        """
        self.client.disconnect()
        self.target.disconnect_client()

        return

    def subscribe_to_configurations(self, configuration_topic):
        """
        Subscribe to the configuration topic that the target should listen to for configuration changes
        """
        uid = self.get_uid()
        print(f'Subscribing to configuration topic for target: {uid}')
        # Subscribe to the configuration topic which will provide target configurations and attach the callback function
        self.client.subscribe(configuration_topic)
        self.client.message_callback_add(
            configuration_topic, self.read_target_configurations_cb)

        return

    def start_clients(self):
        """
        Start the payload MQTT Client
        """
        # Start the manager and target as MQTT clients
        self.client.loop_start()
        self.target.start_client()

        return

    def stop_clients(self):
        """
        Stop the MQTT clients
        """
        # Stop both the manager client and the target client
        self.client.loop_stop()
        self.target.stop_client()

        return

    def __on_connect(self, client, userdata, flags, rc):
        """
        Callback function used to log MQTT client connection status
        """
        self.subscribe_to_configurations(self.configuration_topic)
        if rc == 0:
            logging.debug('Successfully connected to the MQTT broker')
        else:
            logging.debug(
                'Payload was unable to successfully connect to the MQTT client')

        return

    def _get_target_identifier(self):
        """
        Getter used to retrieve the identifier that should be used for a target
        """
        return self.target_identifier

    def _get_target_uid(self):
        """
        Getter used to retrieve the UID that is assigned to a target
        """
        return self.target.get_uid()

    def _get_log_flag(self):
        """
        Get the flag indicating whether the targets should log their log messages
        """
        return self.log

    def _change_target_type(self, target_type):
        """
        Change the target type according to the type provided
        """
        assert target_type in self.acceptable_target_types, 'An invalid target configuration was provided. Acceptable configurations include: IDLE, MASS, PERI, and LINK'

        logging.debug(f'Converting target type to {target_type}')

        if not self.is_sim:
            # Refresh ble client
            self.ble.shutdown()
            self.ble = BLEHardwareTarget()
            self.ble.set_uid(self.target.get_uid())
            self.ble.set_client(self.client)


        # Change the type of target that the target should operate as
        if target_type == 'IDLE':
            self.target = IdleTarget(self.gps, self.ble, self._get_target_identifier(
            ), self._get_target_uid(), self._get_log_flag(), is_sim=self.is_sim)
        elif target_type == 'MASS':
            self.target = MassTarget(self.gps, self.ble, self._get_target_identifier(
            ), self._get_target_uid(), self._get_log_flag(), is_sim=self.is_sim)
        elif target_type == 'PERI':
            self.target = PeriodicTarget(self.gps, self.ble, self._get_target_identifier(
            ), self._get_target_uid(), self._get_log_flag(), is_sim=self.is_sim)
        elif target_type == 'LINK':
            self.target = LinkedTarget(self.gps, self.ble, self._get_target_identifier(
            ), self._get_target_uid(), self._get_log_flag(), is_sim=self.is_sim)

        return

    def _update_target_configurations(self, configs):
        """
        Update the target configurations
        """
        # Ensure that the target types are the same
        assert configs['type'] == self.target.get_target_type()

        # Don't modify any configurations if the config type is IDLE
        if configs['type'] == 'IDLE':
            return

        logging.debug(
            f'Updating the configurations for the {self.target.get_target_type()} target type')

        # If the configurations specify that the capture state is being manually modified, reset the capture state
        if configs['type'] != 'LINK':
            if configs['modify_capture_state']:
                captured = configs['captured']
            else:
                captured = None

        # Update the target configurations according to the type of target currently set
        if configs['type'] == 'MASS':
            self.target.update_target_settings(
                configs['detection_range'], configs['required_payloads'], configs['suppression'], captured=captured)
        elif configs['type'] == 'PERI':
            self.target.update_target_settings(
                configs['detection_range'], configs['required_payloads'], configs['capture_countdown'], configs['suppression'], captured=captured)
        elif configs['type'] == 'LINK':
            self.target.update_target_settings(
                configs['detection_range'], configs['networks_captured'], configs['networks'], configs['suppression'])

        if 'detection_range' in configs:
            self.ble.set_detection_range(configs['detection_range'])

        return

    def read_target_configurations_cb(self, client, userdata, message):
        """
        Callback function responsible for collecting the target configurations distributed
        by the command stations and updating the configurations for the payload according to these configurations
        """
        # Read the message to a JSON element
        json_configurations = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        configurations = json.loads(json_configurations)

        # Update the payload configurations if the configurations are intended for the payload
        if (configurations['uid'] == self._get_target_uid()):
            # Check if this message if for changing the target UID
            if ("set_callsign" in configurations):
                logging.info(f'Received New Callsign: {configurations["callsign"]}')
                # set the new callsign
                self.target.set_callsign(configurations["callsign"])
                return

            logging.debug(f'Received New Configurations: {configurations}')
            # If the configuration type has changed, update the target configuration type
            if configurations['type'] != self.target.get_target_type():
                # Stop and disconnect the existing target client
                self.target.stop_client()
                self.target.disconnect_client()

                # Change the target configuration type
                self._change_target_type(configurations['type'])

                # Start and reconnect the updated client
                self.target.connect_client(self.target_host, self.mqtt_user, self.mqtt_password)
                self.target.start_client()

            # Update the target configurations
            self._update_target_configurations(configurations)

            if 'type' in configurations:
                self.ble.set_target_configuration_characteristic(
                    configurations['type'])
            if 'captured' in configurations:
                self.ble.set_capture_state_characteristic(
                    str(configurations['captured']))

        return

    def update(self):
        """
        If the target requires scanning, execute target scanning
        """
        # Update the current gps location
        self.target.read_gps()
        self.target.scan()
        time.sleep(0.1)
        self.target.advertise()

        return

    def publish(self, topic):
        """
        Publish the target data
        """
        # Publish the target data
        json_payload = self.target.publish_target_state()
        self.logger.debug(f'publishing target state: {json_payload}')
        self.client.publish(topic, json_payload)

        return
