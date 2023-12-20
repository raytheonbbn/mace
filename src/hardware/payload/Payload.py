#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from uuid import getnode as get_mac_address
import paho.mqtt.client as mqtt
import sys
import time
import json
import logging
import ssl
import os

from ble.BLEPayload import BLEPayload
from ble.BLEHardwarePayload import BLEHardwarePayload



class Payload:
    def __init__(self, gps, ble: BLEPayload, relay_topic: str, query_uid_response_topic: str, query_location_response_topic: str,
                 in_range_notification_topic: str, intent_change_notification_topic: str, capture_notification_topic: str, server_config_topic: str,
                   identifier: str='PAYLOAD', uid: str=None, log: bool=False, is_sim: bool=False, ca_certs: str='ca_certificates/ca.crt', callsign: str=None):
        # The list of acceptable intents that a payload may have
        self.acceptable_intents = ['IDLE', 'MASS', 'PERI', 'LINK', 'FAIL']

        # Initialize class variables
        # GPS Object that enables capture of the current GPS location
        self.gps = gps
        # Bluetooth beacon interaction object
        self.ble_manager = ble
        # Unique payload identification number that is used when interacting with the server
        self.uid = uid
        # Target configuration type that the payload intends to interact with
        self.intent = ['MASS', 'PERI', 'LINK']
        # Variable indicating whether the payload is within range of a valid target
        self.in_range = False
        # The type of target in range of the payload
        self.type_in_range = None
        # MQTT client used to publish state and read configs
        self.server_client = None
        # MQTT client used to manage the user-end API (Agent)
        self.agent_client = None
        # Topic that the payloads should use to communicate with each other over
        self.relay_topic = relay_topic
        # Topic that the payloads send uid query responses over
        self.query_uid_response_topic = query_uid_response_topic
        # Topic that the payloads send location query responses over
        self.query_location_response_topic = query_location_response_topic
        # Topic used to notify the agents of the payload coming into range of a target
        self.in_range_notification_topic = in_range_notification_topic
        # Topic used to notify agents that the command station changed the intent
        self.intent_change_notification_topic = intent_change_notification_topic
        # Topic used to notify agents that the target in range was captured
        self.capture_notification_topic = capture_notification_topic
        # Topic used to notify payload that its configuration should change
        self.server_config_topic = server_config_topic
        # Flag used to indicate whether the agent reset the intent
        self.modify_intent = False
        self.battery = 0.0
        self.logger = logging.getLogger(__name__)
        #Path to CA chain to use for MQTT TLS
        self.ca_certs = ca_certs
        # Get home directory (might not always be "pi")
        self.home_dir = os.path.expanduser('~')
        

        if log:
            # Set up the system logger
            self.__setup_logger()

        if uid is None:
            # Set the UID for the device
            self.uid = self.__generate_uid(identifier)

        self.ble_manager.set_uid(self.uid)

        self.is_sim = is_sim
        if is_sim:
            gps.set_target_uid(self.uid)

        self.callsign = callsign
        if self.callsign is None:
            self.callsign = self.__generate_callsign()
            print(f"My callsign is initialized as {self.callsign}")

        self.ble_manager.set_notification_callbacks(
            self._publish_capture_notification, self._publish_in_range_notification)
        
        self.ip_address = self.__generate_ip_address()


    def __setup_logger(self):
        """
        Configure the logger to send system logs to console
        """
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

        return

    def connect_server_client(self, host, port=1883):
        """
        Create a new client and connect it to the desired broker
        """
        self.logger.debug(
            f'Creating a new MQTT server client with the ID: {self.get_uid()}')

        # Create an MQTT Client
        self.server_client = mqtt.Client(self.get_uid())

        # Setting the client callback function for successful connections
        self.server_client.on_connect = self.__on_connect

        print(f'Setting ca cert path to {self.ca_certs}')

        self.server_client.tls_set(ca_certs=self.ca_certs)
        self.server_client.tls_insecure_set(True)

        print(f'Attempting to connect to the host: {host} on port: {port}')

        # Connect the payload client to the mqtt broker
        self.server_client.connect(host, port=port)

        self.ble_manager.set_client(self.server_client)
        self.gps.set_client(self.server_client)

        return

    def connect_agent_client(self, host, port=1883):
        """
        Create a new client and connect it to the desired broker
        """
        self.logger.debug(
            f'Creating a new MQTT API client with the ID: {self.get_uid()}')

        if self.is_sim:
            self.logger.debug('mocking agent mqtt using server mqtt for sim')
            self.agent_client = self.server_client
            return

        # Create an MQTT Client
        self.agent_client = mqtt.Client(self.get_uid())

        # Setting the client callback function for successful connections
        self.agent_client.on_connect = self.__on_connect

        self.logger.debug(f'Attempting to connect to the host: {host}')

        # Connect the payload client to the mqtt broker
        self.agent_client.connect(host, port=port)

        return

    def connect_all_clients(self, server_host, api_host, server_port=1883, agent_port=1883):
        """
        Helper method used to connect all MQTT clients 
        """
        self.logger.debug("connecting_all_clients")
        if server_port == agent_port:
            assert server_host != api_host, self.logger.error(
                'The system is unable to use the same broker and port for both clients')
        try:
            self.connect_server_client(server_host, server_port)
            self.connect_agent_client(api_host, agent_port)
        except (ConnectionRefusedError, OSError) as err:
            logging.warn("Failed to connect: " + str(err))
            return False

        return True

    def disconnect_server_client(self):
        """
        Disconnect the server client from the broker
        """
        self.server_client.disconnect()

        return

    def disconnect_agent_client(self):
        """
        Disconnect the API client from the broker
        """
        self.agent_client.disconnect()

        return

    def disconnect_all_clients(self):
        """
        Helper method used to disconnect all MQTT clients
        """
        self.disconnect_server_client()
        self.disconnect_agent_client()

        return

    def subscribe_to_server_topics(self, configuration_topic, command_topic='agent/command/message/', configuration_qos=2, relay_qos=0):
        """
        Subscribe to the configuration topic that the payload should listen to for configuration changes
        """
        # Define a list of (topic, qos) tuples for the client to subscribe to
        topics = [(configuration_topic, configuration_qos), (self._get_relay_topic(
        ) + self.uid, relay_qos), (command_topic + self.uid, relay_qos)]

        # Subscribe to the desired topics
        self.server_client.subscribe(topics)

        # Attach the desired callbacks
        self.server_client.message_callback_add(
            configuration_topic, self.__read_payload_server_configurations_cb)
        self.server_client.message_callback_add(self._get_relay_topic(
        ) + self.uid, self.__proxy_republished_message_to_agent_cb)
        self.server_client.message_callback_add(
            command_topic + self.uid, self.__proxy_republished_message_to_agent_cb)

        self.logger.debug("subscribed to server topics")
        return

    def subscribe_to_agent_topics(self, query_uid_topic, query_location_topic, send_message_topic, agent_configs_topic, reset_discovered_topic,
                                  query_uid_qos=2, query_location_qos=2, send_message_qos=2, agent_configs_qos=2, reset_discovered_qos=2):
        """
        Subscribe to the topics that the client should listen to for agent calls
        """

        receive_command_topic = (
            self.uid + '/' if self.is_sim else '') + 'command/agent/message'

        # Define a list of (topic, qos) tuples for the client to subscribe to
        topics = [(query_uid_topic, query_uid_qos), (query_location_topic, query_location_qos), (send_message_topic, send_message_qos),
                  (agent_configs_topic, agent_configs_qos), (receive_command_topic, send_message_qos), (reset_discovered_topic, reset_discovered_qos)]

        # Subscribe to the list of topics
        self.agent_client.subscribe(topics)

        # Attach the desired callbacks to each function
        self.agent_client.message_callback_add(
            query_uid_topic, self.__read_agent_uid_request_cb)
        self.agent_client.message_callback_add(
            query_location_topic, self.__read_agent_location_request_cb)
        self.agent_client.message_callback_add(
            send_message_topic, self.__read_agent_message_to_republish_cb)
        self.agent_client.message_callback_add(
            agent_configs_topic, self.__read_payload_agent_configurations_cb)
        self.agent_client.message_callback_add(
            receive_command_topic, self.__read_command_message_to_republish_cb)
        self.agent_client.message_callback_add(
            reset_discovered_topic, self.__read_reset_discovered_command_cb)

        self.logger.debug("subscribed to agent topics")

    def start_server_client(self):
        """
        Start the payload MQTT Client
        """
        # Start the payload as an mqtt client
        self.server_client.loop_start()

        return

    def start_agent_client(self):
        """
        Start the payload API MQTT Client
        """
        # Start the payload API mqtt client
        self.agent_client.loop_start()

        return

    def start_all_clients(self):
        """
        Helper method used to start all payload clients (server and API) at once
        """
        self.start_server_client()
        self.start_agent_client()

        return

    def stop_server_client(self):
        """
        Stop the MQTT client
        """
        self.server_client.loop_stop()

        return

    def stop_agent_client(self):
        """
        Stop the API MQTT client
        """
        self.agent_client.loop_stop()

        return

    def stop_all_clients(self):
        """
        Helper method used to stop all MQTT clients
        """
        self.stop_server_client()
        self.stop_agent_client()

        return

    def __on_connect(self, client, userdata, flags, rc):
        """
        Callback function used to log MQTT client connection status
        """
        print(f'Subscribing to configuration topic for payload: {self.uid}')
        self.subscribe_to_server_topics(self.server_config_topic)

        if rc == 0:
            self.logger.debug('Successfully connected to the MQTT broker')
        else:
            self.logger.debug(
                'Payload was unable to successfully connect to the MQTT client')

        return

    def __generate_uid(self, identifier: str):
        """
        Generate a constant UID based off of the device MAC address and a constant identifier
        """
        mac_address = str(get_mac_address())

        uid = identifier + '-' + mac_address

        return uid
    
    def __generate_callsign(self):
        """
        Set callsign to the contents of a file called 'callsign' located in /home/<user> (~) or default to the device UID
        """
        print(f"Generating callsign for {self.uid}")
        callsign = ""
        if not self.is_sim:
            try:
                with open(f"{self.home_dir}/callsign",'r') as ff:
                    callsign = ff.readline().strip()
                print(f'Payload {self.uid} callsign set to {callsign}')
            except Exception as e:
                print(f"Could not access callsign for file at {self.home_dir}/callsign")
                print(e) 
        # callsign defaults to the device UID if none set in hardware
        if len(callsign) < 1:
            return self.uid
        return callsign

    def get_uid(self):
        """
        Get the payload UID
        """
        return self.uid
    
    # Getter and Setter for the payload callsign
    def get_callsign(self):
        if len(self.callsign) < 1:
            self.callsign = self.__generate_callsign(); 
        return self.callsign
    
    def set_callsign(self, callsign):
        self.callsign = callsign

    def _get_relay_topic(self):
        """
        Helper method used to get the relay topic
        """
        return self.relay_topic

    def _set_in_range(self, in_range: bool):
        """
        Set the in_range variable to indicate whether the payload is within range of a valid target
        """
        self.in_range = in_range

        return

    def _get_in_range(self):
        """
        Get the flag indicating whether the payload is within range of a valid target
        """
        return self.in_range

    def _get_type_in_range(self):
        """
        Getter used to retrieve the type of target in range of the payload
        """
        return str(self.type_in_range)

    def _set_type_in_range(self, target_type: str):
        """
        Setter used to set the type of target in range of the payload
        """
        self.type_in_range = target_type

        return

    def _set_intent(self, intent: list):
        """
        If the provided intent is a valid/acceptable intent, set the payload intent
        """
        for ii in intent:
            assert ii in self.acceptable_intents, 'An invalid payload intent was provided. Acceptable intents include: IDLE, MASS, PERI, LINK, and FAIL'
        
        self.intent.clear()
        self.intent = intent   

        return

    def _get_intent(self):
        """
        Get the intent of the payload
        """
        return self.intent

    def _set_modify_intent(self, modify: bool):
        """
        Setter used to modify the flag indicating whether the intent was reset by the agent
        """
        self.modify_intent = modify

        return

    def _get_modify_intent(self):
        """
        Getter used to retrieve the flag indicating whether the intent was reset by the agent
        """
        return self.modify_intent

    def _get_intent_change_notification_topic(self):
        """
        Getter used to retrieve the topic over which the payload should notify the agent that the command station
        modified the payload's intent
        """
        return self.intent_change_notification_topic

    def _get_capture_notification_topic(self):
        return self.capture_notification_topic

    def _get_query_uid_response_topic(self):
        """
        Getter used to retrieve the topic over which a payload will respond to UID queries
        """
        return self.query_uid_response_topic

    def _get_in_range_notification_topic(self):
        return self.in_range_notification_topic

    def _get_query_location_response_topic(self):
        """
        Getter used to retrieve the topic over which a payload will respond to location queries
        """
        return self.query_location_response_topic

    def read_gps(self):
        """
        Read the current gps location
        """
        self.gps.read_gps()

        return

    def scan(self):
        """
        Scans for updates to nearby targets that match this payload's intent and updates data
        """
        self.ble_manager.scan(self.intent)

    def publish_payload_state(self, topic: str):
        """
        Helper method used to publish payload state to a desired broker and topic
        """
        # Create a dictionary containing the payload state information
        payload_state = {
            'uid': self.get_uid(),
            'callsign': self.get_callsign(),
            'latitude': self.gps.get_latitude(),
            'longitude': self.gps.get_longitude(),
            'altitude': self.gps.get_altitude(),
            'timestamp': int(time.time()*1000),
            'ipAddress': self.get_ip_address(),
            'isOccupied': False,
            'neutralized': False,
            'capabilities': [],
            'modify_intent': self._get_modify_intent(),
            'intent': self._get_intent(),
            'in_range': self.ble_manager.get_type_in_range() is not None,
            'batteryLevel': self.battery,
            'loadout': 'PAYLOAD',
            'yaw': 0.0,
            'isSimPlatform': self.is_sim,
            'type_in_range': str(self.ble_manager.get_type_in_range()),
            'discovered_targets': self.ble_manager.get_discovered_targets()
        }

        # Convert the payload state to JSON
        json_payload_state = json.dumps(payload_state)

        # Publish the payload state to the broker
        self.server_client.publish(topic, json_payload_state)

        # Set the modify intent flag back to false
        self._set_modify_intent(False)

        self.logger.debug(f'Payload state published: {json_payload_state}')

        return

    def __read_payload_server_configurations_cb(self, client, userdata, message):
        """
        Callback function responsible for collecting the payload configurations distributed
        by the command stations and updating the configurations for the payload according to these configurations
        """
        # Read the message to a JSON element
        json_configurations = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        configurations = json.loads(json_configurations)

        # Update the payload configurations if the configurations are intended for the payload
        if configurations['uid'] == self.get_uid():
            self.logger.debug(
                f'Received New Configurations from Server: {configurations}')
            # If this is a change of callsign message, set the new callsign and return
            if ("set_callsign" in configurations):
                logging.info(f'Received New Callsign: {configurations["callsign"]}')
                # set the new callsign
                self.set_callsign(configurations["callsign"])
                return

            # If this is an intent change configuration, handle that
            if ("intent" in configurations):                
                self._set_intent(configurations['intent'])
                # Notify the agent of the intent change
                self._publish_payload_intent_change_notification()
                return
            return

    def _publish_payload_intent_change_notification(self):
        """
        Publish a notification to the agent that the intent was changed by the command station
        """
        # Create a notification message
        notification = {
            'intent': self._get_intent()
        }

        # Convert the message to json
        json_notification = json.dumps(notification)
        self.logger.debug(
            f'notifying agent of payload intent change: {json_notification}')

        # Publish the message
        self.agent_client.publish(
            self._get_intent_change_notification_topic(), json_notification)

        return

    def __read_payload_agent_configurations_cb(self, client, userdata, message):
        """
        Callback function used to handle configuration changes from a payload's respective agent
        """
        # Read the message to a JSON element
        json_configurations = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        configurations = json.loads(json_configurations)

        # Update the payload configurations
        self.logger.debug(
            f'Recieved New Configurations from Agent: {configurations}')
        self._set_intent(configurations['intent'])

        # Update the modify intent flag
        self._set_modify_intent(True)

        # the payload intent change notification acts as an ACK
        self._publish_payload_intent_change_notification()

        return

    def __read_reset_discovered_command_cb(self, client, userdata, message):
        """
        Callback function used to reset the payloads discovered targets
        """

        # Read the message to a JSON element
        json_configurations = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        configurations = json.loads(json_configurations)

        # Update the payload configurations
        self.logger.debug(
            f'Received New Configurations from Agent: {configurations}')

        # Get the targets to reset
        target_to_reset = configurations['targetsAcknowledged']

        self.logger.debug('Reset discovered targets')
        self.ble_manager.reset_discoverd_targets(target_to_reset)
        return

    def __read_agent_uid_request_cb(self, client, userdata, message):
        """
        Callback function used to handle UID queries from a payload's respective agent
        """
        # Create a UID message
        uid = {
            'uid': self.get_uid()
        }

        # Convert the payload state to JSON
        json_uid = json.dumps(uid)
        self.logger.debug(f'returning uid request to agent: {json_uid}')

        # Publish the payload state to the broker
        self.agent_client.publish(
            self._get_query_uid_response_topic(), json_uid)

        return

    def __read_agent_location_request_cb(self, client, userdata, message):
        """
        Callback function used to handle location queries from a payload's respective agent
        """
        # Create a location dictionary
        location = {
            'latitude': self.gps.get_latitude(),
            'longitude': self.gps.get_longitude(),
            'altitude': self.gps.get_altitude(),
        }

        try:
            # Convert the payload state to JSON
            json_location = json.dumps(location)
            self.logger.debug(
                f'returning location request to agent: {json_location}')

            # Publish the payload state to the broker
            self.agent_client.publish(
                self._get_query_location_response_topic(), json_location)
        except TypeError:
            # Catch the mock exception thrown when running test cases
            pass

        return

    def _publish_in_range_notification(self, in_range, target_uid, type_in_range):
        """
        Helper method used to notifiy the agent of changes in notification status
        """
        self.logger.debug(
            f'publishing in range notification: in_range={self._get_in_range()}, type={self._get_type_in_range()}')
        # Create a status message
        status = {
            'in_range': in_range,
            'target_uid': target_uid,
            'type_in_range': type_in_range
        }

        # Convert the message to json
        json_status = json.dumps(status)

        # Publish the message
        self.agent_client.publish(
            self._get_in_range_notification_topic(), json_status)

        return

    def _publish_capture_notification(self, uid):
        """
        Helper method used to notify an agent of a target capture
        """
        self.logger.debug('publishing captured notification: captured')
        # Create a status message
        status = {
            'captured': True,
            'uid': uid
        }

        # Convert the message to json
        json_status = json.dumps(status)

        # Publish the message
        self.agent_client.publish(
            self._get_capture_notification_topic(), json_status)

        return

    def __read_agent_message_to_republish_cb(self, client, userdata, message):
        """
        Callback function used to collect a message provided by the agent and re-publish the message to the other agents in the network
        """
        # Read the message to a JSON element
        json_message = str(message.payload.decode('utf-8', 'ignore'))
        self.logger.debug(f'Sending proxy message: {json_message}')

        # Convert the JSON message to a python dictionary
        republished_message = json.loads(json_message)

        if 'topic' not in republished_message:
            self.logger.error(
                'The message could not be proxied to the agent. Please include the topic that the message should be published to the agent over')
            return

        if 'uid' not in republished_message:
            self.logger.error(
                'The message could not be proxied to the agent. Please include the UID of the proxy payload.')
            return

        destination = republished_message['uid']
        republished_message['from'] = self.uid

        # Convert the message to JSON
        republished_message_json = json.dumps(republished_message)

        # Relay the message to the respective payload
        self.server_client.publish(
            self._get_relay_topic() + destination, republished_message_json)

        return

    def __read_command_message_to_republish_cb(self, client, userdata, message):
        """
        Callback function used to collect messages destined for the command station of end users
        """
        # Read the message to a JSON element
        json_message = str(message.payload.decode('utf-8', 'ignore'))
        self.logger.debug(f'Sending proxy message: {json_message}')

        # Convert the JSON message to a python dictionary
        republished_message = json.loads(json_message)

        republished_message['from'] = self.uid

        # Convert the message to JSON
        republished_message_json = json.dumps(republished_message)

        # Relay the message to the respective payload
        self.server_client.publish(
            republished_message['topic'], republished_message_json)

        return

    def __proxy_republished_message_to_agent_cb(self, client, userdata, message):
        """
        Callback function used to handle a republished message and publish it to the agent over the respective topic
        """
        # Read the message to a JSON element
        json_message = str(message.payload.decode('utf-8', 'ignore'))
        self.logger.debug(f'Receiving proxy message: {json_message}')

        # Convert the JSON message to a python dictionary
        republished_message = json.loads(json_message)

        # Get the topic over which the client should send the data to the agent
        topic = republished_message['topic']

        new_message = json.dumps(republished_message)

        # Publish the message to the agent
        self.agent_client.publish(
            self.uid + '/' + topic if self.is_sim else topic, new_message)

        return


    def __generate_ip_address(self):
        os.system(f"hostname -I > {self.home_dir}/ip_address")
        if not self.is_sim:
            try:
                with open(f'{self.home_dir}/ip_address','r') as ff:
                    ip_address = None
                    ip_addresses = ff.readline().split()
                    for address in ip_addresses:
                        if not address.startswith("169."):
                            ip_address = address
                    if ip_addresses is None:
                        raise Exception("IP address could not be found for this pi.")
                    print(f'Pi ip address is {ip_address}')
                    return ip_address
            except Exception as e:
                print(f"Could not access ip_adress for file at {self.home_dir}/ip_address")
                print(e)  
                return None
        return "sim"
    
    def get_ip_address(self):
        return self.ip_address
