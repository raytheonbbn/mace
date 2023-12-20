#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import json
import sys
import paho.mqtt.client as mqtt
import logging
import time
from binascii import hexlify
from bgapi.module import GATTCharacteristic, GATTService, BlueGigaClient, RemoteError
from ble.BLEPayload import BLEPayload
import ssl

TARGET_CAPTURED_HANDLE = 8
TARGET_CONFIGURATION_HANDLE = 12


class BLESimPayload(BLEPayload):
    def __init__(self, uid, host, port=1883, intent='IDLE', log=False, mqtt_user='target', mqtt_password=None, ca_certs='ca_certificates/ca.crt'):
        """
        """

        # Initialize class variables
        self.port = port
        self.host = host
        self.intent = intent
        self.connected_devices = set()
        self.discovered_devices = set()
        self.in_range_devices = {}
        self.type_in_range = None
        self.logger = logging.getLogger(__name__)
        self.uuid = uid
        self.interaction_topic = "sim/hardware/ble/" + str(self.uuid)
        self.logger.debug(f'interaction topic set to {self.interaction_topic}')
        self.capture_notification_cb = None
        self.in_range_notification_cb = None
        self.mqtt_user = mqtt_user
        self.mqtt_password= mqtt_password
        self.ca_certs=ca_certs
        self.client = None

        # stomp and gps also subscribes to this because it has other important info
        self.telem_topic = "blue_force/server/payload/telem"  # uid is in json message

        if log:
            # Set up the system logger
            self.__setup_logger()

    def get_discovered_targets(self):
        return list(self.discovered_devices)

    def reset_discoverd_targets(self, target=None):
        """
        Reset discovered targets. If target is None, then all targets are removed
        """
        
        # Reset the discovered targets
        if target is None:
            # If target is none reset all
            self.discovered_devices = set()
        else:
            # Remove only the specific target
            try:
                self.discovered_devices.remove(target)
            except KeyError:
                # Not sure if we should care about this? TODO
                pass

    def __setup_logger(self):
        """
        Configure the logger to send system logs to the console
        """
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
        bgapi_logger = logging.getLogger('bgapi')
        bgapi_logger.propagate = False

        return

    def set_notification_callbacks(self, capture_cb, in_range_cb):
        self.capture_notification_cb = capture_cb
        self.in_range_notification_cb = in_range_cb

    def set_uid(self, uid):
        pass

    def set_client(self, client):
        self.client = client
        print(f'Subscribing to {self.interaction_topic}')
        self.client.subscribe(self.interaction_topic)
        self.client.message_callback_add(self.interaction_topic, self.handle_ble_msg)
        print(f'Subscribing to blue_force/server/payload/state')
        self.client.subscribe("blue_force/server/payload/state")
        self.client.message_callback_add(
            "blue_force/server/payload/state", self.handle_telem_update)

    def get_type_in_range(self) -> 'str | None':
        return self.type_in_range


    def handle_telem_update(self, client, userdata, message):
        # Read the message to a JSON element
        json_msg = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        telem = json.loads(json_msg)

        # update intent here
        self.intent = telem['intent']
        self.logger.debug(f'Intent updated to {self.intent}')

        return

    def handle_ble_msg(self, client, userdata, message):
        """handle notification of simulated ble interaction with target"""

        # Read the message to a JSON element
        json_ble_msg = str(message.payload.decode('utf-8', 'ignore'))
        self.logger.debug(f"Received sim ble message: {json_ble_msg}")

        # Convert the JSON message to a python dictionary
        ble_info = json.loads(json_ble_msg)

        target_id = ble_info['target_id']
        target_in_range = bool(ble_info['in_range'])
        # target_capture = ble_info['capture_state']
        target_config = ble_info['config']

        intent = ble_info['intent'] if 'intent' in ble_info else None
        if intent is not None:
            self.set_intent(intent)

        if not target_in_range:
            self._close_ble_device_connection(target_id)
            self.type_in_range = None
            self.in_range_notification_cb(False, target_config)
            return
        else:
            self.type_in_range = target_config
            self.in_range_notification_cb(True, target_config)

        # check to see if this is a new discovery and act accordingly
        if target_id not in self.discovered_devices:
            self.discovered_devices.add(target_id)

        if target_config in intent and target_in_range:
            self._connect_to_ble_device(target_id)

        # close the connection if the payload intent does not match the target configuration type
        else:
            self._close_ble_device_connection(target_id)

        # if target_capture != self.connected_devices[target_id]['status']:
        #     self.logger.debug(f'The target capture state changed. Value: {capture_state}')

        #     # Only maintain the connection of the capture state changes to uncaptured
        #     if target_capture == 'CAPTURED' or target_capture == 'FAILED':
        #         self._close_ble_device_connection()

    def set_intent(self, intent):
        """
        Setter used to store the intent of the respective payload
        """
        self.intent = intent

        return

    def _get_intent(self):
        """
        Getter used to retrieve the intent of the respective payload
        """
        return self.intent

    def _connect_to_ble_device(self, uid):
        """
        Connect a Target BLE Beacon to a Payload BLE Beacon
        """

        if uid not in self.connected_devices:
            self.logger.debug(f'Connecting to target: {uid}')

            # Keep track of the connection so that it may be dropped in the future
            self.connected_devices.add(uid)

        return

    def _close_ble_device_connection(self, uid):
        """
        Close a BLE connection
        If no `uid` is provided then assume we only have one connected device
        """

        # Remove the target from the set of connected targets
        if uid in self.connected_devices:
            self.connected_devices.remove(uid)

            self.logger.debug(f'connection to {uid} closed')

        return

    def scan(self, intent):
        """
        Scan for advertisements being broadcasted by a device
        """
        return

    def get_connected_devices(self):
        return self.connected_devices
