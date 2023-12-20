#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import sys
import logging
import time
import threading
from binascii import hexlify
from bgapi.module import BlueGigaServer, GATTCharacteristic, GATTService
from bgapi.cmd_def import gap_discoverable_mode, gap_connectable_mode
import paho.mqtt.client as mqtt
import pygatt
import json
from pygatt.backends.bgapi import constants
from pygatt.backends.bgapi.bglib import EventPacketType
from pygatt.backends.bgapi.packets import BGAPICommandPacketBuilder
from .BLETarget import BLETarget


TARGET_CAPTURED_HANDLE = 8
TARGET_CONFIGURATION_HANDLE = 12


class BLEHardwareTarget(BLETarget, BlueGigaServer):
    '''Class to manage BLE interactions of beacons using BlueGiga API'''

    def __init__(self, target_configuration: str='IDLE', target_capture_state: str='False', port='/dev/ble_tag', baud=115200, timeout=0.1, log=False):
        super().__init__(port, baud, timeout)
        """
        GAP: Peripheral
        GATT: Master
        """

        logging.getLogger('bgapi').setLevel(logging.CRITICAL)

        # Initialize class variables
        self.target_configuration = target_configuration
        self.target_capture_state = target_capture_state
        self.logger = logging.getLogger()
        self.client: mqtt.Client = None
        self.engagement_topic_prefix = "target/payload/engage/"
        self.uid: str = None
        self.detection_range: float = 0

        self.engaged_devices = {}

        self.lock = threading.Lock()

        # Configure the device
        self.reset_ble_state()
        self.delete_bonding()
        self.allow_bonding()

        # Give the device time to reset
        time.sleep(0.1)

        if log:
            # Set up the system logger
            self.__setup_logger()

    def __setup_logger(self):
        """
        Configure the logger to send system logs to the console
        """
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
        bgapi_logger = logging.getLogger('bgapi')
        bgapi_logger.propagate = False

        return

    def set_uid(self, uid: str):
        '''Set the UID of the device to be used for identification'''
        self.uid = uid

    def get_detection_range(self):
        return self.detection_range

    def set_detection_range(self, range: float):
        '''Set the range that a payload is required to be within to capture this target'''
        self.detection_range = range

    def set_client(self, client: mqtt.Client):
        '''
        Set the MQTT client that is used to handle BLE interactions and subsribe to relevant topics.
        Call before using BLE interaction features.
        '''
        self.client = client

        self.logger.debug(
            f'subscribing to topic {self.engagement_topic_prefix + self.uid}')
        self.client.subscribe(self.engagement_topic_prefix + self.uid)
        self.client.message_callback_add(
            self.engagement_topic_prefix + self.uid, self._payload_engagement_cb)

    def _payload_engagement_cb(self, client, userdata, message):
        self.logger.info("Engaged a payload")
        '''Callback that manages BLE interactions from in range payloads'''

        # Read the message to a JSON element
        json_msg = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary and extract values
        engagement_msg = json.loads(json_msg)

        payload_uid: str = engagement_msg['uid']
        payload_intent: list = engagement_msg['intent']
        distance: float = engagement_msg['distance']

        # update engaged devices if payload is able to attempt capture
        # and distance <= self.detection_range:
        if distance <= self.detection_range:
            self.lock.acquire()
            self.engaged_devices[payload_uid] = {
                'last_contact': time.time(),
                'intent': payload_intent,
                'interacting': self.target_configuration in payload_intent and self.target_configuration != 'IDLE'
            }
            self.logger.info(self.engaged_devices)
            self.lock.release()

    def _refresh_engagements(self, target_configuration_type: str, payload_timeout: float=5):
        '''
        Update engaged devices list. Remove payloads that no longer match intent or have become stale.
        '''

        now = time.time()
        to_remove = []
        self.lock.acquire()
        for uid, d in self.engaged_devices.items():
            if target_configuration_type not in d['intent']:
                d['interacting'] = False
            if (now - d['last_contact']) > payload_timeout:
                to_remove.append(uid)

        for device_uid in to_remove:
            self.engaged_devices.pop(device_uid)
        self.lock.release()

    def advertise(self, uid: str, target_configuration_type: str, captured: bool, power: int=15):
        """
        Advertise the presence of a target
        - Undirected advertisement
        - Connectable
        """
        assert power >= 0 and power <= 15, self.logger.error(
            'The power level must be between 0 and 15')

        self.set_target_configuration_type(target_configuration_type)

        # Stop any existing advertisements
        self.stop_advertising()

        time.sleep(0.1)

        # Set the TX power
        tx_power = 15
        self._api.ble_cmd_hardware_set_txpower(tx_power)

        time.sleep(0.1)

        # Set the flags for the advertisement
        flags = b'\x02\x01\x06\x1a\xff\x09'

        # Create the string to advertise
        reduced_uid = 'T-' + uid.split('-')[1]
        payload_information = reduced_uid + ':' + \
            target_configuration_type + ':' + str(1 if captured else 0)

        # Create the advertisement
        adv = flags + bytes(payload_information, 'utf-8')

        # Set the advertisment data
        self._api.ble_cmd_gap_set_adv_data(0, adv)

        time.sleep(0.1)

        # Start broadcasting the advertisement
        self.start_advertisement(
            adv_mode=gap_discoverable_mode['gap_user_data'], conn_mode=gap_connectable_mode['gap_undirected_connectable'])

        time.sleep(0.1)

        self._refresh_engagements(target_configuration_type)

        time.sleep(0.1)

        return

    def configure_connection(self):
        """
        Configure the connection parameters
        """
        # Set the OOB encryption key
        oob_data = '52434B487F435B259254AAD6D497CCFB'
        # self.server.set_out_of_band_data(oob_data)
        self.set_out_of_band_data(oob_data)

        return

    def ble_evt_connection_status(self, connection, flags, address, address_type, conn_interval, timeout, latency, bonding):
        """
        NOTE: Inheritted from BlueGigaCallbacks
        Callback function used to handle new connections 
        """
        super(BLEHardwareTarget, self).ble_evt_connection_status(
            connection, flags, address, address_type, conn_interval, timeout, latency, bonding)

        self.logger.debug('Connection Status - Handle:%d - Flags:%02X - ' % (connection, flags) +
                          'Address:%s - ' % (hexlify(address[::-1]).decode('ascii').upper(), ) +
                          'Address Type:%d - Interval:%d - Timeout:%d - Latency:%d - Bonding:%d' % (address_type, conn_interval, timeout, latency, bonding))

        self.logger.debug('Writing initial characteristic information')

        # Write the initial state data to the characteristics
        self.set_capture_state_characteristic(self.target_capture_state)
        self.set_target_configuration_characteristic(self.target_configuration)

        self.logger.debug('initial characteristics written successfully')

        return

    def set_capture_state_characteristic(self, captured):
        """
        Write the capture state to the capture state characteristic
        """

        if captured == True:
            captured = 'CAPTURED'
        elif captured == False:
            captured = 'UNCAPTURED'

        # Convert the data to bytes
        captured_bytes = bytes(captured, 'utf-8')

        self._api.ble_cmd_attributes_write(
            handle=TARGET_CAPTURED_HANDLE, offset=0, value=captured_bytes)

        return

    def set_target_configuration_characteristic(self, configuration: str):
        """
        Write the capture state to the capture state characteristic
        """
        # Convert the data to bytes
        configuration_bytes = bytes(configuration, 'utf-8')

        self._api.ble_cmd_attributes_write(
            handle=TARGET_CONFIGURATION_HANDLE, offset=0, value=configuration_bytes)

        return

    def set_target_configuration_type(self, configuration: str):
        """
        Setter used to set the target configuration class variable
        """
        if self.target_configuration == configuration:
            pass

        self.lock.acquire()
        self.target_configuration = configuration
        for k in self.engaged_devices:
            self.engaged_devices[k]['interacting'] = False
        self.lock.release()
        

        return

    def set_target_capture_state(self, captured: str):
        """
        Setter used to set the target capture state class variable
        """
        self.target_capture_state = captured

        return

    def ble_evt_connection_disconnected(self, connection, reason):
        """
        NOTE: Inherited from BlueGigaCallbacks
        Callback function used to handle disconnect events
        """
        super(BLEHardwareTarget, self).ble_evt_connection_disconnected(
            connection, reason)
        self.logger.debug(
            f'LE Event: connection disconnected event because {reason}. Connection: {connection}')

        # Delete the bonding to enable future connections from the device
        self.delete_bonding(connection)

        return

#    def get_total_payloads_in_range(self):
    def get_interacting_payloads(self):
        """
        Get the number of payloads that are interacting properly with the target
        to attempt to capture.
        """
        # return len(self.connections)
        return len(list(filter(lambda x: self.engaged_devices[x]['interacting'], self.engaged_devices)))

    def get_total_in_range_payloads(self):
        """
        A count of the number of payloads that are in range of this target. Does not
        necessarily match intent to properly capture.
        """
        return len(self.engaged_devices)
    
    def shutdown(self):
        #ser = serial.Serial('/dev/ble_tag')
        return super().shutdown()

    


