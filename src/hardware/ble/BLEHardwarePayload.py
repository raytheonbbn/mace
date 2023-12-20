#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import sys
import logging
import time
import json
import math
from binascii import hexlify
from pygatt.backends.bgapi import constants
from pygatt.backends.bgapi.packets import BGAPICommandPacketBuilder
import paho.mqtt.client as mqtt

from pygatt.device import BLEDevice
from bgapi.module import BlueGigaModule, GATTCharacteristic, GATTService, BlueGigaClient, RemoteError
import pygatt
from .BLEPayload import BLEPayload

TARGET_CAPTURED_HANDLE = 8
TARGET_CONFIGURATION_HANDLE = 12


class RunningAvg():
    '''
    A container to store a running average of some numerical value.
    '''

    def __init__(self, start_avg: float, window: int=4):
        '''
        Create a new running average.
        start_avg: initial value
        window: Number of values to average. Old values are discarded.
        '''
        self.window = window
        self.values = [start_avg]

    def get(self):
        '''Get the current average value'''
        return sum(self.values) / len(self.values)

    def push(self, new_value: float):
        '''Add a new value to the running average'''
        self.values.append(new_value)
        if len(self.values) > self.window:
            self.values.pop(0)

    def __str__(self) -> str:
        return self.get()


class BLEHardwarePayload(BlueGigaClient):
    '''Class to manage BLE interactions between beacons using the BlueGiga API'''

    def __init__(self, intent: str='IDLE', port='/dev/ble_tag', baud=115200, timeout=0.1, log=False):
        super().__init__(port, baud, timeout)
        """
        GAP: Central
        """

        logging.getLogger('bgapi').setLevel(logging.CRITICAL)

        # Initialize class variables
        self.intent = intent
        self.engaged_devices = {}
        self.discovered_devices = set()
        self.in_range_devices = {}
        self.logger = logging.getLogger(__name__)
        self.client: mqtt.Client = None
        self.uid: str = None
        self.engagement_topic_prefix = "target/payload/engage/"
        self.capture_notification_cb = None
        self.in_range_notification_cb = None

        # Reset the device state
        self.reset_ble_state()
        self.delete_bonding()
        self.allow_bonding()

        # Give the device time to reset
        time.sleep(0.1)

        self.log = log
        if log:
            # Set up the system logger
            self.__setup_logger()

    def set_notification_callbacks(self, capture_cb, in_range_cb):
        '''
        Set callback functions for the payload. When BLE events are triggered, these may be
        called and allow the payload class to handle events at a higher level.
        The callbacks occur when a target is captured, or when a target goes in/out of range
        '''
        self.capture_notification_cb = capture_cb
        self.in_range_notification_cb = in_range_cb

    def set_client(self, client: mqtt.Client):
        '''Sets the MQTT client to be used for BLE interaction messages. Call before using BLE features'''
        self.client = client

    def set_uid(self, uid: str):
        '''Set the UID of the device to be used for identification'''
        self.uid = uid

    def _engage_device(self, uid: str, distance: float, config: str, rssi):
        '''When a device is within BLE range, update engagement record and possibly ping target about engagement'''

        device = self.engaged_devices.get(uid, None)

        # new engagement make new record
        if device is None:
            device = {
                'last_contact': time.time(),
                'last_transmission': None,
                'type': config,
                # Note that we are using a running average of 4 here
                # this helps to reduce the standard deviation of the samples
                # which gives a better accuracy without taking too much time
                'distance_avg': RunningAvg(distance, 4)
            }
            self.engaged_devices[uid] = device
        else:
            device['last_contact'] = time.time()
            device['type'] = config
            device['distance_avg'].push(distance)

        debounce_time = 2  # sec

        # publish transmission to target if enough time has passed since last transmission
        if device['last_transmission'] is None or (device['last_contact'] - device['last_transmission']) > debounce_time:
            self.engaged_devices[uid]['last_transmission'] = self._publish_engagement(
                uid, device['distance_avg'].get())

    def get_engaged_devices(self) -> 'list[str]':
        return list(self.engaged_devices.keys())

    def get_type_in_range(self) -> 'str | None':
        if len(self.engaged_devices) > 0:
            return self.engaged_devices[self.get_engaged_devices()[0]]['type']
        else:
            return None

    def _publish_engagement(self, target_uid: str, distance: float):
        '''Publish MQTT message to a target to inform that a payload attempts to capture'''

        message = {
            'uid': self.uid,
            'intent': self.intent,
            'distance': distance
        }

        json_message = json.dumps(message)

        self.logger.debug(
            f'publishing engagement to {self.engagement_topic_prefix + target_uid}')
        self.client.publish(self.engagement_topic_prefix +
                            target_uid, json_message)

        return time.time()

    def __setup_logger(self):
        """
        Configure the logger to send system logs to the console
        """
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
        bgapi_logger = logging.getLogger('bgapi')
        bgapi_logger.propagate = False

        return

    def set_intent(self, intent: str):
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

    def _refresh_in_range(self, stale_time: float=2):
        '''
        Update the local list of targets that are in range of this payload.
        Remove targets that haven't been contacted in a predetermined stale_time. 
        '''
        now = time.time()
        to_remove = []
        for uid, body in self.in_range_devices.items():
            if now - body['time'] > stale_time:
                to_remove.append(uid)

        for uid in to_remove:
            self.in_range_notification_cb(
                False, uid, self.in_range_devices[uid]['config'])
            self.in_range_devices.pop(uid)

    def _calc_distance(self, rssi: float, target_uid: str):
        '''Estimate the distance to the target based on the RSSI.'''

        # E[P] = -7.221345 * ln(meters) - 65.921186
        # R^2 = 0.93
        # sigma = 3 to 5.5 dB depending on distance
        a = -7.221345
        b = -65.921186

        # this is the inverse function of power eqation above
        dist = lambda signal: math.exp((signal - b) / a) # distance in meters

        dist_est = dist(rssi)

        if self.log:
            sigma = 4.5 # dB
            mesg = {
                'uid': self.uid,
                'target': target_uid,
                'rssi': rssi,
                'dist_est': dist_est,
                'dist_low': dist(rssi+2*sigma),
                'dist_high': dist(rssi-2*sigma),
                'timestamp': time.time()
            }
            mesg_txt = json.dumps(mesg)
            self.client.publish('logger/rssi/' + self.uid, mesg_txt)

        return dist_est # distance in m

    def scan(self, intent: str):
        """
        Scan for advertisements being broadcasted by a device
        """

        self.intent = intent

        self._refresh_in_range()

        # Scan for advertisements
        responses = self.scan_all(timeout=2)

        if responses is None:
            return

        # Read through all of the scanned responses
        for response in responses:
            response.parse_advertisement_data()

            # Get the signal strength
            rssi: float = response.rssi

            # Look for an advertisement
            for payload in response.adv_payload:
                try:
                    # Decode the advertisement payload
                    device_data: str = payload[2].decode('utf-8').strip()

                    # Make sure that the advertisement is from a target
                    if 'T-' in device_data:

                        # Extract the relevant data
                        split_data = device_data.split(':')
                        # re-expand uid from compressed string
                        uid = 'TARGET-' + split_data[0].split('-')[1]
                        configuration_type = split_data[1]
                        capture_state = 'True' if split_data[2] == '1' else 'False'

                        if uid not in self.in_range_devices:
                            self.in_range_notification_cb(True, uid, configuration_type)

                        if uid in self.in_range_devices and not self.in_range_devices[uid]['capture_state'] and capture_state == 'True':
                            self.capture_notification_cb(uid)

                        # update in_range_devices with most recent data
                        distance = self._calc_distance(rssi, uid)
                        self.in_range_devices[uid] = {
                            'config': configuration_type,
                            'capture_state': capture_state,
                            'time': time.time(),
                            'distance': distance
                        }
                        self.logger.debug(
                            f'In range of target: {uid} with configuration type: {configuration_type}, RSSI: {rssi}, and Distance: {distance} m')

                        # check to see if this is a new discovery and act accordingly
                        if uid not in self.discovered_devices:
                            self.logger.debug(f'discovering target: {uid}')
                            self.discovered_devices.add(uid)

                        # attempt to capture the target by engaging with it. Target decides if the intent matches and if it gets captured
                        self._engage_device(uid, distance, configuration_type, rssi)

                except UnicodeDecodeError:
                    # We don't want that message
                    pass
        return

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
