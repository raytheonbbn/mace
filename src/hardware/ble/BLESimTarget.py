#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from .BLETarget import BLETarget
import paho.mqtt.client as mqtt
import threading
import logging
import json
import ssl


class BLESimTarget(BLETarget):
    def __init__(self, host="localhost", port=1883, mqtt_user='target', mqtt_password=None, ca_certs='ca_certificates/ca.crt'):
        self.lock = threading.Lock()
        self.host = host
        self.port = port
        self.interaction_map: 'dict[str,bool]' = {}
        self.capture_state = None
        self.logger = logging.getLogger(__name__)
        self.mqtt_user = mqtt_user
        self.mqtt_password = mqtt_password
        self.ca_certs = ca_certs
        self.client = None

    def set_target_uid(self, target_uid):
        """
        Used in sim to intialize sim MQTT topics.
        """
        if target_uid is None:
            return
        self.uuid = target_uid
        self.interaction_topic = "sim/hardware/ble/" + str(self.uuid)


    def set_uid(self, uid):
        pass

    def set_client(self, client):
        self.client = client
        print(f'Subscribing to {self.interaction_topic}')
        self.client.subscribe(self.interaction_topic)
        self.client.message_callback_add(self.interaction_topic, self.handle_ble_msg)

    def get_interaction_map(self):
        """
        Getter used to retrieve the targets interaction state 
        """
        self.lock.acquire()
        result = self.interaction_map
        self.lock.release()
        return result

    def set_detection_range(self, range):
        pass

    def get_interacting_payloads(self):
        """
        Get the number of payloads that are interacting properly with the target
        to attempt to capture.
        """
        return len(list(filter(lambda x: self.interaction_map[x], self.get_interaction_map().keys())))

    def get_total_in_range_payloads(self):
        """
        A count of the number of payloads that are in range of this target. Does not
        necessarily match intent to properly capture.
        """
        return len(self.get_interaction_map())


    def set_target_configuration_type(self, configuration: str):
        pass

    def handle_ble_msg(self, client, userdata, message):
        """
        Callback function responsible for updating simulated BLE interactions.
        """
        # Read the message to a JSON element
        json_ble_msg = str(message.payload.decode('utf-8', 'ignore'))
        self.logger.debug(f"Received sim ble message: {json_ble_msg}")

        # Convert the JSON message to a python dictionary
        ble_info = json.loads(json_ble_msg)
        self.lock.acquire()

        interacting = bool(ble_info['interacting'])
        in_range = bool(ble_info['in_range'])
        payload_id: str = ble_info['payload_id']
        if in_range:
            # Add interaction to map
            self.interaction_map[payload_id] = interacting
        elif payload_id in self.interaction_map:
            # Remove interaction from map
            self.interaction_map.pop(payload_id)
        self.lock.release()

    def set_target_configuration_characteristic(self, intent):
        pass

    def set_capture_state_characteristic(self, capture_state):
        if self.capture_state != capture_state:
            self.capture_state = capture_state

            msg = {
                'captured': capture_state == 'CAPTURED' or capture_state == True,
                'uid': self.uuid
            }
            if not msg['captured']:
                return

            json_msg = json.dumps(msg)

            for uid, val in self.interaction_map.items():
                self.client.publish(
                    uid + '/agent/payload/notification/capture', json_msg)
