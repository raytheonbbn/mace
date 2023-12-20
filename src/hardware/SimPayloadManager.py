#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse

from payload.Payload import Payload
from ble.BLESimPayload import BLESimPayload
from gps.SimGPS import SimGPS
import time


class SimPayloadManager:
    def __init__(self, num_quad_payloads, num_rover_payloads, relay_message_topic, query_uid_response_topic,
                 query_location_response_topic,
                 in_range_notification_topic, intent_change_notification_topic,
                 capture_notification_topic, identifier,
                 log, server_config_topic, query_uid_topic, query_location_topic,
                 send_message_topic, agent_config_topic, reset_discovered_topic, payload_cycle_frequency,
                 state_topic, server_port, agent_port, ca_certs):
        self.relay_message_topic = relay_message_topic
        self.query_uid_response_topic = query_uid_response_topic
        self.query_location_response_topic = query_location_response_topic
        self.in_range_notification_topic = in_range_notification_topic
        self.intent_change_notification_topic = intent_change_notification_topic
        self.capture_notification_topic = capture_notification_topic
        self.identifier = identifier
        self.log = log
        self.server_config_topic = server_config_topic
        self.query_uid_topic = query_uid_topic
        self.query_location_topic = query_location_topic
        self.send_message_topic = send_message_topic
        self.agent_config_topic = agent_config_topic
        self.reset_discovered_topic = reset_discovered_topic
        self.payload_cycle_frequency = payload_cycle_frequency
        self.state_topic = state_topic
        self.payloads = []
        self.server_port = server_port
        self.agent_port = agent_port
        self.ca_certs = ca_certs

        # Setup Quadcopter Payloads
        for i in range(num_quad_payloads):
            payload_uid = "Quad_" + str(i)
            self.payloads.append(self.create_payload(payload_uid))

        # Setup Rover Payloads
        for i in range(num_rover_payloads):
            payload_uid = "Rover_" + str(i)
            self.payloads.append(self.create_payload(payload_uid))

    def create_payload(self, payload_uid):
        # Create a new GPS object for the payload to use
        gps = SimGPS(topic="sim/hardware/telem", mqtt_user=None, mqtt_password=None, ca_certs=self.ca_certs)

        # Create a new BLE Manager for the payload to use
        ble_payload = BLESimPayload(host="127.0.0.1", log=self.log,
                                    uid=payload_uid,
                                    mqtt_user="white_force", mqtt_password="maceRef!")

        # for sim have the MQTT broker multiplex the agent side
        topic_prefix = payload_uid + '/'

        # Create a new payload
        payload = Payload(gps, ble_payload,
                          self.relay_message_topic,
                          topic_prefix + self.query_uid_response_topic,
                          topic_prefix + self.query_location_response_topic,
                          topic_prefix + self.in_range_notification_topic,
                          topic_prefix + self.intent_change_notification_topic,
                          topic_prefix + self.capture_notification_topic,
                          topic_prefix + self.server_config_topic,
                          self.identifier, payload_uid, self.log, is_sim=True)

        # Connect the payload to the broker
        connected=False
        while not connected:
            connected = payload.connect_all_clients(
                "localhost", "localhost", server_port=self.server_port, agent_port=self.agent_port)
            if not connected:
                time.sleep(5)

        payload.subscribe_to_agent_topics(
            topic_prefix + self.query_uid_topic,
            topic_prefix + self.query_location_topic,
            topic_prefix + self.send_message_topic,
            topic_prefix + self.agent_config_topic,
            topic_prefix + self.reset_discovered_topic)

        # Start the payload MQTT client
        payload.start_all_clients()
        return payload

    def run(self):
        print("Starting...")
        # Calculate the frequency that publishes should be made
        cycle_rate = 1.0 / self.payload_cycle_frequency

        # Timestamp used to determine whether to publish the state data
        last_cycle = time.perf_counter()

        # Rate to update printed data
        print_rate = 0.5

        # Timestamp used to determine wther to print the console data
        last_print = time.perf_counter()

        # Current state of the running dots
        current_dot = 1

        while True:
            # Get the current timestamp
            current_time = time.perf_counter()

            # print('[Payload Manager] Running', end='\r')

            if current_time - last_print >= print_rate:
                if current_dot == 0:
                    print('[Payload Manager] Running   ', end='\r')
                    current_dot += 1
                elif current_dot == 1:
                    print('[Payload Manager] Running.  ', end='\r')
                    current_dot += 1
                elif current_dot == 2:
                    print('[Payload Manager] Running.. ', end='\r')
                    current_dot += 1
                elif current_dot == 3:
                    print('[Payload Manager] Running...', end='\r')
                    current_dot = 0

                last_print = current_time

            # Publish the data if at the publish interval
            if current_time - last_cycle >= cycle_rate:
                for payload in self.payloads:
                    self.run_cycle(payload)
                    last_cycle = time.perf_counter()

    def run_cycle(self, payload):
        # Read the latest GPS reading
        payload.read_gps()

        # scan for updated targets
        payload.scan()

        payload.publish_payload_state(self.state_topic)


def main():
    parser = argparse.ArgumentParser(
        description='System used to simulate payloads.')

    # Add the desired arguments
    parser.add_argument('--num_quad_payloads', type=int, default=2,
                        help='The number of rotorcraft payloads to simulate')
    parser.add_argument('--num_rover_payloads', type=int, default=2,
                        help='The number of rover payloads to simulate')
    parser.add_argument('--server_port', type=int, default=1883,
                        help='The port number of the MQTT broker that the server client should connect to')
    parser.add_argument('--mqtt_user', type=str, default='target',
                        help='The username the mqtt client should use to authenticate with the server')
    parser.add_argument('--mqtt_password', type=str, default='maceFlag!',
                        help='The password the mqtt client should use to authenticate with the server')
    parser.add_argument('--ca_certs', type=str, default='ca_certificates/ca.crt',
                        help='The CA certs the mqtt client should use to authenticate with the server')
    parser.add_argument('--agent_port', type=int, default=1884,
                        help='The port number of the MQTT broker that the agent client should connect to')
    parser.add_argument('--identifier', type=str, default='PAYLOAD',
                        help='The identifier that should be used to distinguish payloads from targets')
    parser.add_argument('--payload_uid', type=str, default=None,
                        help='The UID that the payload should be assigned. This should only be assigned if more than one payload instance is running on a system')
    parser.add_argument('--server_config_topic', type=str, default='hardware/server/payload/configurations',
                        help='The topic that the payload should listen to for configuration change requests sent by the analytics server')
    parser.add_argument('--state_topic', type=str, default='server/hardware/payload/state',
                        help='The topic that the payload should publish its respective state data to')
    parser.add_argument('--query_uid_topic', type=str, default='payload/agent/query/uid',
                        help='The topic that the payload should subscribe to for UID queries')
    parser.add_argument('--query_location_topic', type=str, default='payload/agent/query/location',
                        help='The topic that the payload should subscribe to for location queries')
    parser.add_argument('--query_uid_response_topic', type=str, default='agent/payload/query/uid',
                        help='The topic that the created agents should subscribe to for UID query responses')
    parser.add_argument('--query_location_response_topic', type=str, default='agent/payload/query/location',
                        help='The topic that the created agents should subscribe to for location query responses')
    parser.add_argument('--relay_message_topic', type=str, default='payload/payload/proxy',
                        help='The topic that the payloads should use for relaying messages to each other')
    parser.add_argument('--send_message_topic', type=str, default='agent/agent/message',
                        help='The topic that the payload should subscribe to for requests to send messages')
    parser.add_argument('--agent_config_topic', type=str, default='payload/agent/configurations',
                        help='The topic that the payload should subscribe to for configuration change requests sent by the agent')
    parser.add_argument('--reset_discovered_topic', type=str, default='payload/agent/acknowledge_discovered',
                        help="The topic that resets the payloads discovered targets")
    parser.add_argument('--intent_change_notification_topic', type=str, default='agent/payload/notification/intent',
                        help='The topic that the payload sends notifications to the agent over regarding configuration changes made by the command station')
    parser.add_argument('--in_range_notification_topic', type=str, default='agent/payload/notification/in_range',
                        help='The topic that the payload sends notifications over regarding the payload coming into or exiting a target\'s detection range')
    parser.add_argument('--capture_notification_topic', type=str, default='agent/payload/notification/capture',
                        help='The topic that the payload sends target capture notifications over')
    parser.add_argument('--frequency', type=float, default=1,
                        help='The number of times per second that the payload should update simulated payload state')
    parser.add_argument('--sim', action='store_true',
                        help='Set to True to run the payload as a simulated process')
    parser.add_argument('--log', action='store_true',
                        help='Set to True to display all logging information to the console')

    # Parse the arguments
    args = parser.parse_args()
    manager = SimPayloadManager(args.num_quad_payloads, args.num_rover_payloads,
                                args.relay_message_topic, args.query_uid_response_topic,
                                args.query_location_response_topic, args.in_range_notification_topic,
                                args.intent_change_notification_topic,
                                args.capture_notification_topic, args.identifier, args.log,
                                args.server_config_topic, args.query_uid_topic,
                                args.query_location_topic, args.send_message_topic,
                                args.agent_config_topic, args.reset_discovered_topic, args.frequency, args.state_topic,
                                args.server_port, args.agent_port, args.ca_certs)
    manager.run()


if __name__ == '__main__':
    main()
