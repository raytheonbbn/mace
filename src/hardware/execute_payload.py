#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse
import time
from ble.BLESimPayload import BLESimPayload
from gps.SimGPS import SimGPS
from payload.Payload import Payload
from gps import GPS
from ble.BLEHardwarePayload import BLEHardwarePayload


def main():
    # Create a new argument parser
    parser = argparse.ArgumentParser(
        description='System used to configure the device as a payload and to execute payload behavior')

    # Add the desired arguments
    parser.add_argument('--server_host', type=str, default='127.0.0.1',
                        help='The URL or IP address of the analytics server broker that the payload MQTT client should connect to')
    parser.add_argument('--agent_host', type=str, default='127.0.0.1',
                        help='The URL or IP address of the payload broker that the payload and agent MQTT clients should connect to')
    parser.add_argument('--server_port', type=int, default=1883,
                        help='The port number of the MQTT broker that the server client should connect to')
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
    parser.add_argument('--relay_message_topic', type=str, default='payload/payload/proxy/',
                        help='The topic prefix that the payloads should use for relaying messages to each other')
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
    parser.add_argument('--frequency', type=float, default=0.5,
                        help='The number of times per second that the payload should publish its state')
    parser.add_argument('--sim', action='store_true',
                        help='Set to True to run the payload as a simulated process')
    parser.add_argument('--log', action='store_true',
                        help='Set to True to display all logging information to the console')

    # Parse the arguments
    args = parser.parse_args()

    # Create a new GPS object for the payload to use
    gps = SimGPS(topic="sim/hardware/telem", mqtt_user=None, mqtt_password=None, ca_certs=self.ca_certs) if args.sim else GPS()

    # Create a new BLE Manager for the payload to use
    ble_payload = BLESimPayload(args.payload_uid, host=args.server_host,
                                log=args.log) if args.sim else BLEHardwarePayload(log=args.log)

    if args.sim:
        # for sim have the MQTT broker multiplex the agent side
        topic_prefix = args.payload_uid + '/'

        args.query_uid_response_topic = topic_prefix + args.query_uid_response_topic
        args.query_location_response_topic = topic_prefix + \
            args.query_location_response_topic
        args.in_range_notification_topic = topic_prefix + args.in_range_notification_topic
        args.intent_change_notification_topic = topic_prefix + \
            args.intent_change_notification_topic
        args.capture_notification_topic = topic_prefix + args.capture_notification_topic

        args.query_uid_topic = topic_prefix + args.query_uid_topic
        args.query_location_topic = topic_prefix + args.query_location_topic
        args.send_message_topic = topic_prefix + args.send_message_topic
        args.agent_config_topic = topic_prefix + args.agent_config_topic

    # Create a new payload
    payload = Payload(
        gps,
        ble_payload,
        args.relay_message_topic,
        args.query_uid_response_topic,
        args.query_location_response_topic,
        args.in_range_notification_topic,
        args.intent_change_notification_topic,
        args.capture_notification_topic,
        args.server_config_topic,
        args.identifier,
        args.payload_uid,
        args.log,
        is_sim=args.sim)

    # Connect the payload to the broker
    payload.connect_all_clients(
        args.server_host, args.agent_host, args.server_port, args.agent_port)

    payload.subscribe_to_agent_topics(
        args.query_uid_topic,
        args.query_location_topic,
        args.send_message_topic,
        args.agent_config_topic,
        args.reset_discovered_topic)

    # Start the payload MQTT client
    payload.start_all_clients()

    # Calculate the frequency that publishes should be made
    publish_rate = 1.0 / args.frequency

    # Timestamp used to determine whether to publish the state data
    last_publish = time.perf_counter()

    # Rate to update printed data
    print_rate = 0.5

    # Timestamp used to determine wether to print the console data
    last_print = time.perf_counter()

    # Current state of the running dots
    current_dot = 1

    while(True):
        # Get the current timestamp
        current_time = time.perf_counter()

        if args.log and current_time - last_print >= print_rate:
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

        # Read the latest GPS reading
        payload.read_gps()

        # scan for updated targets
        payload.scan()

        # Publish the data if at the publish interval
        if current_time - last_publish >= publish_rate:
            payload.publish_payload_state(args.state_topic)
            last_publish = time.perf_counter()


if __name__ == '__main__':
    main()
