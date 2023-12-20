#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse
import time
import logging
from target import TargetManager
from gps import GPS
from gps.SimGPS import SimGPS
from ble.BLESimTarget import BLESimTarget
from ble.BLEHardwareTarget import BLEHardwareTarget
from ble.BLETarget import BLETarget


def main():
    # Create a new argument parser
    parser = argparse.ArgumentParser(
        description='System used to configure the device as a payload and to execute payload behavior')

    # Add the desired arguments
    parser.add_argument('--manager_host', type=str, default='127.0.0.1',
                        help='The URL or IP address of the broker that the target manager MQTT client should connect to')
    parser.add_argument('--target_host', type=str, default='127.0.0.1',
                        help='The URL or IP address of the broker that the target MQTT client should connect to')
    parser.add_argument('--manager_port', type=int, default=1883,
                        help='The port number of the MQTT host that the manager client should connect to')
    parser.add_argument('--target_port', type=int, default=1883,
                        help='The port number of the MQTT host that the target client should connect to')
    parser.add_argument('--mqtt_user', type=str, default='target',
                        help='The username the mqtt client should use to authenticate')
    parser.add_argument('--mqtt_password', type=str, default='maceFlag!',
                        help='The password the mqtt client should use to authenticate')
    parser.add_argument('--ca_certs', type=str, default='ca_certificates/ca.crt',
                        help='The CA certs the mqtt client should use to authenticate with the server')
    parser.add_argument('--manager_identifier', type=str, default='MANAGER',
                        help='The identifier that should be used to distinguish the target manager from the targets and payloads')
    parser.add_argument('--target_identifier', type=str, default='TARGET',
                        help='The identifier that should be used to distinguish targets from target managers and payloads')
    parser.add_argument('--manager_uid', type=str, default=None,
                        help='The UID that the target manager should be assigned. This should only be assigned if more than one manager instance is running on a system')
    parser.add_argument('--target_uid', type=str, default=None,
                        help='The UID that the target should be assigned. This should only be assigned if more than one target instance is running on a system')
    parser.add_argument('--config_topic', type=str, default='hardware/server/target/configurations',
                        help='The topic that the target manager should listen to for target configuration changes')
    parser.add_argument('--state_topic', type=str, default='server/hardware/target/state',
                        help='The topic that the target should publish its respective state data to')
    parser.add_argument('--frequency', type=float, default=0.5,
                        help='The number of times per second that the target should publish its state')
    parser.add_argument('--log', action='store_true',
                        help='Set to True to display all logging information to the console')
    parser.add_argument('--sim', action='store_true',
                        help='Set to True to use simulated GPS and BLE events')

    # Parse the arguments
    args = parser.parse_args()

    if args.log:
        logging.basicConfig(level=logging.DEBUG)

    # Create a new GPS object and BLE Manager for the payload to use
    ble_client = None
    if args.sim:
        print("Using simulation stack.")
        gps = SimGPS(host=args.target_host, port=args.target_port,
            mqtt_user=self.mqtt_user, mqtt_password=self.mqtt_password, ca_certs=self.ca_certs)
        ble_client = BLESimTarget(host=args.target_host, port=args.target_port)
    else:
        gps = GPS()
        ble_client = BLEHardwareTarget()

    # Create a new target
    manager = TargetManager(gps, ble_client, args.manager_identifier, args.target_identifier,
                            args.config_topic, args.manager_uid, args.target_uid, args.log, is_sim=args.sim)

    # Connect the manager and the target to the respective host
    manager.connect_clients(
        args.manager_host, args.target_host, args.manager_port, args.target_port,
        args.mqtt_user, args.mqtt_password)

    # Start the payload MQTT client
    manager.start_clients()

    # Calculate the frequency that publishes should be made
    publish_rate = 1.0 / args.frequency

    # Timestamp used to determine whether to publish the state data
    last_publish = time.perf_counter()

    # Rate to update printed data
    print_rate = 0.5

    # Timestamp used to determine wther to print the console data
    last_print = time.perf_counter()

    # Current state of the running dots
    current_dot = 0

    while(True):

        # Get the current timestamp
        current_time = time.perf_counter()

        # if args.log and current_time - last_print >= print_rate:
        #     if current_dot == 0:
        #         print('[Target Manager] Running   ', end='\r')
        #         current_dot += 1
        #     elif current_dot == 1:
        #         print('[Target Manager] Running.  ', end='\r')
        #         current_dot += 1
        #     elif current_dot == 2:
        #         print('[Target Manager] Running.. ', end='\r')
        #         current_dot += 1
        #     elif current_dot == 3:
        #         print('[Target Manager] Running...', end='\r')
        #         current_dot = 0

        #     last_print = current_time

        # Publish the data if at the publish interval
        if current_time - last_publish >= publish_rate:
            # Execute target scanning
            manager.update()

            manager.publish(args.state_topic)
            last_publish = time.perf_counter()


if __name__ == '__main__':
    main()
