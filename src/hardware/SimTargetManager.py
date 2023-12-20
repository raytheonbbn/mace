#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse

from ble import BLESimTarget
from gps.SimGPS import SimGPS
import time

from target import TargetManager


class SimTargetManager:
    def __init__(self, num_targets, target_host, target_port, manager_host, manager_port, mqtt_user, mqtt_password, manager_identifier,
                 target_identifier, log, config_topic, target_cycle_frequency, state_topic, ca_certs):
        self.targets = []
        self.num_targets = num_targets
        self.target_host = target_host
        self.target_port = target_port
        self.manager_host = manager_host
        self.manager_port = manager_port
        self.mqtt_user = mqtt_user
        self.mqtt_password = mqtt_password
        self.manager_identifier = manager_identifier
        self.target_identifier = target_identifier
        self.log = log
        self.config_topic = config_topic
        self.target_cycle_frequency = target_cycle_frequency
        self.state_topic = state_topic
        self.ca_certs = ca_certs

        # Setup Targets
        for i in range(num_targets):
            manager_uid = "Target_Manager_" + str(i)
            target_uid = "Target_" + str(i)
            self.targets.append(self.create_target(manager_uid, target_uid))

    def create_target(self, manager_uid, target_uid):
        # Create a new GPS object and BLE Manager for the target to use
        gps_topic = "sim/hardware/position/" + target_uid
        gps = SimGPS(host=self.target_host,
                     port=self.target_port, topic=gps_topic,
                     mqtt_user=self.mqtt_user, mqtt_password=self.mqtt_password)
        ble_client = BLESimTarget(host=self.target_host, port=self.target_port, mqtt_user=self.mqtt_user, mqtt_password=self.mqtt_password)

        # Create a new target
        manager = TargetManager(gps, ble_client, self.manager_identifier, self.target_identifier, self.config_topic, manager_uid, target_uid, self.log, is_sim=True)

        # Connect the manager and the target to the respective host
        connected=False
        while not connected:
            connected = manager.connect_clients(
                self.manager_host, self.target_host, self.manager_port, self.target_port, self.mqtt_user, self.mqtt_password)
            if not connected:
                time.sleep(5)
        # Start the payload MQTT client
        manager.start_clients()
        return manager

    def run(self):
        print("Starting...")
        # Calculate the frequency that publishes should be made
        cycle_rate = 1.0 / self.target_cycle_frequency

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
                    print('[Target Manager] Running   ', end='\r')
                    current_dot += 1
                elif current_dot == 1:
                    print('[Target Manager] Running.  ', end='\r')
                    current_dot += 1
                elif current_dot == 2:
                    print('[Target Manager] Running.. ', end='\r')
                    current_dot += 1
                elif current_dot == 3:
                    print('[Target Manager] Running...', end='\r')
                    current_dot = 0

                last_print = current_time

            # Publish the data if at the publish interval
            if current_time - last_cycle >= cycle_rate:
                for target in self.targets:
                    self.run_cycle(target)
                    last_cycle = time.perf_counter()

    def run_cycle(self, manager):
        # Execute target scanning
        manager.update()

        manager.publish(self.state_topic)


def main():
    parser = argparse.ArgumentParser(
        description='System used to simulate payloads.')

    # Add the desired arguments
    parser.add_argument('--num_targets', type=int, default=2,
                        help='The number of targets to simulate')
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
    parser.add_argument('--config_topic', type=str, default='hardware/server/target/configurations',
                        help='The topic that the target manager should listen to for target configuration changes')
    parser.add_argument('--state_topic', type=str, default='server/hardware/target/state',
                        help='The topic that the target should publish its respective state data to')
    parser.add_argument('--frequency', type=float, default=1,
                        help='The number of times per second to update and publish simulated target states')
    parser.add_argument('--log', action='store_true',
                        help='Set to True to display all logging information to the console')

    # Parse the arguments
    args = parser.parse_args()
    print("Parsed")
    manager = SimTargetManager(args.num_targets, args.target_host, args.target_port, args.manager_host,
                               args.manager_port, args.mqtt_user, args.mqtt_password, args.manager_identifier,
                               args.target_identifier, args.log,
                               args.config_topic, args.frequency, args.state_topic, args.ca_certs)
    manager.run()


if __name__ == '__main__':
    main()
