#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from uuid import getnode as get_mac_address
from ble.BLETarget import BLETarget
import paho.mqtt.client as mqtt
import sys
import logging
import os


class Target:
    def __init__(self, target_type: str, gps, ble: BLETarget, identifier: str, uid: str=None, log: bool=False, is_sim: bool=False, callsign: str=None, ):

        # Initialize class variables
        # The type of target that the target is currently configured as
        self.target_type = target_type
        # GPS Object that enables capture of the current GPS location
        self.gps = gps

        self.ble_manager = ble              # Bluetooth beacon interaction object
        self.ble_manager.set_target_configuration_type(target_type)

        self.uid = uid                      # Unique target identification number
        self.client = None                  # The MQTT client of the target
        self.logger = logging.getLogger()   # Logger for the class
        self.is_sim = is_sim
        # The number of payloads in the current detection range
        self.payloads_in_range: int = 0

        if self.uid is None:
            # Set the UID for the device
            self.uid = self.__generate_uid(identifier)

        self.callsign = callsign                # The human-readable name of this target
        if self.callsign is None:
            # Set callsign for the device
            self.callsign = self.__generate_callsign()

        self.sim_shutdown_topic = "sim/target/stop/" + str(self.uid)

        if log:
            # Set up the system logger
            self.__setup_logger()

        if is_sim:
            gps.set_target_uid(self.uid)
            self.ble_manager.set_target_uid(self.uid)

        self.ip_address = self.__generate_ip_address()


    def __setup_logger(self):
        """
        Configure the logger to send system logs to the console
        """
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

        return

    def connect_client(self, host, port=1883, mqtt_user=None, mqtt_password=None):
        """
        Connect the client to the desired broker
        """
        pass

    def disconnect_client(self):
        """
        Disconnect the client from the broker
        """
        return

    def start_client(self):
        """
        Start the payload MQTT Client
        """
        return

    def stop_client(self):
        """
        Stop the MQTT client
        """
        pass

    def __on_connect(self, client, userdata, flags, rc):
        """
        Callback function used to log MQTT client connection status
        """
        if rc == 0:
            logging.debug('Successfully connected to the MQTT broker')
            if self.is_sim:
                print("Subscribing to shutdown topic " + self.sim_shutdown_topic)
                self.client.subscribe(self.sim_shutdown_topic)
                self.client.on_message = self.handle_sim_msg
        else:
            logging.debug(
                'Payload was unable to successfully connect to the MQTT client')

        return

    def handle_sim_msg(self, client, userdata, message):
        """
        Callback function responsible for updating the simulated GPS position.
        """
        print("Received message.")
        # Read the message to a JSON element
        topic: str = message.topic
        print("Got msg on topic " + topic)
        json_msg = str(message.payload.decode('utf-8', 'ignore'))
        if topic == self.sim_shutdown_topic and self.is_sim:
            logging.info('Received stop signal over MQTT. Shutting down.')
            sys.exit(0)

    def __generate_uid(self, identifier: str):
        """
        Generate a constant UID based off of the device MAC address and a constant identifier
        """
        # Get system MAC address
        mac_address = str(get_mac_address())
        uid = identifier + '-' + mac_address

        return uid
    
    def __generate_callsign(self):
        """
        Set callsign to the contents of a file called 'callsign' located in /home/<user> (~) or default to the device UID
        """
        callsign = None
        if not self.is_sim:
            try:
                with open(f'{self.get_home_dir()}/callsign','r') as ff:
                    callsign = ff.readline().strip()
                logging.info(f'Target callsign set to {callsign}')

            except Exception as e:
                print(f"Could not access callsign for file at {self.get_home_dir()}/callsign")
                print(e)
        
        # callsign defaults to the device UID
        if not callsign:
            return self.uid
        
        return callsign

    def get_uid(self):
        """
        Getter to retrieve the target unique identification code
        """
        return self.uid

    def get_target_type(self):
        """
        Get the configuration type that the target is currently set to
        """
        return self.target_type

    def read_gps(self):
        """
        Read the current GPS location
        """
        self.gps.read_gps()

        return

    def count_connected_devices(self):
        """
        Return the number of connected payloads
        """
        return self.ble_manager.get_interacting_payloads()

    def _count_payloads_in_range(self):
        """
        Updates the target's payloads_in_range class variable
        """
        payloads = self.count_connected_devices()
        self.payloads_in_range = payloads

        return

    def _get_payloads_in_range(self):
        """
        Getter used to retrieve the number of payloads in range
        """
        return self.payloads_in_range

    def scan(self):
        """
        Public method tou update the capture status of the target
        NOTE: Abstract method
        """
        raise NotImplementedError()

    def advertise(self):
        """
        Advertises the target's current status on BLE
        """
        self.ble_manager.advertise(
            self.uid, self.get_target_type(), self.is_captured())

    def is_captured(self):
        """
        Is this target in a captured state
        NOTE: Abstract method
        """
        raise NotImplementedError()

    def publish_target_state(self):
        """
        Method used to publish the target state information
        Note: This is an abstract method that should be implemented by child classes
        """
        raise NotImplementedError
    
    def get_callsign(self):
        return self.callsign
    
    def set_callsign(self, callsign):
        self.callsign = callsign

    def __generate_ip_address(self):
        os.system(f"hostname -I > {self.get_home_dir()}/ip_address")
        if not self.is_sim:
            try:
                with open(f'{self.get_home_dir()}/ip_address','r') as ff:
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
                print(f"Could not access ip_adress for file at {self.get_home_dir()}/ip_address")
                print(e)  
                return None
        return "sim"
    
    def get_ip_address(self):
        return self.ip_address
    
    # Get home directory (might not always be "pi")
    def get_home_dir(self):
        return os.path.expanduser('~')
    
    
