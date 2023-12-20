#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import sys
sys.path.append('..')
from target import *
from payload import Payload
from agent import Agent


class StompCli:
    def __init__(self, target_config_topic, payload_server_config_topic, payload_agent_config_topic, query_uid_topic,
        query_location_topic, send_agent_message_topic, receive_uid_topic, receive_location_topic, receive_message_topic):

        # Dictionaries containing uid-component pairs
        self.targets = {}                                         # Dictionary containing all registered targets (key: uid, value: target)
        self.payloads = {}                                              # Dictionary containing all registered payloads (key: uid, value: payload)
        self.networks = {}                                              # Dictionary containing all registered networks (key: uid, value: network)
        self.agents = {}                                                # Dictionary containing all registered agents
        self.ports = {}                                                 # Dictionary containing all ports and their associated payload
        
        # Lists responsible for handling the ports
        self.available_ports = []                                       # List of ports that the payloads have opened for agent interaction but are not used
        self.occupied_ports = []                                        # List of ports that are occupied by payloads and their respective agent
        
        # CLI MQTT
        self.client = None                                              # The MQTT client used by the CLI
        self.host = None                                                # The IP Address/URL of the MQTT broker
        
        # Component counts used for MQTT naming
        self.current_target_number = 0                                    # The current number of targets that have been generated
        self.current_payload_number = 0                                 # The current number of payloads that have been generated
        self.current_network_number = 0                                 # The current number of networks that have been generated
        self.current_agent_number = 0                                   # The current number of agents that have been generated
        
        # Topics
        self.payload_server_config_topic = payload_server_config_topic  # Topic that the payload configurations should be sent over
        self.target_config_topic = target_config_topic      # Topic that the target configurations should be sent over
        self.payload_agent_config_topic = payload_agent_config_topic    # Topic that the agents use to send configurations over
        self.send_agent_message_topic = send_agent_message_topic        # Topic that the agents use to send messages to other agents 
        self.query_payload_uid_topic = query_uid_topic                  # Topic that the agents use to query their payload's uid   
        self.query_payload_location_topic = query_location_topic        # Topic that the agents use to query their payload's location
        self.receive_uid_response_topic = receive_uid_topic             # Topic that the agents subscribe to for uid query responses
        self.receive_location_response_topic = receive_location_topic   # Topic that the agents subscribe to for location query responses 
        self.receive_message_topic = receive_message_topic              # Topic that the agents subscribe to to receive messages from other agents

        # Misc
        self.first_display = True                                       # Flag indicating whether this is the first display of the application
        self.DEFAULT_PORT = 1884                                        # Constant that dictates the default port number


    def _isfloat(self, num):
        """
        Determine whether a provided number is a float
        """
        try:
            float(num)
            return True
        except ValueError:
            return False

    
    def _print_info(self, message):
        """
        Helper method used to apply consistent tags to info statements in the CLI
        """
        print('[INFO] ' + message)

        return


    def _print_error(self, message):
        """
        Helper method used to apply consistent tags to error messages in the CLI
        """
        print('[ERROR] ' + message)

        return


    def _get_input(self, message):
        """
        Helper method used to retreive an input with consistent tags
        """

        response = input('[INPUT] ' + message)

        return response


    def _check_exit(self, response):
        """
        Helper method used to determine whether a user is trying to exist the current prompt
        """

        if response == 'exit':
            return True

        return False

    
    def _get_default_port(self):
        """
        Get the default port for payload/clients
        """
        return self.DEFAULT_PORT


    def connect(self, host, uid):
        """
        Connect the client to the desired broker
        """
        # Intialize the class host
        self.host = host

        # Create an MQTT Client
        self.client = mqtt.Client(uid)

        # Connect the client to the mqtt broker
        self.client.connect(host)

        return


    def disconnect_client(self):
        """
        Disconnect the client from the broker
        """
        self.client.disconnect()

        return


    def start_client(self):
        """
        Start the MQTT Client
        """
        self.client.loop_start() 

        return


    def stop_client(self):
        """
        Stop the MQTT client
        """
        self.client.loop_stop()

        return

    def _get_payloads(self):
        """
        Retrieve the payloads dictionary
        """
        return self.payloads

    
    def _get_payload(self, uid):
        """
        Retrieve the payload object from the dictionary
        """
        return self.payloads[uid]

    
    def _get_ports(self):
        """
        Retrieve the dictionary of ports
        """
        return self.ports


    def _get_agents(self):
        """
        Retrieve the agents dictionary
        """
        return self.agents


    def _get_agent(self, uid):
        """
        Retrieve the desired agent
        """
        return self.agents[uid]


    def _get_networks(self):
        """
        Retrieve the networks dictionary
        """
        return self.networks

    
    def _get_targets(self):
        """
        Retrieve the targets dictionary
        """
        return self.targets


    def _get_target(self, uid):
        """
        Retrieve a single target
        """
        return self.targets[uid]


    def _get_payload_config_topic(self):
        """
        Getter used to retrieve the payload configuration topic
        """
        return self.payload_server_config_topic


    def _get_target_config_topic(self):
        """
        Getter used to retrieve the target configuration topic
        """
        return self.target_config_topic

    
    def _add_target(self, target, host, mqtt_id):
        """
        Add a new target to the dictionary of targets
        """
        # Only add a target if the key does not already exist
        if not target.get_uid() in self._get_targets():
            # Connect the new target to the broker
            target.connect(host, mqtt_id)

            # Start the new target
            target.start_client()

            # Add the target
            self.targets[target.get_uid()] = target

        return


    def _overwrite_target(self, uid, target, host, mqtt_id):
        """
        Replace the target value with a new target
        """
        # Stop the existing target
        self.targets[uid].stop_client()
        self.targets[uid].disconnect_client()

        # Connect the new target
        target.connect(host, mqtt_id)
        target.start_client()

        # Overwrite the existing value in the targets dictionary
        self.targets[uid] = target

        return

    
    def _get_next_available_port(self):
        """
        Helper method used to get the next available port number
        """
        if len(self.available_ports) == 0 and len(self.occupied_ports) == 0:
            return self._get_default_port()
        elif len(self.available_ports) == 0:
            return max(self.occupied_ports) + 1
        else:
            return max(self.available_ports) + 1


    def _add_payload(self, payload, host, mqtt_id):
        """
        Add a new payload to the dictionary of payloads
        """
        # Only add a payload if the key does not already exist
        if not payload.get_uid() in self._get_payloads():
            # Connect the payload to the broker
            payload.connect(host, mqtt_id)

            # Start the payload
            payload.start_client()

            # Add the payload to the dictionary
            self.payloads[payload.get_uid()] = payload

            # Determine which port the payload was added on
            port = self._get_next_available_port()

            # Add the new port to the list of available ports
            self._add_available_port(port)

            # Add the port to the dictionary of ports
            self._add_port(port, payload.get_uid())

        return


    def _add_network(self, network, host, mqtt_id):
        """
        Add a new network to the dictionary of payloads
        """
        # Only add a network if the network does not already exist
        if not network.get_uid() in self._get_networks():
            # Start the network
            network.connect(host, mqtt_id)
            network.start_client()

            # Add the network to the dictionary
            self.networks[network.get_uid()] = network

        return


    def _get_network(self, uid):
        """
        Retrieve the desired network from the dictionary of networks
        """
        return self.networks[uid]


    def _delete_network(self, uid):
        """
        Delete a network from the network dictionary
        """
        self.networks.pop(uid)

        return

    
    def subscribe_to_state(self, target_state_topic, payload_state_topic, target_state_qos=1, payload_state_qos=1):
        """
        Subscribe to the topics that the cli should listen to for state changes
        """
        # Define a list of (topic, qos) tuples for the client to subscribe to
        topics = [(target_state_topic, target_state_qos), (payload_state_topic, payload_state_qos)]

        # Subscribe to the list of topics
        self.client.subscribe(topics)

        # Attach the desired callbacks to each function
        self.client.message_callback_add(target_state_topic, self._read_target_state_cb)
        self.client.message_callback_add(payload_state_topic, self._read_payload_state_cb)

        return

    
    def _get_host(self):
        """
        Get the host URL/IP
        """
        return self.host


    def _get_current_target_number(self):
        """
        Getter used to enable unique mqtt identifiers to be assigned to the target clients
        """
        return self.current_target_number


    def _increment_target_number(self):
        """
        Increment the number of targets that have been registered
        This is used to enable unique client ids to be assigned to the target mqtt clients
        """
        self.current_target_number += 1

        return

    
    def _generate_target_mqtt_id(self, increment=True):
        """
        Generate a new uid for the local target uid
        This is done because the target's existing uid is already used as the mqtt uid by the hardware
        itself and cannot be replicated
        """
        # Create the new target mqtt id
        name = 'CLI-TARGET-' + str(self._get_current_target_number())

        # If the target number should be incremented, do so
        if increment:
            self._increment_target_number()

        return name


    def _get_current_payload_number(self):
        """
        Getter used to enable unique mqtt identifiers to be assigned to the payload clients
        """
        return self.current_payload_number


    def _increment_payload_number(self):
        """
        Increment the number of payloads that have been registered
        This is used to enable unique client ids to be assigned to the payload mqtt clients
        """
        self.current_payload_number += 1

        return

    
    def _generate_payload_mqtt_id(self, increment=True):
        """
        Generate a new uid for the local payload uid
        This is done because the payload's existing uid is already used as the mqtt uid by the hardware
        itself and cannot be replicated
        """
        # Create the new payload mqtt id
        name = 'CLI-PAYLOAD-' + str(self._get_current_payload_number())

        # If the payload number should be incremented, do so
        if increment:
            self._increment_payload_number()

        return name


    def _get_current_network_number(self):
        """
        Getter used to enable unique mqtt identifiers to be assigned to the network clients
        """
        return self.current_network_number


    def _increment_network_number(self):
        """
        Increment the number of networks that have been registered
        This is used to enable unique client ids to be assigned to the network mqtt clients
        """
        self.current_network_number += 1

        return

    
    def _generate_network_mqtt_id(self, increment=True):
        """
        Generate a new uid for the local network uid
        This is done because the network's existing uid is already used as the mqtt uid by the analytics server
        itself and cannot be replicated
        """
        # Create the new network mqtt id
        name = 'CLI-NETWORK-' + str(self._get_current_network_number())

        # If the network number should be incremented, do so
        if increment:
            self._increment_network_number()

        return name


    def _get_current_agent_number(self):
        """
        Getter used to enable unique mqtt identifiers to be assigned to the agent clients
        """
        return self.current_agent_number


    def _increment_agent_number(self):
        """
        Increment the number of agents that have been registered
        This is used to enable unique client ids to be assigned to the agent mqtt clients
        """
        self.current_agent_number += 1

        return

    
    def _generate_agent_mqtt_id(self, increment=True):
        """
        Generate a new uid for the agent uid
        """
        # Create the new agent mqtt id
        name = 'CLI-AGENT-' + str(self._get_current_agent_number())

        # If the agent number should be incremented, do so
        if increment:
            self._increment_agent_number()

        return name


    def _get_agent_config_topic(self):
        """
        Getter used to retrieve the agent configuration topic
        """
        return self.payload_agent_config_topic


    def _get_send_agent_message_topic(self):
        """
        Getter used to retrieve the topic that agents should use to send other agents messages
        """
        return self.send_agent_message_topic


    def _get_query_payload_uid_topic(self):
        """
        Getter used to retrieve the topic that agents should use to query their payload's UID
        """
        return self.query_payload_uid_topic


    def _get_query_payload_location_topic(self):
        """
        Getter used to retrieve the topic that agents should use to query their payload's location
        """
        return self.query_payload_location_topic


    def _get_receive_uid_response_topic(self):
        """
        Getter used to retrieve the topic that agents should used to receive UID query responses over
        """
        return self.receive_uid_response_topic


    def _get_receive_location_response_topic(self):
        """
        Getter used to retrieve the topic that agents should use to receive location query responses over
        """
        return self.receive_location_response_topic


    def _get_receive_message_topic(self):
        """
        Getter used to retrieve the topic that agents should use to receive messages from other agents over
        """
        return self.receive_message_topic


    def _add_port(self, port, payload_uid):
        """
        Add an entry into the ports dictionary
        """
        if not port in self._get_ports():
            self.ports[port] = payload_uid

        return


    def _get_port_payload(self, port):
        """
        Get the paylaod associated with a given port
        """
        return self.ports[port]


    def _get_available_ports(self):
        """
        Getter used to retrieve the list of available ports
        """
        return self.available_ports


    def _remove_available_port(self, port):
        """
        Helper method used to remove a port from the list of available ports
        """
        self.available_ports.remove(port)

        return


    def _check_if_port_is_available(self, port):
        """
        Helper method used to determine if a given port is available
        """
        return port in self._get_available_ports()


    def _add_available_port(self, port):
        """
        Helper method used to add a new available port
        NOTE: Does not add the port if it is already an occupied or available port
        """
        if not self._check_if_port_is_occupied(port) and not self._check_if_port_is_available(port):
            self.available_ports.append(port)

        return

    
    def _get_occupied_ports(self):
        """
        Getter used to retrieve the list of occupied ports
        """
        return self.occupied_ports


    def _remove_occupied_port(self, port):
        """
        Helper method used to remove an occupied port
        """
        self.occupied_ports.remove(port)

        return


    def _check_if_port_is_occupied(self, port):
        """
        Helper method used to determine if a port is occupied
        """
        return port in self._get_occupied_ports()

    
    def _add_occupied_port(self, port):
        """
        Helper method used to add a new occupied port
        NOTE: Does not add the port if already an occupied or available port
        """
        if not self._check_if_port_is_available(port) and not self._check_if_port_is_occupied(port):
            self.occupied_ports.append(port)

        return

    
    def _add_agent(self, agent, host, mqtt_id, uid_response_topic, location_response_topic, receive_message_topic):
        """
        Add a new network to the dictionary of payloads
        """
        # Only add a network if the network does not already exist
        if not agent.get_uid() in self._get_agents():
            # Start the network
            agent.connect(host, mqtt_id)
            agent.start_client()

            # Subscribe to the API topics
            agent.subscribe_to_api_topics(uid_response_topic, location_response_topic, receive_message_topic)

            # Add the network to the dictionary
            self.agents[agent.get_uid()] = agent

        return


    def _set_network_capture_state(self, target_uid, captured):
        """
        Find the network that a target is in and set its capture state
        """
        # Iterate through the networks in the dictionary
        for network in self._get_networks().items():
            # Reset the capture state of the network if the target is part of the network
            if target_uid in network:
                network.set_network_captured(captured)

                # Stop the search
                break

        return

    
    def _update_target_state(self, state):
        """
        Update the desired target's state
        """
        try:
            # Reset each of the object's state properties according to the message
            if state['type'] == 'IDLE':
                self.targets[state['uid']].update_state(state['latitude'], state['longitude'], state['altitude'])
            if state['type'] == 'MASS':
                self.targets[state['uid']].update_state(state['latitude'], state['longitude'], state['altitude'], state['payloads'], state['captured'])
            elif state['type'] == 'PERI':
                self.targets[state['uid']].update_state(state['latitude'], state['longitude'], state['altitude'], state['payloads'], state['current_duration'], state['captured'])
            elif state['type'] == 'LINK':
                self.targets[state['uid']].update_state(state['latitude'], state['longitude'], state['altitude'], state['payloads'], state['captured'], state['network_captured'])

                # Update the respective network state
                self._set_network_capture_state(state['uid'], state['network_captured'])
        except TypeError:
            # A type error will occur if we are attempting to modify configurations for a target and receive a state before the configs have been sent
            pass

        return


    def _read_target_state_cb(self, client, userdata, message):
        """
        Callback function used to read a target state message provided by the analytics server
        """
        # Read the message to a JSON element
        json_state = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a dictionary
        state = json.loads(json_state)

        # Check if the target already exists
        if state['uid'] not in self._get_targets():
            if state['type'] == 'IDLE':
                target = IdleTarget(state['uid'])
            elif state['type'] == 'MASS':
                target = MassTarget(state['uid'])
            elif state['type'] == 'PERI':
                target = PeriodicTarget(state['uid'])
            elif state['type'] == 'LINK':
                target = LinkedTarget(state['uid'])

            # Create a new target and add it to the dictionary of targets if the target doesn't already exist
            self._add_target(target, self._get_host(), self._generate_target_mqtt_id())

        # Update the state of the target according to the rest of the message parameters
        self._update_target_state(state)

        return


    def _read_payload_state_cb(self, client, userdata, message):
        """
        Callback function used to read a state payload state message provided by the analytics server
        """
        # Read the message to a JSON element
        json_state = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        state = json.loads(json_state)

        if state['uid'] not in self._get_payloads():
            self._add_payload(Payload(state['uid']), self._get_host(), self._generate_payload_mqtt_id())

        # Update the payload state
        self.payloads[state['uid']].update_state(state['latitude'], state['longitude'], state['altitude'], state['in_range'], state['type_in_range'],
            state['modify_intent'], state['intent'])

        return


    def _print_line(self):
        """
        Print a separator line
        """
        print('----------------------------------------------------------------------')

        return


    def _print_dict_keys(self, dict_to_print):
        """
        Helper method used to print out the keys in a dictionary
        """
        i = 1

        for key in dict_to_print:
            print(f'{i}. {key}')
            i += 1

        print('\n')

        return


    def _print_components_list(self):
        """
        Print the existing list of registered hardware devices
        """
        # Print out the header
        self._print_line()
        print('|                            COMPONENTS                              |')
        self._print_line()

        # Print out the targets
        print('Targets:')
        self._print_dict_keys(self._get_targets())

        # Print out the linked target networks
        print('Networks:')
        self._print_dict_keys(self._get_networks())

        # Print out the registered payloads
        print('Payloads:')
        self._print_dict_keys(self._get_payloads())

        # Print out the registered agents
        print('Agents:')
        self._print_dict_keys(self._get_agents())

        return


    def _print_menu(self):
        """
        Print out the menu of commands that a user may provide as input
        """
        # Print header
        self._print_line()
        print('|                               MENU                                 |')
        self._print_line()

        # Print menu items
        print('1. Display Menu')
        print('2. Display Components List')
        print('3. Display Target State')
        print('4. Display Network State')
        print('5. Display Payload State')
        print('6. Display Agent State')
        print('7. Modify Target Configurations')
        print('8. Enter Network Configuration Menu')
        print('9. Modify Payload Configurations')
        print('10. Enter Agent Menu')
        print('[NOTE] Enter \"exit\" at any point to exit the current prompt')

        return


    def _get_user_input(self):
        """
        Get the desired menu commands from the user and execute the respective command
        """
        # Get the initial input
        self._print_line()
        value = self._get_input('Enter your Menu Selection: ')

        # Ensure that the input is a digit
        if not value.isdigit():
            self._print_error('Invalid Menu Item selected. Valid selections include items 1-10.')
            return

        # Cast the number to an int
        value = int(value)

        # Validate that the menu selection is a valid option
        if value > 10 or value < 1:
            self._print_error('Invalid Menu Item selected. Valid selections include items 1-10.')
            return

        # Display the menu
        if value == 1:
            self._print_menu()

        # Display the hardware list
        elif value == 2:
            # Reprint out the hardware display and menu
            self._print_components_list()

        # Display target state
        elif value == 3:
            self._print_target_state()

        # Display network staste
        elif value == 4:
            self._print_network_state()

        # Display payload state
        elif value == 5:
            self._print_payload_state()

        # Display agent state
        elif value == 6:
            self._print_agent_state()
        
        # Configure a target
        elif value == 7:   
            self._read_target_configurations()

        # Configure a network
        elif value == 8:
            self._print_network_modification_menu()
            continue_network_config = True

            # Read and publish the configurations
            while continue_network_config:
                continue_network_config = self._read_network_user_input()
                
        # Configure a payload
        elif value == 9:
            self._read_payload_server_configurations()

        # Manage an Agent
        elif value == 10:
            self._print_agent_menu()
            continue_agent_interaction = True

            while continue_agent_interaction:
                continue_agent_interaction = self._read_agent_user_input()

        return


    def _print_target_state(self):
        """
        Print the state of the target
        """
        # Make sure that there is a target whose state can be checked
        if len(self._get_targets()) == 0:
            self._print_error('There are no registered targets to display')
            return

        # Get the uid of the target whose state will be accessed
        uid = self._get_input('Enter the UID of the target whose state you wish to display: ')

        if self._check_exit(uid):
            return

        # Ensure that a valid uid is provided
        while uid not in self._get_targets():
            self._print_error('The target provided is not a registered target. Please enter a valid target UID')
            uid = self._get_input('Enter the UID of the target whose state you wish to display: ')

            if self._check_exit(uid):
                return

        # Print the state header
        self._print_line()
        print('|                       TARGET STATE                           |')
        self._print_line()

        # Print the target state
        self._get_target(uid).print_state()

        return


    def _print_network_state(self):
        """
        Print the state of the network
        """
        # Make sure that there is a network whose state can be accessed
        if len(self._get_networks()) == 0:
            self._print_error('There are no registered networks to display')
            return

        # Get the uid of the network
        uid = self._get_input('Enter the UID of the network whose state you wish to display: ')

        if self._check_exit(uid):
            return

        # Ensure that a valid network uid is provided
        while uid not in self._get_networks():
            self._print_error('The network provided is not a registered network. Please enter a valid UID')
            uid = self._get_input('Enter the UID of the network whose state you wish to display: ')
            
            if self._check_exit(uid):
                return

        # Print the state header
        self._print_line()
        print('|                          NETWORK STATE                             |')
        self._print_line()

        self._get_network(uid).print_state()

        return


    def _print_payload_state(self):
        """
        Print the state of the payload
        """
        # Ensure that there are payloads to access
        if len(self._get_payloads()) == 0:
            self._print_error('There are no registered payloads to display')
            return

        # Get the payload uid whose state will be displayed
        uid = self._get_input('Enter the UID of the Payload whose state you wish to display: ')

        if self._check_exit(uid):
            return

        # Ensure that a valid payload is provided
        while uid not in self._get_payloads():
            self._print_error('The payload provided is not a registered payload. Please enter a valid payload UID')
            uid = self._get_input('Enter the UID of the payload whose state you wish to display: ')

            if self._check_exit(uid):
                return

        # Print the state header
        self._print_line()
        print('|                          PAYLOAD STATE                             |')
        self._print_line()

        self._get_payload(uid).print_state()

        return


    def _print_agent_state(self):
        """
        Print the state of the payload
        """
        # Ensure that there are agents to access
        if len(self._get_agents()) == 0:
            self._print_error('There are no registered agents to display')
            return

        # Get the agent uid whose state will be displayed
        uid = self._get_input('Enter the UID of the agent whose state you wish to display: ')

        if self._check_exit(uid):
            return

        # Ensure that a valid payload is provided
        while uid not in self._get_agents():
            self._print_error('The agent provided is not a registered agent. Please enter a valid agent UID')
            uid = self._get_input('Enter the UID of the agent whose state you wish to display: ')

            if self._check_exit(uid):
                return

        # Print the state header
        self._print_line()
        print('|                           AGENT STATE                              |')
        self._print_line()

        self._get_agent(uid).print_state()

        return


    def _read_target_configurations(self):
        """
        Get the desired configurations for the target from the user
        """
        # List containing the configurable target types
        valid_types = ['IDLE', 'MASS', 'PERI']

        # Ensure that there are targets that can be configured         
        if len(self._get_targets()) == 0:
            self._print_error('There are no registered targets to configure')
            return

        # Get the target uid to configure
        uid = self._get_input('Enter the UID of the target to configure: ')

        if self._check_exit(uid):
            return

        # Ensure that the uid is valid
        while uid not in self._get_targets():
            self._print_error('The target provided is not a registered target. Please enter a valid target UID')
            uid = self._get_input('Enter the UID of the target to configure: ')

            if self._check_exit(uid):
                return

        # Determine whether the target type should be modified
        modify_target_type = self._get_input('Would you like to modify the target type? (y/n): ')

        if self._check_exit(modify_target_type):
            return

        # Modify the target type
        if modify_target_type == 'y':

            if self._get_target(uid).get_target_type() == 'LINK':
                self._print_error('The provided target is currently configured as a LINK target')
                remove = self._get_input('Would you like to remove the target from its network and proceed? (y/n): ')

                if remove == 'y':
                    self._find_and_remove_target_from_network(uid)
                else:
                    self._print_error('Cannot change the configuration type of a target that is currently included in a network')
                    return

            # Get the desired configuration type
            modified_type = self._get_input('Enter the desired target configuration type (IDLE, MASS, PERI): ')

            if self._check_exit(modified_type):
                return

            # Ensure that a valid configuration type was provided
            while modified_type not in valid_types:
                self._print_error('Invalid type provided. Please enter a valid target type (IDLE, MASS, PERI)')
                modified_type = self._get_input('Enter the desired target configuration type (IDLE, MASS, PERI): ')
                    
                if self._check_exit(modified_type):
                    return

            # Convert the target to the desired type 
            if modified_type != self._get_target(uid).get_target_type():
                if modified_type == 'IDLE':
                    self._overwrite_target(uid, IdleTarget(uid), self._get_host(), self._generate_target_mqtt_id())
                elif modified_type == 'MASS':
                    self._overwrite_target(uid, MassTarget(uid), self._get_host(), self._generate_target_mqtt_id())
                elif modified_type == 'PERI':
                    self._overwrite_target(uid, PeriodicTarget(uid), self._get_host(), self._generate_target_mqtt_id())

        # Get the current target type
        target_type = self._get_target(uid).get_target_type()

        # Prevent modification of linked targets
        if target_type == 'LINK':
            self._print_error('Cannot modify the configurations of a linked target directly. To modify the configurations, modify the network or change the target type.')
            return

        # Read the configuration type specific configurations
        if target_type == 'IDLE':
            self._print_info('There are no additional configurations to modify for an idle target')
        elif target_type == 'MASS':
            self._read_mass_target_configurations(uid)
        elif target_type == 'PERI':
            self._read_periodic_target_configurations(uid)

        self._print_info('Publishing the target configurations')

        # Publish the target configurations to the analytics server
        self._get_target(uid).publish_configs(self._get_target_config_topic())

        return


    def _read_mass_target_configurations(self, uid):
        """
        Read the configurations that a user desires to set for a mass target
        """
        # Determine whether the user would like to reset the detection range
        reset_detection_range = self._get_input('Would you like to configure the detection range? (y/n): ')

        if self._check_exit(reset_detection_range):
            return
            
        # Reset the detection range if desired
        if reset_detection_range == 'y':
            # Get the desired detection range
            detection_range = self._get_input('Enter the desired detection range for the target: ')

            if self._check_exit(detection_range):
                return

            # Ensure that the user provides a valid detection range
            while not self._isfloat(detection_range) and not detection_range.isdigit():
                self._print_error('An invalid entry was provided for the detection range. Please provide a numerical range.')
                detection_range = self._get_input('Enter the desired detection range for the target: ')

                if self._check_exit(detection_range):
                    return

            # Set the detection range
            self.targets[uid].set_detection_range(float(detection_range))

        # Determine whether the user would like to reset the number of required payloads for capture
        reset_required_payloads = self._get_input('Would you like to configure the number of required payloads? (y/n): ')

        if self._check_exit(reset_required_payloads):
            return

        # Reset the number of required paylaods for capture
        if reset_required_payloads == 'y':
            # Get the number of required payloads
            required_payloads = self._get_input('Enter the desired number of required payloads: ')

            if self._check_exit(required_payloads):
                return

            # Ensure that the user provides a valid number of required payloads
            while not required_payloads.isdigit():
                self._print_error('An invalid entry was provided for the number of required payloads. Please provide an integer.')
                required_payloads = self._get_input('Enter the desired number of required payloads: ')

                if self._check_exit(required_payloads):
                    return

            # Set the number of required payloads
            self.targets[uid].set_required_payloads(int(required_payloads))

        # Determine whether the user would like to reset the capture state
        reset_capture_state = self._get_input('Would you like to manually configure the capture state? (y/n): ')

        if self._check_exit(reset_capture_state):
            return
            
        # Reset the capture state
        if reset_capture_state == 'y':
            # Set the modify capture state flag to true
            self.targets[uid].set_modify_capture_state(True)

            # Get the desired capture state from the user
            capture_state = self._get_input('Enter the desired capture state: (captured/uncaptured): ')

            if self._check_exit(capture_state):
                return

            # Ensure that a valid capture state is provided
            while capture_state != 'captured' and capture_state != 'uncaptured':
                self._print_error('An invalid capture state was provided. Please enter either: captured or uncaptured')
                capture_state = self._get_input('Enter the desired capture state: (captured/uncaptured): ')

                if self._check_exit(capture_state):
                    return

            # Set the capture state
            if capture_state == 'captured':
                self.targets[uid].set_captured(True)
            else:
                self.targets[uid].set_captured(False)
        else:
            self.targets[uid].set_modify_capture_state(False)

        return


    def _read_periodic_target_configurations(self, uid):
        """
        Read the desired configurations for a periodic target from a user
        """
        # Determine whether the detection range should be modified
        reset_detection_range = self._get_input('Would you like to configure the detection range? (y/n): ')

        if self._check_exit(reset_detection_range):
            return
            
        # Update the detection range
        if reset_detection_range == 'y':
            # Get the desired detection range
            detection_range = self._get_input('Enter the desired detection range for the target: ')

            if self._check_exit(detection_range):
                return

            # Ensure that a valid detection range was provided
            while not self._isfloat(detection_range) and not detection_range.isdigit():
                self._print_error('An invalid entry was provided for the detection range. Please provide a numerical range.')
                detection_range = self._get_input('Enter the desired detection range for the target: ')

                if self._check_exit(detection_range):
                    return

            # Set the detection range
            self.targets[uid].set_detection_range(float(detection_range))

        # Determine whether to reset the required number of payloads
        reset_required_payloads = self._get_input('Would you like to configure the number of required payloads? (y/n): ')

        if self._check_exit(reset_required_payloads):
            return

        # Reset the required number of payloads
        if reset_required_payloads == 'y':
            # Get the number of required payloads to be configured
            required_payloads = self._get_input('Enter the desired number of required payloads: ')

            if self._check_exit(required_payloads):
                return

            # Ensure that a valid input was provided
            while not required_payloads.isdigit():
                self._print_error('An invalid entry was provided for the number of required payloads. Please provide an integer.')
                required_payloads = self._get_input('Enter the desired number of required payloads: ')

                if self._check_exit(required_payloads):
                    return

            # Set the required number of payloads
            self.targets[uid].set_required_payloads(int(required_payloads))

        # Determine whether to reset the countdown timer
        reset_capture_countdown = self._get_input('Would you like to reset the countdown timer? (y/n): ')

        if self._check_exit(reset_capture_countdown):
            return

        # Reset the countdown timer
        if reset_capture_countdown == 'y':
            # Get the desired timer duration
            countdown = self._get_input('Enter the desired countdown timer: ')

            if self._check_exit(countdown):
                return

            # Ensure that a valid duration is provided
            while not self._isfloat(countdown) and not countdown.isdigit():
                self._print_error('Please enter a valid numerical countdown duration.')
                countdown = self._get_input('Enter the desired countdown timer: ')

                if self._check_exit(countdown):
                    return

            # Set the timer duration
            self.targets[uid].set_capture_countdown(float(countdown))

        # Determine whether to reset the capture state
        reset_capture_state = self._get_input('Would you like to manually configure the capture state? (y/n): ')

        if self._check_exit(reset_capture_state):
            return
            
        # Reset the capture state
        if reset_capture_state == 'y':
            # Set the modify capture state flag
            self.targets[uid].set_modify_capture_state(True)

            # Read the desired capture state
            capture_state = self._get_input('Enter the desired capture state: (captured/uncaptured): ')

            if self._check_exit(capture_state):
                return

            # Ensure that a desired capture state is provided
            while capture_state != 'captured' and capture_state != 'uncaptured':
                self._print_error('An invalid capture state was provided. Please enter either: captured or uncaptured')
                capture_state = self._get_input('Enter the desired capture state: (captured/uncaptured): ')

                if self._check_exit(capture_state):
                    return

            # Set the capture state
            if capture_state == 'captured':
                self.targets[uid].set_captured(True)
            else:
                self.targets[uid].set_captured(False)
        else:
            self.targets[uid].set_modify_capture_state(False)

        return


    def _print_network_modification_menu(self):
        """
        Helper method used to print the network configuration menu
        """
        self._print_line()
        print('|                    NETWORK MODIFICATION MENU                       |')
        self._print_line()
        print('1. Display Network Modification Menu')
        print('2. Create a new linked network')
        print('3. Configure an existing linked network\'s settings')
        print('4. Rename a network')
        print('5. Delete a network')
        print('6. Add a target to an existing linked network')
        print('7. Remove a target from an existing network')
        print('8. Exit Network Modification Menu')

        self._print_line()

        return


    def _read_network_user_input(self):
        """
        Read the configurations for a linked target network
        """
        # Get the user's menu item selection
        menu_selection = self._get_input('Enter a Network Modification Menu Item: ')

        # Ensure that the input is a digit
        while not menu_selection.isdigit():
            self._print_error('Invalid Network Menu Item selected. Valid selections include items 1-8.')
            self._print_line()
            menu_selection = self._get_input('Enter a Network Modification Menu Item: ')

        # Cast the number to an int
        menu_selection = int(menu_selection)

        # Validate that the menu selection is a valid option
        while menu_selection > 8 or menu_selection < 1:
            self._print_error('Invalid Network Menu Item selected. Valid selections include items 1-8.')
            self._print_line()
            menu_selection = int(self._get_input('Enter a Network Modification Menu Item: '))
        
        # Display the menu
        if menu_selection == 1:
            self._print_network_modification_menu()

        # Create a new linked network
        elif menu_selection == 2:
            self._create_new_linked_network()

        # Configure an existing network
        elif menu_selection == 3:
            self._read_network_configurations()

        # Rename a network
        elif menu_selection == 4:
            self._read_new_network_name()

        # Delete a network       
        elif menu_selection == 5:
            self._read_network_deletion()

        # Add a target to the network
        elif menu_selection == 6:
            self._read_new_targets_to_add_to_network()

        # Remove targets from a network
        elif menu_selection == 7:
            self._read_targets_to_delete_from_network()

        # Exit the network modification menu
        elif menu_selection == 8:
            self._print_info('Exiting Network Modification Menu')
            self._print_menu()
            return False

        return True


    def _create_new_linked_network(self):
        """
        Helper method used to create new linked target networks
        """
        # Ensure that there are targets that can be added to a network
        if len(self._get_targets()) == 0:
            self._print_error('There are no registered targets to add to a network')
            return

        # Get the uid of the network
        uid = self._get_input('Enter the UID of the network you wish to create: ')

        if self._check_exit(uid):
            return

        # Prevent existing UIDs from being used to create networks
        while uid in self._get_networks():
            self._print_error('The provided network UID already exists in the list of networks. Please enter a unique network UID.')
            uid = self._get_input('Enter the UID of the network you wish to create: ')

            if self._check_exit(uid):
                return

        # List used to store the network targets
        network_targets = []

        # Get the UID of the first target that should be used to create the network
        first_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

        if self._check_exit(first_target_uid):
            return

        # Ensure that the provided target UID is a valid target
        while first_target_uid not in self._get_targets():
            print(f'[ERROR] The provided target UID ({first_target_uid}) is not a valid target. Please enter a valid UID.')
            first_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

            if self._check_exit(first_target_uid):
                return

        # Determine if the target is a linked target already (it belongs to a network)
        if self._get_target(first_target_uid).get_target_type() == 'LINK':

            # Determine whether the user would like to remove the target from the existing network
            remove = self._get_input('The provided target belongs to another existing network. Would you like to remove the target from the other network? (y/n): ')

            if self._check_exit(remove):
                return

            # Remove the target from the existing network
            if remove == 'y':
                # Find and remove the target from the existing network
                self._find_and_remove_target_from_network(first_target_uid)

                # Add the target to the new networks targets list
                network_targets.append(self._get_target(first_target_uid))
            else:
                print('The target will not be added to the linked target network.')
        else:
            # Adjust the target type to be a linked target
            first_target = LinkedTarget(first_target_uid)
            network_targets.append(first_target)
            self._overwrite_target(first_target_uid, first_target, self._get_host(), self._generate_target_mqtt_id())

        # Flag indicating whether to continue adding targets
        keep_adding_targets = True

        # Add targets until the user decides to stop adding targets or there are no more targets to add
        while keep_adding_targets and len(network_targets) < len(self._get_targets()):

            # Determine whether the user wants to add another target to the network
            add_target = self._get_input('Would you like to add another target? (y/n): ')

            if self._check_exit(add_target):
                return

            # Add another target to the network
            if add_target == 'y':
                # Get the UID of the target to add
                new_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

                if self._check_exit(new_target_uid):
                    return

                # Ensure that the provided input is valid
                while new_target_uid not in self._get_targets():
                    print(f'[ERROR] The provided target UID ({new_target_uid}) is not a valid target. Please enter a valid UID.')
                    new_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

                    if self._check_exit(new_target_uid):
                        return

                # Ensure that the target isn't already in the network
                while new_target_uid in network_targets:
                    print(f'[ERROR] The provided target UID ({new_target_uid} already exists in the network. Please provide a different UID.')    
                    new_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ') 

                    if self._check_exit(new_target_uid):
                        return

                # Determine whether te provided target is a linked target
                if self._get_target(new_target_uid).get_target_type() == 'LINK':
                    # Determine whether the user wants to remove the target from the existing network and add it to the current
                    remove = self._get_input('The provided target belongs to another existing network. Would you like to remove the target from the other network? (y/n): ')

                    if self._check_exit(remove):
                        return

                    # Remove the target from the existing network
                    if remove == 'y':
                        self._find_and_remove_target_from_network(new_target_uid)
                        network_targets.append(self._get_target(new_target_uid))
                    else:
                        print('The target will not be added to the linked target.')
                else:
                    # Convert the target to a linked target
                    new_target = LinkedTarget(new_target_uid)

                    # Add the target to the network
                    network_targets.append(new_target)

                    # Overwrite the existing target
                    self._overwrite_target(new_target_uid, new_target, self._get_host(), self._generate_target_mqtt_id())
            else:
                # Make sure that the user provided at least one target
                if len(network_targets) == 0:
                    self._print_error('At least one target must be used to create a linked network. Please restart the network creation process.')
                    return

                # Break
                keep_adding_targets = False

        # Get the detection range for the network
        detection_range = self._get_input('Enter the detection range that the targets within the network should apply: ')

        if self._check_exit(detection_range):
            return

        # Ensure that the provided detection range is a valid number
        while not self._isfloat(detection_range) and not detection_range.isdigit():
            self._print_error('An invalid detection range was provided. Please provide a numeric detection range.')
            detection_range = self._get_input('Enter the detection range that the targets within the network should apply: ')
            
            if self._check_exit(detection_range):
                return

        # Update the detection range of the target in the UI
        for target in network_targets:
            self.targets[target.get_uid()].set_detection_range(detection_range)

        # Create a new linked target network using the configurations
        new_network = LinkedNetwork(uid, float(detection_range), network_targets)

        # Add the network to the dictionary of networks
        self._add_network(new_network, self._get_host(), self._generate_network_mqtt_id())

        # Publish the configurations to the analytics server
        new_network.publish_configs(self._get_target_config_topic())

        self._print_info(f'Successfully created a new linked network: {uid}')

        return


    def _find_and_remove_target_from_network(self, target):
        """
        Helper method responsible for finding a target in a linked network and removing it
        """
        for network_uid in self._get_networks():
            if target in self._get_network(network_uid).get_network_uids():
                self._print_info(f'Found the target in {network_uid}. Removing the target from the network.')
                self.networks[network_uid].remove_target_from_network(self._get_target(target))
                self.networks[network_uid].publish_configs(self._get_target_config_topic())

        return

    
    def _read_network_configurations(self):
        """
        Read the configurations for an existing network from the user
        """
        # Make sure that there is a network whose state can be accessed
        if len(self._get_networks()) == 0:
            self._print_error('There are no registered networks to reconfigure')
            return

        # Get the uid of the network
        configured_network = self._get_input('Enter the UID of the network you wish to reconfigure: ')

        if self._check_exit(configured_network):
            return

        # Ensure that a valid network uid is provided
        while configured_network not in self._get_networks():
            self._print_error('The network provided is not a registered network. Please enter a valid UID')
            configured_network = self._get_input('Enter the UID of the network whose state you wish to reconfigure: ')

            if self._check_exit(configured_network):
                return

        # Determine whether the user wants to reset the detection range of the network
        reset_detection_range = self._get_input('Would you like to reset the detection range? (y/n): ')

        if self._check_exit(reset_detection_range):
            return

        # Reset the detection range
        if reset_detection_range == 'y':
            # Get the desired detection range
            new_detection_range = self._get_input('Enter the detection range that you wish to establish for the network: ')

            if self._check_exit(new_detection_range):
                return

            # Ensure that the provided detection range is a valid range
            while not self._isfloat(new_detection_range) and not new_detection_range.isdigit():
                self._print_error('An invalid detection range was provided. Please enter a numeric detection range')
                new_detection_range = self._get_input('Enter the detection range that you wish to establish for the network: ')

                if self._check_exit(new_detection_range):
                    return

            # Set the network's detection range
            self.networks[configured_network].set_detection_range(float(new_detection_range))

            # Update the detection range of the target in the UI
            for target in self.networks[configured_network].get_network():
                self.targets[target].set_detection_range(new_detection_range)

        # Determine whether the user wantes to modify the capture state
        reset_capture_state = self._get_input('Would you like to modify the capture state of the network? (y/n): ')

        if self._check_exit(reset_capture_state):
            return

        # Modify the capture state
        if reset_capture_state == 'y':
            # Update the modify capture state flag
            self.networks[configured_network].set_modify_capture_state(True)

            # Get the desired capture state
            capture_state = self._get_input('Enter the new capture state for the linked network (captured/uncaptured): ')

            if self._check_exit(capture_state):
                return

            # Ensure that a valid capture state has been provided
            while capture_state != 'captured' and capture_state != 'uncaptured':
                self._print_error('An invalid capture state was provided. Please enter a valid capture state for the network.')
                capture_state = self._get_input('Enter the new capture state for the linked network (captured/uncaptured): ')

                if self._check_exit(capture_state):
                    return

            # Determine whether to set the network to captured or uncaptured
            if capture_state == 'captured':
                capture_bool = True
            else:
                capture_bool = False

            # Set the capture state
            self.networks[configured_network].set_captured(capture_bool)

        self._print_info('Publishing the new network configurations')

        # Publish the target configurations
        self.networks[configured_network].publish_configs(self._get_target_config_topic())

        return


    def _read_new_network_name(self):
        """
        Helper method used to read the new name of a network and set the networks new name
        """
        # Make sure that there is a network whose name can be accessed
        if len(self._get_networks()) == 0:
            self._print_error('There are no registered networks to rename')
            return

        # Get the uid of the network
        renamed_network = self._get_input('Enter the UID of the network you wish to rename: ')

        if self._check_exit(renamed_network):
            return

        # Ensure that a valid network uid is provided
        while renamed_network not in self._get_networks():
            self._print_error('The network provided is not a registered network. Please enter a valid UID')
            renamed_network = self._get_input('Enter the UID of the network whose state you wish to rename: ')

            if self._check_exit(renamed_network):
                return

        # Get the new name of the network
        new_uid = self._get_input('Enter the UID that you wish to rename the network to: ')

        if self._check_exit(new_uid):
            return

        self._print_info(f'Renaming the network, {renamed_network}, to be {new_uid}')

        # Rename the network to the desired name
        self.networks[renamed_network].rename(new_uid, self._get_target_config_topic())

        return


    def _read_network_deletion(self):
        """
        Helper method used to enable a user to delete a network
        """
        # Make sure that there is a network whose state can be accessed
        if len(self._get_networks()) == 0:
            self._print_error('There are no registered networks to delete')
            return

        # Get the uid of the network
        deleted_network = self._get_input('Enter the UID of the network you wish to delete: ')

        if self._check_exit(deleted_network):
            return

        # Ensure that a valid network uid is provided
        while deleted_network not in self._get_networks():
            self._print_error('The network provided is not a registered network. Please enter a valid UID')
            deleted_network = self._get_input('Enter the UID of the network whose state you wish to delete: ')

            if self._check_exit(deleted_network):
                return

        # Verify that the user really does want to delete the network
        are_you_sure = self._get_input('Are you sure that you wish to delete the linked target network? (y/n): ')

        if self._check_exit(are_you_sure):
            return

        # Delete the network
        if are_you_sure == 'y':
            self.networks[deleted_network].delete_network(self._get_target_config_topic())

            # Convert all targets in the network to idle targets
            for target in self._get_network(deleted_network).get_network_uids():
                converted_target = IdleTarget(target)
                self._overwrite_target(target, converted_target, self._get_host(), self._generate_target_mqtt_id())
                converted_target.publish_configs(self._get_target_config_topic())

            self._delete_network(deleted_network)
        else:
            self._print_info('Linked target network deletion canceled')

        self._print_info('The linked target network has been deleted')

        return


    def _read_payload_server_configurations(self):
        """
        Read the payload configurations from the user
        """
        valid_intents = ['IDLE', 'MASS', 'LINK', 'PERI', 'FAIL']

        # Ensure that there are payloads that can be configured
        if len(self._get_payloads()) == 0:
            self._print_error('There are no registered payloads to configure')
            return

        # Get the payload to configure
        uid = self._get_input('Enter the UID of the Payload whose configurations you wish to modify: ')

        if self._check_exit(uid):
            return

        # Ensure that the payload uid is valid
        while uid not in self._get_payloads():
            self._print_error('The payload provided is not a registered payload. Please enter a valid payload UID.')
            uid = self._get_input('Enter the UID of the Payload whose configurations you wish to modify: ')

            if self._check_exit(uid):
                return

        # Determine whether the user would like to reset the intent
        reset_intent = self._get_input('Would you like to reset the intent? (y/n): ')

        if self._check_exit(reset_intent):
            return

        # Reset the intent
        if reset_intent == 'y':
            # Get the new intent
            intent = self._get_input('Intent (IDLE, MASS, LINK, PERI, or FAIL): ')

            if self._check_exit(intent):
                return

            # Ensure that the new intent is a valid option
            while intent not in valid_intents:
                self._print_error('Invalid intent provided. Please enter a valid intent (IDLE, MASS, LINK, PERI, or FAIL)')
                intent = self._get_input('Intent (IDLE, MASS, LINK, PERI, or FAIL): ')

                if self._check_exit(intent):
                    return

            # Set the intent
            self.payloads[uid].set_intent(intent)

            self._print_info('Publishing the payload configurations')

            # Publish the new configurations
            self.payloads[uid].publish_configs(self._get_payload_config_topic())
        
        else:
            self._print_info('No payload configurations have been set or published')

        return


    def _read_new_targets_to_add_to_network(self):
        """
        Helper method used to read in the targets that should be added to an existing linked network
        """
        # Make sure that there is a network whose state can be accessed
        if len(self._get_networks()) == 0:
            self._print_error('There are no registered networks to modify')
            return

        # Get the uid of the network
        modified_network = self._get_input('Enter the UID of the network you wish to modify: ')

        if self._check_exit(modified_network):
            return

        # Ensure that a valid network uid is provided
        while modified_network not in self._get_networks():
            self._print_error('The network provided is not a registered network. Please enter a valid UID')
            modified_network = self._get_input('Enter the UID of the network whose state you wish to modify: ')

            if self._check_exit(modified_network):
                return

        # Get the UID of the first target to add to the network
        first_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

        if self._check_exit(first_target_uid):
            return

        # Ensure that a valid UID is provided
        while first_target_uid not in self._get_targets():
            print(f'[ERROR] The provided target UID ({first_target_uid}) is not a valid target. Please enter a valid UID.')
            first_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

            if self._check_exit(first_target_uid):
                return

        # Ensure that the target isn't already in the network
        while first_target_uid in self._get_network(modified_network).get_network():
            print(f'[ERROR] The provided target UID ({first_target_uid} already exists in the network. Please provide a different UID.')    
            first_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')          

            if self._check_exit(first_target_uid):
                return

        # Determine whether the target already belongs to a network
        if self._get_target(first_target_uid).get_target_type() == 'LINK':
            # Determien whether the user wants to add the target to the current network
            remove = self._get_input('The provided target belongs to another existing network. Would you like to remove the target from the other network? (y/n): ')

            if self._check_exit(remove):
                return

            # Remove the target from the network
            if remove == 'y':
                # Find and remove the target from the linked network
                self._find_and_remove_target_from_network(first_target_uid)

                self._print_info(f'Adding {first_target_uid} to the linked network')

                self.networks[modified_network].add_target_to_network(self._get_target(first_target_uid))
            else:
                print('The target will not be added to the linked target network.')
        else:
            # Convert the target to a Linked target
            first_target = LinkedTarget(first_target_uid)

            self._print_info(f'Adding {first_target_uid} to the linked network')

            # Add the target to the network
            self.networks[modified_network].add_target_to_network(first_target)

            # Overwrite the existing target
            self._overwrite_target(first_target_uid, first_target, self._get_host(), self._generate_target_mqtt_id())

        # Flag indicating whether to continue adding targets or not
        keep_adding_targets = True

        # Loop until the user wants to stop adding targets or there are no more targets to add
        while keep_adding_targets and len(self._get_network(modified_network).get_network()) < len(self._get_targets()):
            # Determine whether the user wants to add another target
            add_target = self._get_input('Would you like to add another target? (y/n): ')

            if self._check_exit(add_target):
                return

            # Add another target
            if add_target == 'y':

                # Get the UID of the target to add
                new_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

                if self._check_exit(new_target_uid):
                    return

                # Ensure that the uid provided is valid
                while new_target_uid not in self._get_targets():
                    print(f'[ERROR] The provided target UID ({new_target_uid}) is not a valid target. Please enter a valid UID.')
                    new_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')

                    if self._check_exit(new_target_uid):
                        return

                # Ensure that the target isn't already in the network
                while new_target_uid in self._get_network(modified_network).get_network():
                    print(f'[ERROR] The provided target UID ({new_target_uid} already exists in the network. Please provide a different UID.')    
                    new_target_uid = self._get_input('Enter the UID of a target to add to a linked network: ')    

                    if self._check_exit(new_target_uid):
                        return

                # Determine whether the target is already in another network
                if self._get_target(new_target_uid).get_target_type() == 'LINK':
                    # Determine whether to remove the target from the existing network or not
                    remove = self._get_input('The provided target belongs to another existing network. Would you like to remove the target from the other network? (y/n): ')

                    if self._check_exit(remove):
                        return

                    # Remove the target from the other existing network
                    if remove == 'y':
                        self._find_and_remove_target_from_network(new_target_uid)
                        self._print_info(f'Adding {new_target_uid} to the linked network')
                        self.networks[modified_network].add_target_to_network(self._get_target(new_target_uid))
                    else:
                        print('The target will not be added to the linked target.')
                else: 
                    # Modify the configuration type of the target to be a linked target and add it to the network
                    new_target = LinkedTarget(new_target_uid)
                    self._print_info(f'Adding {new_target_uid} to the linked network')
                    self.networks[modified_network].add_target_to_network(new_target)
                    self._overwrite_target(new_target_uid, new_target, self._get_host(), self._generate_target_mqtt_id())
            else:
                # Break
                keep_adding_targets = False

        self._print_info('Publishing the new configuration changes to the server')

        # Publish the changes
        self.networks[modified_network].publish_configs(self._get_target_config_topic())

        return


    def _read_targets_to_delete_from_network(self):
        """
        Helper method used to read in the targets that a user would like to delete from a network
        """
        # Make sure that there is a network whose state can be accessed
        if len(self._get_networks()) == 0:
            self._print_error('There are no registered networks to modify')
            return

        # Get the uid of the network
        modified_network = self._get_input('Enter the UID of the network you wish to modify: ')

        if self._check_exit(modified_network):
            return

        # Ensure that a valid network uid is provided
        while modified_network not in self._get_networks():
            self._print_error('The network provided is not a registered network. Please enter a valid UID')
            modified_network = self._get_input('Enter the UID of the network whose state you wish to modify: ')

            if self._check_exit(modified_network):
                return

        # Display the existing targets in the network
        self._print_info('The available targets to delete from the network include:')
        for target in self._get_network(modified_network).get_network():
            print(f'- {target.get_uid()}')

        # Get the target UID to remove
        target_to_remove = self._get_input('Enter the UID of the target that you wish to remove from the linked target network: ')

        if self._check_exit(target_to_remove):
            return

        # Ensure that the target is in the network
        while target_to_remove not in self._get_network(modified_network).get_network_uids():
            self._print_error('The provided target is not a member of the linked network. Please provide a valid target UID.')
            target_to_remove = self._get_input('Enter the UID of the target that you wish to remove from the linked target network: ')

            if self._check_exit(target_to_remove):
                return

        self._print_info(f'Removing {target_to_remove} from the linked network')        

        # Remove the target from the network
        self.networks[modified_network].remove_target_from_network(self._get_target(target_to_remove))

        # Convert the target from a linked target to an idle target
        updated_target = IdleTarget(target_to_remove)
        self._overwrite_target(target_to_remove, updated_target, self._get_host(), self._generate_target_mqtt_id())
        updated_target.publish_configs(self._get_target_config_topic())

        # Flag indicating whether to keep deleting targets
        keep_deleting_targets = True

        # Iterate until the user wnats to stop deleting targets
        while keep_deleting_targets and len(self._get_network(modified_network).get_network()) > 0:
            # Determine whether the user would like to continue removing targets
            keep_deleting_input = self._get_input(f'Would you like to remove another target from the network (There are {len(self._get_network(modified_network).get_network())} targets remaining)? (y/n): ')

            if self._check_exit(keep_deleting_input):
                return

            # Continue removing targets from the network
            if keep_deleting_input == 'y':
                # Print out the targets that can be deleted
                self._print_info('The available targets to delete from the network include:')
                for target in self._get_network(modified_network).get_network():
                    print(f'- {target.get_uid()}')

                # Get the target that should be removed
                target_to_remove = self._get_input('Enter the UID of the target that you wish to remove from the linked target network: ')

                if self._check_exit(target_to_remove):
                    return

                # Ensure that the target can be removed from the network
                while target_to_remove not in self._get_network(modified_network).get_network_uids():
                    self._print_error('The provided target is not a member of the linked network. Please provide a valid target UID.')
                    target_to_remove = self._get_input('Enter the UID of the target that you wish to remove from the linked target network: ')

                    if self._check_exit(target_to_remove):
                        return  
    
                self._print_info(f'Removing {target_to_remove} from the linked network')
                        
                # Remove the target from the network
                self.networks[modified_network].remove_target_from_network(self._get_target(target_to_remove))

                # Convert the target to an idle target
                updated_target = IdleTarget(target_to_remove)
                self._overwrite_target(target_to_remove, updated_target, self._get_host(), self._generate_target_mqtt_id())
                updated_target.publish_configs(self._get_target_config_topic())
            else:
                # Break
                self._print_info('No additional targets will be removed from the network.')
                keep_deleting_targets = False

        self._print_info('Publishing the changes to the server')

        # Publish the changes to the network
        self.networks[modified_network].publish_configs(self._get_target_config_topic())

        return

    
    def _print_agent_menu(self):
        """
        Helper method used to print the network configuration menu
        """
        self._print_line()
        print('|                             AGENT MENU                             |')
        self._print_line()
        print('1. Display Agent Menu')
        print('2. Create a new agent')
        print('3. Display agent state')
        print('4. Query an agent\'s payload\'s UID')
        print('5. Query an agent\'s payload\'s location')
        print('6. Set an agent\'s payload\'s intent')
        print('7. Send another agent a message')
        print('8. Delete an agent')
        print('9. Exit Agent Menu')

        self._print_line()

        return


    def _read_agent_user_input(self):
        """
        Read the user interactions with a rudimentary agent
        """
        # Get the user's menu item selection
        menu_selection = self._get_input('Enter an Agent Menu Item: ')

        # Ensure that the input is a digit
        while not menu_selection.isdigit():
            self._print_error('Agent Menu Item selected. Valid selections include items 1-9.')
            self._print_line()
            menu_selection = self._get_input('Enter an Agent Menu Item: ')

        # Cast the number to an int
        menu_selection = int(menu_selection)

        # Validate that the menu selection is a valid option
        while menu_selection > 9 or menu_selection < 1:
            self._print_error('Agent Menu Item selected. Valid selections include items 1-9.')
            self._print_line()
            menu_selection = int(self._get_input('Enter a Agent Menu Item: '))
        
        # Display the menu
        if menu_selection == 1:
            self._print_agent_menu()

        # Create a new agent
        elif menu_selection == 2:
            self._create_new_agent()

        # Display agent state
        elif menu_selection == 3:
            self._print_agent_state()

        # Query payload UID
        elif menu_selection == 4:
            self._query_payload_uid()

        # Query payload location
        elif menu_selection == 5:
            self._query_payload_location()

        # Set payload intent      
        elif menu_selection == 6:
            self._read_payload_agent_configurations()

        # Send a message
        elif menu_selection == 7:
            self._send_agent_message()

        # Delete an agent
        elif menu_selection == 8:
            self._read_agent_deletion()

        # Exit the agent menu
        elif menu_selection == 9:
            self._print_info('Exiting Agent Menu')
            self._print_menu()
            return False

        return True


    def _create_new_agent(self):
        """
        Helper method used to create new agents
        """
        # Ensure that there are targets that can be added to a network
        if len(self._get_payloads()) == 0:
            self._print_error('There are no registered payloads that may be assigned to an agent')
            return

        if len(self._get_available_ports()) == 0:
            self._print_error('There are no available ports that an agent can connect to')
            return

        # Get the uid of the network
        uid = self._get_input('Enter the UID of the agent you wish to create: ')

        if self._check_exit(uid):
            return

        # Prevent existing UIDs from being used to create networks
        while uid in self._get_agents():
            self._print_error('The provided agent UID already exists in the list of agents. Please enter a unique agent UID.')
            uid = self._get_input('Enter the UID of the agent you wish to create: ')

            if self._check_exit(uid):
                return

        # Print out the occupied ports for reference
        self._print_occupied_ports()

        # Print out the available ports for reference
        self._print_available_ports()

        # Get the port number
        port = self._get_input('Enter the port of the payload that you wish to assign to the agent: ')

        while not port.isdigit():
            self._print_error('The provided port is not a valid port number. Please provide an integer-based port number')
            port = self._get_input('Enter the port of the payload that you wish to assign to the agent: ')

            if self._check_exit(port):
                return

        # Ensure that a valid port number is provided
        while self._check_if_port_is_available(port):
            self._print_error('The provided port is not an available port. Please enter a valid available port number')
            self._print_available_ports()
            port = self._get_input('Enter the port of the payload that you wish to assign to the agent: ')

            if self._check_exit(port):
                return

        # Cast the port to an integer
        port = int(port)

        # Remove the port from the available list
        self._remove_available_port(port)
        
        # Remove the port from the occupied ports list
        self._add_occupied_port(port)

        # Create a new agent
        agent = Agent(uid, port)

        # Add the agent to the dictionary of agents and start its client
        self._add_agent(agent, self._get_host(), self._generate_agent_mqtt_id(), self._get_receive_uid_response_topic(), 
            self._get_receive_location_response_topic(), self._get_receive_message_topic())

        self._print_info(f'Successfully created agent: {uid}')

        return

        
    def _print_occupied_ports(self):
        """
        Helper method used to print out the occupied ports
        """
        print('Occupied Ports: ')

        for port in self._get_occupied_ports():
            print(f'- {port}')

        return


    def _print_available_ports(self):
        """
        Helper method used to print out the available ports
        """
        print('Available Ports: ')

        for port in self._get_available_ports():
            print(f'- {port}')

        return


    def _query_payload_uid(self):
        """
        Determine which agent to send the uid query through and send the query
        """
        # Make sure that an agent exists
        if len(self._get_agents()) == 0:
            self._print_error('There are no registered agents whose payloads may be queried')
            return

        # Get the uid of the agent
        agent = self._get_input('Enter the UID of the agent you whose payload you wish to access: ')

        if self._check_exit(agent):
            return

        # Ensure that a valid agent uid is provided
        while agent not in self._get_agents():
            self._print_error('The agent provided is not a registered agent. Please enter a valid UID')
            agent = self._get_input('Enter the UID of the agent you whose payload you wish to access: ')

            if self._check_exit(agent):
                return

        # Query the payload
        self.agents[agent].query_payload(self._get_query_payload_uid_topic())

        self._print_info('The payload UID has been queried')

        return

    
    def _query_payload_location(self):
        """
        Determine which agent to send the location query through and send the query
        """
        # Make sure that there is an agent to access
        if len(self._get_agents()) == 0:
            self._print_error('There are no registered agents whose payloads may be queried')
            return

        # Get the uid of the agent
        agent = self._get_input('Enter the UID of the agent you whose payload you wish to access: ')

        if self._check_exit(agent):
            return

        # Ensure that a valid agent uid is provided
        while agent not in self._get_agents():
            self._print_error('The agent provided is not a registered agent. Please enter a valid UID')
            agent = self._get_input('Enter the UID of the agent you whose payload you wish to access: ')

            if self._check_exit(agent):
                return

        # Query the payload
        self.agents[agent].query_payload(self._get_query_payload_location_topic())

        self._print_info('The payload location has been queried')

        return


    def _read_payload_agent_configurations(self):
        """
        Read the payload configurations from the user
        """
        valid_intents = ['IDLE', 'MASS', 'LINK', 'PERI', 'FAIL']

        # Ensure that there are agents that can be configured
        if len(self._get_agents()) == 0:
            self._print_error('There are no registered agents whose payloads can be configured')
            return

        # Get the payload to configure
        uid = self._get_input('Enter the UID of the agent whose payload you wish to modify: ')

        if self._check_exit(uid):
            return

        # Ensure that the agent uid is valid
        while uid not in self._get_agents():
            self._print_error('The agent provided is not a registered agent. Please enter a valid agent UID.')
            uid = self._get_input('Enter the UID of the agent whose payload you wish to modify: ')

            if self._check_exit(uid):
                return

        # Determine whether the user would like to reset the intent
        reset_intent = self._get_input('Would you like to reset the intent? (y/n): ')

        if self._check_exit(reset_intent):
            return

        # Reset the intent
        if reset_intent == 'y':
            # Get the new intent
            intent = self._get_input('Intent (IDLE, MASS, LINK, PERI, or FAIL): ')

            if self._check_exit(intent):
                return

            # Ensure that the new intent is a valid option
            while intent not in valid_intents:
                self._print_error('Invalid intent provided. Please enter a valid intent (IDLE, MASS, LINK, PERI, or FAIL)')
                intent = self._get_input('Intent (IDLE, MASS, LINK, PERI, or FAIL): ')

                if self._check_exit(intent):
                    return

            self._print_info('Publishing the agent payload configurations')

            # Publish the new configurations
            self.agents[uid].set_payload_intent(self._get_agent_config_topic(), intent)
        
        else:
            self._print_info('No payload configurations have been set or published')

        return


    def _send_agent_message(self):
        """
        Get a message to send to another agent and send the message
        """
        # Ensure that there are payloads that can be configured
        if len(self._get_agents()) < 2:
            self._print_error('There are an insufficient amount of agents to send messages between. There must be at least two agents.')
            return

        # Get the payload to configure
        sender_uid = self._get_input('Enter the UID of the agent that will send the message: ')

        if self._check_exit(sender_uid):
            return

        # Ensure that the agent uid is valid
        while sender_uid not in self._get_agents():
            self._print_error('The agent provided is not a registered agent. Please enter a valid agent UID.')
            sender_uid = self._get_input('Enter the UID of the agent that will send the message: ')

            if self._check_exit(sender_uid):
                return

        # Get the payload to configure
        receiver_uid = self._get_input('Enter the UID of the agent that will receive the message: ')

        if self._check_exit(receiver_uid):
            return

        # Ensure that the agent uid is valid
        while receiver_uid not in self._get_agents():
            self._print_error('The agent provided is not a registered agent. Please enter a valid agent UID.')
            receiver_uid = self._get_input('Enter the UID of the agent that will send the message: ')

            if self._check_exit(receiver_uid):
                return

        # Get the UID of the payload that the message will be proxied through
        proxy_payload = self._get_port_payload(self._get_agent(receiver_uid).get_port())

        # Get the message
        message = self._get_input('Enter the message that you wish to send the agent: ')

        if self._check_exit(message):
            return

        # Send the message
        self.agents[sender_uid].send_agent_message(self._get_send_agent_message_topic(), proxy_payload, message)

        self._print_info(f'{sender_uid} has just sent a message to {receiver_uid}')

        return
        


    def _read_agent_deletion(self):
        """
        Determine which agent to delete and delete it
        """
        # Ensure that there are payloads that can be configured
        if len(self._get_agents()) == 0:
            self._print_error('There are no registered agents to delete')
            return

        # Get the payload to configure
        uid = self._get_input('Enter the UID of the agent whose payload you wish to delete: ')

        if self._check_exit(uid):
            return

        # Ensure that the agent uid is valid
        while uid not in self._get_agents():
            self._print_error('The agent provided is not a registered agent. Please enter a valid agent UID.')
            uid = self._get_input('Enter the UID of the agent whose payload you wish to delete: ')

            if self._check_exit(uid):
                return

         # Verify that the user really does want to delete the network
        are_you_sure = self._get_input('Are you sure that you wish to delete the agent? (y/n): ')

        if self._check_exit(are_you_sure):
            return

        # Delete the agent
        if are_you_sure == 'y':
            agent = self.agents.pop(uid)

            # Move the port from occupied to available
            self._remove_occupied_port(agent.get_port())
            self._add_available_port(agent.get_port())
        else:
            self._print_info('Agent deletion canceled')

        self._print_info('The agent has been deleted')

        return


    def print_output(self):
        """
        Display the CLI output
        """
        # Print the menu on first run
        if self.first_display:
            self._print_menu()
            self.first_display = False

        # Get the user input
        self._get_user_input()

        return