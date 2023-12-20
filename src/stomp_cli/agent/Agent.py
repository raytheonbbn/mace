#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json


class Agent:
    def __init__(self, uid, port):
        # Initialize class variables
        self.uid = uid
        self.port = port
        self.assigned_payload_uid = None
        self.assigned_payload_latitude = None
        self.assigned_payload_longitude = None
        self.assigned_payload_altitude = None
        self.uid_response_topic = None
        self.location_response_topic = None
        self.receive_message_topic = None
        self.client = None
        self.messages_received = []


    def connect(self, host, uid):
        """
        Connect the client to the desired broker
        """
        # Create an MQTT Client
        self.client = mqtt.Client(uid)

        # Connect the client to the mqtt broker
        self.client.connect(host, port=self.get_port())

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
        # Start an mqtt client
        self.client.loop_start()

        return


    def stop_client(self):
        """
        Stop the MQTT client
        """
        self.client.loop_stop()

        return


    def subscribe_to_api_topics(self, uid_response_topic, location_response_topic, receive_message_topic, 
        uid_response_qos=2, location_response_qos=2, receive_message_qos=2):
        """
        Helper method used to subscribe to the topics over which the agent will receive query responses
        """
        self.set_receive_message_topic(receive_message_topic)

        topics = [(uid_response_topic, uid_response_qos), (location_response_topic, location_response_qos), (receive_message_topic, receive_message_qos)]

        self.client.subscribe(topics)

        self.client.message_callback_add(uid_response_topic, self.__read_uid_query_response_cb)
        self.client.message_callback_add(location_response_topic, self.__read_location_query_response_cb)
        self.client.message_callback_add(receive_message_topic, self.__read_incoming_message_cb)

    
    def get_uid(self):
        """
        Getter used to retrieve the agent uid
        """
        return self.uid


    def get_receive_message_topic(self):
        """
        Getter used to retrieve the topic that messages sent to the payload should be returned over
        """
        return self.receive_message_topic


    def set_receive_message_topic(self, topic):
        """
        Setter used to set the topic that the messages intended for an agent should be returned over
        """
        self.receive_message_topic = topic

        return


    def get_assigned_payload_uid(self):
        """
        Getter used to retrieve the UID of the payload that the agent is carrying
        """
        return self.assigned_payload_uid

    
    def set_assigned_payload_uid(self, uid):
        """
        Setter used to set the stored UID of the payload that the agent is carrying
        """
        self.assigned_payload_uid = uid

        return


    def get_assigned_payload_latitude(self):
        """
        Getter used to retrieve the latitude of the assigned payload
        """
        return self.assigned_payload_latitude


    def set_assigned_payload_latitude(self, latitude):
        """
        Setter used to set the latitude of the assigned payload
        """
        self.assigned_payload_latitude = latitude

        return

    def get_assigned_payload_longitude(self):
        """
        Getter used to retrieve the longitude of the assigned payload
        """
        return self.assigned_payload_longitude


    def set_assigned_payload_longitude(self, longitude):
        """
        Setter used to set the longitude of the assigned payload
        """
        self.assigned_payload_longitude = longitude

        return


    def get_assigned_payload_altitude(self):
        """
        Getter used to retreive the altitude of the assigned payload
        """
        return self.assigned_payload_altitude


    def set_assigned_payload_altitude(self, altitude):
        """
        Setter used to set the altitude of the assigned payload
        """
        self.assigned_payload_altitude = altitude

        return

    
    def add_message_to_received_messages(self, message):
        """
        Add a received message to the list of received messages
        """
        self.messages_received.append(message)

        return


    def get_messages_received(self):
        """
        Getter used to retrieve all messages sent to the agent
        """
        return self.messages_received


    def get_port(self):
        """
        Getter used to retrieve the port over which the agent is interacting with its payload
        """
        return self.port


    def set_port(self, port):
        """
        Setter used to set the port over which the agent is interacting with its payload
        """
        self.port = port

        return

    
    def query_payload(self, topic):
        """
        Helper method used to abstract out queries
        """
        request = 'request'

        # Publish the query
        self.client.publish(topic, request)

        return


    def send_agent_message(self, topic, intended_payload_uid, message):
        """
        Send another agent a message
        """
        # Create the message to send
        packaged_message = {
            'uid': intended_payload_uid,
            'topic': self.get_receive_message_topic(),
            'message': message
        }

        # Convert the message to json
        packaged_message_json = json.dumps(packaged_message)

        # Publish the message
        self.client.publish(topic, packaged_message_json)

        return


    def set_payload_intent(self, topic, intent):
        """
        Set the intent of the payload that the agent is carrying
        """
        # Create the message
        intent_message = {
            'intent': intent
        }

        # Convert the message to json
        json_intent_message = json.dumps(intent_message)

        # Publish the message
        self.client.publish(topic, json_intent_message)

        return

    
    def __read_uid_query_response_cb(self, client, userdata, message):
        """
        Callback function used to read the UID query response from a payload
        """
        # Read the message to a JSON element
        json_response = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        response = json.loads(json_response)

        # Store the associated payload UID
        self.set_assigned_payload_uid(response['uid'])

        return


    def __read_location_query_response_cb(self, client, userdata, message):
        """
        Callback function used to read the location query response from a payload
        """
        # Read the message to a JSON element
        json_response = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        response = json.loads(json_response)

        # Store the response
        self.set_assigned_payload_latitude(response['latitude'])
        self.set_assigned_payload_longitude(response['longitude'])
        self.set_assigned_payload_altitude(response['altitude'])

        return


    def __read_incoming_message_cb(self, client, userdata, message):
        """
        Callback function used to read the message sent from another agent
        """
        # Read the message to a JSON element
        json_agent_message = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        agent_message = json.loads(json_agent_message)

        self.add_message_to_received_messages(agent_message['message'])

        return


    def print_state(self):
        """
        Helper method used to print the current state of the agent
        """
        print(f'Port: {self.get_port()}')        
        print(f'Assigned Payload UID: {self.get_assigned_payload_uid()}')
        print(f'Assigned Payload Latitude: {self.get_assigned_payload_latitude()}')
        print(f'Assigned Payload Longitude: {self.get_assigned_payload_longitude()}')
        print(f'Assigned Payload Altitude: {self.get_assigned_payload_altitude()}')
        print(f'Received Messages: ')

        for message in self.get_messages_received():
            print(f'- {message}')

        return