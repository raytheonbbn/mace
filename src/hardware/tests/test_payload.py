#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import unittest
from mock import Mock
from uuid import getnode as get_mac_address
import paho.mqtt.client as mqtt
import time
import json
import sys
sys.path.append('..')
from payload.Payload import Payload


class TestPayload(unittest.TestCase):

    def set_up_test_client(self, user_server_host=True, query_uid_response_topic=None, send_message_response_topic=None):
        """
        Create a test client to use for payload MQTT client testing
        """

        if user_server_host:
            host = 'test.mosquitto.org'
        else:
            host = 'broker.emqx.io'

        test_client = mqtt.Client("Test Client")
        test_client.connect(host)

        topics = []

        if query_uid_response_topic is not None:
            topics.append((query_uid_response_topic, 2))
            test_client.message_callback_add(query_uid_response_topic, self.read_query_response)
            self.query_response = None

        if send_message_response_topic is not None:
            topics.append((send_message_response_topic, 2))
            test_client.message_callback_add(send_message_response_topic, self.read_message_sent)
            self.received_message = None

        if len(topics) > 0:
            test_client.subscribe(topics)

        # Start the test client
        test_client.loop_start()

        return test_client

    
    def shutdown_test_client(self, test_client):
        """
        Helper method to shut down a test client
        """
        test_client.loop_stop()
        test_client.disconnect()

        return


    def read_query_response(self, client, userdata, message):
        """
        Callback function used to update the query response
        """
        # Read the message to a JSON element
        json_response = str(message.payload.decode('utf-8', 'ignore'))

        # Convert the JSON message to a python dictionary
        response = json.loads(json_response)

        self.query_response = response['uid']

        return

    
    def read_message_sent(self, client, userdata, message):
        """
        Callback function used to handle a sent message
        """
        json_response = str(message.payload.decode('utf-8', 'ignore'))

        response = json.loads(json_response)

        self.received_message = response['message']

        return


    def set_up_payload(self, 
        start_client=False, 
        agent_test_host='broker.emqx.io', 
        server_test_host='test.mosquitto.org', 
        relay_topic='test/relay', 
        query_uid_response_topic='test/query/uid', 
        query_location_response_topic='test/query/location', 
        in_range_notification_topic='test/notification/in_range', 
        intent_change_notification_topic='test/notification/intent', 
        identifier='PAYLOAD', 
        uid=None, 
        server_configuration_topic='test/1', 
        query_uid_topic='test/2', 
        query_location_topic='test/3', 
        send_message_topic='test/4', 
        agent_configuration_topic='test/5',
        reset_discovered_topic='test/6'
        target_captured_notification_topic='test/notification/capture'):
        """
        Helper method used to generate and start a new payload
        """
        # Create a payload
        payload = Payload(Mock(), Mock(), relay_topic, query_uid_response_topic, query_location_response_topic,
            in_range_notification_topic, intent_change_notification_topic, target_captured_notification_topic, identifier=identifier, uid=uid)
        
        if start_client:
            # Connect the client to the broker
            payload.connect_all_clients(server_test_host, agent_test_host)

            # Set the payload up to read configurations
            payload.subscribe_to_server_topics(server_configuration_topic)
            payload.subscribe_to_agent_topics(query_uid_topic, query_location_topic, send_message_topic, agent_configuration_topic, reset_discovered_topic)

            # Start the payload client
            payload.start_all_clients()

        return payload

    
    def shutdown_payload(self, payload):
        """
        Helper method used to shut down a payload
        """
        # payload.stop_all_clients()
        payload.stop_server_client()
        # payload.disconnect_all_clients()
        payload.disconnect_server_client()

        return


    def test_set_uid(self):
        """
        Ensure that the system is using the correct MAC address and correct identifier
        """
        test_identifier = 'TEST-PAYLOAD'

        payload = self.set_up_payload(identifier=test_identifier)

        current_mac = str(get_mac_address())
        
        # Assert that the device MAC address is being used in the UID
        self.assertTrue(current_mac in payload.get_uid())

        # Assert that the correct identifier is being used in the UID
        self.assertTrue(test_identifier in payload.get_uid())

        return
        

    def test_get_intent(self):
        """
        Ensure that the system effectively sets the intent of the object
        """
        payload = self.set_up_payload()

        intent = 'IDLE'

        payload.intent = intent

        self.assertEqual(payload._get_intent(), intent)

        return
        

    def test_set_valid_intent(self):
        """
        Test setting a valid intent for a payload
        """
        payload = self.set_up_payload()

        intent = 'IDLE'

        payload._set_intent(intent)

        self.assertEqual(payload._get_intent(), 'IDLE')

        return
        

    def test_set_invalid_intent(self):
        """
        Test setting an invalid intent for a payload
        """
        payload = self.set_up_payload()

        intent = 'invalid_intent'

        self.assertRaises(AssertionError, payload._set_intent, intent)

        return
        

    def test_valid_read_payload_configurations(self):
        """
        Test whether the system correctly changes configurations when delivered
        """
        configuration_topic = 'payload_test/configuration/'

        # Create a new test client
        test_client = self.set_up_test_client()

        # Create a new payload
        payload = self.set_up_payload(start_client=True, server_configuration_topic=configuration_topic)

        test_intent = 'MASS'

        test_configurations = {
            'uid': payload.get_uid(),
            'intent': test_intent
        }

        # Convert the payload state to JSON
        json_configurations = json.dumps(test_configurations)

        # Publish the payload state to the broker
        for _ in range(100):
            test_client.publish(configuration_topic, json_configurations)

        # Delay to ensure that the system is able to publish the state
        time.sleep(2)

        # Shutdown the clients
        self.shutdown_payload(payload)
        self.shutdown_test_client(test_client)

        self.assertEqual(payload._get_intent(), test_intent)

        return

    def test_send_agent_message(self):
        """
        Test sending a message from one agent to another agent
        NOTE: Sometimes this test will fail because of failure to interact with the broker
        """
        # Topics
        send_message_topic = 'payload_test/send/message'
        send_message_response_topic = 'payload_test/send/message/response'

        # Create a new test agent to simulate an agent
        test_client = self.set_up_test_client(user_server_host=False, send_message_response_topic=send_message_response_topic)
        
        # Create a new payload
        payload = self.set_up_payload(start_client=True, send_message_topic=send_message_topic)

        message = 'testing123456'

        test_message = {
            'uid': payload.get_uid(),
            'topic': send_message_response_topic,
            'message': message
        }

        json_message = json.dumps(test_message)

        for _ in range(20):
            # Publish the payload state to the broker
            test_client.publish(send_message_topic, json_message)

        # Wait for the messages to be sent
        time.sleep(2)

        # Shutdown the clients
        self.shutdown_payload(payload)
        self.shutdown_test_client(test_client)

        self.assertEqual(self.received_message, message)


    def test_query_uid(self):
        """
        Test querying the payload UID
        """
        # Topics
        test_query_topic = 'agent/payload/query/uid'
        response_topic = 'payload/agent/query/uid'

        # Create a new test client
        test_client = self.set_up_test_client(user_server_host=False, query_uid_response_topic=response_topic)

        # Create a new payload
        payload = self.set_up_payload(start_client=True, query_uid_response_topic=response_topic, query_uid_topic=test_query_topic)

        for _ in range(100):
            # Publish the payload state to the broker
            test_client.publish(test_query_topic, 'request')

        # Delay to ensure that the system is able to publish the state
        time.sleep(2)

        # Shutdown the system
        self.shutdown_payload(payload)
        self.shutdown_test_client(test_client)

        self.assertEqual(self.query_response, payload.get_uid())

        return


if __name__ == 'main':
    unittest.main()