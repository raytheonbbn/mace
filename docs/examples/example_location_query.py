#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time


def read_location_query_response_cb(client, userdata, message):
    """
    Callback function used to read the UID query response from a payload
    """
    # Read the message to a JSON element
    json_response = str(message.payload.decode('utf-8', 'ignore'))

    # Convert the JSON message to a python dictionary
    response = json.loads(json_response)

    print(response['latitude'])
    print(response['longitude'])
    print(response['altitude'])

    return


def main():
    # Create an MQTT Client
    client = mqtt.Client('MY-TEST-CLIENT')

    # Connect the payload client to the mqtt broker
    client.connect('169.254.0.1', port=1884)

    # Subscribe to the response topic
    client.subscribe('agent/payload/query/location')

    # Set the callback function
    client.message_callback_add('agent/payload/query/location', read_location_query_response_cb)

    # Start the client
    client.loop_start()
    
    # Publish the query
    client.publish('payload/agent/query/location', '')

    # Wait for the message to be received and for the callback to be called
    time.sleep(2)

    # Stop and disconnect the client
    client.loop_stop()
    client.disconnect()

    return


if __name__ == '__main__':
    main()