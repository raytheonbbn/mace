#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time

"""
This script when run on the payload raspberry pi will print a notification method when it 
detects a target within range of it's BLE sensor.
"""


def read_in_range_notification_cb(client, userdata, message):
    """
    Callback function used to read an in_range notification
    """
    # Read the message to a JSON element
    json_notification = str(message.payload.decode('utf-8', 'ignore'))

    # Convert the JSON message to a python dictionary
    notification = json.loads(json_notification)

    # Get the notification properties
    in_range = notification['in_range']
    target_uid = notification['target_uid']
    target_type = notification['type_in_range']
    if in_range:
        print(f'I just came into range of target {target_uid} which is of type {target_type}')

    return


def main():
    # Create an MQTT Client
    client = mqtt.Client('MY-TEST-CLIENT')

    # Connect the payload client to the mqtt broker
    client.connect('localhost', port=1884)

    # Subscribe to the agent topic
    client.subscribe('agent/payload/notification/in_range')

    # Set the callback function
    client.message_callback_add('agent/payload/notification/in_range', read_in_range_notification_cb)

    try:
        # Start the client
        client.loop_start()

        # Wait for the message to be received and for the callback to be called
        time.sleep(60)
    except:
        pass
    finally:
        # Stop and disconnect the client
        client.loop_stop()
        client.disconnect()

    return


if __name__ == '__main__':
    main()