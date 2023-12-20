#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time


def read_capture_notification_cb(client, userdata, message):
    """
    Callback function used to read a capture notification
    """
    # Read the message to a JSON element
    json_notification = str(message.payload.decode('utf-8', 'ignore'))

    # Convert the JSON message to a python dictionary
    notification = json.loads(json_notification)

    # Get the notification properties
    captured = notification['captured']
    device_captured = notification['uid']

    if captured:
        print(f'I just captured device {device_captured}')
    else:
        print(f'I failed to capture {device_captured}')

    return


def main():
    # Create an MQTT Client
    client = mqtt.Client('MY-TEST-CLIENT')

    # Connect the payload client to the mqtt broker
    client.connect('169.254.0.1', port=1884)

    # Subscribe to the agent topic
    client.subscribe('agent/payload/notification/capture')

    # Set the callback function
    client.message_callback_add('agent/payload/notification/capture', read_capture_notification_cb)

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