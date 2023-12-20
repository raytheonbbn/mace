#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time


def read_intent_notification_cb(client, userdata, message):
    """
    Callback function used to read the intent change notification
    """
    # Read the message to a JSON element
    json_notification = str(message.payload.decode('utf-8', 'ignore'))

    # Convert the JSON message to a python dictionary
    notification = json.loads(json_notification)

    # Get the new intent
    new_intent = notification['intent']

    print(f'I just received notification that the command station change the intent to: {new_intent}')

    return


def main():
    # Create an MQTT Client
    client = mqtt.Client('MY-TEST-CLIENT')

    # Connect the payload client to the mqtt broker
    client.connect('169.254.0.1', port=1884)

    # Subscribe to the agent topic
    client.subscribe('agent/payload/notification/intent')

    # Set the callback function
    client.message_callback_add('agent/payload/notification/intent', read_intent_notification_cb)

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