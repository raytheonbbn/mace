#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time

def read_message_cb(client, userdata, message):
    """
    Callback function used to read message sent
    """
    # Read the message to a JSON element
    json_message = str(message.payload.decode('utf-8', 'ignore'))

    # Convert the JSON message to a python dictionary
    custom_message = json.loads(json_message)

    print(custom_message)

    return


def main():
    # Create an MQTT Client
    client = mqtt.Client('RECEIVER-CLIENT')

    # Connect the command client to the mqtt broker
    client.connect('192.168.1.100', port=1883)

    # Subscribe to the agent topic
    client.subscribe('command/agent/message/example')

    # Set the callback function
    client.message_callback_add('command/agent/message/example', read_message_cb)

    try:
        # Start the client
        client.loop_start()

        # Wait for the message to be received and for the callback to be called
        time.sleep(15)
    except:
        pass
    finally:
        # Stop and disconnect the client
        client.loop_stop()
        client.disconnect()

    return

if __name__ == '__main__':
    main()