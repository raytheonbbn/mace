#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time

def main():
    # Create an MQTT Client
    client = mqtt.Client('SENDER-CLIENT')

    # Connect the command client to the mqtt broker
    client.connect('192.168.1.100', port=1883)

    # Start the client
    client.loop_start()

    # Create a message to send to the agent
    example_message = {
        'topic': 'agent/command/message/example',
        'example_element': 'example_value'
    }

    # Convert the message to JSON
    json_example_message = json.dumps(example_message)
    
    # Publish the message
    client.publish('agent/command/message/PAYLOAD-123456789', json_example_message)

    # Stop and disconnect the client
    client.loop_stop()
    client.disconnect()

    return

if __name__ == '__main__':
    main()