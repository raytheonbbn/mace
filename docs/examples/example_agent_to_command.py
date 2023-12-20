#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time

def main():
    # Create an MQTT Client
    client = mqtt.Client('SENDER-CLIENT')

    # Connect the payload client to the mqtt broker
    client.connect('169.254.0.1', port=1884)

    # Start the client
    client.loop_start()

    # Create a message to send to the agent
    example_message = {
        'topic': 'command/agent/message/example',
        'example_element': 'example_value'
    }

    # Convert the message to JSON
    json_example_message = json.dumps(example_message)
    
    # Publish the message
    client.publish('command/agent/message', json_example_message)

    # Stop and disconnect the client
    client.loop_stop()
    client.disconnect()

    return

if __name__ == '__main__':
    main()