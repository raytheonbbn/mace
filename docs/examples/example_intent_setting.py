#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import sys


def main():   

    intent_val = 'IDLE'
    if len(sys.argv) == 2:
        intent_val = sys.argv[1]

    # Create an MQTT Client
    client = mqtt.Client('MY-TEST-CLIENT')

    # Connect the payload client to the mqtt broker
    client.connect('169.254.0.1', port=1884)

    # Start the client
    client.loop_start()

    # Create a new intent message
    intent = {
        'intent': intent_val
    }

    # Convert the dictionary to JSON
    json_intent = json.dumps(intent)

    # Publish the query
    client.publish('payload/agent/configurations', json_intent)

    # Shut down and disconnect the client
    client.loop_stop()
    client.disconnect()


if __name__ == '__main__':
    main()