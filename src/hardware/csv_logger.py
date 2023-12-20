#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import json
import time
import sys

'''
Helper script to logg MQTT messages to a file (ex: CSV) or terminal
'''
class Logger():

    def __init__(self, output_file: 'str|None', columns: 'list[str]'):
        self.file_name = output_file
        self.columns = columns

        if self.file_name is not None:
            with open(self.file_name, 'a') as file:
                file.write(f'{",".join(self.columns)}\n')

    def log_callback(self, client, userdata, message):
        """
        logging callback function
        """
        # Read the message to a JSON element
        line = str(message.payload.decode('utf-8', 'ignore'))

        #print(line)
        json_val = json.loads(line)
        out = f'{",".join([str(json_val[c]) for c in self.columns])}\n'
        if self.file_name is not None:
            with open(self.file_name, 'a') as file:
                file.write(out)
        else:
            print(out, end=None)

        return


def main(args: 'list[str]'):

    output_file = args[1] if len(args) == 2 else None

    # Create an MQTT Client
    client = mqtt.Client('DATA-LOG')

    # Connect the payload client to the mqtt broker
    client.connect('192.168.1.100', port=1883)

    # Subscribe to the agent topic to logg
    topic = 'logger/rssi/#'
    client.subscribe(topic)

    logger = Logger(output_file, ['timestamp', 'uid', 'target', 'rssi', 'dist_est', 'dist_low', 'dist_high'])

    # Set the callback function
    client.message_callback_add(topic, logger.log_callback)

    try:
        # Start the client
        client.loop_start()

        # Wait for the message to be received and for the callback to be called until program exited
        while True:
            time.sleep(5)
    except:
        pass
    finally:
        # Stop and disconnect the client
        client.loop_stop()
        client.disconnect()

    return


if __name__ == '__main__':
    main(sys.argv)
