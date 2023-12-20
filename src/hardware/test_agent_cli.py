#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse
import paho.mqtt.client as mqtt
import logging
import json
import time

'''
Helper script to interact with the Agent API using a CLI interface.
'''

logger = logging.getLogger(__name__)


def __on_connect(self, client, userdata, flags, rc):
    """
    Callback function used to log MQTT client connection status
    """
    if rc == 0:
        print('Successfully connected to the MQTT broker')
    else:
        print('Payload was unable to successfully connect to the MQTT client')

    return


def _topic_callback(client, userdata, message):
    json_msg = str(message.payload.decode('utf-8', 'ignore'))

    # Convert the JSON message to a python dictionary
    msg = json.loads(json_msg)

    print(f'incoming message on {userdata}: {msg}')


def subscribe(client: mqtt.Client, payload_uid, is_sim=True):
    '''Subscribe to MQTT topics'''
    if is_sim:
        client.subscribe(payload_uid + '/agent/#')
        client.message_callback_add(payload_uid + '/agent/#', _topic_callback)
    else:
        client.subscribe('agent/#')
        client.message_callback_add('agent/#', _topic_callback)


def setup_client(args):
    '''Setup mQTT client for Agent API'''

    # Create an MQTT Client
    client = mqtt.Client('AGENT_CLI_' + args.payload_uid)

    # Setting the client callback function for successful connections
    client.on_connect = __on_connect

    logger.debug(f'Attempting to connect to the host: {args.agent_host}')

    # Connect the payload client to the mqtt broker
    client.connect(args.agent_host, port=args.agent_port)

    #subscribe(client, args.payload_uid)

    return client


def set_intent(client: mqtt.Client, topic, intent):
    '''Send a payload intent configuration message'''

    msg = {
        'intent': intent
    }

    msg_json = json.dumps(msg)

    client.publish(topic, msg_json)


def send_msg(client: mqtt.Client, topic, dest_payload, dest_topic, body):
    '''Send a message to another agent.'''
    msg = {
        'uid': dest_payload,
        'topic': dest_topic,
        'body': body
    }

    msg_json = json.dumps(msg)

    client.publish(topic, msg_json)


def get_help():
    return '''
    Agent CLI commands:
        help                            this help text
        query [uid|location]            query payload data
        send payload_uid topic msg      send a different agent a json formatted message
        config [IDLE|MASS|PERI|LINK]    configure the intent of the attached payload
    '''


def main_loop(args, prefix, client):
    # enter execution loop
    while True:
        cmd_raw = input(f'agent_{args.payload_uid}> ')

        cmd = cmd_raw.split()

        if len(cmd) < 1:
            continue

        # switch on cmd keyword
        if cmd[0] == 'query':  # query command
            if cmd[1] == 'uid':  # query uid
                logger.debug(f'publishing to {prefix + args.query_uid_topic}')
                client.publish(prefix + args.query_uid_topic, '')
            elif cmd[1] == 'location':  # query location
                logger.debug(
                    f'publishing to {prefix + args.query_location_topic}')
                client.publish(prefix + args.query_location_topic, '')
            else:
                print(f'unknown argument: {cmd[1]}')
                continue
        elif cmd[0] == 'send':  # send message to another agent
            if len(cmd) != 4:
                print('invalid parameters for agent to agent message')
            else:
                send_msg(client, prefix + args.send_message_topic,
                         dest_payload=cmd[1], dest_topic=cmd[2], body=cmd[3])
        elif cmd[0] == 'config':  # configure payload
            if len(cmd) == 2 and cmd[1] in ['IDLE', 'MASS', 'PERI', 'LINK']:
                set_intent(client, prefix + args.agent_config_topic, cmd[1])
            else:
                print('invalid parameter to config option.')
        elif cmd[0] == 'help':  # help text
            print('Help text:')
            print(get_help())
        elif cmd[0] == 'exit':
            break
        else:
            print(f'unknown command: {cmd[0]}')
            print(get_help())


def run():

    logging.basicConfig(level=logging.ERROR)

    # Create a new argument parser
    parser = argparse.ArgumentParser(
        description='System used to configure the device as a payload and to execute payload behavior')

    # Add the desired arguments
    parser.add_argument('--agent_host', type=str, default='127.0.0.1',
                        help='The URL or IP address of the payload broker that the payload and agent MQTT clients should connect to')
    parser.add_argument('--agent_port', type=int, default=1884,
                        help='The port number of the MQTT broker that the agent client should connect to')
    parser.add_argument('--identifier', type=str, default='PAYLOAD',
                        help='The identifier that should be used to distinguish payloads from targets')
    parser.add_argument('--payload_uid', type=str, default=None,
                        help='The UID that the payload should be assigned. This should only be assigned if more than one payload instance is running on a system')
    parser.add_argument('--query_uid_topic', type=str, default='payload/agent/query/uid',
                        help='The topic that the payload should subscribe to for UID queries')
    parser.add_argument('--query_location_topic', type=str, default='payload/agent/query/location',
                        help='The topic that the payload should subscribe to for location queries')
    parser.add_argument('--query_uid_response_topic', type=str, default='agent/payload/query/uid',
                        help='The topic that the created agents should subscribe to for UID query responses')
    parser.add_argument('--query_location_response_topic', type=str, default='agent/payload/query/location',
                        help='The topic that the created agents should subscribe to for location query responses')
    parser.add_argument('--send_message_topic', type=str, default='agent/agent/message',
                        help='The topic that the payload should subscribe to for requests to send messages')
    parser.add_argument('--agent_config_topic', type=str, default='payload/agent/configurations',
                        help='The topic that the payload should subscribe to for configuration change requests sent by the agent')
    parser.add_argument('--intent_change_notification_topic', type=str, default='agent/payload/notification/intent',
                        help='The topic that the payload sends notifications to the agent over regarding configuration changes made by the command station')
    parser.add_argument('--in_range_notification_topic', type=str, default='agent/payload/notification/in_range',
                        help='The topic that the payload sends notifications over regarding the payload coming into or exiting a target\'s detection range')
    parser.add_argument('--capture_notification_topic', type=str, default='agent/payload/notification/capture',
                        help='The topic that the payload sends target capture notifications over')
    parser.add_argument('--sim', action='store_true')

    args = parser.parse_args()

    prefix = args.payload_uid + '/' if args.sim else ''  # is sim

    if args.sim:
        args.agent_port = 1883

    client = setup_client(args)
    time.sleep(1)

    try:
        client.loop_start()

        main_loop(args, prefix, client)

    except:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == '__main__':
    run()
