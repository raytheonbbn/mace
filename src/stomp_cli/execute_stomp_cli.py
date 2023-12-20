#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import argparse
from display import StompCli


def main():
    # Create a new argument parser
    parser = argparse.ArgumentParser(description='System used to configure the device as a payload and to execute payload behavior')

    # Add the desired arguments
    parser.add_argument('--host', type=str, default='127.0.0.1', help='The URL or IP address of the broker that the STOMP CLI client should connect to')
    parser.add_argument('--target_state_topic', type=str, default='white_force/server/target/state', help='The topic that the STOMP CLI should use to read target state data')
    parser.add_argument('--payload_state_topic', type=str, default='command/server/payload/state', help='The topic that the STOMP CLI should use to read payload state data')
    parser.add_argument('--target_config_topic', type=str, default='server/white_force/target/configurations', help='The topic that the STOMP CLI should use to publish target configurations over')
    parser.add_argument('--payload_server_config_topic', type=str, default='server/command/payload/configurations', help='The topic that the STOMP CLI should use to publish payload configurations over')
    parser.add_argument('--payload_agent_config_topic', type=str, default='payload/agent/configurations', help='The topic that the created agents should use to publish payload configurations over')
    parser.add_argument('--query_payload_uid_topic', type=str, default='payload/agent/query/uid', help='The topic that the created agents should use to query their respective payload\'s UID over')
    parser.add_argument('--query_payload_location_topic', type=str, default='payload/agent/query/location', help='The topic that the created agents should use to query their respective payload\'s location over')
    parser.add_argument('--send_agent_message_topic', type=str, default='agent/agent/message', help='The topic that the created agents should use to send messages to each other over')
    parser.add_argument('--receive_payload_uid_topic', type=str, default='agent/payload/query/uid', help='The topic that the created agents should subscribe to for UID query responses')
    parser.add_argument('--receive_payload_location_topic', type=str, default='agent/payload/query/location', help='The topic that the created agents should subscribe to for location query responses')
    parser.add_argument('--receive_message_topic', type=str, default='agent/agent/message/response', help='The topic that the created agents should use to listen to for new messages sent by other agents')

    # Parse the arguments
    args = parser.parse_args()

    # Start the CLI
    cli = StompCli(args.target_config_topic, args.payload_server_config_topic, args.payload_agent_config_topic, args.query_payload_uid_topic,
        args.query_payload_location_topic, args.send_agent_message_topic, args.receive_payload_uid_topic, args.receive_payload_location_topic,
        args.receive_message_topic)

    cli.connect(args.host, 'StompCli')

    cli.start_client()
    
    cli.subscribe_to_state(args.target_state_topic, args.payload_state_topic)

    # Run the CLi
    while True:
        cli.print_output()
        

if __name__ == '__main__':
    main()