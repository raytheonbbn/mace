#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import ssl
import json
import numpy as np
import paho.mqtt.client as mqtt

class CommunicationManager():

  def __init__(self, host="localhost", port=1883):
    """
    Connects to the MQTT client and subscribes to the target and payload state topics
    """
    # Save the UUID, host and port
    self.host = host
    self.port = port

    # List of items to subscribe to
    self.subscribe_list = []
    self.subscribe_list.append("blue_force/server/target/state")
    self.subscribe_list.append("command/server/payload/state")

    # Holds information about the different systems
    self.target_information = {}
    self.payload_information   = {}

    # Connect to the client to for sending
    self.client = mqtt.Client()
    self.client.on_connect = self.on_connect
    self.client.tls_set(ca_certs="./ca_certificates/ca.crt", tls_version=ssl.PROTOCOL_TLSv1_2)
    self.client.connect(host=self.host, port=self.port)

    # Start the client
    self.client.loop_start()

  def on_connect(self, client, userdata, flags, rc):
    """
    Subscribe to each of the topics and point them to the message_callback function
    """
    for topic in self.subscribe_list:
      client.subscribe(topic)
    client.on_message = self.message_callback

  def message_callback(self, client, userdata, message):
    """
    Callback function that handles each of the incoming messages. 
    It saves the target and payload information in self.target_information and self.payload_information
    """

    # Read the message to a JSON element
    json_msg = str(message.payload.decode('utf-8', 'ignore'))
    data = json.loads(json_msg)

    # If its a target
    if "Target_" in data["uid"]:

      # Ignore IDLE type
      if data["type"] == "IDLE":
        return

      # If we have not seen this target before, create the it
      if (data["uid"] not in self.target_information):
        self.target_information[data["uid"]] = {}

      # Update the information
      self.target_information[data["uid"]]["type"]              = data["type"]
      self.target_information[data["uid"]]["latitude"]          = data["latitude"]
      self.target_information[data["uid"]]["longitude"]         = data["longitude"]
      self.target_information[data["uid"]]["altitude"]          = data["altitude"]
      self.target_information[data["uid"]]["discovered"]        = data["discovered"]
      self.target_information[data["uid"]]["captured"]          = data["captured"]
      self.target_information[data["uid"]]["ready_for_capture"] = True

      if data["type"] == "LINK":
        targets = list(data["networks"].values())[0]
        self.target_information[data["uid"]]["required_payloads"]       = len(targets)
        self.target_information[data["uid"]]["networks"]                = str(targets)
        self.target_information[data["uid"]]["networks_captured"]       = list(data["networks_captured"].values())[0]
      else:
        self.target_information[data["uid"]]["required_payloads"]       = data["required_payloads"]
        self.target_information[data["uid"]]["networks"]                = str([data["uid"]])
        self.target_information[data["uid"]]["networks_captured"]       = data["captured"]

      # If this target has a countdown and current duration compute the readiness based on this
      if "current_duration" in data.keys():
        self.target_information[data["uid"]]["ready_for_capture"] = not (0 - 1e-6 <= float(data["current_duration"]) <= 0 + 1e-6)
      
    # If its a quad rotor
    elif "Quad_" in data["uid"] or "Rover_" in data["uid"]:
      # If we have not seen this target before, create the it
      if data["uid"] not in self.target_information:
        self.payload_information[data["uid"]] = {}
        
      # Update the information
      self.payload_information[data["uid"]]["latitude"]    = data["latitude"]
      self.payload_information[data["uid"]]["longitude"]   = data["longitude"]
      self.payload_information[data["uid"]]["altitude"]    = data["altitude"]

      # If its a rover mark it as a ground robot
      self.payload_information[data["uid"]]["ground"]      = False
      if "Rover_" in data["uid"]:
        self.payload_information[data["uid"]]["ground"]    = True
        
    # Else we are not sure what this message is
    else:
      print("Error message type not known: {}".format(data))
     
  def disconnect(self):
    """
    Disconnect from each of the topics  
    """
    self.client.loop_stop()
    for topic in self.subscribe_list:
      self.client.unsubscribe(topic)
    self.client.disconnect()

  def get_target_information(self):
    """
    Get the target information
    """
    return self.target_information

  def get_payload_information(self):
    """
    Return the payload information
    """
    return self.payload_information

  def payload_goto(self, uid, position):
    """
    Send a given payload to a given position
    """
    # Get the lat lon and alt
    lat, lon, alt = position
    # Create the task template
    task_template = '{agent:"%s", "latitude": %f, "longitude": %f, "altitude": %f}'
    task = task_template % (uid, lat, lon, alt)
    # Send the message
    self.client.publish("sim/agent/tasking/goto", task) 

  def reset_simulation(self, tag):
    """
    Send the reset command, and a tag which is used to save the log file
    """
    # Create the task template
    reset_template = '{command:"%s", "tag":"%s"}'
    task = reset_template % ("reset", tag)
    # Send the message
    self.client.publish("command/server/command", task) 

    