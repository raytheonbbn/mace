#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import paho.mqtt.client as mqtt
import logging
import json
import time
import geopy.distance
import math

'''
This script shows an example of tasking a simulated vehicle through MQTT.  It sends a list of points for the vehicle to drive to with some time delay between them and prints out the vehicle's didistance from the current point.
'''

#Change the agent_uuid to match the sim vehicle you wish to task.
agent_uuid = "Quad_0"

#These values must point to an active MQTT server such as mosquitto
host = "localhost"
port = 1883

#This topic is used to task sim vehicles
tasking_topic = "sim/agent/tasking/goto"

#This topic receives the payload state of all simulated and real hardware from the analytics server.
state_topic = "command/server/payload/state"

#This is the basic format of a json message used to task a sim vehicle. See get_task_message().
task_template = '{agent:"%s", "latitude": %f, "longitude": %f, "altitude": %f}'

#A list of (latitude, longitude, altitude) positions to task to.
position_list = [(42.3900, -71.1462, 10), (42.3898, -71.1462, 10), (42.3895, -71.1462, 10)]

#The current goal position.
goal = None

def get_task_msg(lat, lon, alt):
  '''
  Construct a task in json format to be sent over MQTT to a sim vehicle.
  '''
  global agent_uuid
  task = task_template % (agent_uuid, lat, lon, alt)
  return task

def main():
  global host, port, goal
  connect_client(host, port)
  start_client()

  print("Beginning tasking.")
  for lat, lon, alt in position_list:
    goal = (lat, lon, alt)
    task = get_task_msg(lat, lon, alt)
    print("Sending task " + task)
    client.publish(tasking_topic, task) 
    time.sleep(15)

  print('Done.  Keeping alive to listen to state.')
  while True:
    time.sleep(60)


def connect_client(host, port=1883):
  """
  Connect the client to the desired broker
  """
  logging.debug('Creating a new MQTT client.')
  
  # Create an MQTT Client
  global client
  client = mqtt.Client()
  
  # Setting the client callback function for successful connections
  client.on_connect = __on_connect
  
  print(f'Attempting to connect to the host: {host}')
  
  # Connect the payload client to the mqtt broker
  client.connect(host, port=port)

  return


def disconnect_client():
  """
  Disconnect the client from the broker
  """
  global client
  client.disconnect()
  
  return


def start_client():
  """
  Start the payload MQTT Client
  """
  # Start the payload as an mqtt client
  global client
  client.loop_start()
  
  return


def stop_client():
  """
  Stop the MQTT client
  """
  global client, tasking_topic 
  # This should be state_topic
  client.unsubscribe(tasking_topic)
  client.loop_stop()
   
  return


def __on_connect(client, userdata, flags, rc):
  """
  Callback function used to log MQTT client connection status
  """
  global state_topic
  if rc == 0:
    print('Successfully connected to the MQTT broker')
    print(f'Subscribing to {state_topic}')
    client.subscribe(state_topic)
    client.on_message = handle_state_msg
  else:
      logging.debug('Unable to successfully connect to the MQTT client')

  return


def handle_state_msg(client, userdata, message):
  """
  Callback function responsible for receiving the vehicle's GPS position.
  """
  # Read the message to a JSON element
  global state_topic, agent_uuid, goal
  json_msg = str(message.payload.decode('utf-8', 'ignore'))

  # Convert the JSON message to a python dictionary
  state = json.loads(json_msg)
  if agent_uuid in state_topic or agent_uuid == state['uid']:
    #Optionally, uncomment the line below to see the full sim vehicle state.
    print('Received state update: ' + json_msg)   
    lat = state['latitude']
    lon = state['longitude']
    alt = state['altitude']
    distance_from_goal = get_distance((lat, lon, alt), goal)
    print("Platform is " + str(distance_from_goal) + " meters from the goal.")

def get_distance(source, destination):
  if source is None or destination is None:
    return None 
  s_lat = source[0]
  s_lon = source[1]
  s_alt = source[2]
  d_lat = destination[0]
  d_lon = destination[1]
  d_alt = destination[2]
  horizontal_dist = geopy.distance.distance((s_lat, s_lon), (d_lat, d_lon)).meters
  vertical_dist = d_alt - s_alt
  dist = math.sqrt(horizontal_dist*horizontal_dist + vertical_dist*vertical_dist)
  return dist



if __name__ == '__main__':
  main()
