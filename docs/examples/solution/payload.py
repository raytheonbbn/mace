#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import copy

from mission_states import PayloadState
from mission_object import MissionObject


# Implements a payload
class Payload(MissionObject):

  def __init__(self, uid=None, position=None, state=PayloadState.IDLE, comm_link=None, ground_vehicle=False, altitude_constant=0):
    """
    Sets the uid, position, and state. It also holds a communication link, home based and current target and waypoint
    """
    super().__init__(uid=uid, position=position, state=state)

    # Holds the comm link required to send the robot goals
    if comm_link is None:
      print("Error: payload requires a comm_link")
      exit()
    self.comm_link = comm_link

    # Declare if its a ground vehicle or not
    self.ground_vehicle = ground_vehicle

    # Declare a home base, current robot state
    self.home_base = None

    # Declare the altitude of this vehicle
    self.altitude = 0
    if not ground_vehicle:
      # self.altitude = 0.5 + (0.75 * altitude_constant)
      self.altitude = 1

    # Holds the current target uid
    self.target_uid = None

    # Holds the current waypoint
    self.waypoint = None

  def set_position(self, position):
    """
    Updates the current position of the payload. 
    If no home base has been set, the current position is its home base
    """
    super().set_position(position=position)

    # Also update the home base
    if self.home_base is None:
      self.home_base = copy.copy(self.position)

  def return_home(self):
    """
    Send the payload to its home base
    """
    self.send_to_waypoint(self.home_base)

  def distance_to_current_waypoint(self):
    """
    Compute the distance between the payloads current position and the current waypoint
    """
    return self.distance_to_point(self.waypoint)

  def send_to_waypoint(self, waypoint):
    """
    Send the payload to a given waypoint
    """
    # Update the robot goal
    self.waypoint = list(waypoint)
    # Replace the waypoints z position with the correct altitude
    self.waypoint[2] = self.altitude
    self.waypoint = tuple(self.waypoint)
    # Send the waypoint on over the comms to the simulation
    self.comm_link.payload_goto(self.uid, self.waypoint)

  def update_state(self, state_information):
    """
    Update the state of the payload
    """
    
    # Update the position
    pos = (state_information["latitude"], state_information["longitude"], state_information["altitude"])
    self.set_position(pos)