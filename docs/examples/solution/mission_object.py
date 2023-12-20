#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import math
import copy
import geopy.distance

# Base class for all mission objects
class MissionObject():
  def __init__(self, uid=None, position=None, state=None):
    """
    Parent class that holds all mission objectives uids, positions, and states
    """
    # Save the UID, position and state
    self.uid = uid
    self.position = position
    self.state = state

  def set_position(self, position):
    """
    Updates the position of the mission objective
    """
    self.position = position

  def distance_to_point(self, point):
    """
    Computes the distance between the mission object and a given point in meters
    """
    # Init the distance to inf
    distance = math.inf
    # If there is a waypoint and current position, compute the distance
    if (point is not None) and (self.position is not None):
      robot_pos = (self.position[0], self.position[1])
      goal_pos  = (point[0], point[1])
      distance = geopy.distance.distance(robot_pos, goal_pos).meters
    # Return the distance
    return distance

  def update_state(self, state_information):
    """
    Used to update the state of the mission objective
    Note: This is an abstract method that should be implemented by child classes
    """
    raise NotImplementedError