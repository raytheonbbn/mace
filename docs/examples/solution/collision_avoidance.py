#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import copy

from typing import List
from utils import valid_position
from shapely.geometry import LineString

import numpy as np

class CollisionAvoidance():

  def __init__(self, payload_uids: List[str], payload_locations: List[List[float]]):
    """
    Saves the current location of all payloads and payload UIDS.
    Assumes that the payload uids are the same order as the payload locations
    """

    # Validate that you have provided the same number of payload_uids and locations
    assert(len(payload_uids) == len(payload_locations))

    # Save the payload uids
    self.payload_uids = payload_uids

    # Used to hold the payload locations and goals
    self.payload_locations  = {}
    self.payload_goals      = {}

    # Update the payload locations
    for uid, location in zip(payload_uids, payload_locations):

      # Validate that each location is 3 values representing long, lat, alt
      assert(len(location) == 3)
      assert(valid_position(location))

      # Save the position
      self.payload_locations[uid] = location
      self.payload_goals[uid] = location

  def is_path_allowed(self, uid: str,  position: List[float]) -> bool:
    """
    Takes in two uids, and locations, and checks if they cross paths
    """

    # Get the other payloads we need to consider
    considered_payloads = copy.copy(self.payload_uids)
    considered_payloads.remove(uid)

    # For each of the considered payloads
    intersections = []
    for considered_uid in considered_payloads:
      suggested_path = LineString([self.payload_locations[uid][0:2], position[0:2]])
      other_payload_path = LineString([self.payload_locations[considered_uid][0:2], self.payload_goals[considered_uid][0:2]])
      col = suggested_path.intersects(other_payload_path)
      intersections.append(col)

    # If all are false we know there are no intersections
    if np.sum(intersections) == 0:
      return True
    else:
      return False

  def update_position(self, uid: str, position: List[float]) -> bool:
    """
    Updates the position of a specific payload, if the location is valid
    """
    # Update the payload location
    if valid_position(position):
      self.payload_locations[uid] = position
      return True
    # If its not a valid position return False
    return False

  def update_goal(self, uid: str, position: List[float]) -> bool:
    """
    Updates the goal of a specific payload, if the location is valid
    """
    # Update the payload location
    if valid_position(position):
      self.payload_goals[uid] = position
      return True
    # If its not a valid position return False
    return False
