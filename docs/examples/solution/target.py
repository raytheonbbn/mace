#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import ast

from mission_states import TargetState
from mission_object import MissionObject


# Implements a target
class Target(MissionObject):
  
  def __init__(self, uid=None, position=None, state=None, target_type=None):
    """
    Sets the uid, position, and state.
    It also holds the type of target, the required number of payloads and its discovered, captured, and ready for capture state
    """
    super().__init__(uid=uid, position=position, state=state)

    # Save what type of target this is, and the required number to capture
    self.type = target_type
    self.required_payloads = None

    # Declare if its discovered or captured
    self.discovered = False
    self.captured = False
    self.ready_for_capture = False

  def update_state(self, state_information):
    """
    This updates the target position, discovered status, capture status and require payload information
    """
    
    # Update the position
    pos = (state_information["latitude"], state_information["longitude"], state_information["altitude"])
    self.set_position(pos)

    # Update the discovered and captured
    self.discovered         = state_information["discovered"]
    self.captured           = state_information["captured"]
    self.ready_for_capture  = state_information["ready_for_capture"]
    self.network_captured   = state_information["networks_captured"]

    # Update the network0
    self.network            = ast.literal_eval(state_information["networks"])

    print("Target {} - Captured {}".format(self.uid, self.captured))

    # Update the state appropriately
    if self.state == TargetState.UNDISCOVERED and self.discovered:
      self.state = TargetState.DISCOVERED
    if self.state == TargetState.ASSIGNED_CAPTURE and self.captured and self.network_captured:
      self.state = TargetState.CAPTURED
    

    # Update the number of payloads required
    self.required_payloads = state_information["required_payloads"]
