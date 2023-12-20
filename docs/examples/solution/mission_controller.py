#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import time
import numpy as np

from target import Target
from payload import Payload
from utils import valid_position
from random_walk import RandomWalk
from mission_states import TargetType
from mission_states import TargetState
from mission_states import PayloadState
from parallel_sweep import ParallelSweep
from archimedean_spiral import ArchimedeanSpiral
from collision_avoidance import CollisionAvoidance
from communication_manager import CommunicationManager
from random_walk_with_coverage import RandomWalkWithCoverage

class MissionController():

  def __init__(self, staged=True, search="RandomWalk", tag=""):
    """
    Start the mission controller:
    1) Launch the communication manager
    2) Create a search pattern
    3) Get all the target and payload information
    4) Set the mission status to incomplete
    """

    # Determines if we solved it staged or not
    self.staged = staged

    # Create the communication manager
    self.comm_link = CommunicationManager()

    # Save the tag
    self.tag = tag

    # Wait to ensure that the communication link is established and has data
    time.sleep(2)

    # Get all the target and payload information
    target_info = self.comm_link.get_target_information()
    payload_info = self.comm_link.get_payload_information()

    # Create a list of targets
    self.targets = []
    for uid in target_info:
      # Create the target and add it to the list
      t = Target(uid=uid, position=None, state=TargetState.UNDISCOVERED, target_type=TargetType[target_info[uid]["type"]])
      self.targets.append(t)

    # Create a list of payloads
    payload_uids = []
    self.payloads = []
    for i, uid in enumerate(payload_info):
      # Create the target and add it to the list
      p = Payload(uid=uid,
                  position=None,
                  state=PayloadState.IDLE,
                  comm_link=self.comm_link,
                  ground_vehicle=payload_info[uid]["ground"],
                  altitude_constant=i)
      payload_uids.append(uid)
      self.payloads.append(p)

    # Create the search pattern
    if search == "RandomWalk":
      self.search_pattern = RandomWalk()
    elif search == "RandomWalkCoverage":
      self.search_pattern = RandomWalkWithCoverage(resolution=30)
    elif search == "ParallelSweep":
      self.search_pattern = ParallelSweep(robot_names=payload_uids, meters_between_lines=5)
    elif search == "ArchimedeanSpiral":
      self.search_pattern = ArchimedeanSpiral(robot_names=payload_uids)
    else:
      print("Error: Unknown search pattern")
      exit()

    # Get all the information we need for the collision avoidance class
    payload_uids = []
    payload_positions = []
    for uid in payload_info:
      # Save the payload uid
      payload_uids.append(uid)
      lat = payload_info[uid]["latitude"]
      lon = payload_info[uid]["longitude"]
      alt = payload_info[uid]["altitude"]
      # Save the payload pos
      payload_positions.append([lat, lon, alt])
    
    # Create the collision avoidance class
    self.collision_avoidance = CollisionAvoidance(payload_uids=payload_uids, payload_locations=payload_positions)

    # List used to keep track of newly discovered targets
    self.targets_assigned_capture = []

    # Declare the mission status
    self.mission_complete = False
    
  def start(self, reset=True):
    """
    Starts the mission. While the mission is not complete it repeats the following:
    1) Update the target and payload status
    2) Assign idle payloads to a goal 
    3) If new goal might cause a collision, reselect a goal
    4) Check if any payloads have reached their goal
    5) Attempt to capture any discovered payloads (if the mission is staged this step is skipped until all targets are discovered)
    6) Check if there is a payload that can no longer be captured
    7) Start a recapture process if a payload can no longer be captured
    8) Check if any targets has gone from uncaptured to captured
    9) Set mission complete if all targets are captured
    """
    
    # Waits for the GPS data to become fully available
    gps_initialized = False

    # while the mission is incomplete
    while not self.mission_complete:

      # Get the latest payload and target information
      target_info = self.comm_link.get_target_information()
      payload_info = self.comm_link.get_payload_information()

      # Update the position and state of all payloads and targets using the comm_link
      for target in self.targets:
        target.update_state(target_info[target.uid])
      for payload in self.payloads:
        payload.update_state(payload_info[payload.uid])

      # Check you are getting valid GPS data for all drones
      if not gps_initialized:
        # Check each payload
        gps_initialized = True
        for p in self.payloads:
          if not valid_position(p.position):
            gps_initialized = False
        
        # Restart the loop
        continue

      # Display log information to terminal
      self.display_status()

      # Assign goals to all idle payloads
      self.assign_idle_payloads()

      # Check if any payload that is currently searching has completed its search path
      self.check_payload_complete_search_path()

      # Send payloads to capture discovered targets
      if self.staged:
        if self.all_targets_discovered():
          self.attempt_capture_of_discovered()
      else:
        self.attempt_capture_of_discovered()

      # Check if any payload can no longer capture a waypoint due to it becoming unable to capture
      self.check_targets_can_still_be_captured()

      # Any targets that are assigned for recapture check if they can begin the recapture
      self.handle_recapturing()

      # Check if any payload that was trying to capture a node has completed capturing it
      self.check_payload_complete_capture()
      
      # Update the mission state
      self.mission_complete = self.check_mission_status()

      # Keep the loop rate
      time.sleep(1)

    # If the mission is complete return all payloads to base
    for payload in self.payloads:
      payload.return_home()

    # Check if all payloads have returned home
    print("Return home sent")
    all_returned_home = False
    while not all_returned_home:

      # Get the latest payload and target information
      payload_info = self.comm_link.get_payload_information()

      # Update the position and state of all payloads and targets using the comm_link
      for payload in self.payloads:
        payload.update_state(payload_info[payload.uid])

      # Declare that none are home
      all_home = np.full(len(self.payloads), False)
      # Loop through each payload
      for i, p in enumerate(self.payloads):
        # Compute the distance to the home base
        distance_to_home = p.distance_to_point(p.home_base)
        # If they are at home
        if distance_to_home < 0.5:
          all_home[i] = True

      # Check if they are all home
      if np.sum(all_home) == len(self.payloads):
        if reset:
          print("All have returned home, resetting and tagging log file")
          self.comm_link.reset_simulation(self.tag)
        else:
          print("All have returned home, not resetting")
        all_returned_home = True
        
      # Keep the loop rate
      time.sleep(1)

    # Disconnect when we are done
    self.comm_link.disconnect()

  def assign_idle_payloads(self):
    """
    For any payload that is in an IDLE state, get a goal from the search pattern and set that as the payloads goal
    """
    # For each payload
    for payload in self.payloads:
      # If the payload is Idle
      if payload.state == PayloadState.IDLE:

        # Try assign a path to the current payload 10 time
        ATTEMPT_LIMIT = 10

        # While the new goal wont cause a collision
        path_attempt = 0
        path_allowed = False
        while not path_allowed:

          # If we have reached the limit
          if path_attempt >= ATTEMPT_LIMIT:
            return

          # Use the search pattern to get a new point and send it to that point
          waypoint = self.search_pattern.get_point(payload.position, payload.uid)

          # Check if this new payload is likely to cause collision
          if self.search_pattern.collision_avoidance_required:
            path_allowed = self.collision_avoidance.is_path_allowed(payload.uid, waypoint)
            path_attempt += 1
          else:
            path_allowed = True

        # Once we have a collision free goal, update the payload
        payload.send_to_waypoint(waypoint)
        payload.state = PayloadState.SEARCHING

        # Update the new goal of the payload in the collision avoidance
        self.collision_avoidance.update_goal(payload.uid, waypoint)

        # The point has been accepted let the search class know
        self.search_pattern.point_accepted(current_location=payload.position, goal=waypoint)

    return

  def check_payload_complete_search_path(self):
    """
    Look at each of the payloads and check if it has reached it goal. If it has, set it to and IDLE state
    """
    # For each payload
    for payload in self.payloads:
      if payload.state == PayloadState.SEARCHING:
        # Declare a arrival threshold
        AT_GOAL_THRESHOLD = 5
        # Check if the payload has arrived at a goal
        distance_to_current_waypoint = payload.distance_to_current_waypoint()
        if distance_to_current_waypoint < AT_GOAL_THRESHOLD:
          payload.state = PayloadState.IDLE
          # Update the payloads position and goal in the collision avoidance class
          self.collision_avoidance.update_position(payload.uid, payload.position)
          self.collision_avoidance.update_goal(payload.uid, payload.position)
  
    return

  def compute_detected_and_captured_targets(self):
    """
    For all targets create a list of detected and captured targets
    """
    detected_targets = []
    captured_targets = []

    # For each target
    for target in self.targets:
      # Check its state
      if target.state == TargetState.DISCOVERED or target.state == TargetState.ASSIGNED_CAPTURE:
        detected_targets.append(target)
      elif target.state == TargetState.CAPTURED:
        captured_targets.append(target)

    # Return it in the appropriate list
    return detected_targets, captured_targets

  def display_status(self):
    """
    Display payload and target information to terminal
    """

    # Print the payload states
    print("----------------------")
    print("Payload States:")
    for payload in self.payloads:
      print("|--{}: {} -- target: {}".format(payload.uid, payload.state, payload.target_uid))

    # Compute the detected and captured targets
    detected_targets, captured_targets = self.compute_detected_and_captured_targets()

    # Print all detected targets
    print("Detected Targets {}/{}".format(len(detected_targets), len(self.targets)))
    for target in detected_targets:
      print("|--{}: {}".format(target.uid, target.state))

    # Display all captured targets
    print("Captured Targets {}/{}".format(len(captured_targets), len(self.targets)))
    for target in captured_targets:
      print("|--{}".format(target.uid))
    return

  def check_mission_status(self):
    """
    Returns true if all targets are captured
    """

    # For each target
    for target in self.targets:
      # Check its state
      if target.state != TargetState.CAPTURED:
        return False
    
    # Otherwise all are captured return true
    return True

  def check_targets_can_still_be_captured(self):
    """
    If any target becomes expired, assign all payloads trying to capture that target into a recapture state
    """

    # Compute the detected and captured targets
    detected_targets, captured_targets = self.compute_detected_and_captured_targets()

    # Compute all targets that have expired
    expired_target_uids = []
    for target in detected_targets:
      if not target.ready_for_capture:
        expired_target_uids.append(target.uid)

    # Move all payloads that have are assigned a target which cant be captured to a recapture state
    for payload in self.payloads:
      # If we are waiting at a target
      if payload.distance_to_current_waypoint() <= 5:
        # And its expired
        if payload.target_uid in expired_target_uids:
          # Move away
          payload.state = PayloadState.RECAPTURE_REQUIRED

    # If all payloads for a given target are in recapture mode
    for target in self.targets:
      # If all the payloads for a given target require recapture
      if self.all_payloads_for_target_are_given_state(target.uid, PayloadState.RECAPTURE_REQUIRED):
        for payload in self.payloads:
          # Move all of these to a recapturing state
          if payload.target_uid == target.uid:
            payload.state = PayloadState.RECAPTURING
            payload.return_home()

    return

  def all_payloads_for_target_are_given_state(self, uid, state):
    """
    Check if all payloads for a given target and in a given state.
    """

    all_require_recapture = False
    # Loop through all payloads 
    for payload in self.payloads:
      # If this payload has the given target_uid
      if uid == payload.target_uid:
        if payload.state != state:
          return False
        else:
          # There is at least one
          all_require_recapture = True

    return all_require_recapture

  def handle_recapturing(self):
    """
    For all payloads that are in a recapture state, perform the recapture operation
    This moves all payloads away from the target, until the target becomes available for capture again
    """
    can_recapture = False
    recapture_target = None
    # Check if we can start return for recapturing
    for payload in self.payloads:
      # Check if we are in recapturing mode
      if (payload.state == PayloadState.RECAPTURING):
        # Check if we can recapture that target
        for target in self.targets:
          if ((target.uid == payload.target_uid) and (target.ready_for_capture)):
            can_recapture = True
            recapture_target = target
            break 

    # If we can recapture
    if can_recapture:
      # Send all payloads back to capturing
      for payload in self.payloads:
        if payload.target_uid == recapture_target.uid:
          payload.send_to_waypoint(recapture_target.position)
          payload.state = PayloadState.ASSIGNED_TARGET
          payload.target_uid = recapture_target.uid
    return

  def check_payload_complete_capture(self):
    """
    Check if the payload has completed capturing its given target
    """

    # Compute the detected and captured targets
    detected_targets, captured_targets = self.compute_detected_and_captured_targets()
    captured_targets_uids = [target.uid for target in captured_targets]

    # Move all payloads that have captured a target to idle
    for payload in self.payloads:
      # If it was captured
      if payload.target_uid in captured_targets_uids:
        payload.state = PayloadState.IDLE
        payload.target_uid = None
    return

  def attempt_capture_of_discovered(self):
    """
    For all targets in a discovered state, assign the x closest payloads to capture that state, where x is defined by the target.
    """

    # For each of the discovered targets assign robots for capture
    for target in self.targets:

      # Check if we have discovered that target, and that we haven't already send payloads to capture it, and that we are able to capture it
      if (target.state == TargetState.DISCOVERED) and (target.ready_for_capture == True):

        # Get the list of available payloads
        available_payloads = []
        for payload in self.payloads:
          # Only select payloads that are idle or searching
          if (payload.state == PayloadState.IDLE) or (payload.state == PayloadState.SEARCHING):
            available_payloads.append(payload)

        # If there are too few payloads try the next target
        if len(available_payloads) < target.required_payloads:
          continue

        # Get the targets to capture
        targets_to_capture = []

        # Check if this is a LINK payload.
        if target.type == TargetType.LINK:
          all_discovered = True
          # See if all targets in this network are discovered
          for target_name in target.network:
            # Find the target
            t = self.get_target_from_uid(target_name)
            all_discovered = t.discovered and all_discovered

          # If any of the targets are undiscovered we cant capture it yet
          if not all_discovered:
            continue

          # Assign payloads to each of the targets in the network
          for target_name in target.network:
            t = self.get_target_from_uid(target_name)
            targets_to_capture.append(t)

        # If this is not a LINK node    
        else:
          targets_to_capture.append(target)

        # Now that we have the targets we want to capture.
        for target in targets_to_capture:

          # Get the available payloads (as this will change when using links) and distance to targets
          available_payloads = []
          distances_to_target = []
          for payload in self.payloads:
            # Only select payloads that are idle or searching
            if (payload.state == PayloadState.IDLE) or (payload.state == PayloadState.SEARCHING):
              available_payloads.append(payload)
              distances_to_target.append(payload.distance_to_point(target.position))

          # Sort the lists based on distance
          distances_to_target, available_payloads = zip(*sorted(zip(distances_to_target, available_payloads)))
          # Assign the closest payloads to capture the target
          if target.type == TargetType.LINK:
            selected_payloads = available_payloads[0:1]
          else:
            selected_payloads = available_payloads[0:target.required_payloads]

          # Assign them to capture
          for payload in selected_payloads:
            payload.send_to_waypoint(target.position)
            payload.state = PayloadState.ASSIGNED_TARGET
            payload.target_uid = target.uid
        
          # We have assigned payloads to capture this target
          target.state = TargetState.ASSIGNED_CAPTURE

  def all_targets_discovered(self) -> bool:
    """
    Returns true if all targets are discovered
    """
    for target in self.targets:
      if (target.state == TargetState.UNDISCOVERED):
        return False
    return True

  def get_target_from_uid(self, uid: str) -> Target:
    # Look through the targets
    for t in self.targets:
      # If the UID's match
      if t.uid == uid:
        return t
    return None