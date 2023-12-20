#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import enum
import geopy
import geopy.distance

from search_pattern import SearchPattern
from utils import break_search_area_into_rectangles

# Holds the path state
class PathState(enum.Enum):
    UP            = 1
    FIRST_RIGHT   = 2
    DOWN          = 3
    SECOND_RIGHT  = 4

# Breaks the Search Area up into a set of rectangles than are then explored using a parallel sweep
class ParallelSweep(SearchPattern):
  
  def __init__(self, robot_names, meters_between_lines=1, search_area=[[42.3899, 42.3914], [-71.1461, -71.1469]]):
    """
    Sets a set search area, and search height
    Defaults to the BBN parking lot
    """
    super().__init__(search_area=search_area)

    # Turn off collision avoidance for this technique
    self.collision_avoidance_required = False

    # Save the total number of robots
    self.robot_names = robot_names
    self.total_robots = len(self.robot_names)

    # Break the search area up into equally sized rectangles
    divided_search_areas = break_search_area_into_rectangles(x_range=self.x_range,
                                                             y_range=self.y_range,
                                                             number_partitions=self.total_robots)

    # Get the individual search ranges
    self.divided_x_ranges = divided_search_areas[0]
    self.divided_y_ranges = divided_search_areas[1]

    # Pre-compute the search path
    self.robot_paths = self.compute_parallel_sweep_path(meters_between_lines)

    # Create a current point index, which allows us to track which point each robot is currently visiting
    self.current_point_index = {}
    for robot in self.robot_names:
      self.current_point_index[robot] = 0
    
  def get_point(self, current_location, robot_name):
    """
    Returns the next point for a given robot
    """
    # If this robot name is not known return none
    if robot_name not in self.robot_names:
      return None
    # Get the next point
    index = self.current_point_index[robot_name]
    point = self.robot_paths[robot_name][index]
    self.current_point_index[robot_name] += 1

    # Make sure we dont go over all the indices
    if self.current_point_index[robot_name] >= len(self.robot_paths[robot_name]):
      self.current_point_index[robot_name] = 0

    # Return the point
    return (point[0], point[1], self.altitude)

  def point_accepted(self, current_location, goal):
    """
    This class does not require any post point behavior
    """
    return

  def compute_parallel_sweep_path(self, delta):
    """
    Computes each of the parallel sweep paths where each parallel line is
    delta distance away from the previous one
    """

    # Declare a dictionary to hold the final paths
    final_paths = {}

    # For each of the robots
    for robot, cur_x_range, cur_y_range in zip(self.robot_names, self.divided_x_ranges, self.divided_y_ranges):

      # Create the path as a list
      final_paths[robot] = []

      # Create the starting point
      start_point = (cur_x_range[0], cur_y_range[0])
      final_paths[robot].append(start_point)

      # Define the ending and current x
      current_x = cur_x_range[0]
      ending_x = cur_x_range[1]

      # Holds the path state
      state = PathState.UP

      # Create the path
      while current_x < ending_x:

        # Based on our state, add to the path
        if state == PathState.UP:
          next_point = (current_x, cur_y_range[1])

        elif state == PathState.DOWN:
          next_point = (current_x, cur_y_range[0])

        elif (state == PathState.FIRST_RIGHT) or (state == PathState.SECOND_RIGHT):
          # Use geopy to add delta to the current point
          starting_point = geopy.Point(final_paths[robot][-1])
          d = geopy.distance.distance(meters=delta)
          new_point = d.destination(point=starting_point, bearing=0)

          # Save the new point
          next_point = (new_point[0], new_point[1])
          current_x = new_point[0]

        # Add the next point to the path
        final_paths[robot].append(next_point)
        # Go to the next state
        next_state = state.value + 1
        if next_state >= 5:
          next_state = 1
        state = PathState(next_state)

    # Return the paths
    return final_paths
      


