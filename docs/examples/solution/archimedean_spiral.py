#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import math
import geopy
import geopy.distance
import numpy as np

from search_pattern import SearchPattern
from utils import break_search_area_into_rectangles

# Breaks the Search Area up into a set of rectangles than are then explored using a Archimedean spiral
class ArchimedeanSpiral(SearchPattern):

  def __init__(self, robot_names, search_area=[[42.3899, 42.3914], [-71.1461, -71.1469]]):
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
    self.robot_paths = self.compute_archimedean_spiral()

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

  def compute_archimedean_spiral(self):
    """
    For each of the robots it creates a spiral inside the bounding box of the search area
    """

    # Declare a dictionary to hold the final paths
    final_paths = {}

    # For each of the robots
    for robot, cur_x_range, cur_y_range in zip(self.robot_names, self.divided_x_ranges, self.divided_y_ranges):

      # Create the path as a list
      final_paths[robot] = []

      # Compute the center of the search area
      center_x = (cur_x_range[0] + cur_x_range[1]) / 2.0
      center_y = (cur_y_range[0] + cur_y_range[1]) / 2.0

      # Create the starting point
      start_point = (center_x, center_y)

      # Generate the spiral from that point
      spiral_x, spiral_y = self.generate_spiral(start_point)

      # Make sure the spiral does not go outside its bounds
      spiral_x = np.clip(spiral_x, min(cur_x_range), max(cur_x_range))
      spiral_y = np.clip(spiral_y, min(cur_y_range), max(cur_y_range))

      for x, y in zip(spiral_x, spiral_y):
        final_paths[robot].append([x, y])
     
    # Return the paths
    return final_paths

  def generate_spiral(self, center):
    """
    Generates a spiral from the center point
    """

    x = []
    y = []

    NUMBER_OF_ROTATIONS = 20
    for i in range(0, 360 * NUMBER_OF_ROTATIONS, 20):
        
        # Declare the constants
        SCALING_CONSTANT = 0.1
        TIGHTNESS = 0.75

        # Get the angle
        theta = math.radians(i)

        # Get the distance from the center
        radial_distance = SCALING_CONSTANT * math.pow(theta, 1/TIGHTNESS)
        
        # Use geopy to add delta to the current point
        starting_point = geopy.Point(center)
        d = geopy.distance.distance(meters=radial_distance)
        new_point = d.destination(point=starting_point, bearing=math.degrees(theta))
        
        # Save into the array
        x.append(new_point[0])
        y.append(new_point[1])

    # Return the data
    return x, y