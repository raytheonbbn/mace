#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import random
import numpy as np
import matplotlib.pyplot as plt

from search_pattern import SearchPattern

# Implements the random walk search pattern
class RandomWalkWithCoverage(SearchPattern):
  def __init__(self, search_area=[[42.3899, 42.3914], [-71.1461, -71.1469]], resolution=25):
    """
    Sets a set search area, and search height
    Defaults to the BBN parking lot
    """
    # Call the original search area
    super().__init__(search_area=search_area)

    # Save the search area
    self.search_area = search_area

    # Used to compute the mapping between geo location and coverage grid
    self.x_index = None
    self.y_index = None

    # Create a coverage grid
    self.resolution = resolution
    self.coverage_grid = np.zeros((self.resolution, self.resolution), dtype=float)

    # Create the plot
    plt.figure(1)
    plt.title("Coverage grid")
    plt.ion()

    plt.show()

  def get_point(self, current_location, robot_name=None):
    """
    Returns a random point inside the given search area, based on the current point
    """
    # Declare a sample size
    # We will try and generate sample size random positions before giving up
    SAMPLE_SIZE = 10

    # Create the list of suggested points and scores
    current_point   = None
    current_score   = -np.inf

    # Generate sample size random points
    for i in range(SAMPLE_SIZE):

      # Generate random sample points
      x = random.uniform(np.min(self.x_range), np.max(self.x_range))
      y = random.uniform(np.min(self.y_range), np.max(self.y_range))
      z = self.altitude

      # Make sure the point is valid
      x = np.clip(x, np.min(self.x_range), np.max(self.x_range))
      y = np.clip(y, np.min(self.y_range), np.max(self.y_range))

      #  Compute the score of the current point
      # Number of cells covered - number of cells already visited
      score = self._compute_coverage_score(current_location, (x,y))

      # If this is the best path, save it
      if score > current_score:
        current_score = score
        current_point = (x, y, z)

    # Return the new point
    return current_point

  def point_accepted(self, current_location, goal):
    """
    Updates the coverage grid and displays it
    """
    # Update the coverage grid
    self.update_coverage_grid(start_point=current_location[0:2], end_point=goal[0:2])

    # Display the coverage grid
    # self.display_coverage_grid()

    return

  def display_coverage_grid(self):
    """
    Displays the coverage grid
    """
    # Flip the x axis, as these are negative co-ordinates
    plt.imshow(self.coverage_grid, interpolation='none', origin='lower', cmap="binary")
    plt.gca().invert_xaxis()
    plt.draw()
    plt.pause(0.001)

  def _compute_coverage_score(self, start_point, end_point):
    """
    Computes how many of the cells visited are new
    Score is length - number of cells already visited
    """
    # How severely do we punish revisiting something
    VISIT_FACTOR = 2

    # Compute all points along the line
    intermediate_points = self._intermediates(start_point[0:2], end_point, number_points=self.resolution)

    # Holds the set of coverage cells this path will visit
    unique_cells_visited = set()

    # For each point in its path
    for p in intermediate_points:
      # Convert the geo point to a index in the coverage array
      index = self._map_geo_to_coverage(p)
      # Valid index
      assert(index is not None)
      unique_cells_visited.add(index)

    # Compute how many of the cells have already been visited

    visited_value = 0
    for p in unique_cells_visited:
      visited_value += self.coverage_grid[p[0], p[1]] * VISIT_FACTOR

    # Compute the score
    score = len(unique_cells_visited) - visited_value
      
    return score

  def update_coverage_grid(self, start_point, end_point):
    """
    Updates all points along a line in the coverage grid
    """
    # Compute all points along the line
    intermediate_points = self._intermediates(start_point[0:2], end_point, number_points=self.resolution)

    # Determines how fast the previous search areas decays
    DECAY_RATE = 0.95

    # dim the previous paths
    self.coverage_grid = self.coverage_grid * DECAY_RATE
    # Make sure that no value goes below 0.2
    self.coverage_grid[(self.coverage_grid>=0.1) & (self.coverage_grid<0.5)] = 0.2

    # For each point in its path
    for p in intermediate_points:
      # Convert the geo point to a index in the coverage array
      index = self._map_geo_to_coverage(p)
      # Valid index
      assert(index is not None)
      # Update the coverage array
      self.coverage_grid[index[0], index[1]] = 1
      
    return 

  def _has_point_been_visited(self, point):
    """
    Returns true if the point has been visited before
    """
    # Convert the geo point to a index in the coverage array
    index = self._map_geo_to_coverage(point)
    # Return true if the cell at that point is greater than 0
    return self.coverage_grid[index[0], index[1]] > 0.1

  def _compute_mapping_arrays(self): 
    """
    Computes the mapping arrays.
    Each array self.x_index and self.y_index holds the equivalent geo location
    """
    # Declare a small threshold so that arange's >= sign is not triggered
    thresh = 1e-6

    # Compute the x mapping
    x_start = self.search_area[0][0]
    x_end   = self.search_area[0][1]
    increment = (x_end - x_start) / (self.resolution - 1)
    self.x_index = np.arange(x_start, x_end + thresh, increment)

    # Compute the y mapping
    y_start = self.search_area[1][0]
    y_end   = self.search_area[1][1]
    increment = (y_end - y_start) / (self.resolution - 1)
    self.y_index = np.arange(y_start, y_end + thresh, increment)

    return

  def _smooth_point(self, point):
    """
    Makes sure the point is in the search area
    """ 
    # Put values in temp variables
    x1 = self.search_area[0][0]
    x2 = self.search_area[0][1]
    y1 = self.search_area[1][0] 
    y2 = self.search_area[1][1]

    new_x = point[0]
    new_y = point[1]

    if min(x1, x2) > point[0]:
      new_x = min(x1, x2)
    if point[0] > max(x1, x2):
      new_x = max(x1,x2)
    if min(y1, y2) > point[1]:
      new_y = min(y1, y2)
    if point[1] > max(y1, y2):
      new_y = max(y1, y2)
    
    # Return the point
    return (new_x, new_y)


  def _is_valid_point(self, point):
    """
    Computes whether the current point is inside the search area
    """
    # Put values in temp variables
    x1 = self.search_area[0][0]
    x2 = self.search_area[0][1]
    y1 = self.search_area[1][0] 
    y2 = self.search_area[1][1]

    # Check the x point
    if not (min(x1, x2) < point[0] < max(x1, x2)):
      return False
    # Check the y point
    elif not (min(y1, y2) < point[1] < max(y1, y2)):
      return False
    # The point is valid
    else:
      return True

  def _map_geo_to_coverage(self, point):
    """
    Takes in a point in long and late and converts it into an index used by the coverage graph
    """

    # Compute the mapping array
    if (self.x_index is None) or (self.x_index is None):
      self._compute_mapping_arrays()

    # Smooth the point (Occasionally the point would be out of the search area due to rounding. This was my workaround.)
    point = self._smooth_point(point)
    
    # Check that the point is valid 
    if not self._is_valid_point(point):
      print("This is not a valid point")
      print(point)
      print(self.search_area)
      return None
    
    # Compute the difference between all the points and the current point.
    x_difference = abs(self.x_index - point[0])
    y_difference = abs(self.y_index - point[1])

    # The points closest to zero will be the index
    x_point = np.argmin(x_difference)
    y_point = np.argmin(y_difference)

    # Return that point
    return (x_point, y_point) 

  def _intermediates(self, p1, p2, number_points):
    """"
    Return a list of nb_points equally spaced points between p1 and p2
    """
    # If we have 8 intermediate points, we have 8+1=9 spaces between p1 and p2
    x_spacing = (p2[0] - p1[0]) / (number_points + 1)
    y_spacing = (p2[1] - p1[1]) / (number_points + 1)

    # Get the intermediate points inbetween the start and end point
    intermediate_points = [p1]
    intermediate_points += [[p1[0] + i * x_spacing, p1[1] +  i * y_spacing] for i in range(1, number_points+1)]
    intermediate_points += [p2]

    return intermediate_points

