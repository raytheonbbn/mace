#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import random

from search_pattern import SearchPattern

# Implements the random walk search pattern
class RandomWalk(SearchPattern):
  
  def get_point(self, current_location=None, robot_name=None):
    """
    Returns a random point inside the given search area
    """
    x = random.uniform(self.x_range[0], self.x_range[1])
    y = random.uniform(self.y_range[0], self.y_range[1])
    z = self.altitude
    return (x, y, z)

  def point_accepted(self, current_location, goal):
    """
    This class does not require any post point behavior
    """
    return