#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



class SearchPattern():
  
  def __init__(self, search_area=[[42.3899, 42.3914], [-71.1461, -71.1469]], altitude=3):
    """
    Sets a set search area, and search height
    Defaults to the BBN parking lot
    """
    self.x_range = search_area[0]
    self.y_range = search_area[1]
    self.altitude = altitude
    self.collision_avoidance_required = True
    
  def get_point(self, current_location):
    """
    Returns the next search point in the search pattern
    Note: This is an abstract method that should be implemented by child classes
    """
    raise NotImplementedError

  def point_accepted(self, current_location, goal):
    """
    Handles all behavior after a point has been accepted by the mission controllers
    Note: This is an abstract method that should be implemented by child classes
    """
    raise NotImplementedError

