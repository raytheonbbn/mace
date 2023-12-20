#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



from typing import List
from typing import Tuple

def valid_position(position: List[float]) -> bool:
    """
    Computes if a position is a valid longitude latitude altitude position
    """
    # Position
    if position[0] < -90 or position[0] > 90:
        return False
    if position[1] < -90 or position[1] > 90:
        return False
    if position[2] < 0 or position[2] > 1000:
        return False
    return True

def break_search_area_into_rectangles(x_range: List[float], y_range: List[float], number_partitions: int) -> Tuple[List[List[float]], List[List[float]]]:
    """
    Takes in a rectangular search area and breaks it up into number_partitions equally sized rectangular partitions.
    """

    # Make sure the range's are both 2 numbers
    assert(len(x_range) == 2)
    assert(len(y_range) == 2)

    # Compute the size of each new rectangular search area
    x_magnitude = abs(x_range[0] - x_range[1])
    new_rectangular_size = x_magnitude / number_partitions

    # Holds the new search areas 
    new_x_ranges = []
    new_y_ranges = []

    # Compute the new search area
    start_x = x_range[0]
    for i in range(number_partitions):

        # Compute the end_x
        end_x = round(start_x + new_rectangular_size, 10)

        # Compute the new x_range
        new_x_range = [start_x, end_x]
        new_y_range = [y_range[0], y_range[1]]

        # Save these value to the new search areas
        new_x_ranges.append(new_x_range)
        new_y_ranges.append(new_y_range)

        # The new start_x is the ending_x
        start_x = end_x

    # Return the new areas
    return new_x_ranges, new_y_ranges