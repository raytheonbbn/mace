#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import random
import math
import geopy
import geopy.distance

from parallel_sweep import ParallelSweep
from archimedean_spiral import ArchimedeanSpiral

import matplotlib.pyplot as plt

# robot_list = ["Quad1", "Quad2", "Quad3", "Quad4", "Quad5", "Quad6", "Quad7"]
robot_list = ["Quad1", "Quad2"]

test = ParallelSweep(robot_list, meters_between_lines=4)

plt.figure("Parallel Sweep")
# Plot each of the robots search areas
for x_range, y_range in zip(test.divided_x_ranges, test.divided_y_ranges):
    x = [x_range[0], x_range[1], x_range[1], x_range[0], x_range[0]]
    y = [y_range[0], y_range[0], y_range[1], y_range[1], y_range[0]]
    plt.plot(x, y, c="black")

plotting_data = []
for i, robot in enumerate(test.robot_paths):
  for p1, p2 in zip(test.robot_paths[robot][0:-2], test.robot_paths[robot][1:-1]):
    plotting_data.append([p1, p2])

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], c="C{}".format(i))


test = ArchimedeanSpiral(robot_list)
plt.figure("Archimedean Spiral")
# Plot each of the robots search areas
for x_range, y_range in zip(test.divided_x_ranges, test.divided_y_ranges):
    x = [x_range[0], x_range[1], x_range[1], x_range[0], x_range[0]]
    y = [y_range[0], y_range[0], y_range[1], y_range[1], y_range[0]]
    plt.plot(x, y, c="black")

# Plot the spiral
plotting_data = []
for i, robot in enumerate(test.robot_paths):
  print(len(test.robot_paths[robot]))
  for p1, p2 in zip(test.robot_paths[robot][0:-2], test.robot_paths[robot][1:-1]):
    plotting_data.append([p1, p2])

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], c="C{}".format(i))

plt.show()