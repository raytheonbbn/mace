#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  





import argparse
from mission_controller import MissionController

def main():
  """
  Main function gets the arguments and launches a mission controller
  """

  # Get the command line arguments
  parser = argparse.ArgumentParser()
  parser.add_argument('--staged', action="store_true", default=False)
  parser.add_argument('--search_pattern', type=str, default="RandomWalk", help="(RandomWalk, RandomWalkCoverage)")
  parser.add_argument('--tag', type=str, default="", help="The tag used when saving the log file")
  args = parser.parse_args()

  # Start the mission
  print("Starting mission")
  mission = MissionController(staged=args.staged, search=args.search_pattern, tag=args.tag)
  mission.start()
  print("Mission complete")

if __name__ == '__main__':
  main()
