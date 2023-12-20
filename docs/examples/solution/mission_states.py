#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



import enum

# Create a state Enum to keep track of the robots current state
class PayloadState(enum.Enum):
  IDLE                = 0
  SEARCHING           = 1
  ASSIGNED_TARGET     = 2
  RECAPTURE_REQUIRED  = 3
  RECAPTURING         = 4
  RETURN_TO_BASE      = 5

# Create an enum to keep track of the target status
class TargetState(enum.Enum):
  UNDISCOVERED      = 0
  DISCOVERED        = 1
  ASSIGNED_CAPTURE  = 2
  CAPTURED          = 3
  TIMEOUT           = 4

# Create an enum to hold the different target types
class TargetType(enum.Enum):
  IDLE = 0
  MASS = 1
  PERI = 2
  LINK = 3