#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



# Constant values for hardware devices

TARGET_CAPTURED_HANDLE = 8
TARGET_CONFIGURATION_HANDLE = 12

# Target confituration Maps
TARGET_CONFIGURATION_VALUES = {
    'I': 'IDLE',
    'M': 'MASS',
    'P': 'PERI',
    'L': 'LINK'
}

TARGET_CONFIGURATION_KEYS = {
    'IDLE': 'I',
    'MASS': 'M',
    'PERI': 'P',
    'LINK': 'L'
}

# Target capture state Maps
TARGET_CAPTURED_VALUES = {
    'U': 'UNCAPTURED',
    'C': 'CAPTURED',
    'F': 'FAILED'
}

TARGET_CAPTURED_KEYS = {
    'UNCAPTURED': 'U',
    'False': 'U',
    'CAPTURED': 'C',
    'captured': 'C',
    'True': 'C',
    'FAILED': 'F'
}
