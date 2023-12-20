#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

mosquitto_pub -t "sim/agent/tasking/goto" -m '{agent:"Quad_0", "latitude": 42.3899, "longitude":-71.1463, "altitude":10}'

