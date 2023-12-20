#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  


MACE_CONTAINER_ID=$(docker ps -aqf "name=mace")

docker exec -it $MACE_CONTAINER_ID /bin/bash 
