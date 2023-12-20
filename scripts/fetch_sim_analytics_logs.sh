#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

mkdir -p ./docker/logs
docker cp mace:/home/mace/scripts/docker/logs ./docker
docker cp mace:/home/mace/logs ./docker
cp -r logs ./docker
#docker cp mace:/home/mace/scripts/logs ./docker
