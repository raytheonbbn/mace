::  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

::  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



@ECHO off

::Open Docker Desktop
echo Opening Docker, give it time to startup
START C:\"Program Files"\Docker\Docker\"Docker Desktop"

::Give Docker time to startup
timeout 30

::Remove previous mace image if it exists
echo Removing previous MACE image if present
docker rm mace

::Load mace image
echo Loading current MACE image
docker load -i mace-docker.tar.gz