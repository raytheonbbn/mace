::  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

::  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



::docker run -p 1883:1883 -p 9091:9091 -p 9092:9092 -p 11312:11312 -p 11313:11313 -it mace

set rover_ports=""
set rosbridge_port="9091"
set rosmaster_port="11312"
::TODO: parse the values below from localDevice.properties
set num_rovers="2"
set num_solos="5"
set num_targets="20"

for /l %%x in (1, 1, %num_rovers%) do (
	set /a "rosbrdige_port=%rosbridge_port%+1"
	set rover_ports="%rover_ports%-p %rosbridge_port%:%rosbridge_port "
	set /a "rosmaster_port=%rosmaster_port%+1"
	set rover_ports="%rover_ports%-p %rosbridge_master%:%rosmaster_port "
)
::echo %rover_ports%
docker run --rm -p 1883:1883 %rover_ports% --name mace-docker -it mace ./scripts/docker/start_all.sh %num_solso% %num_rovers% $num_targets%
