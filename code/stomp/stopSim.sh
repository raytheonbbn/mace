#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash

set +e
./stomp_stop_payloads.sh
./stomp_stop_targets.sh
pushd ../../scripts
./stop_analytics_server.sh
rm -r paho*
popd
set -e
exit
