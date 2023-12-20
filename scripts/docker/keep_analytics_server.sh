#! /bin/bash

#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  

pushd src/analytics_server
java -cp build/libs/analytics_server-0.1.jar:build/dependencies/* com.bbn.mace.server.Server
