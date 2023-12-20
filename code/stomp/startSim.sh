#  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

#  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



#!/bin/bash
source ccastConfig.properties
source localDevice.properties

pushd ../../scripts
./start_analytics_server.sh
popd

./stomp_start_targets.sh
./stomp_start_payloads.sh

java -cp build/libs/stomp-0.1-all.jar -Dsun.awt.disableMixing=True com.bbn.ccast.visualizer.WorldWindVisualization --propertiesFile ccastConfig.properties --propertiesFile sim.properties --propertiesFile localDevice.properties --propertiesFile regions/cambridge/cambridgeRegion.properties 

./stopSim.sh