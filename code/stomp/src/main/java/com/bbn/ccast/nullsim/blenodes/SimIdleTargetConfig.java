//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

public class SimIdleTargetConfig extends SimTargetConfig{

    public SimIdleTargetConfig(String targetId, double lat, double lon, double alt, double discoverDistance) {
        super(targetId, lat, lon, alt, discoverDistance, SimTargetType.IDLE.name());
    }

    public SimIdleTargetConfig(SimTargetConfig config) {
        super(config, SimTargetType.IDLE.name());
    }
}
