//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

import java.util.HashMap;

public abstract class SimTargetConfig {
    protected String targetId;
    protected double latitude;
    protected double longitude;
    protected double altitude;
    protected String type;
    protected boolean suppression;
    protected double discoverDistance;

    public SimTargetConfig(String targetId, double lat, double lon, double alt, double discoverDistance, String type){
        this.targetId = targetId;
        this.latitude = lat;
        this.longitude = lon;
        this.altitude = alt;
        this.discoverDistance = discoverDistance;
        this.type = type;   
    }

    public SimTargetConfig(SimTargetConfig config, String type) {
        this.targetId = config.targetId;
        this.latitude = config.latitude;
        this.longitude = config.longitude;
        this.altitude = config.altitude;
        this.discoverDistance = config.discoverDistance;
        this.type = type;
    }
}
