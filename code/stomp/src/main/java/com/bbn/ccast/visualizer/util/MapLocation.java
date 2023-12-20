//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import gov.nasa.worldwind.geom.Position;

public enum MapLocation {
    RAYTHEON("Raytheon BBN", Position.fromDegrees(42.389938, -71.146389, 1000)),
    CAPITOL("US Capitol", Position.fromDegrees(38.889819, -77.010123, 1000)),
    COLORADO("Colorado Fields",Position.fromDegrees(39.0174, -104.894105, 3000)),
    WEST_POINT("West Point",Position.fromDegrees(41.391124, -73.953023, 1000));

    private final String name;
    private final Position pos;

    MapLocation(String name, Position position) {
        this.name = name;
        this.pos = position;
    }

    public String getName() {
        return name;
    }

    public Position getPos() {
        return pos;
    }
}
