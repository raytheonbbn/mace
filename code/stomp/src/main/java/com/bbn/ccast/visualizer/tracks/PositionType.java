//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

public enum PositionType {
    GOTO("GOTO"),
    LOOKPOINT("LOOKPOINT"),
    LOITER("LOITER"),
    LAND("LAND"),
    RTL("RTL")
    ;

    private final String textStr;

    PositionType(final String textStr) {
        this.textStr = textStr;
    }

    @Override
    public String toString() {
        return textStr;
    }
}