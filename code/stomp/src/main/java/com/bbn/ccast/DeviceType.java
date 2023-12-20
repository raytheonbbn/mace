//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast;

public enum DeviceType {

    ROTOR_CRAFT("Quad", "a-f-A-M-H-Q", "34ae1613-9645-4222-a9d2-e5f243dea2865/Military/UAV_quad.png"),
    ROVER("Rover", "a-f-G", "34ae1613-9645-4222-a9d2-e5f243dea2865/Transportation/Car8.png"),
    FIXED_WING("FixedWing", "a-f-A", "34ae1613-9645-4222-a9d2-e5f243dea2865/Transportation/Plane8.png"),
    
    // TODO ensure that we're okay with empty strings here
    DISPATCHER("Dispatcher", "", "");
	
    private String name;
    private String cotType;
    private String iconsetpath;

    DeviceType(String name, String cotType, String iconsetpath) {
        this.name = name;
        this.cotType = cotType;
        this.iconsetpath = iconsetpath;
    }

    public String getName() {
        return this.name;
    }

    public String getCotType() {
        return this.cotType;
    }

    public String getIconsetpath() {
        return this.iconsetpath;
    }

}
