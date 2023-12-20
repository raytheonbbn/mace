//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.util;

import java.io.IOException;
import java.io.Serializable;

/**
 * Simple LLA class to help with JSON serialization/deserialization
 * TODO: think about adding units/type to any or all of these
 */
public class LatLonAlt implements Serializable {
	
	private static final long serialVersionUID = 1L;
	
    public double getLat() {
        return lat;
    }

    public void setLat(double lat) {
        this.lat = lat;
    }

    public double getLon() {
        return lon;
    }

    public void setLon(double lon) {
        this.lon = lon;
    }

    public double getAlt() {
        return alt;
    }

    public void setAlt(double alt) {
        this.alt = alt;
    }

    public double lat;
    public double lon;
    public double alt;

    public LatLonAlt() {}

    public LatLonAlt(double lat, double lon, double alt) {
        this.lat = lat;
        this.lon = lon;
        this.alt = alt;
    }
    @Override
	public String toString() {
        return this.lat +","+ this.lon +"," + this.alt;
    }
    public static LatLonAlt llaFromString(String str) throws NumberFormatException, IOException {
        String[] lla = str.split(",");
        if(lla == null || lla.length != 3)
            throw new IOException("Wrong number of items in LLA string (expected 3): " + str);
        return new LatLonAlt(
                Double.parseDouble(lla[0]),
                Double.parseDouble(lla[1]),
                Double.parseDouble(lla[2])
        );
    }
}