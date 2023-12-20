//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.util;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import org.apache.log4j.Logger;

import javax.vecmath.Vector3d;
import java.io.Serializable;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by kusbeck on 6/19/17.
 */

public class MathHelper {
    private static final Logger logger = Logger.getLogger(MathHelper.class.getName());

    public static gov.nasa.worldwind.geom.LatLon toNasaLLA(LatLon pathPoint) {
        if(pathPoint == null || pathPoint.lat == null || pathPoint.lon == null || pathPoint.alt == null) return null;
        gov.nasa.worldwind.geom.LatLon ret = new Position(Angle.fromDegrees(pathPoint.lat), Angle.fromDegrees(pathPoint.lon), pathPoint.alt);
        return ret;
    }

    public static List<gov.nasa.worldwind.geom.LatLon> toNasaLLAList(List<LatLon> path) {
        List<gov.nasa.worldwind.geom.LatLon> nasaPath = new LinkedList<>();
        for(LatLon pathPoint : path) {
            nasaPath.add(toNasaLLA(pathPoint));
        }
        return nasaPath;
    }

    public static gov.nasa.worldwind.geom.Position toNasaPosition(LatLon pathPoint) {
        if(pathPoint == null || pathPoint.lat == null || pathPoint.lon == null || pathPoint.alt == null) return null;
        gov.nasa.worldwind.geom.Position ret = new Position(Angle.fromDegrees(pathPoint.lat), Angle.fromDegrees(pathPoint.lon), pathPoint.alt);
        return ret;
    }

    
    public static List<Position> toNasaPositionList(List<LatLon> path) {
        List<Position> nasaPath = new LinkedList<>();
        for(LatLon pathPoint : path) {
            nasaPath.add(toNasaPosition(pathPoint));
        }
        return nasaPath;
    }

    
    public static class HorizontalVertical {
        public float horizontal;
        public float vertical;
    }

    /**
     * Takes an angle [0-360] and strength [0-100] and converts it to horizontal and vertical
     * components [-1,1]
     *
     * @param angle
     * @param strength
     * @return
     */
    public static HorizontalVertical convertAngleStrengthToHorizontalVertical(
            int angle, int strength) {
        HorizontalVertical ret = new HorizontalVertical();
        ret.horizontal = 0;
        ret.vertical = 0;

        // normalize strength
        float normalizedStrength = strength / 100.0f;

        // 0 degrees starts on the right, 90 at top, 180 at left, 270 at bottom
        float normalizedAngle = angle + 360 % 360;

        // Polar to Cartesian conversion
        // x = r * cos( theta )
        ret.horizontal = normalizedStrength
                * (float) Math.cos(Math.toRadians(normalizedAngle));

        // y = r * sin( theta )
        ret.vertical = normalizedStrength
                * (float) Math.sin(Math.toRadians(normalizedAngle));

        return ret;
    }

    public static class LatLon implements Serializable {
        /**
		 * 
		 */
		private static final long serialVersionUID = 2625775765181010026L;
		public Double lat;
        public Double lon;
        public Double alt;
        
        @Override
		public String toString() {
        	return lat + ", " + lon + ", " + alt;
        }
    }

    public static LatLon createLLA(Vector3d latLonAlt) {
        if(latLonAlt == null) return null;
        LatLon ret = new LatLon();
        ret.lat = latLonAlt.x;
        ret.lon = latLonAlt.y;
        ret.alt = latLonAlt.z;
        return ret;
    }

    /**
     * @param gimbalPitch      angle in degrees from (-90.0 to 0.0) exclusive
     * @param droneLat
     * @param droneLon
     * @param droneAlt
     * @param droneOrientation
     * @return
     */
    public static LatLon getProjectionPoint(double gimbalPitch,
                                            double droneLat, double droneLon, double droneAlt,
                                            double droneOrientation) {

        if (Double.isNaN(gimbalPitch) || Double.isNaN(droneLat)
                || Double.isNaN(droneLon) || Double.isNaN(droneAlt)
                || Double.isNaN(droneOrientation)) {
            logger.debug("MathHelper: " +
                    "invalid gimbalPitch, droneLat, droneLon, droneAlt, droneOrientation "
                            +
                            gimbalPitch + ", " + droneLat + ", " + droneLon
                            + ", " + droneAlt + ", "
                            + droneOrientation);
            return null;
        }

        //       a
        //       |\
        //       |  \ sideC
        // sideB |    \
        //       |      \
        //       c ------ b
        //          sideA

        double c = 90.0;
        double a = (90.0 + gimbalPitch);
        double b = (180.0 - c) - a; // as long as c and a are valid angles, this should be OK.
        double sideB = droneAlt;
        double sideA = (sideB * Math.sin(Math.toRadians(a)))
                / Math.sin(Math.toRadians(b));
        double sideC = 0.0;// TODO: we don't need sideC yet

        // Now compute the directional offsets (dn & de)
        c = 90.0;
        a = 270.0 - (droneOrientation + 180.0); // yaw (-180, 180) and seems to be offset 270
        // degrees.
        // a just gets used in a toRadians call, which should take care of normalization
        b = (180.0 - c) - a;
        sideC = sideA; // distance from before
        sideB = (sideC * Math.sin(Math.toRadians(b)))
                / Math.sin(Math.toRadians(c));
        sideA = (sideC * Math.sin(Math.toRadians(a)))
                / Math.sin(Math.toRadians(c));

        // Now get the new lat/lon for the SPoI
        // https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        double de = sideB;
        double dn = sideA;

        // Earth’s radius, sphere
        double R = 6378137.0;

        double lat = droneLat;
        double lon = droneLon;

        double dLat = dn / R;
        double dLon = de / (R * Math.cos(Math.toRadians(lat)));

        // OffsetPosition, decimal degrees
        LatLon spoiLatLon = new LatLon();
        spoiLatLon.lat = lat + dLat * 180.0 / Math.PI;
        spoiLatLon.lon = lon + dLon * 180.0 / Math.PI;

        return spoiLatLon;
    }

}
