//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.util;

import org.apache.log4j.Logger;
import org.gavaghan.geodesy.Ellipsoid;
import org.gavaghan.geodesy.GeodeticCalculator;
import org.gavaghan.geodesy.GlobalPosition;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

public class DistanceUtils {
	private static final Logger logger = Logger.getLogger(DistanceUtils.class.getName());

	public static double LATITUDE_TO_METERS_AT_MERIDIAN = 110567.2;
	public static double LONGITUDE_TO_METERS_AT_EQUATOR = 111321.0;

	public static Ellipsoid geoReference = Ellipsoid.WGS84;
	static public GeodeticCalculator geoCalc = new GeodeticCalculator();

	private static double roughLatScaler = 1.0 / LATITUDE_TO_METERS_AT_MERIDIAN;
	private static double roughLonScaler = 1.0 / LONGITUDE_TO_METERS_AT_EQUATOR;

	/**
	 * Calculate the distance between two geographic points in 3 dimensions
	 * @param pos1 a 3-dimensional position in geo coordinates (decimal lat/lon/alt)
	 * @param pos2 a 3-dimensional position in geo coordinates (decimal lat/lon/alt)
	 * @param isGeodesic whether to calculate the distance with respect to an ellipsoid or using a straight line
	 * @return distance between pos1 and pos2 in meters
	 */
	public static double calculateDistance(Vector3d pos1, Vector3d pos2, boolean isGeodesic) {
		
		if (pos1 == null || pos2 == null) {
			logger.warn("Error:  null vector in calculateDistance!");
			return Double.NaN;
		}

		if (isGeodesic) {
			GlobalPosition p1 = new GlobalPosition(pos1.getX(), pos1.getY(), pos1.getZ());
			GlobalPosition p2 = new GlobalPosition(pos2.getX(), pos2.getY(), pos2.getZ());
			return geoCalc.calculateGeodeticMeasurement(geoReference, p1, p2).getPointToPointDistance();
		}

		if (!pos1.epsilonEquals(pos2, 0.0)) {
			Vector3d temp = new Vector3d();
			temp.sub(pos1, pos2);
			return temp.length();
		}

		return 0.0;
	}

	public static double calculateDistance(Vector2d pos1, Vector2d pos2, boolean isGeodesic) {
		
		if (pos1 == null || pos2 == null) {
			logger.warn("Error:  null vector in calculateDistance!");
			return Double.NaN;
		}

		if (isGeodesic) {
			GlobalPosition p1 = new GlobalPosition(pos1.getX(), pos1.getY(), 0);
			GlobalPosition p2 = new GlobalPosition(pos2.getX(), pos2.getY(), 0);
			return geoCalc.calculateGeodeticMeasurement(geoReference, p1, p2).getPointToPointDistance();
		}

		if (!pos1.epsilonEquals(pos2, 0.0)) {
			Vector2d temp = new Vector2d();
			temp.sub(pos1, pos2);
			double val = temp.length();
			return val;
		}

		return 0.0;
	}	
	
	public static double distanceInCRS(double distance) {
		double result = ((distance * roughLatScaler) + (distance * roughLonScaler)) / 2;
		return result;
	}

	public static double distanceInCRS(double distance, Vector3d pos1, Vector3d pos2) {
		double a = DistanceUtils.calculateDistance(pos1, pos2, false);
		double b = DistanceUtils.calculateDistance(pos1, pos2, true);
		double result = distance * (a / b);
		return result;
	}

	public static Vector3d advancePastEndPosition(Vector3d start, Vector3d end, double distance, boolean isGeodesic) {
		double s_e_dist = DistanceUtils.calculateDistance(start,  end, isGeodesic);
		double fraction = (s_e_dist + distance) / s_e_dist;
		return intermediatePoint(start, end, fraction, isGeodesic);
	}
	
	public static Vector3d intermediatePoint(Vector3d start, Vector3d end, double fraction, boolean isGeodetic) {
		if (isGeodetic) {
			return intermediatePointGeodetic(start, end, fraction);
		}
		return intermediatePointCartesian(start, end, fraction);
	}

	public static Vector3d intermediatePointCartesian(Vector3d start, Vector3d end, double fraction) {
		Vector3d point = new Vector3d(start);
		Vector3d temp = new Vector3d(end);
		temp.sub(start);
		temp.scale(fraction);
		point.add(temp);
		return point;
	}

	/*
	 * https://www.movable-type.co.uk/scripts/latlong.html
	 */
	public static Vector3d intermediatePointGeodetic(Vector3d start, Vector3d end, double fraction) {
		double lat1 = Math.toRadians(start.x);
		double lon1 = Math.toRadians(start.y);
		double lat2 = Math.toRadians(end.x);
		double lon2 = Math.toRadians(end.y);
		double sinlat1 = Math.sin(lat1);
		double coslat1 = Math.cos(lat1);
		double sinlon1 = Math.sin(lon1);
		double coslon1 = Math.cos(lon1);
		double sinlat2 = Math.sin(lat2);
		double coslat2 = Math.cos(lat2);
		double sinlon2 = Math.sin(lon2);
		double coslon2 = Math.cos(lon2);

		// distance between points
		double deltalat = lat2 - lat1;
		double deltalon = lon2 - lon1;
		double a = Math.sin(deltalat / 2) * Math.sin(deltalat / 2)
				+ Math.cos(lat1) * Math.cos(lat2) * Math.sin(deltalon / 2) * Math.sin(deltalon / 2);
		double delta = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

		double A = Math.sin((1 - fraction) * delta) / Math.sin(delta);
		double B = Math.sin(fraction * delta) / Math.sin(delta);

		double x = A * coslat1 * coslon1 + B * coslat2 * coslon2;
		double y = A * coslat1 * sinlon1 + B * coslat2 * sinlon2;
		double z = A * sinlat1 + B * sinlat2;

		double lat3 = Math.atan2(z, Math.sqrt(x * x + y * y));
		double lon3 = Math.atan2(y, x);

		// handle altitude
		double finalAlt = start.z + (fraction * (end.z - start.z));

		double finalLat = Math.toDegrees(lat3);
		// normalize lon to −180..+180°
		double finalLon = ((Math.toDegrees(lon3) + 540) % 360.0 - 180.0);
		return new Vector3d(finalLat, finalLon, finalAlt); 

	}
	
	/**
	 * Convert a lat/long point to a euclidean point describing a position relative to another lat/long.
	 * Note that the conversion from degrees latitude to meters may be approximated as constant,
	 * but the conversion from degrees longitude to meters varies based on the <i>latitude</i> 
	 * of a given point.
	 * Altitudes are not converted.
	 * 
	 * @param anchor	The origin, the lat/long which corresponds to a local euclidean value of (0, 0).
	 * @param point		The lat/long point to convert to local euclidean coordinates.
	 * @return			The local euclidean equivalent of the point.
	 */
	public static Vector3d convertToEuclideanPoint(Vector3d anchor, Vector3d point) {
		double LONGITUDE_TO_METERS = Math.cos(Math.toRadians(anchor.getX())) * DistanceUtils.LONGITUDE_TO_METERS_AT_EQUATOR;

		Vector3d newPoint = new Vector3d((point.x - anchor.getX()) * DistanceUtils.LATITUDE_TO_METERS_AT_MERIDIAN,
				(point.y - anchor.getY()) * LONGITUDE_TO_METERS, point.z);
		return newPoint;
	}
	
	/**
	 * Convert a local euclidean point to a lat/long value, using the lat/long value of the origin of the
	 * local euclidean space.
	 * 
	 * @param anchor	The "origin" point, the lat/long which corresponds to a local euclidean value of (0, 0).
	 * @param point		The local euclidean point to convert to lat/long coordinates.
	 * @return			The lat/long equivalent of the point.
	 */
	public static Vector3d convertFromEuclideanPoint(Vector3d anchor, Vector3d point) {
		double LONGITUDE_TO_METERS = Math.cos(Math.toRadians(anchor.getX())) * DistanceUtils.LONGITUDE_TO_METERS_AT_EQUATOR;

		Vector3d newPoint = new Vector3d((point.x / DistanceUtils.LATITUDE_TO_METERS_AT_MERIDIAN) + anchor.getX(),
				(point.y / LONGITUDE_TO_METERS) + anchor.getY(), point.z);
		return newPoint;
	}
	
	public static double getDistance(Vector3d start, Vector3d end)
	{
		Vector3d lookVec = new Vector3d(end);
		lookVec.sub(start);
		return lookVec.length();
	}
	
	public static double getDistance(Vector2d start, Vector2d end)
	{
		Vector2d lookVec = new Vector2d(end);
		lookVec.sub(start);
		return lookVec.length();
	}

	public static Vector3d addXYZ(Vector3d pos, double x, double y, double z, boolean isGeodesic) {
		if (isGeodesic) {
			Vector3d anchor = new Vector3d(pos);
			Vector3d posEuclidean = DistanceUtils.convertToEuclideanPoint(anchor, pos);
			 Vector3d newPos = DistanceUtils.addXYZ(posEuclidean, x, y, z, false);
			 return DistanceUtils.convertFromEuclideanPoint(anchor, newPos);
		}
		return new Vector3d(pos.getX()+x, pos.getY()+y, pos.getZ()+z);
	}
	
	
	public static void main(String[] args) {
		DistanceUtils.geoCalc = new GeodeticCalculator();

		// double res1 = DistanceUtils.distanceInCRS(100.0);

		// Vector3d s1 = new Vector3d(35,-71,0);
		// Vector3d e1 = new Vector3d(35.5, -71.5,0);

		// double res2 = DistanceUtils.distanceInCRS(100.0, s1, e1);
		// double val = DistanceUtils.calculateDistance(s1, e1, true);
	}

}
