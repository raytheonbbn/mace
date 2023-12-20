//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast;

import com.bbn.ccast.config.Configuration;
import com.bbn.ccast.util.DistanceUtils;
import com.bbn.openmap.geo.Geo;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import org.apache.log4j.Logger;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.SimpleTimeZone;

/**
 * Various geospherical / geometry helpers. NB this statically and
 * asynchronously auto-loads a fair bit of data for its model of the world.
 */
public class Utils {
	private static final Logger logger = Logger.getLogger(Utils.class);

	public static final AgentEarth EARTH = new AgentEarth();
	public static final Globe GLOBE = EARTH;

	private static final double METERS_PER_DEGREE = 111111.0;

	public static boolean isNull(Object o) {
		return o == null;
	}

	public static Object NULL() {
		return null;
	}

	public static Set<?> EMPTYSET() {
		return new HashSet<>();
	}

	public static final double ALTIUDE_EPSILON = 1.5; // in meters


	public static double[] nedToLla(double[] homeLla, float[] ned) {

		double[] result = new double[3];
		result[0] = homeLla[0] + ned[0] / METERS_PER_DEGREE;
		result[1] = homeLla[1] + ned[1] / (METERS_PER_DEGREE * Math.cos(Math.toRadians(homeLla[0])));

		// For alt we subtract, since we're converting from +down to +up
		result[2] = homeLla[2] - ned[2];
		return result;
	}

	public static Vector3d nedVectorInMetersFromVector3dToVector3d(Vector3d currentPos, Vector3d target) {
		
		if (target == null) {
			logger.debug("No target specified");
			return new Vector3d();
		}
		
		double scaledNorthDiff = 111111.0 * (target.getX() - currentPos.getX());
		double scaledEastDiff = 111111.0 * Math.cos(Math.toRadians(target.getX()))
				* (target.getY() - currentPos.getY());
		double altDiff = (target.getZ() - currentPos.getZ());
		
		if (logger.isTraceEnabled()) {
			logger.trace("Start position: " + currentPos.toString());
			logger.trace("Target position: " + target.toString());
			logger.trace("Scaled North difference: " + scaledNorthDiff);
			logger.trace("Scaled East difference: " + scaledEastDiff);	
			logger.trace("(N=" + scaledNorthDiff + ", E=" + scaledEastDiff + ", D=" + -altDiff + ")");
		}
		
		// Negative altDiff here, since we want a NED vector
		return new Vector3d(scaledNorthDiff, scaledEastDiff, -altDiff);
	}

	public static Double distanceInMetersFromLlaToLla(LatLon a, LatLon b) {
		return LatLon.greatCircleDistance(a, b).getRadians() * EARTH.getRadius();
	}

	public static LatLon vecToLLA(Vector3d vec) {
		return Position.fromDegrees(vec.x, vec.y, vec.z);
	}

	static SimpleDateFormat dateFormat;

	static {
		dateFormat = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.S'Z'");
		dateFormat.setTimeZone(new SimpleTimeZone(0, "UTC"));
	}

	public static String timeStrFromMillis(long millisSinceEpochUtc) {
		return dateFormat.format(millisSinceEpochUtc);
	}


	public static double getBearingBetweenTwoPoints(Vector2d point1, Vector2d point2) {
		LatLon latLon1 = LatLon.fromDegrees(point1.getX(), point1.getY());
		LatLon latLon2 = LatLon.fromDegrees(point2.getX(), point2.getY());
		double degrees = LatLon.greatCircleAzimuth(latLon1, latLon2).degrees;
		if (degrees < 0) {
			degrees += 360;
		}
		return degrees;
	}

	public static double getBearingBetweenTwoPoints(Vector3d point1, Vector3d point2) {
		LatLon latLon1 = LatLon.fromDegrees(point1.getX(), point1.getY());
		LatLon latLon2 = LatLon.fromDegrees(point2.getX(), point2.getY());
		double degrees = LatLon.greatCircleAzimuth(latLon1, latLon2).degrees;
		if (degrees < 0) {
			degrees += 360;
		}
		return degrees;

	}


	public static double getBearingFromVector(Vector3d dir) {
		Vector3d bearingZero = new Vector3d(1, 0, 0);
		Vector3d xyDir = new Vector3d(dir.x, dir.y, 0);

		double bearing = Math.toDegrees(xyDir.angle(bearingZero));

		if (dir.y < 0) {
			bearing = 360 - bearing;
		}

		return bearing;
	}


	public static LatLon calculatePointAlongLine(LatLon start, LatLon end, double percentage) {
		return LatLon.interpolate(percentage, start, end);
	}



	public static double getConfigurationParamOrDefault(Configuration configuration, String parameterKey,
			double defaultValue) {
		Double ret = configuration.getDoubleProperty(parameterKey);
		if (ret == null) {
			ret = defaultValue;
		}
		return ret;
	}

	public static double[] toDoubleArray(Object[] array) {
		double[] ret = new double[array.length];
		int i = 0;
		for (Object obj : array) {
			if (obj != null) {
				ret[i++] = Double.parseDouble(obj.toString());
			} else {
				ret[i++] = Double.NaN;
			}
		}
		return ret;
	}

	/**
	 * @param ll
	 * @param alt
	 * @return Vector3d from lat/lon degrees and alt meters
	 */
	public static Vector3d llaToVec(LatLon ll, double alt) {
		return new Vector3d(ll.getLatitude().getDegrees(), ll.getLongitude().getDegrees(), alt);
	}

	// Internal class just to hold values
	private static class PointMinMaxDetails {
		double minX;
		double minY;
		double maxX;
		double maxY;
	}

	public static Double mslToHae(Double msl) {
		// XXX: needs to be implemented!!!
		return msl;
	}

	public static final double EPSILON = 0.00001;

	public static boolean isApproxZero(Double test) {
		if (test < EPSILON && test > -EPSILON)
			return true;
		return false;
	}

	public static int restartService(String service) {
		String[] cmdArray = { "ssh", "bottom", "sudo systemctl restart " + service };
		ProcessBuilder pb = new ProcessBuilder();
		// pb.environment().put("PATH",System.getenv("PATH"));
		pb.command(cmdArray);
		int result = 1;
		try {
			Process p = pb.start();
			p.waitFor();
		} catch (IOException e) {
			logger.error("Failed to restart service " + service + ": ", e);
		} catch (InterruptedException e) {
			logger.warn("Interrupted while waiting for result of systemctl restart " + service);
		}
		return result;
	}

	public static final boolean isNotNullAndTrue(Boolean test) {
		if (test == null)
			return false;
		return test;
	}

	public static boolean containsIgnoreCase(String src, String what) {
		final int length = what.length();
		if (length == 0)
			return true; // Empty string is contained

		final char firstLo = Character.toLowerCase(what.charAt(0));
		final char firstUp = Character.toUpperCase(what.charAt(0));

		if (src == null){
			return false;
		}

		for (int i = src.length() - length; i >= 0; i--) {
			// Quick check before calling the more expensive regionMatches() method:
			final char ch = src.charAt(i);
			if (ch != firstLo && ch != firstUp)
				continue;

			if (src.regionMatches(true, i, what, 0, length))
				return true;
		}

		return false;
	}

	public static ArrayList<Vector3d> getFrustumPoints(Vector3d targetPoint, Vector3d lookPoint, double hFov,
			double vFov, boolean isGeodetic, double maxDistance, boolean restrictToAboveGround) {
		ArrayList<Vector3d> frustumPoints = new ArrayList<Vector3d>();

		if (isGeodetic) {
			ArrayList<Vector3d> euclideanFrustumPoints = getFrustumPoints(
					DistanceUtils.convertToEuclideanPoint(targetPoint, targetPoint),
					DistanceUtils.convertToEuclideanPoint(targetPoint, lookPoint), hFov, vFov, false, maxDistance,
					restrictToAboveGround);

			if (euclideanFrustumPoints != null) {
				for (Vector3d euclideanFrustumPoint : euclideanFrustumPoints) {
					frustumPoints.add(DistanceUtils.convertFromEuclideanPoint(targetPoint, euclideanFrustumPoint));
				}
			}

			return frustumPoints;
		}

		Vector3d lookVec = new Vector3d(lookPoint);
		lookVec.sub(targetPoint);

		Vector3d up = new Vector3d(0, 0, 1);

		Vector3d xDir = new Vector3d();
		xDir.cross(lookVec, up);
		if (xDir.length() == 0) {
			logger.error("Can't calculate frustum points when vertically aligned without further data");
			return null;
		}
		xDir.normalize();

		Vector3d yDir = new Vector3d();
		yDir.cross(xDir, lookVec);
		yDir.normalize();

		double hFovRad = hFov * Math.PI / 180;
		double vFovRad = vFov * Math.PI / 180;

		double distance = lookVec.length();
		Vector3d notionalLookPoint = new Vector3d(lookPoint);

		if (maxDistance > 0 && distance > maxDistance) {
			lookVec.scale(maxDistance / distance);
			notionalLookPoint.add(targetPoint, lookVec);
			distance = maxDistance;
		}

		double halfWidth = distance * Math.tan(hFovRad / 2);
		double halfHeight = distance * Math.tan(vFovRad / 2);

		if (restrictToAboveGround) {
			Vector3d lowestPoint = new Vector3d(notionalLookPoint);
			Vector3d yOffset = new Vector3d(yDir);
			yOffset.scale(halfHeight * -1);
			lowestPoint.add(yOffset);
			if (lowestPoint.z < 0) {
				double scaleFactor = targetPoint.z / (targetPoint.z - lowestPoint.z);
				lookVec.scale(scaleFactor);
				notionalLookPoint.add(targetPoint, lookVec);
				distance = lookVec.length();
				halfWidth = distance * Math.tan(hFovRad / 2);
				halfHeight = distance * Math.tan(vFovRad / 2);
			}
		}

		for (int i = 0; i < 4; i++) {
			Vector3d xOffset = new Vector3d(xDir);
			xOffset.scale(halfWidth);

			if (i == 1 || i == 3) {
				xOffset.scale(-1);
			}

			Vector3d yOffset = new Vector3d(yDir);
			yOffset.scale(halfHeight);

			if (i == 2 || i == 3) {
				yOffset.scale(-1);
			}

			Vector3d offset = new Vector3d();
			offset.add(xOffset, yOffset);

			Vector3d frustumPoint = new Vector3d();
			frustumPoint.add(notionalLookPoint, offset);

			frustumPoints.add(frustumPoint);
		}

		return frustumPoints;
	}



	/**
	 * Normalizes an angle to an absolute angle. The normalized angle will be in the
	 * range from 0 to 360, where 360 itself is not included.
	 *
	 * @param angle
	 *            the angle to normalize
	 * @return the normalized angle that will be in the range of [0,360[
	 */
	public static double normalAbsoluteAngleDegrees(double angle) {
		double angle2 = angle;
		return (angle2 %= 360) >= 0 ? angle2 : (angle2 + 360);
	}
	
	
	public static Geo[] convertToGeoArray(ArrayList<Vector2d> points) {
		Geo[] geoArray = new Geo[points.size()];
		for (int i = 0; i < points.size(); i++) {
			geoArray[i] = new Geo(points.get(i).getX(), points.get(i).getY());
		}
		return geoArray;
	}
	
	
	public static double normalizeBearing(double bearing) {
		
		double result = bearing;
		
		// Make it in the range -180 to 180, for easier managing
		if (result < -180) {
			result += 360;
		}

		if (result > 180) {
			result -= 360;
		}
		
		return result;
	}
	

	public static void main(String[] args) {
		
		Vector3d dir = new Vector3d(1, 1, 0);
		double angle = getBearingFromVector(dir);
		System.out.println("++: " + angle);

		dir = new Vector3d(-1, 1, 0);
		angle = getBearingFromVector(dir);
		System.out.println("-+: " + angle);

		dir = new Vector3d(1, -1, 0);
		angle = getBearingFromVector(dir);
		System.out.println("+-: " + angle);

		dir = new Vector3d(-1, -1, 0);
		angle = getBearingFromVector(dir);
		System.out.println("--: " + angle);

		// Tuple testStart = new ArrayTupleImpl(40.79178, -73.91837, 36.0);
		// Tuple testEnd = new ArrayTupleImpl(40.7913983, -73.9185090, 36.0);

		// System.out.println(nedVectorInMetersFromLlaToLla(testStart, testEnd));
		// System.out.println(Arrays.toString(llaToNed(40.79178, -73.91837, 40.7913983,
		// -73.9185090)));
		// System.out.println(Arrays.toString(latLonToXY(40.79178, -73.91837, 0,
		// 40.7913983, -73.9185090)));
		// System.out.println(Arrays.toString(latLonToXY(40.79178, -73.91837, 90,
		// 40.7913983, -73.9185090)));
		// System.out.println(Arrays.toString(latLonToXY(40.79178, -73.91837, 45,
		// 40.7913983, -73.9185090)));

	}

}
