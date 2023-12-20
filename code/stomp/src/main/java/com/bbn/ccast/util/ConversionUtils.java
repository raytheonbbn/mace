//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.util;

/**
 * Created by jsilva on 7/26/17.
 */

public class ConversionUtils {
	public static final Double EPSILON = Math.pow(10, -5);
	
	public static final double METERS_TO_FEET = 3.280839895;

	// TODO do we actually intend to use MAX_DECIMAL_PLACES?
//	public static final int MAX_DECIMAL_PLACES = 5;

	/**
	 * Return the string representation of a number. If input number is less than
	 * pre-defined EPSILON, return "0.0".
	 *
	 * @param number
	 * @return String representation of number
	 */
	public static String getString(Double number) {
		if (number.isNaN()) {
			return "0.0";
		}

		if (isZero(number)) {
			return "0.0";
		}

		return String.format("%.10f", number);
	}

	public static String getString(Integer number) {
		return String.format("%d", number);
	}

	public static boolean isZero(double value) {
		return isZero(value, EPSILON);
	}

	public static boolean isZero(double value, double threshold) {
		return value >= -threshold && value <= threshold;
	}
	
	public static double degreesToRadians(double degrees) {
		return degrees * Math.PI / 180.0;
	}

    public static double metersToFeet(double meters)
    {
        return meters * METERS_TO_FEET;
    }

    public static double feetToMeters(double feet)
    {
        return feet / METERS_TO_FEET;
    }


}
