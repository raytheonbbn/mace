//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.utils;


/*
 * Data container class used to store and represent a GPS location
 */
public class GpsLocation {
	
	// Class variables
	private double latitude;	// The location latitude
	private double longitude;	// The location longitude
	private double altitude;	// The location altitude
	
	public GpsLocation(double latitude, double longitude, double altitude) {
		
		// Initialize the class variables
		this.latitude = latitude;
		this.longitude = longitude;
		this.altitude = altitude;
	}
	
	
	/*
	 * Getter used to retrieve the latitude
	 */
	public double getLatitude() {
		return this.latitude;
	}
	
	
	/*
	 * Getter used to retrieve the longitude
	 */
	public double getLongitude() {
		return this.longitude;
	}
	
	
	/*
	 * Getter used to retrieve the altitude
	 */
	public double getAltitude() {
		return this.altitude;
	}
	
	
	/*
	 * Setter used to set the latitude
	 */
	public void setLatitude(double latitude) {
		this.latitude = latitude;
	}
	
	
	/*
	 * Setter used to set the longitude
	 */
	public void setLongitude(double longitude) {
		this.longitude = longitude;
	}
	
	
	/*
	 * Setter used to set the altitude
	 */
	public void setAltitude(double altitude) {
		this.altitude = altitude;
	}
}
