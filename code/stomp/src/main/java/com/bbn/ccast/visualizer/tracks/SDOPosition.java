//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.tracks;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;

@JsonIgnoreProperties({ "latitude", "longitude", "altitude" })
public class SDOPosition extends Position {
    
    private PositionType positionType;
    private double duration;
    private double radius;
	
	public SDOPosition() {
        super(Angle.ZERO, Angle.ZERO, 0d);
	}

	public SDOPosition(PositionType positionType) {
        super(Angle.ZERO, Angle.ZERO, 0d);
		this.positionType = positionType;
	}
	
	public SDOPosition(PositionType positionType, Angle latitude, Angle longitude, double elevation, double duration, double radius) {
        super(latitude, longitude, elevation);
    		this.positionType = positionType;
    		this.duration = duration;
    		this.radius = radius;
	}

	public SDOPosition(PositionType positionType, double latitude, double longitude, double elevation, double duration, double radius) {
        super(Angle.fromDegrees(latitude), Angle.fromDegrees(longitude), elevation);
    		this.positionType = positionType;
    		this.duration = duration;
    		this.radius = radius;
	}

	public double getLatitudeDegrees() {
		return this.getLatitude().getDegrees();
	}

	public double getLongitudeDegrees() {
		return this.getLongitude().getDegrees();
	}
		
	/**
	 * @return the duration in milliseconds
	 */
	public double getDuration() {
		return duration;
	}

	/**
	 * @return the radius
	 */
	public double getRadius() {
		return radius;
	}

	/**
	 * @return the positionType
	 */
	public PositionType getPositionType() {
		return positionType;
	}
	
	@Override
	public String toString() {
		StringBuffer buf = new StringBuffer();
		buf.append(super.toString());
		buf.append(", type: " + this.getPositionType().toString());
		buf.append(", duration: " + this.getDuration());
		buf.append(", radius: " + this.getRadius());
		return buf.toString();
	}


}
