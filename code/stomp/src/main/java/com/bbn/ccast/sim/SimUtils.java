//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.sim;

import com.bbn.ccast.Utils;
import gov.nasa.worldwind.geom.Angle;

/**
 * Helpers to manage coordinate system deltas.
 * 
 * @author kmoffitt
 */
public class SimUtils {

	public static double getAltitude(double latitude, double longitude) {

		// Calculate altitude based on world terrain model
		// TODO parameterize this, and switch AirSim set from CLA to RPC
		// TODO only calculate anchor altitude once (after we sort out synchronizing elevation model load)
		final double airSimAnchorLat = 31.136424;
		final double airSimAnchorLon = -89.065786;
		
		// Fudge factor to err on the side of being just above terrain, so we can see rendered lines
		final double elevationOffsetFudge = 1;

		// AirSim calls this point elevation "0" - we need to know what WorldWind calls it
		// WW puts it at 12 m above AS, so correct for that, sigh
		// Note that there's the delta to sea level, and then the delta between AS and WW.
		// We're already accounting for the former.  This here is to account for the latter.
		double wwAltitudeAtAirSimAnchorPoint = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(airSimAnchorLat),
				Angle.fromDegreesLongitude(airSimAnchorLon));

		double wwAltitude = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(latitude),
				Angle.fromDegreesLongitude(longitude));
		double altitudeRelativeToAirSimAnchor = wwAltitude - wwAltitudeAtAirSimAnchorPoint + elevationOffsetFudge;

		return altitudeRelativeToAirSimAnchor;
	}
}
