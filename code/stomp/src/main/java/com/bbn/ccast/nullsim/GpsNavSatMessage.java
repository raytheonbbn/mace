//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;

public class GpsNavSatMessage {

	static final JsonObject build(double[] lla) {

		// Header
		JsonObject header = RosMessageHeader.build("gps_link");

		// Pose (position and orientation)
		JsonObject status = Json.createObjectBuilder().add("status", 0).add("service", 0).build();

		// Covariance all zero for now, which means 'unknown' to ROS
		int numCovarianceArrayEntries = 9;

		JsonArrayBuilder arrayBuilder = Json.createArrayBuilder();
		for (int i = 0; i < numCovarianceArrayEntries; i++) {
			arrayBuilder.add(0);
		}

		JsonArray covariance = arrayBuilder.build();

		JsonObject topLevelJson = Json.createObjectBuilder().add("header", header).add("status", status)
				.add("latitude", lla[0]).add("longitude", lla[1]).add("altitude", lla[2])
				.add("position_covariance", covariance).add("position_covariance_type", 0).build();

		return topLevelJson;
	}
}
