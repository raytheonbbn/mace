//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObject;

/**
 * Utility to build Java representation of ROS nav_msgs/Odometry.msg Used for
 * sending faux odom updates from lightweight sim to ROS nav stack
 * 
 * @author kmoffitt
 */
class OdometryMessage {

	static final JsonObject build(double x, double y, double z, double relativeHeading) {

		// Header
		JsonObject header = RosMessageHeader.build("odom");

		// Pose (position and orientation)
		JsonObject position = Json.createObjectBuilder().add("x", x).add("y", y).add("z", z).build();

		// In ROS, positive yaw means counter-clockwise
		Rotation headingAdjustment = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0, 0,
				-relativeHeading);

		JsonObject orientation = Json.createObjectBuilder().add("x", headingAdjustment.getQ1())
				.add("y", headingAdjustment.getQ2()).add("z", headingAdjustment.getQ3())
				.add("w", headingAdjustment.getQ0()).build();

		JsonObject pose = Json.createObjectBuilder().add("position", position).add("orientation", orientation).build();

		// Covariance all zero for now, which means 'unknown' to ROS
		int numCovarianceArrayEntries = 36;

		JsonArrayBuilder arrayBuilder = Json.createArrayBuilder();
		for (int i = 0; i < numCovarianceArrayEntries; i++) {
			arrayBuilder.add(0);
		}

		JsonArray covariance = arrayBuilder.build();

		JsonObject poseWithCovariance = Json.createObjectBuilder().add("pose", pose).add("covariance", covariance)
				.build();

		// Twist (linear and angular velocity)
		// Probably don't need to worry about these. All 0's for now. TODO confirm ok
		JsonObject linearTwist = Json.createObjectBuilder().add("x", 0).add("y", 0).add("z", 0).build();

		JsonObject angularTwist = Json.createObjectBuilder().add("x", 0).add("y", 0).add("z", 0).build();

		JsonObject twist = Json.createObjectBuilder().add("linear", linearTwist).add("angular", angularTwist).build();

		JsonObject twistWithCovariance = Json.createObjectBuilder().add("twist", twist).add("covariance", covariance)
				.build();

		JsonObject topLevelJson = Json.createObjectBuilder().add("header", header).add("child_frame_id", "base_link")
				.add("pose", poseWithCovariance).add("twist", twistWithCovariance).build();

		return topLevelJson;
	}
}
