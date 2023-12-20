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
 * Java representation of ROS sensor_msgs/Imu.msg Used for sending faux IMU
 * updates from lightweight sim to ROS nav stack
 * 
 * @author kmoffitt
 */
class ImuMessage {

	/**
	 * Build JSON representing IMU state for ROS. UGV-only for now, so linear X,
	 * angular Z, and orientation (relative to start) only.
	 * 
	 * @param linearX         Forward-backward acceleration in m/s^2
	 * @param angularZ        Angular yaw velocity, in radians. Positive is
	 *                        counterclockwise.
	 * @param relativeHeading Bearing from start, in radians. Positive is
	 *                        counterclockwise.
	 * @return
	 */
	static final JsonObject build(double linearX, double angularZ, double relativeHeading) {

		/* 
		   From http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html -

		Header header

		geometry_msgs/Quaternion orientation
		float64[9] orientation_covariance # Row major about x, y, z axes

		geometry_msgs/Vector3 angular_velocity
		float64[9] angular_velocity_covariance # Row major about x, y, z axes

		geometry_msgs/Vector3 linear_acceleration
		float64[9] linear_acceleration_covariance # Row major x, y z
		*/

		// Covariance all zero for now, which means 'unknown' to ROS
		int numCovarianceArrayEntries = 9;

		JsonArrayBuilder arrayBuilder = Json.createArrayBuilder();
		for (int i = 0; i < numCovarianceArrayEntries; i++) {
			arrayBuilder.add(0);
		}
		JsonArray covariance = arrayBuilder.build();

		// In ROS, positive yaw means counter-clockwise
		Rotation headingAdjustment = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0, 0,
				-relativeHeading);

		JsonObject header = RosMessageHeader.build("base_link");

		JsonObject orientation = Json.createObjectBuilder().add("x", headingAdjustment.getQ1())
				.add("y", headingAdjustment.getQ2()).add("z", headingAdjustment.getQ3())
				.add("w", headingAdjustment.getQ0()).build();

		JsonObject angularVelocity = Json.createObjectBuilder().add("x", 0).add("y", 0).add("z", angularZ).build();

		JsonObject linearAcceleration = Json.createObjectBuilder().add("x", linearX).add("y", 0).add("z", 0).build();

		JsonObject topLevelJson = Json.createObjectBuilder().add("header", header).add("orientation", orientation)
				.add("orientation_covariance", covariance).add("angular_velocity", angularVelocity)
				.add("angular_velocity_covariance", covariance).add("linear_acceleration", linearAcceleration)
				.add("linear_acceleration_covariance", covariance).build();

		return topLevelJson;
	}

}
