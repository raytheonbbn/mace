//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import com.bbn.ccast.Utils;
import com.bbn.ccast.config.Configuration;
import com.bbn.ccast.ros.GoalStatus;
import com.bbn.ccast.sim.SimUtils;
import com.bbn.ccast.sim.TimeServer;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import edu.brown.robotics.rosbridge.RosClientSubscription;
import edu.brown.robotics.rosbridge.RosClientSubscription.JsonMessageCallback;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;

import javax.json.JsonObject;
import javax.vecmath.Vector3d;

public class SimRover extends SimVehicle {

	public static final float BATTERY_LIFE_SECONDS = 60.0f * 60.0f;
	protected static final float BATTERY_USAGE_PER_SECOND = 1.0f/BATTERY_LIFE_SECONDS;

	private final String rosHost;
	private static final int BASE_ROS_PORT = 9091;
	private static final String NAV_SAT_TYPE = "sensor_msgs/NavSatFix";

	private final int rosPort;

	private final String myMoveBaseOutputTopic;
	private final String myGoalStatusTopic;
	private final String myMoveBaseInputTopic;
	private final String myNavSatTopic;

	// TODO separate topics per rover
	private static final String IMU_TOPIC = "/imu";
	private static final String WHEEL_ODOM_TOPIC = "/odom_wheel";

	private static final String MOVE_BASE_INPUT_TOPIC = "/move_base_simple/goal";
	private static final String MOVE_BASE_OUTPUT_TOPIC = "/cmd_vel";
	private static final String MOVE_BASE_STATUS_TOPIC = "/move_base/status";

	private static final String MOVE_BASE_TYPE = "geometry_msgs/PoseStamped";
	private static final String CMD_VEL_TYPE = "geometry_msgs/Twist";

	private static final String MOVE_BASE_STATUS_MESSAGE_TYPE = "actionlib_msgs/GoalStatusArray";

	private static final String IMU_MESSAGE_TYPE = "sensor_msgs/Imu";
	private static final String WHEEL_ODOM_TYPE = "nav_msgs/Odometry";

	private static final String GLOBAL_FRAME_ID = "utm";


	private final String name;

	private final Object initSync = new Object();

	// KM911 probably we remove this, but maybe we consider keeping it for featherweight?
	private RosClientSubscription rosMoveBaseSubscription;
	private RosClientSubscription rosCmdVelSubscription;

	private int currentMoveBaseGoalId;

	private float finalTargetHeading;

	private float speed;
	private float angularVelocity;

	// In this context, lightweight means that this class replaces the whole ROS nav stack
	private final boolean lightweightSim;

	SimRover(String uid, TimeServer timeServer, int port, int ordinal, boolean isLightweight) {

		super(uid, timeServer);

		Configuration config = Configuration.instance();

		rosHost = config.getNavRosHost();

		this.name = config.getAgentPrefix() + "Rover_" + ordinal;
		this.rosPort = BASE_ROS_PORT + ordinal;
		this.lightweightSim = isLightweight;

//		Random rand = new Random();
//		setBatteryStdDev(config.getSimBatteryStdDev());
//		setBatteryLifeSeconds((float) ((rand.nextGaussian() * getBatteryStdDev() + config.getSimRoverBattery()) * 60));
//		setBatteryUsagePerSecond(1.0f/getBatteryLifeSeconds());

		this.myMoveBaseOutputTopic = MOVE_BASE_OUTPUT_TOPIC;
		this.myGoalStatusTopic = MOVE_BASE_STATUS_TOPIC;
		this.myMoveBaseInputTopic = MOVE_BASE_INPUT_TOPIC;
		this.myNavSatTopic = "/gps_navsat";

		// Connect asynchronously
		new Thread() {

			@Override
			public void run() {

				synchronized (initSync) {

					if (lightweightSim) {
						while (rosMoveBaseSubscription == null) {

							try {
								logger.debug("Starting SimVehicle connection to ROS move_base at " + rosHost + ":" + port);
								rosMoveBaseSubscription = RosClientSubscription.makeSubscription(rosHost, rosPort,
										myMoveBaseInputTopic, MOVE_BASE_TYPE,
										moveBaseCallback);
							} catch (Exception e) {
								// Don't care; probably just no ROSbridge server yet, so just fall through
							}

							if (rosMoveBaseSubscription == null) {
								logger.warn("Failed to connect SimRover to ROS movement topic.  Retrying in a second...");
								try {
									Thread.sleep(1000);
								} catch (InterruptedException e) {
									// Don't care
								}
							}
						}
					}

					while (rosCmdVelSubscription == null) {

						try {
							logger.debug("Starting SimVehicle connection to ROS cmd_vel at " + rosHost + ":" + port);
							rosCmdVelSubscription = RosClientSubscription.makeSubscription(rosHost, rosPort,
									myMoveBaseOutputTopic, CMD_VEL_TYPE, cmdVelCallback);
						} catch (Exception e) {
							// Don't care; probably just no ROSbridge server yet, so just fall through
						}

						if (rosCmdVelSubscription == null) {
							logger.warn("Failed to connect SimRover to ROS movement topic.  Retrying in a second...");
							try {
								Thread.sleep(1000);
							} catch (InterruptedException e) {
								// Don't care
							}
						}
					}

					logger.info("Subscribed to ROS cmd_vel topic at " + rosHost + ":" + rosPort);
				}
			}
		}.start();
	}

	@Override
	protected void setFinalTargetHeading(float targetHeading) {
		synchronized (stateLock) {
			finalTargetHeading = targetHeading;
		}
	}

	@Override
	protected void handleCustomModeSwitch(long custom_mode) {

		//NOP for now


//		synchronized (stateLock) {
//			if (custom_mode == CustomRoverMode.HOLD.getValue()) {
//				clearVelocity();
//				state = State.LOITER;
//			}
//		}
	}

	@Override
	public boolean checkForGroundCollision() {
		return false;
	}

	@Override
	void setTimeout() {
		// NOP for rovers, for now at least (since ROS nav doesn't timeout)
		// TODO determine whether this is right in general
	}

	@Override
	void setImmediateYawTarget(float target) {
		synchronized (stateLock) {
			finalTargetHeading = target;
			state = State.TURN_TO_FINAL_HEADING;
		}
	}

	@Override
	void clearVelocity() {
		synchronized (stateLock) {
			speed = 0;
			angularVelocity = 0;
			velocity[0] = 0;
			velocity[1] = 0;
			velocity[2] = 0;
		}
	}

	void setLinearAndAngularVelocity(float linear, float angular) {

		synchronized (stateLock) {
			speed = linear;

			angularVelocity = angular;
			finalTargetHeading = NO_FINAL_HEADING_ADJUSTMENT;
			state = State.MOVE;
		}
	}

	@Override
	void setVelocity(float vx, float vy, float vz) {

		synchronized (stateLock) {

			float targetYaw = (float) Math.toDegrees(Math.atan2(vx, vy));

			// Now this is standard math, in which the origin is E and rotation is CCW. We
			// want origin N and rotation CW. So...
			targetYaw *= -1;
			targetYaw += 90.0f;
			if (targetYaw < 0) {
				targetYaw += 360.0f;
			}

			setAbsoluteYawTarget(targetYaw);

			float forwardSpeed = (float) (Math.sqrt(vx * vx) + Math.sqrt(vy * vy));
			setLinearAndAngularVelocity(forwardSpeed, 0);

			state = State.TURN_TO_MOVE;
		}
	}

	@Override
	public void setTarget(LatLonAlt target) {

		synchronized (stateLock) {
			targetPosition = target;
			Vector3d newVelocity = Utils.nedVectorInMetersFromVector3dToVector3d(new Vector3d(lla),
					new Vector3d(target.lat, target.lon, target.alt));
			newVelocity.z = 0;
			double EPSILON = 0.01;
			if (newVelocity.length() > EPSILON) {
				newVelocity.normalize();
			}
			setVelocity((float) newVelocity.x, (float) newVelocity.y, (float) newVelocity.z);
			// By default, indicate that we just want to leave ourselves at resting yaw when
			// we arrive.
			setFinalTargetHeading(NO_FINAL_HEADING_ADJUSTMENT);
			logger.debug("Set target for " + getUid() + " to " + newVelocity.x + ", " + newVelocity.y + ", "
					+ newVelocity.z);
		}
	}

	@Override
	public String getUid() {
		return name;
	}

	@Override
	public void returnToLaunch() {
		synchronized (stateLock) {
			setTarget(new LatLonAlt(homeLla[0], homeLla[1], homeLla[2]));
			state = State.RTL_TURN_TO_MOVE;
		}
	}

	@Override
	public void land() {
	}

	@Override
	void takeoff(float altitude) {
	}

	@Override
	void applyPhysics() {

		long currentTime = timeServer.currentTimeMillis();
		float secondsSinceLastUpdate = (currentTime - lastPhysicsUpdateTime) / 1000.0f;

		synchronized (stateLock) {

			// As long as we're not in the first frame (since we need to know
			// secondsSinceLastUpdate)...
			if (lastPhysicsUpdateTime != 0) {

				// Update orientation
				if (state == State.TURN_TO_MOVE || state == State.RTL_TURN_TO_MOVE || state == State.TURN_TO_FINAL_HEADING) {

					float targetHeading = (state == State.TURN_TO_MOVE || state == State.RTL_TURN_TO_MOVE) ? targetRollPitchYaw[2] : finalTargetHeading;
					float relativeAngleDelta = applyRotationPhysics(secondsSinceLastUpdate, targetHeading);

					// If we're not done, prep for next frame
					if (state == State.TURN_TO_MOVE || state == State.RTL_TURN_TO_MOVE || state == State.TURN_TO_FINAL_HEADING) {

						prepareForNextRotationPhysicsFrame(targetHeading, relativeAngleDelta);
					}
				}
			}

			if (targetPosition == null && targetPositions.size() > 0 ){
				SDOPosition nextTarget = targetPositions.poll();
				switch(nextTarget.getPositionType()){
					case GOTO:
						LatLonAlt moveTgt = new LatLonAlt(nextTarget.getLatitudeDegrees(), nextTarget.getLongitudeDegrees(), nextTarget.getAltitude());
						setTarget(moveTgt);
						break;
					case LOOKPOINT:
						Position lookPos = new Position(Angle.fromDegrees(nextTarget.getLatitudeDegrees()),
								Angle.fromDegrees(nextTarget.getLongitudeDegrees()),
								nextTarget.getAltitude());
						LatLonAlt curLla = getPosition();
						Position curPos = new Position(Angle.fromDegrees(curLla.getLat()), Angle.fromDegrees(curLla.getLon()), curLla.getAlt());
						double targetHeading = LatLon.greatCircleAzimuth(curPos, lookPos).getDegrees();
						if (targetHeading < 0) {
							targetHeading += 360;
						}
						setAbsoluteYawTarget((float) targetHeading);
						break;
					case LOITER:
						break;
					case LAND:
						land();
						break;
					case RTL:
						returnToLaunch();
						break;
				}
			}

			if (state == State.MOVE || state == State.RTL) {
				applyMovementPhysics(secondsSinceLastUpdate);
			} else {
				// If we're moving, we carefully update lla in the physics update, else here
				double[] newLla = Utils.nedToLla(originLla, positionNed);
				lla[0] = newLla[0];
				lla[1] = newLla[1];
				lla[2] = 0;
			}

			if (state != State.LANDED && state != State.LOITER) {
				setBatteryRemaining(batteryRemaining - secondsSinceLastUpdate * BATTERY_USAGE_PER_SECOND);
//				setBatteryRemaining(batteryRemaining - secondsSinceLastUpdate * getBatteryUsagePerSecond());
			}
		}

		lastPhysicsUpdateTime = currentTime;
	}

	private void applyMovementPhysics(float secondsSinceLastUpdate) {

		// Rovers move based on linear speed and angular velocity.
		// We set the NED-based velocity[] (for now) for outgoing Pixhawk state

		// Update yaw
		rollPitchYaw[2] += angularVelocity * secondsSinceLastUpdate;
		if (rollPitchYaw[2] < 0) {
			rollPitchYaw[2] += 360.0;
		}
		if (rollPitchYaw[2] > 360.0) {
			rollPitchYaw[2] -= 360.0;
		}

		// And now update position.  First a little trig...
		// In our vehicles, north is 0, +heading rotates clockwise
		velocity[0] = 0;
		velocity[1] = 0;
		if (speed != 0) {

			float heading = getYaw();
			velocity[0] = (float) Math.cos(Math.toRadians(heading));
			velocity[1] = (float) Math.sin(Math.toRadians(heading));

			velocity[0] *= speed;
			velocity[1] *= speed;
		}

		// Update position (TODO handle collisions)
		// We'll update N and E, then LLA, then D from LL using elevation map.
		// (A is always 0 for rovers.)
		for (int i = 0; i < 2; i++) {

			positionNed[i] += velocity[i] * secondsSinceLastUpdate;
			//	            logger.info("s = " + secondsSinceLastUpdate);
			//	            logger.info("vel[" + i + "] = " + velocity[i]);
			//	            logger.info("NED[" + i + "] = " + positionNed[i]);
		}

		double[] newLla = Utils.nedToLla(originLla, positionNed);
		lla[0] = newLla[0];
		lla[1] = newLla[1];
		lla[2] = 0;

		// Calculate altitude based on world terrain model
		// And since AirSim wants NED relative to home, subtract home alt
		positionNed[2] = -(float)(SimUtils.getAltitude(lla[0], lla[1]) - originLla[2]);

		if (targetPosition != null) {

			Double distance = Utils.distanceInMetersFromLlaToLla(LatLon.fromDegrees(lla[0], lla[1]),
					LatLon.fromDegrees(targetPosition.lat, targetPosition.lon));
			//		        logger.info("Distance to target: " + distance);

			// TODO figure out why this happens (epsilon mis-match, or something?)
			if (velocity[0] == 0 && velocity[1] == 0 && velocity[2] == 0) {
				logger.debug("State is MOVE but velocity is 0?");
				distance = 0d;
			}

			if (distance < EPSILON_TO_TARGET) {
				logger.debug(name + " distance is " + distance + ", so we think we've reached our destination");
				clearVelocity();
				state = (finalTargetHeading == NO_FINAL_HEADING_ADJUSTMENT) ? State.LOITER
						: State.TURN_TO_FINAL_HEADING;
				targetPosition = null;
			}
		}
	}

	private float applyRotationPhysics(float secondsSinceLastUpdate, float targetHeading) {

		// First: calculate delta, in [-180, 180] so we don't have to worry about ye ole
		// 360/0 discontinuity
		float relativeAngleDelta = calculateRelativeAngleDelta(targetHeading, rollPitchYaw[2]);

		//            	if (state == State.TURN_TO_FINAL_HEADING) {
		//            		logger.info("TTFH: cur yaw = " + rollPitchYaw[2] + ", target yaw = " + targetHeading + ", delta = " + delta);
		//            	}

		// If target is to the right, turn right
		if (relativeAngleDelta > 0) {

			relativeAngleDelta = DEFAULT_ROTATION_SPEED_DEGREES_PER_SECOND * secondsSinceLastUpdate;

		} else if (relativeAngleDelta < 0) {

			relativeAngleDelta = -DEFAULT_ROTATION_SPEED_DEGREES_PER_SECOND * secondsSinceLastUpdate;

		} else {

			if (state == State.TURN_TO_MOVE) {
				// If we were turning to move and now we're done, start moving
				logger.trace("Switching to MOVE");
				state = State.MOVE;
			}else if (state == State.RTL_TURN_TO_MOVE){
				logger.trace("Switching to RTL");
				state = State.RTL;
			}else if (state == State.TURN_TO_FINAL_HEADING) {
				// If we were performing our final turn, we're now done
				logger.trace("Switching to LOITER");
				state = State.LOITER;

				if (lightweightSim) {
					sendGoalStatus(GoalStatus.SUCCEEDED);
				}

				// Finished with that goal, so bump this (FIXME what if movement interrupted?)
				currentMoveBaseGoalId++;
			}
		}

		return relativeAngleDelta;
	}

	private void prepareForNextRotationPhysicsFrame(float targetHeading, float delta) {
		// Figure out what the next value would be...
		float nextValue = (rollPitchYaw[2] + delta) % 360.0f;

		// But before we actually use nextValue, check whether it represents an
		// overshoot
		// (And if it has, just snap to target and be done)
		float nextDelta = calculateRelativeAngleDelta(targetHeading, nextValue);

		if (state == State.TURN_TO_FINAL_HEADING) {
			logger.trace("TTFH: nextValue = " + nextValue + ", delta = " + delta + ", nextDelta = " + nextDelta);
		}

		if ((delta > 0 && nextDelta < 0) || (delta < 0 && nextDelta > 0)) {

			// Snap to final heading. Next frame will officially end turn
			rollPitchYaw[2] = targetHeading;

		} else {

			rollPitchYaw[2] += delta;

			if (rollPitchYaw[2] < 0) {
				rollPitchYaw[2] += 360.0f;
			}

			if (rollPitchYaw[2] >= 360.0f) {
				rollPitchYaw[2] -= 360.0f;
			}
		}
	}

	private void sendGoalStatus(GoalStatus status) {

		// TODO abstract/refactor with code in RosNavigationIface, and pick one JSON lib!
		long time = timeServer.currentTimeMillis();
		int timeSeconds = (int) (time / 1000);
		int timeMilliFraction = (int) (time % 1000);
		int timeNanoFraction = (int) (timeMilliFraction / 1e6);

		int dummySequence = 0;
		int dummyGoalTimestamp = 0;

		com.google.gson.JsonObject stamp = new com.google.gson.JsonObject();
		stamp.addProperty("sec", timeSeconds);
		stamp.addProperty("nsec", timeNanoFraction);

		com.google.gson.JsonObject header = new com.google.gson.JsonObject();
		header.addProperty("seq", dummySequence);
		header.add("stamp", stamp);
		header.addProperty("frame_id", "");

		String textToSend = "Sending status " + status + " for goal ID " + currentMoveBaseGoalId;

		com.google.gson.JsonObject goalId = new com.google.gson.JsonObject();
		goalId.addProperty("stamp", dummyGoalTimestamp);
		goalId.addProperty("id", Integer.toString(currentMoveBaseGoalId));

		com.google.gson.JsonObject goalStatus = new com.google.gson.JsonObject();
		goalStatus.add("goal_id", goalId);
		goalStatus.addProperty("status", status.ordinal());
		goalStatus.addProperty("text", textToSend);

		com.google.gson.JsonArray goalStatusArray = new com.google.gson.JsonArray();
		goalStatusArray.add(goalStatus);

		com.google.gson.JsonObject topLevelJson = new com.google.gson.JsonObject();
		topLevelJson.add("header", header);
		topLevelJson.add("status_list", goalStatusArray);

		JsonObject jsonToSend = RosClientSubscription.parseJson(topLevelJson.toString());

		// This is crude but will work for now.  TODO organize/robustify sim vehicle ROS init some
		synchronized (initSync) {
			if (rosMoveBaseSubscription != null) {
				rosMoveBaseSubscription.publish(myGoalStatusTopic, MOVE_BASE_STATUS_MESSAGE_TYPE,
						jsonToSend);
			}
		}

		logger.info("Sent goal status '" + textToSend + "' to topic " + myGoalStatusTopic);
	}

	private float[] relativeToNed(float x, float y) {

		float[] result = new float[3];

		// In our vehicles, north is 0, +heading rotates clockwise
		float headingAdjustment = rollPitchYaw[2];

		double c = Math.cos(Math.toRadians(headingAdjustment));
		double s = Math.sin(Math.toRadians(headingAdjustment));

		result[0] = (float) (x * c - y * s);
		result[1] = (float) (x * s + y * c);
		result[2] = 0;

		logger.trace("Rel to NED: heading = " + rollPitchYaw[2] + ", in = " + x + ", " + y + "; out = " + result[0]
				+ ", " + result[1]);

		return result;
	}

	/**
	 * Convert from ROS CMD_VEL (BBN Roboclaw style) space to Pixhawk space.
	 * @param deltaYaw Relative yaw in radians.  + is left.
	 * @return relative yaw in degrees.  + is right.
	 */
	private float cmdVelDeltaYawToPixhawkDeltaYaw(double deltaYaw) {
		return (float)Math.toDegrees(-deltaYaw);
	}

	private JsonMessageCallback cmdVelCallback = new JsonMessageCallback() {

		@Override
		public void onNewMessage(JsonObject topMsg) {

			JsonObject msg = topMsg.getJsonObject("msg");

			JsonObject linear = msg.getJsonObject("linear");
			JsonObject angular = msg.getJsonObject("angular");

			float linearX = (float) linear.getJsonNumber("x").doubleValue();
			float angularZ = (float) angular.getJsonNumber("z").doubleValue();

			setLinearAndAngularVelocity(linearX, cmdVelDeltaYawToPixhawkDeltaYaw(angularZ));
		}
	};

	private JsonMessageCallback moveBaseCallback = new JsonMessageCallback() {

		@Override
		public void onNewMessage(JsonObject topMsg) {

			JsonObject msg = topMsg.getJsonObject("msg");
			JsonObject pose = msg.getJsonObject("pose");
			JsonObject positionJson = pose.getJsonObject("position");
			float x = (float) positionJson.getJsonNumber("x").doubleValue();
			float y = (float) positionJson.getJsonNumber("y").doubleValue();
			float z = (float) positionJson.getJsonNumber("z").doubleValue();

			JsonObject rotationJson = pose.getJsonObject("orientation");
			double q0 = rotationJson.getJsonNumber("x").doubleValue();
			double q1 = rotationJson.getJsonNumber("y").doubleValue();
			double q2 = rotationJson.getJsonNumber("z").doubleValue();
			double q3 = rotationJson.getJsonNumber("w").doubleValue();

			Rotation quat = new Rotation(q0, q1, q2, q3, false);
			double deltaHeadingDegrees = Math
					.toDegrees(quat.getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR)[0]);



			JsonObject header = msg.getJsonObject("header");
			String frameId = header.getString("frame_id");

			if (frameId.equalsIgnoreCase(GLOBAL_FRAME_ID)){
				LatLon target = new LatLon(Angle.fromDegrees(x), Angle.fromDegrees(y));
				LatLon current = new LatLon(Angle.fromDegrees(lla[0]), Angle.fromDegrees(lla[1]));
				double distance = Utils.distanceInMetersFromLlaToLla(current, target);
				if (distance >= EPSILON_TO_TARGET) {
					setTarget(new LatLonAlt(x, y, z));
				} else {
					logger.trace("Distance was below epsilon, so we're jumping straight to TURN_TO_FINAL_HEADING");
					state = State.TURN_TO_FINAL_HEADING;
				}
			}else {
				double distance = Math.sqrt(x * x + y * y);
				logger.trace("Distance to target: " + distance);
				if (distance >= EPSILON_TO_TARGET) {
					// Okay, now convert the point and heading from relative to absolute
					float ned[] = relativeToNed(x, y);
					double target[] = Utils.nedToLla(lla, ned);
					setTarget(new LatLonAlt(target[0], target[1], target[2]));
				} else {
					logger.trace("Distance was below epsilon, so we're jumping straight to TURN_TO_FINAL_HEADING");
					state = State.TURN_TO_FINAL_HEADING;
				}
			}

			finalTargetHeading = (float) (rollPitchYaw[2] + deltaHeadingDegrees);
			if (finalTargetHeading < 0) {
				finalTargetHeading += 360;
			}
			if (finalTargetHeading >= 360) {
				finalTargetHeading -= 360;
			}

			if (lightweightSim) {
				sendGoalStatus(GoalStatus.ACTIVE);
			}

			logger.trace("Got move message.  Current yaw = " + rollPitchYaw[2] + ". Delta = " + deltaHeadingDegrees
					+ ". Final target heading: " + finalTargetHeading);
		}
	};

	@Override
	void pushRosState() {

		// Send IMU update
		// NB we do not (currently, anyway) use/need acceleration when the rover starts/stops
		double heading = -Math.toRadians(getYaw());
		if (heading > Math.PI) {
			heading = 2*Math.PI - heading;
		}

		// This is crude but will work for now.  TODO organize/robustify sim vehicle ROS init some
		synchronized (initSync) {

			if (rosCmdVelSubscription != null && rosCmdVelSubscription.isOpen()) {
				JsonObject jsonToSend = ImuMessage.build(0, 0, heading);
				rosCmdVelSubscription.publish(IMU_TOPIC, IMU_MESSAGE_TYPE, jsonToSend);

				// Send wheel odom update
				// It seems that ROS wants position in original vehicle frame, which for us is effectively NWU.  TODO resolve with Giovani
				jsonToSend = OdometryMessage.build(positionNed[0], -positionNed[1], -positionNed[2], heading);
				rosCmdVelSubscription.publish(WHEEL_ODOM_TOPIC, WHEEL_ODOM_TYPE, jsonToSend);

				// Send GPS update
				// We could skip the EKF and run this at a lower rate, but we'll exercise the full IRL-style path for now
				jsonToSend = GpsNavSatMessage.build(lla);
				rosCmdVelSubscription.publish(myNavSatTopic, NAV_SAT_TYPE, jsonToSend);
			}
		}
	}

}
