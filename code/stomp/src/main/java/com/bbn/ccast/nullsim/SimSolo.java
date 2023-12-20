//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import com.bbn.ccast.Utils;
import com.bbn.ccast.config.Configuration;
import com.bbn.ccast.sim.TimeServer;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;

import javax.vecmath.Vector3d;
import java.util.List;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.LinkedBlockingQueue;


public class SimSolo extends SimVehicle {

	public static final float DEFAULT_SPEED = TOP_SPEED;

	public static final float BATTERY_LIFE_SECONDS = 22.0f * 60.0f;
	protected static final float BATTERY_USAGE_PER_SECOND = 1.0f/BATTERY_LIFE_SECONDS;

	protected static final float DEFAULT_TAKEOFF_SPEED = 2.0f;

    protected static final float DEFAULT_LAND_SPEED = 2.0f;
    protected static final float DEFAULT_LANDED_ALTITUDE = 0.025f;

    protected static final int COMMAND_PERSISTENCE_DURATION_MS = 3000;

	private static final int HEARTBEAT_PERIOD = 1000;
	private static final int GPS_PERIOD = 500;

    protected float targetTakeoffAltitudeAgl;

	private final Timer commandTimeoutTimer = new Timer();

	private long nextHeartbeatTime;
	private long nextGpsTime;

	SimSolo(String uid, TimeServer timeServer) {
		super(uid, timeServer);
	}

	@Override
	void start() {
//
//		MavlinkTransport transport = new PipedMavlinkTransport(name, true);
//		this.mavThread = new NullSimMavLinkThread(this, transport);
//		mavThread.start();
	}

    @Override
	public void returnToLaunch() {

        synchronized (stateLock) {
        	//Use current altitude, so we can move to home then land
        	setTarget(new LatLonAlt(homeLla[0], homeLla[1], lla[2]));
            state = State.RTL_TURN_TO_MOVE;

			// Now that we've set velocity in setTarget(), never mind the actual position,
			// since RTL has its own specialized completion test
//			targetPosition = null;
        }
    }

    @Override
	public void land() {
        synchronized (stateLock) {
            state = State.LANDING;
            velocity[0] = 0;
            velocity[1] = 0;
            velocity[2] = DEFAULT_LAND_SPEED;
        }
    }

    @Override
	void takeoff(float altitude) {

        synchronized (stateLock) {
            state = State.TAKEOFF;
            targetTakeoffAltitudeAgl = altitude;
            velocity[0] = 0;
            velocity[1] = 0;
            velocity[2] = -DEFAULT_TAKEOFF_SPEED;
        }

//        mavThread.acknowledgeTakeoffComplete();
    }

    @Override
	void applyPhysics() {

		synchronized (stateLock) {
	        long currentTime = timeServer.currentTimeMillis();
	        float secondsSinceLastUpdate = (currentTime - lastPhysicsUpdateTime) / 1000.0f;

	        // Don't break poor MAVLink even if we sit at a breakpoint for a while
	        secondsSinceLastUpdate = (float) Math.min(secondsSinceLastUpdate, 0.5);

	        // As long as we're not in the first frame (since we need to know secondsSinceLastUpdate)...
	        if (lastPhysicsUpdateTime != 0) {

	            // Update position (TODO handle collisions)
	            for (int i = 0; i < 3; i++) {
	                positionNed[i] += velocity[i] * secondsSinceLastUpdate;
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

				if (targetPosition != null && state != State.TURN_TO_MOVE && state != State.RTL_TURN_TO_MOVE) {

					Vector3d vectorToTarget = Utils.nedVectorInMetersFromVector3dToVector3d(new Vector3d(lla),
							new Vector3d(targetPosition.lat, targetPosition.lon, targetPosition.alt));

					double distance = vectorToTarget.length();

					// TODO figure out why this happens (epsilon mis-match, or something? -Oh, it's
					// because the util method may return a zero vector)
					if (velocity[0] == 0 && velocity[1] == 0 && velocity[2] == 0) {
						logger.debug("State is MOVE but velocity is 0");
						distance = 0d;
					}

					if (distance < EPSILON_TO_TARGET) {
						logger.debug(getUid() + " distance is " + distance + ", so we think we've reached our destination");
						clearVelocity();
						targetPosition = null;
						if (state == State.RTL){
							land();
						}else {
							state = State.LOITER;
						}
					}
				}

	            lla = Utils.nedToLla(originLla, positionNed);
	            int numDegreesOfFreedomAdjusted = 0;

	            // Update orientation
	            for (int i = 0; i < 3; i++) {

	                // First: calculate delta, in [-180, 180] so we don't have to worry about ye ole
	                // 360/0 discontinuity
	                float delta = calculateRelativeAngleDelta(targetRollPitchYaw[i], rollPitchYaw[i]);

	                // If target is to the right, turn right
	                if (delta > 0) {
	                    delta = DEFAULT_ROTATION_SPEED_DEGREES_PER_SECOND * secondsSinceLastUpdate;
	                } else if (delta < 0) {
	                    delta = -DEFAULT_ROTATION_SPEED_DEGREES_PER_SECOND * secondsSinceLastUpdate;
	                } else {
	                    continue;
	                }

	                // Figure out what the next value would be...
	                float nextValue = (rollPitchYaw[i] + delta) % 360.0f;

	                // But before we actually use nextValue, check whether it represents an
	                // overshoot
	                // (And if it has, just snap to target and be done)
	                float nextDelta = calculateRelativeAngleDelta(targetRollPitchYaw[i], nextValue);
	                if ((delta > 0 && nextDelta < 0) || (delta < 0 && nextDelta > 0)) {
	                    rollPitchYaw[i] = targetRollPitchYaw[i];
	                } else {
	                    rollPitchYaw[i] += delta;
	                    if (rollPitchYaw[i] < 0) {
	                        rollPitchYaw[i] += 360.0f;
	                    }
	                    if (rollPitchYaw[i] >= 360.0f) {
	                        rollPitchYaw[i] -= 360.0f;
	                    }

	                    numDegreesOfFreedomAdjusted++;
	                }
	            }

	            // Have we finished turning to move?  If so, move.
	            if ((state == State.TURN_TO_MOVE || state == State.RTL_TURN_TO_MOVE) && numDegreesOfFreedomAdjusted == 0) {

	        		Vector3d newVelocity = Utils.nedVectorInMetersFromVector3dToVector3d(new Vector3d(lla),
	        				new Vector3d(targetPosition.lat, targetPosition.lon, targetPosition.alt));

	        		double EPSILON = 0.01;
	        		if (newVelocity.length() > EPSILON) {
	        			newVelocity.normalize();
	        		}

					State finalState;
					if (state == State.RTL_TURN_TO_MOVE){
						finalState = State.RTL;
					}else{
						finalState = State.MOVE;
					}

	        		setVelocity((float) newVelocity.x * DEFAULT_SPEED, (float) newVelocity.y * DEFAULT_SPEED, (float) newVelocity.z * DEFAULT_SPEED);

	        		state = finalState;
	            }


	            // Check for takeoff completion - targetTakeoffAltitude is Above Ground Level,
	            // so use positionNed, vs. LLA
	            if (state == State.TAKEOFF && positionNed[2] < -targetTakeoffAltitudeAgl) {
	                clearVelocity();
	                if (targetPosition == null) {
						state = State.LOITER;
					}else{
	                	setTarget(targetPosition);
					}
	            } else if ((state == State.LANDING) && positionNed[2] > -DEFAULT_LANDED_ALTITUDE) {
	            	positionNed[2] = -DEFAULT_LANDED_ALTITUDE;
	                clearVelocity();
	                state = State.LANDED;
	                // Disarm on land detection, as ArduCopter does in practice
	                armDisarm(false);
	            }
	        }

	        if (state != State.LANDED) {
	        	setBatteryRemaining(batteryRemaining - secondsSinceLastUpdate * BATTERY_USAGE_PER_SECOND);
	        }

	        lastPhysicsUpdateTime = currentTime;
		}
    }

	@Override
	public void setTarget(LatLonAlt target) {

		synchronized (stateLock) {
			// First, arm and takeoff as neccessary, then rotate to face target, then move to it
			targetPosition = target;

			if (!isArmed()){
				armDisarm(true);
			}
			if (state == State.LANDED){
				takeoff((float) target.getAlt());
				return;
			}

			Vector3d newVelocity = Utils.nedVectorInMetersFromVector3dToVector3d(new Vector3d(lla),
					new Vector3d(target.lat, target.lon, target.alt));

			double EPSILON = 0.01;
			if (newVelocity.length() > EPSILON) {
				newVelocity.normalize();
			}

			float targetYaw = (float) Math.toDegrees(Math.atan2(newVelocity.x, newVelocity.y));

			// Now this is standard math, in which the origin is E and rotation is CCW. We
			// want origin N and rotation CW. So...
			targetYaw *= -1;
			targetYaw += 90.0f;
			if (targetYaw < 0) {
				targetYaw += 360.0f;
			}

			setAbsoluteYawTarget(targetYaw);

			// By default, indicate that we just want to leave ourselves at resting yaw when we arrive.
			setFinalTargetHeading(NO_FINAL_HEADING_ADJUSTMENT);
			logger.debug("Based on new target " + target + ", set velocity for " + getUid() + " to " + newVelocity.x + ", "
					+ newVelocity.y + ", " + newVelocity.z);
			setVelocity((float) newVelocity.x, (float) newVelocity.y, (float) newVelocity.z);

			state = State.TURN_TO_MOVE;
		}
	}

	@Override
	void setVelocity(float vx, float vy, float vz) {

		synchronized (stateLock) {

			state = State.MOVE;

			velocity[0] = vx;
			velocity[1] = vy;
			velocity[2] = vz;
		}
	}

	@Override
	void clearVelocity() {

		synchronized (stateLock) {
			velocity[0] = 0;
			velocity[1] = 0;
			velocity[2] = 0;
		}
	}

	@Override
	void setTimeout() {

		if (timeoutTask != null) {
			timeoutTask.cancel();
		}

		timeoutTask = new TimerTask() {

			@Override
			public void run() {

				synchronized (stateLock) {

					state = State.LOITER;
					velocity[0] = 0;
					velocity[1] = 0;
					velocity[2] = 0;
				}
			}
		};

		commandTimeoutTimer.schedule(timeoutTask, (long)(COMMAND_PERSISTENCE_DURATION_MS / timeServer.getTimeScale()));
	}

	@Override
	void pushMavState() {
//
//		if (!mavThread.isConnected()) {
//			return;
//		}
//
//		long startTime = timeServer.currentTimeMillis();
//
//		// If it's time to send heartbeats, do that
//		if (startTime >= nextHeartbeatTime) {
//				sendHeartbeat();
//
//			nextHeartbeatTime = startTime + HEARTBEAT_PERIOD;
//		}
//
//		// If it's time to send GPS, do that
//		if (startTime >= nextGpsTime) {
//
//			sendGps();
//			nextGpsTime = startTime + GPS_PERIOD;
//		}
	}

//	private void sendGps() {
//
//		// TODO consider a single static message vs. a new one every time
//		msg_global_position_int gpsMessage = new msg_global_position_int();
//
//		gpsMessage.lat = (int) (lla[0] * 1e7);
//		gpsMessage.lon = (int) (lla[1] * 1e7);
//		gpsMessage.alt = (int) (lla[2] * 1000);
//
//		gpsMessage.vx = (short) (velocity[0] * 100);
//		gpsMessage.vy = (short) (velocity[1] * 100);
//		gpsMessage.vz = (short) (velocity[2] * 100);
//
//		gpsMessage.relative_alt = (int) (-positionNed[2] * 1000);
//
//		gpsMessage.hdg = (int) (rollPitchYaw[2] * 100.0);
//
//		msg_attitude attitudeMessage = new msg_attitude();
//
//		attitudeMessage.roll = (float) Math.toRadians(rollPitchYaw[0]);
//		attitudeMessage.pitch = (float) Math.toRadians(rollPitchYaw[1]);
//		attitudeMessage.yaw = (float) Math.toRadians(rollPitchYaw[2]);
//
//		msg_battery_status batteryStatus = new msg_battery_status();
//
//		batteryStatus.battery_remaining = (byte) (batteryRemaining * 100.0f);
//
//		try {
//			mavThread.sendMessage(gpsMessage);
//			mavThread.sendMessage(attitudeMessage);
//			mavThread.sendMessage(batteryStatus);
//		} catch (IOException e) {
//			logger.error("Failed to send GPS position over MAVLink");
//			e.printStackTrace();
//			try {
//				// Probably we're at end of session. In any case, don't spin here; give it a
//				// second before we try again
//				Thread.sleep(1000);
//			} catch (InterruptedException e1) {
//				// Don't care
//			}
//		}
//	}
//
//	void sendHeartbeat() {
//
//		if (!mavThread.isConnected()) {
//			return;
//		}
//
//		try {
//			mavThread.sendHeartbeat();
//			mavThread.sendHomePosition(homeLla[0], homeLla[1], homeLla[2]);
//		} catch (IOException e) {
//			logger.warn("Exception sending heartbeat for " + getName() + ":" + e);
//			e.printStackTrace();
//		}
//	}

	@Override
	protected void handleCustomModeSwitch(long custom_mode) {
		// NOP for the moment
	}

	@Override
	public boolean checkForGroundCollision() {
		if (state == State.LANDED || state == state.LANDING){
			return false;
		}
		return getPosition().getAlt() <= 0;
	}


}
