//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.Utils;
import com.bbn.ccast.nullsim.blenodes.SimTargetManager;
import com.bbn.ccast.sim.TimeServer;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.util.MathHelper;
import com.bbn.ccast.visualizer.tracks.SDOPosition;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import gov.nasa.worldwind.geom.Angle;
import org.apache.log4j.Logger;
import org.msgpack.rpc.error.RemoteError;

import java.util.*;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * A vehicle in the NullSim (very lightweight physics) context. Transacts with
 * MAVLink to send updates and receive commands, and with AirSim to push state
 * and (TODO) handle collision events.
 *
 * @author kmoffitt
 */
public abstract class SimVehicle {

    enum State {
        LANDED, LANDING, RTL, RTL_TURN_TO_MOVE, TAKEOFF, LOITER, MOVE, TURN_TO_MOVE, TURN_TO_FINAL_HEADING
    }

    public enum Intent {
        IDLE, MASS, PERI, LINK, FAIL
    }

    static final float DEFAULT_ROTATION_SPEED_DEGREES_PER_SECOND = 40.0f;

    static final float TOP_SPEED = 3.0f;

    final TimeServer timeServer;

    // TODO can we get this to match IRL?
    protected static final float EPSILON_TO_TARGET = TOP_SPEED / NullSim.FPS;

    protected static final float NO_FINAL_HEADING_ADJUSTMENT = Float.MAX_VALUE;

    protected static final Logger logger = Logger.getLogger(SimVehicle.class.getName());

    protected final Queue<SDOPosition> targetPositions = new LinkedBlockingQueue<>();


    State state = State.LANDED;

    String uid;

    String callsign;

    // TODO find or make vector / Quat classes
    float positionNed[] = new float[3];
    double lla[] = new double[3];
    double originLla[] = new double[3];
    double homeLla[] = new double[3];
    float rollPitchYaw[] = new float[3];
    float targetRollPitchYaw[] = new float[3];
    float velocity[] = new float[3];
    LatLonAlt targetPosition;
    Collection<Intent> intent = new ArrayList<>();

    long lastPhysicsUpdateTime;
    float batteryRemaining = 1.0f;

    TimerTask timeoutTask;

    final Object stateLock = new Object();

    boolean armed = false;

    abstract void applyPhysics();

    public abstract void land();

    public abstract void returnToLaunch();

    abstract void takeoff(float altitude);


    SimVehicle(String uid, TimeServer timeServer) {
        this.uid = uid;
        this.callsign = uid;
        this.timeServer = timeServer;
        this.intent.add(Intent.IDLE);
        // for (Intent intent : Intent.values()) {
        //     if (!intent.equals(Intent.IDLE) && !intent.equals(Intent.FAIL)) {
        //         this.intent.add(intent);
        //     }
        // }
    }

    void start() {
    }

    public String getUid() {
        return uid;
    }

    public abstract void setTarget(LatLonAlt target);

    abstract void setTimeout();

    abstract void clearVelocity();

    abstract void setVelocity(float vx, float vy, float vz);

    void pushRosState() {
    }

    void update() {
        synchronized (stateLock) {
            try {
                applyPhysics();
                pushMavState();
                pushRosState();
            } catch (RemoteError e) {
                logger.error("Failed to transact with AirSim RPC interface.  Aborting sim update frame.");
            } catch (Exception x) {
                logger.error("Exception while processing sim frame.  " + x);
                x.printStackTrace();
            }
        }
    }

    AgentTelemPackage getTelemetry() {
        synchronized (stateLock) {

            double lat = lla[0];
            double lon = lla[1];
            double elev = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(lat),
                    Angle.fromDegreesLongitude(lon));
            double alt = lla[2] + elev;
            double yaw = rollPitchYaw[2];
            HashSet<String> capabilities = new HashSet<>();
            String ipAddr = "sim";

            //TODO: Set these values?
            boolean isOccupied = false;
            boolean isNeutralized = false;

            AgentTelemPackage pkg = new AgentTelemPackage(uid, lat, lon, alt, yaw,
                    timeServer.currentTimeMillis(), "PAYLOAD", capabilities, intent, true, ipAddr,
                    batteryRemaining * 100.0, isOccupied, isNeutralized, callsign
            );
            MathHelper.LatLon goal = new MathHelper.LatLon();

            if (targetPosition != null) {
                goal.lat = targetPosition.getLat();
                goal.lon = targetPosition.getLon();
                goal.alt = targetPosition.getAlt();
                pkg.setCurrentGoalPosition(goal);

                SDOPosition[] targetList = getTargetList();
                if (targetList != null) {
                    List<MathHelper.LatLon> targetPositions = new ArrayList<>();
                    targetPositions.add(goal);
                    for (SDOPosition pos : targetList) {
                        MathHelper.LatLon simplePos = new MathHelper.LatLon();
                        simplePos.lat = pos.getLatitudeDegrees();
                        simplePos.lon = pos.getLongitudeDegrees();
                        simplePos.alt = pos.getAltitude();
                        targetPositions.add(simplePos);
                    }
                    pkg.setCurrentGoalPositionList(targetPositions);
                }

            }

            return pkg;
        }
    }


    /**
     * Express the difference between two angles as a delta between -180 degrees and
     * 180 degrees.
     *
     * @param target  Desired angle, in degrees.
     * @param current Current angle, in degrees.
     * @return delta, in range [-180, 180]
     */
    static float calculateRelativeAngleDelta(float target, float current) {

        float delta = target - current;

        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }

        return delta;
    }


    void pushMavState() {
    }

    // (Rovers behave specially for this)
    void setImmediateYawTarget(float target) {
        setAbsoluteYawTarget(target);
    }

    public void setAbsoluteYawTarget(float target) {
        synchronized (stateLock) {
            targetRollPitchYaw[2] = target;
        }
    }

    public float getYaw() {
        synchronized (stateLock) {
            return rollPitchYaw[2];
        }
    }

    public LatLonAlt getPosition() {
        synchronized (stateLock) {
            return new LatLonAlt(lla[0], lla[1], lla[2]);
        }
    }

    /**
     * @param targetHeading The desired heading to which to yaw after movement is complete.
     */
    protected void setFinalTargetHeading(float targetHeading) {
    }

    protected abstract void handleCustomModeSwitch(long custom_mode);

    public void setOriginLla(double baseLla[], float north, float east, float down) {

        originLla = Utils.nedToLla(baseLla, new float[]{north, east, down});

        homeLla[0] = originLla[0];
        homeLla[1] = originLla[1];
        homeLla[2] = originLla[2];
    }

    protected void setHomeLla(double lat, double lon, double alt) {
        homeLla[0] = lat;
        homeLla[1] = lon;
        homeLla[2] = alt;
    }

    public void armDisarm(boolean arm) {
        armed = arm;
        if (arm) {
            setHomeLla(lla[0], lla[1], lla[2]);
        }
    }

    public boolean isArmed() {
        return armed;
    }

    public synchronized void setBatteryRemaining(float batteryRemaining) {
        this.batteryRemaining = batteryRemaining;
    }

    public float getBatteryRemaining() {
        return this.batteryRemaining;
    }

    public void setTargetList(List<SDOPosition> targetList) {
        synchronized (stateLock) {
            this.targetPositions.clear();
            this.targetPositions.addAll(targetList);
        }
    }

    public SDOPosition[] getTargetList() {
        synchronized (stateLock) {
            return targetPositions.toArray(new SDOPosition[0]);
        }
    }

    /**
     * Return true if we unintentionally hit the ground.
     *
     * @return true if we unintentionally hit the ground
     */
    public abstract boolean checkForGroundCollision();

    public Collection<Intent> getIntent() {
        synchronized (stateLock) {
            return this.intent;
        }
    }

    public JsonArray getIntentToJsonArray() {
        JsonArray gson = new Gson().toJsonTree(intent).getAsJsonArray();
        return gson;
    }

    public static JsonArray getIntentToJsonArray(Collection<Intent> intent) {
        JsonArray gson = new Gson().toJsonTree(intent).getAsJsonArray();
        return gson;
    }

    public void setIntent(Collection<Intent> intent) {
        // update simTargetManager
        SimTargetManager.instance.resetInteractions(this.getUid());

        synchronized (stateLock) {
            this.intent = intent;
        }
    }

    public void clearIntent() {
        this.intent.clear();
    }
}
