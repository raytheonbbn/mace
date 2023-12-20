//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import com.bbn.ccast.AgentTelemPackage;
import com.bbn.ccast.Utils;
import com.bbn.ccast.config.Configuration;
import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.bbn.ccast.nullsim.blenodes.SimTargetManager;
import com.bbn.ccast.sim.OutOfPlatformPositions;
import com.bbn.ccast.sim.PlatformStartPositions;
import com.bbn.ccast.sim.TimeServer;
import com.bbn.ccast.util.LatLonAlt;
import com.bbn.ccast.util.MathHelper;
import com.bbn.ccast.visualizer.WorldWindAppFrame;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import org.apache.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttException;

import javax.swing.*;
import java.io.File;
import java.util.*;

/**
 * Implement very lightweight vehicle simulation. Intended to replace ArduPilot
 * and per-rotor physics, to allow for many more concurrent platforms at the
 * cost of some fidelity.
 *
 * @author kmoffitt
 */
public class NullSim implements Runnable {

	public static final String PAYLOAD_SIM_TELEM_MQTT_TOPIC = "sim/hardware/telem";

	static final float FPS = 10.0f;

	private static final Logger logger = Logger.getLogger(NullSim.class.getName());
	private static final String SOLO_PREFIX = "Quad_";
	private static final String SOLO_DOWNWARD_PREFIX = "SoloDownFacing_";
	private static final String ROVER_PREFIX = "Rover_";
	private static final long TELEM_UPDATE_TIME = 500;
	private static final double COLLISION_DISTANCE = 0.5; //meter

	private static final String USERNAME = "white_force";
	private static final String PASSWORD = "maceRef!";

	private static NullSim instance = null;

	private final TimeServer timeServer;

	private final Map<String, SimVehicle> allVehicles = new HashMap<>();
	private final MqttSimTaskingListener mqttSimTaskingListener;
	private SimVehicle[] vehicleList;
	private boolean active;
	private boolean collisionEnabled;
	private static long framePeriodMillis = (long) (1000 * (1.0f / FPS));

	private final MaceMessageTransport maceMessageTransport;
	private final SimTargetManager simTargetManager;
	private HashSet<Collision> collidingPlatforms = new HashSet<>();
	private HashSet<String> groundCollidingPlatforms = new HashSet<>();
	private final WorldWindAppFrame worldWindAppFrame;

	public NullSim(String mqttBroker, TimeServer timeServer, WorldWindAppFrame worldWindAppFrame) {
		String uid = "PlatformSim-" + UUID.randomUUID().toString();
		this.maceMessageTransport = new MaceMessageTransport(mqttBroker, uid, USERNAME, PASSWORD);
		
		// Allow everything else to start up. This prevents getting several warnings about the inability to connect.
		while (this.maceMessageTransport.isConnected() == false)
		{
			try
			{
				Thread.sleep(1000);
				logger.info("Waiting for MQTT connection.");
			}
			catch(InterruptedException ex)
			{
				Thread.currentThread().interrupt();
			}
		}

		this.timeServer = timeServer;
		this.simTargetManager = new SimTargetManager(maceMessageTransport);
		this.worldWindAppFrame = worldWindAppFrame;
		this.mqttSimTaskingListener = new MqttSimTaskingListener(this, maceMessageTransport);
	}

	private static class Collision{
		String vehicle1Uid;
		String vehicle2Uid;

		public Collision(String vehicle1Uid, String vehicle2Uid){
			this.vehicle1Uid = vehicle1Uid;
			this.vehicle2Uid = vehicle2Uid;
		}

		@Override
		public boolean equals(Object other){
			if (!(other instanceof Collision)){
				return false;
			}
			return vehicle1Uid.equals(((Collision)other).vehicle1Uid)
					&& vehicle2Uid.equals(((Collision)other).vehicle2Uid);
		}

		@Override
		public int hashCode(){
			return vehicle1Uid.hashCode() ^ vehicle2Uid.hashCode();
		}
	}

	/**
	 * Get the current NullSim instance. If there is no NullSim, it will return null
	 * instead of creating one
	 *
	 * @return current NullSim instance, or null
	 */
	public static NullSim instance() {
		return instance;
	}

	private static void setInstance(NullSim instance) {
		NullSim.instance = instance;
	}

	public Map<String, SimVehicle> getAllVehicles() {
		return allVehicles;
	}

	public WorldWindAppFrame getWorldWindAppFrame(){
		return worldWindAppFrame;
	}

	public void init(int baseMavlinkPort, int numRovers, int numSolos, int numDownFacingSolos, File platfromStartPositionsFile) {

		PlatformStartPositions positions = null;

		// Get agents' positions here, since we can't get them from AirSim
		positions = PlatformStartPositions.load(platfromStartPositionsFile);
		positions.validateAvailability(numSolos, numDownFacingSolos, numRovers);

		vehicleList = new SimVehicle[numSolos + numDownFacingSolos + numRovers];
		int baseIdx = 0;
		// TODO refactor these four loops
		for (int i = 0; i < numSolos; i++) {
			int port = baseMavlinkPort + 10 * i;
			String uid = SOLO_PREFIX + i;
			SimVehicle newVehicle = new SimSolo(uid, timeServer);
			allVehicles.put(uid, newVehicle);
			vehicleList[i] = newVehicle;

			List<Double> nedPosition;
			try {
				nedPosition = positions.getNextSoloPosition();
			} catch (OutOfPlatformPositions e) {
				logger.error("Wanted " + numSolos + " Solo positions, but we only have " + i);
				break;
			}

			newVehicle.setOriginLla(positions.getOriginPosition(), nedPosition.get(0).floatValue(),
					nedPosition.get(1).floatValue(), nedPosition.get(2).floatValue());
			newVehicle.start();
		}

		baseIdx = numSolos;
		for (int i = 0; i < numDownFacingSolos; i++) {
			int port = baseMavlinkPort + 10 * (numSolos + i);
			String uid = SOLO_DOWNWARD_PREFIX + i;
			SimVehicle newVehicle = new SimSolo(uid, timeServer);
			allVehicles.put(uid, newVehicle);
			vehicleList[i + baseIdx] = newVehicle;
			List<Double> nedPosition;
			try {
				nedPosition = positions.getNextSoloPosition();
			} catch (OutOfPlatformPositions e) {
				logger.error("Wanted " + numDownFacingSolos + " down-facing Solo positions, but we only have " + i);
				break;
			}

			newVehicle.setOriginLla(positions.getOriginPosition(), nedPosition.get(0).floatValue(),
					nedPosition.get(1).floatValue(), nedPosition.get(2).floatValue());
			newVehicle.start();
		}


		baseIdx = numSolos + numDownFacingSolos;
		//TODO: Rovers with ROS
		boolean isLighweight = true;
		for (int i = 0; i < numRovers; i++) {
			int port = baseMavlinkPort + 10 * (numSolos + numDownFacingSolos + i);
			String uid = ROVER_PREFIX + i;
			SimVehicle newVehicle = new SimRover(uid, timeServer, port, i, isLighweight);
			allVehicles.put(uid, newVehicle);
			vehicleList[baseIdx + i] = newVehicle;

			List<Double> nedPosition;
			try {
				nedPosition = positions.getNextRoverPosition();
			} catch (OutOfPlatformPositions e) {
				logger.error("Wanted " + numRovers + " Rover positions, but we only have " + i);
				break;
			}

			newVehicle.setOriginLla(positions.getOriginPosition(), nedPosition.get(0).floatValue(),
					nedPosition.get(1).floatValue(), nedPosition.get(2).floatValue());


			newVehicle.start();
		}


		NullSim.setInstance(this);

		logger.info("Initialized " + numSolos + " Solos, "
//				+ numRovers + " Rovers, "
				+ numDownFacingSolos + " Ground-facing Solos in the lightweight sim");
	}

	private void startTelemThread(){
		new Thread(new Runnable() {
			@Override
			public void run() {
				while (!Thread.currentThread().isInterrupted()) {
					for (SimVehicle vehicle : getAllVehicles().values()) {
						sendTelemetry(vehicle.getTelemetry());
					}
					try {
						Thread.sleep(TELEM_UPDATE_TIME);
					} catch (InterruptedException e) {
						Thread.currentThread().interrupt();
					}
				}
			}
		}).start();
	}

	@Override
	public void run() {

		// TODO allow join-y shutdown
		active = true;
		// Start with collsion off
		collisionEnabled = false;
		startTelemThread();

		while (active) {

			long startTime = timeServer.currentTimeMillis();

			// Now we have all the vehicles in the world run their physics and send their
			// updates to AirSim`
			HashSet<String> oldCollidingVehicles = groundCollidingPlatforms;
			groundCollidingPlatforms = new HashSet<>();
			for (SimVehicle vehicle : allVehicles.values()) {
				vehicle.update();
				LatLonAlt lla = vehicle.getPosition();
				Position vehiclePos = new Position(Angle.fromDegrees(lla.getLat()),
						Angle.fromDegrees(lla.getLon()), lla.getAlt());
				simTargetManager.interactWithTargetsInRange(vehiclePos, vehicle.getUid(), vehicle.getIntent());
				simTargetManager.discoverTargetsInRange(vehiclePos, vehicle.getUid(), vehicle.getIntent());
				boolean hitGround = vehicle.checkForGroundCollision();
				if (hitGround){
					String uid = vehicle.getUid();
					groundCollidingPlatforms.add(uid);
					if (!oldCollidingVehicles.contains(uid)) {
						notifyGroundCollision(uid);
					}
				}
			}

			//Check for collisions
			if (collisionEnabled) {
				HashSet<Collision> oldCollisions = collidingPlatforms;
				collidingPlatforms = new HashSet<>();
				for (int i = 0; i < allVehicles.size(); i++) {
					for (int j = i + 1; j < allVehicles.size(); j++) {
						SimVehicle vehicle1 = vehicleList[i];
						SimVehicle vehicle2 = vehicleList[j];
						boolean collided = checkForCollision(vehicle1, vehicle2);
						if (collided) {
							Collision collision = new Collision(vehicle1.getUid(), vehicle2.getUid());
							collidingPlatforms.add(collision);
							if (!oldCollisions.contains(collision)) {
								notifyCollision(collision);
							}
						}
					}
				}
			}


			// And finally we carefully make an effort to make our frame time appropriate
			long endTime = timeServer.currentTimeMillis();

			long frameProcessingDuration = endTime - startTime;
			logger.trace("Processed " + allVehicles.size() + " vehicles in " + frameProcessingDuration + " ms");

			// Sleep up to the point where we hit framePeriodMillis (or at _least_ sleep a
			// milli)
			long sleepDurationMillis = framePeriodMillis - frameProcessingDuration;

			if (sleepDurationMillis < 0) {
				logger.debug("Allocated " + framePeriodMillis + " ms per NullSim frame (for " + allVehicles.size()
						+ " platforms), but consumed " + frameProcessingDuration + " ms");
			}

			sleepDurationMillis = Math.max(sleepDurationMillis, 1);

			try {
				Thread.sleep((long) (sleepDurationMillis / timeServer.getTimeScale()));
			} catch (InterruptedException e) {
				// Don't care
			}
		}
	}

	public void setCollisionEnabled(boolean collisionEnabledIn) {
		collisionEnabled = collisionEnabledIn;
	}

	private void notifyCollision(Collision collision) {
		SwingUtilities.invokeLater(() -> JOptionPane.showMessageDialog(worldWindAppFrame, "Potential collision between " + collision.vehicle1Uid + " and " + collision.vehicle2Uid + "!", "Collision Warning",
				JOptionPane.ERROR_MESSAGE));
	}

	private void notifyGroundCollision(String uid) {
		SwingUtilities.invokeLater(() -> JOptionPane.showMessageDialog(worldWindAppFrame, uid + " hit the ground while not attempting to land!", "Collision Warning",
				JOptionPane.ERROR_MESSAGE));
	}

	private boolean checkForCollision(SimVehicle simVehicle1, SimVehicle simVehicle2) {
		LatLonAlt pos1 = simVehicle1.getPosition();
		LatLonAlt pos2 = simVehicle2.getPosition();
		if ((pos1.getLat() == 0 && pos1.getLon() == 0 ) || (pos2.getLat() == 0 && pos2.getLon() == 0)){
			//If one vehicle has no gps fix, assume we're not colliding.
			return false;
		}
		LatLon lla1 = new LatLon(Angle.fromDegrees(pos1.getLat()), Angle.fromDegrees(pos1.getLon()));
		LatLon lla2 = new LatLon(Angle.fromDegrees(pos2.getLat()), Angle.fromDegrees(pos2.getLon()));
		double distanceH = Utils.distanceInMetersFromLlaToLla(lla1, lla2);
		double distanceV = pos1.getAlt() - pos2.getAlt();
		double distance = Math.sqrt(distanceH*distanceH + distanceV*distanceV);
		return distance <= COLLISION_DISTANCE;
	}




	public void sendTelemetry(AgentTelemPackage telem) {
		try {
			String json = telem.toJson();
			maceMessageTransport.publish(PAYLOAD_SIM_TELEM_MQTT_TOPIC, json, MaceMessageTransport.QualityOfService.UNRELIABLE);
		} catch (MqttException e) {
			logger.warn("Failed to send simulated platform telemetry: ", e);
		}
		List<MathHelper.LatLon> path = telem.getCurrentGoalPositionList();
		if (path == null && telem.getCurrentGoalPosition() != null) {
			path = new ArrayList<>();
			path.add(telem.getCurrentGoalPosition());
		}
		worldWindAppFrame.getVisualization().setAgentGoalLine(telem.getUid(), path,
				telem.getLatitude(), telem.getLongitude(), telem.getAltitude());
	}

	public static NullSim createAndStartNullSim(Configuration configuration, WorldWindAppFrame worldWindAppFrame){
		File platformStartPositionsFile = configuration.getPlatfromStartPositionsFile();
		double initialTimeScale = configuration.getInitialTimeScale();
		TimeServer timeServer = configuration.isTimeScalable() ? new TimeServer(initialTimeScale) : new TimeServer();

		int numSolos = configuration.getNumSolos() == null ? 0 : configuration.getNumSolos();
		int numDownFacingSolos = configuration.getNumDownFacingSolos() == null ? 0 : configuration.getNumDownFacingSolos();
		int numRovers = configuration.getNumRovers() == null ? 0 : configuration.getNumRovers();

		NullSim sim = new NullSim(configuration.getMqttBroker(), timeServer, worldWindAppFrame);
		Thread simThread = new Thread(sim, "Main NullSim driver");
		sim.init(15760,
				numRovers,
				numSolos,
				numDownFacingSolos,
				platformStartPositionsFile);
		simThread.start();
		return sim;
	}

	public void addSimTarget(Position position){
		simTargetManager.createSimTarget(position);
	}

	public void removeSimTarget(String targetId){
		simTargetManager.removeSimTarget(targetId);
	}

	public static void main(String[] args) {

		Configuration configuration = Configuration.loadArgsAndProperties(args);

		int numRovers = configuration.getNumRovers();
		int numSolos = configuration.getNumSolos();
		int numDownFacingSolos = configuration.getNumDownFacingSolos();

		NullSim sim = new NullSim(configuration.getMqttBroker(), new TimeServer(), null);
		Thread simThread = new Thread(sim);

		sim.init(15760, numRovers, numSolos, numDownFacingSolos,  new File("regions/cambridge/platform-start-positions.yaml"));

		simThread.start();
	}

	public SimTargetManager getSimTargetManager() {
		return simTargetManager;
	}

	public void setSimTargetPosition(String targetId, Position position) {
		simTargetManager.createOrUpdateSimTarget(targetId, position);
	}
}
