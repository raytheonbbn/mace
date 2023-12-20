//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.target;

import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.MaceMessageTransport;
import com.bbn.mace.utils.QualityOfService;
import com.bbn.mace.utils.UdpNetworkingUtils;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.text.SimpleDateFormat;
import java.util.SimpleTimeZone;


/*
 * Name: Target
 * Description: Abstract class used to define the common behavior and implementation
 * 				across the targets.
 */
public abstract class Target {

	private static final SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.S'Z'");
	private static final long STALE_TIMEOUT_MS = 30*1000; //30 seconds

	static{
		DATE_FORMAT.setTimeZone(new SimpleTimeZone(0, "UTC"));
	}


	// Class Variables
	protected MaceMessageTransport client;
	protected final Logger logger;		// Logger used to log class events and exceptions
	protected final String uid;			// UID used by the associated hardware target
	protected final String targetType;	// Configuration type of the target that the object is associated with
	protected GpsLocation location;		// Current location of the target
	protected final String host;		// Host that the MQTT client should connect to
	public String callsign;             // A human-readable name for this target
	private String ipAddress;			// IP address of the pi running the target, "sim" if isSim

	// Constructor
	public Target(String id, String type, String host, MaceMessageTransport client, String callsign) throws MqttException {

		// Initialize the class variables
		this.host = host;
		this.uid = id;
		this.targetType = type;
		this.location = null;
		this.callsign = callsign;

		// Initialize the class logger
		this.logger = LogManager.getLogger(Target.class.getName());

		this.logger.info("Initializing a new " + type + " target: " + uid);

		// Initialize the new MQTT client
		logger.info("Attempting to create new MQTT client for the target (" + uid + ") object");

		this.client = client;
	}

	/*
	 * Getter used to retrieve the target UID
	 */
	public String getUid() {
		return this.uid;
	}

	/*
	 * Getter/Setter for the target callsign. A human-readable name that can be changed
	 */
	public String getCallsign() {
		if (this.callsign == null) {
			return this.uid;
		}
		return this.callsign;
	}

	public void setCallsign(String callsign) {
		this.callsign = callsign;
	}

	/*
	 * Getter/Setter for the target ip address
	 */
	public String getIpAddress() {
		if (this.ipAddress == null){
			return "sim";
		}
		return this.ipAddress;
	}

	public void setIpAddress(String ipAddress) {
		this.ipAddress = ipAddress;
	}

	/*
	 * Getter used to retrieve the target configuration type
	 */
	public String getTargetType() {
		return this.targetType;
	}


	/*
	 * Getter used to retrieve the current location of a target
	 */
	public GpsLocation getLocation() {
		return this.location;
	}


	/*
	 * Set the location of the target
	 */
	protected void setLocation(GpsLocation location) {
		this.location = location;
	}


	/*
	 * Public method used to publish the target configurations
	 */
	public abstract void publishTargetConfigs(String topic, QualityOfService serviceLevel);

	/*
	 * Public method used by the target to publish its state to the command stations
	 */
	public abstract void publishTargetState(String topic, QualityOfService serviceLevel, boolean discovered);

	/*
	 * Public method used by the target to publish its state as CoT
	 */
	public void publishTargetStateCot(String destinations, boolean discovered){
		if (destinations == null || destinations.isBlank()){
			return;
		}
		if (discovered) {
			String cot = getCot();
			for (String destination : destinations.split(",")) {
				String[] addressPortSplit = destination.split(":");
				String addressStr = addressPortSplit[0].trim();
				String portStr = addressPortSplit[1].trim();
				try {
					InetAddress address = InetAddress.getByName(addressStr);
					int port = Integer.parseInt(portStr);
					UdpNetworkingUtils.sendUdpCot(cot, address, port);
				} catch (NumberFormatException e) {
					logger.error("Failed to parse CoT port from " + destinations + " : ", e);
				} catch (UnknownHostException e) {
					logger.error("Failed to parse CoT host " + addressStr + " from " + destinations + " : ", e);
				}
			}
		}
		//We currently don't publish CoT for undiscovered targets.
	}

	public String getCot(){
		long currentTime = System.currentTimeMillis();
		String timeStr = DATE_FORMAT.format(currentTime);
		String staleStr = DATE_FORMAT.format(currentTime + STALE_TIMEOUT_MS);
		GpsLocation loc = getLocation();
		String toSend = "";
		toSend += "<?xml version='1.0' encoding='UTF-8'?>";
		toSend += "<event version='2.0' uid='" + uid + "' type='" + "a-h-G" + "' time='" + timeStr + "' start='"
				+ timeStr + "' stale='" + staleStr + "' how='m-g'>";
		toSend += "  <point lat='" + loc.getLatitude() + "' lon='" + loc.getLongitude() + "' hae='" + loc.getAltitude() + "' ce='100.0' le='100.0'/>";
		toSend += "</event>";
		return toSend;
	}


	/*
	 * Shut down and close the MQTT client
	 */
	public void shutdown() {
		this.logger.info("Attempting to shutdown the target (" + this.getUid() + ")");
		this.client.disconnectAndClose();
		this.logger.info("Target (" + this.getUid() + ") shutdown successful");
	}
}
