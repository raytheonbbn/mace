//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.payload;

import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.MaceMessageTransport;
import com.bbn.mace.utils.QualityOfService;
import com.bbn.mace.utils.UdpNetworkingUtils;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import org.apache.commons.lang3.StringUtils;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttAsyncClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import java.lang.reflect.Type;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;


/*
 * Name: Payload
 * Description: Class used to represent a payload and to interact with the payload hardwares
 */
public class Payload {

	private static final SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.S'Z'");
	private static final long STALE_TIMEOUT_MS = 30*1000; //30 seconds

	static{
		DATE_FORMAT.setTimeZone(new SimpleTimeZone(0, "UTC"));
	}

	// keeps track of all the targets that all payloads have encountered
	private static final ConcurrentHashMap<String, Boolean> discoveredTargets = new ConcurrentHashMap<>();

	// Class Variables
	private MaceMessageTransport client;	
    private final Logger logger;	// Logger used to log class events and exceptions
    private final String uid;		// UID used by the associated hardware target
    private GpsLocation location;   // Current location of the target
    private Collection<PayloadIntent> intent;			// The type of target that the payload should interact with
    private boolean modifyIntent;	// Flag indicating whether the agent set the payload's intent
    private boolean inRange;		// Boolean indicating whether or not the payload is within range of a target
    private String typeInRange;		// String denoting the type of target in range of the payload
    private final String host;		// MQTT host that the client should connect to
	private boolean isSim;			// The payload is a simulated platform
	private double batteryLevel;	// The battery level of the platform (0-100)
	public String callsign;         // A human-readable name for this payload
	private String ipAddress;		// IP address of the pi running the payload, "sim" if isSim



	// Constructor
	public Payload(String id, String host, MaceMessageTransport client, String callsign) throws MqttException {

		// Initialize the class variables
		this.host = host;
		this.uid = id;
		this.location = null;
		this.inRange = false;
		this.modifyIntent = false;
		this.intent = new ArrayList<PayloadIntent>();
		this.callsign = callsign;


		// Set payloads to idle at start
		this.intent.add(PayloadIntent.IDLE);
		// for (PayloadIntent intent : PayloadIntent.values()) {
		// 	if (!intent.equals(PayloadIntent.IDLE) && !intent.equals(PayloadIntent.FAIL)) {
		// 		this.intent.add(intent);
		// 	}
		// }
		
		// Initialize the class logger
		this.logger = LogManager.getLogger(Payload.class.getName());
		this.client = client;

		logger.info("Payload client (" + uid + ") successfully created");

	}

	/*
	 * Getter used to retrieve the payload UID
	 */
	public String getUid() {
		return this.uid;
	}

	/*
	 * Getter/Setter for the payload callsign. A human-readable name that can be changed
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
	 * Getter used to retrieve the payload IP
	 */
	public String getIpAddress() {
		return this.ipAddress;
	}



	/*
	 * Getter used to retrieve the current intent for the payload
	 */
	public Collection<PayloadIntent> getIntent() {
		return intent;
	}

	public JsonArray getIntentToJsonArray() {
		JsonArray gson = new Gson().toJsonTree(intent).getAsJsonArray();
		return gson;
	}


	/*
	 * Setter used to set the intent of the payload
	 * This is public because this is the sole configurable property that can be set
	 * by the command stations; therefore, it is not necessary to create a redundant public
	 * wrapper.
	 */
	public void setIntent(Collection<PayloadIntent> intent) {
		this.intent = intent;
	}


	/*
	 * Getter used to retrieve the location of the target
	 */
	public GpsLocation getLocation() {
		return this.location;
	}


	/*
	 * Set the location of the target
	 */
	private void setLocation(GpsLocation location) {
		this.location = location;
	}


	/*
	 * Setter used to set the current in range state of the payload
	 */
	private void setInRange(boolean inRange) {
		this.inRange = inRange;
	}


	/*
	 * Getter used to retrieve the in range state of the payload
	 */
	public boolean getInRange() {
		return this.inRange;
	}


	/*
	 * Getter used to retrieve the current type of target in range of the payload
	 */
	public String getTypeInRange() {
		return this.typeInRange;
	}


	/*
	 * Setter used to set the type of target in range of the payload
	 */
	private void setTypeInRange(String type) {
		this.typeInRange = type;
	}


	/*
	 * Getter used to retrieve the modify intent flag which denotes whether an agent directly set the payload intent
	 */
	private boolean getModifyIntent() {
		return this.modifyIntent;
	}


	/*
	 * Setter used to retrieve the modify intent flag which denotes whether an agent directly set the payload intent
	 */
	private void setModifyIntent(boolean modify) {
		this.modifyIntent = modify;
	}


	/**
	 * Function for discovering a target. If a target has already been discovered there is no effect.
	 * @param targetId The uid of the target being discovered
	 * @return `true` if already discovered, `null` if newly discovered
	 */
	public static Boolean discoverTarget(String targetId) {
		return Payload.discoveredTargets.putIfAbsent(targetId, true);
	}

	/**
	 * Checks whether any payload has discovered this target. This does not imply that it is captured,
	 * but that the target is visible.
	 * @param targetId uid of the target
	 * @return If the target should be visible to the Blue Force
	 */
	public static boolean targetDiscovered(String targetId) {
		return Payload.discoveredTargets.containsKey(targetId);
	}

	/**
	 * Reset the program state such that no targets have been discovered
	 */
	public static void resetTargetDiscovery() {
		Payload.discoveredTargets.clear();
	}

	/**
	 * Reset a target such that it has not been discovered
	 * @param targetId uid of the target to undiscover
	 */
	public static void undiscoverTarget(String targetId){
		Payload.discoveredTargets.remove(targetId);
	}

	/*
	 * Public setter used by the associated hardware payloads to update the stored state of the payload
	 */
	public void setPayloadState(GpsLocation location, boolean inRange, String type, boolean modifyIntent, JsonArray intent, boolean isSim, double batteryLevel, String ipAddress) {
		this.setLocation(location);
		this.setInRange(inRange);
		this.setTypeInRange(type);
		this.setModifyIntent(modifyIntent);
		this.isSim = isSim;
		this.batteryLevel = batteryLevel;
		this.ipAddress = ipAddress;

		// Only modify the intent if the modifyIntent flag is set
		if (modifyIntent) {
			Gson gson = new Gson();
			Type conversionType = new TypeToken<ArrayList<PayloadIntent>>(){}.getType();
			ArrayList<PayloadIntent> newIntent = gson.fromJson(intent, conversionType);
			this.setIntent(newIntent);
		}
	}


	/*
	 * Public method used to acknowledge target discoveries
	 */
	public void publishPayloadState(String topic, QualityOfService serviceLevel, String ackTarget) {

		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();

		// Populate the message
		jsonMessage.addProperty("uid", uid);
		jsonMessage.addProperty("targetsAcknowledged", ackTarget);

		// Convert the message to a string
		String stringMessage = jsonMessage.toString();

		// Create a new MQTT message
		MqttMessage message = new MqttMessage();

		// Set the MQTT message payload to the JSON object
		message.setPayload(stringMessage.getBytes());

		// Set the quality of service level for the message
		message.setQos(serviceLevel.qos);

		// Publish the message
		try {
			this.client.publish(topic, message);
		} catch (MqttException e) {
			this.logger.error("The payload experienced an exception while publishing a message: ", e);
		}
	}

	/*
	 * Public method used by the target to publish its state to the command stations
	 */
	public void publishPayloadState(String topic, QualityOfService serviceLevel) {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();

		// Populate the message
		jsonMessage.addProperty("uid", this.getUid());
		jsonMessage.addProperty("callsign", this.getCallsign());
		jsonMessage.addProperty("latitude", this.getLocation().getLatitude());
		jsonMessage.addProperty("longitude", this.getLocation().getLongitude());
		jsonMessage.addProperty("altitude", this.getLocation().getAltitude());
		jsonMessage.addProperty("in_range", this.getInRange());
		jsonMessage.addProperty("type_in_range", this.getTypeInRange());
		jsonMessage.addProperty("modify_intent", this.getModifyIntent());
		jsonMessage.add("intent", this.getIntentToJsonArray());
		jsonMessage.addProperty("batteryLevel", this.batteryLevel);
		jsonMessage.addProperty("loadout", "PAYLOAD");
		jsonMessage.addProperty("neutralized", false);
		jsonMessage.addProperty("isSimPlatform", this.isSim);
		jsonMessage.addProperty("yaw", 0.0);
		jsonMessage.add("capabilities", new JsonArray());
		jsonMessage.addProperty("heading", 0.0);
		jsonMessage.addProperty("timestamp", System.currentTimeMillis());
		jsonMessage.addProperty("ipAddress", this.getIpAddress());
		jsonMessage.addProperty("isoccupied", false);

		// Convert the message to a string
		String stringMessage = jsonMessage.toString();

		// Create a new MQTT message
		MqttMessage message = new MqttMessage();

		// Set the MQTT message payload to the JSON object
		message.setPayload(stringMessage.getBytes());

		// Set the quality of service level for the message
		message.setQos(serviceLevel.qos);

		// Publish the message
		try {
			this.client.publish(topic, message);
		} catch (MqttException e) {
			this.logger.error("The payload experienced an expection while publishing a message: ", e);
		}
	}

	/*
	 * Public method used by the payload to publish its state as CoT
	 */
	public void publishPayloadStateCot(String destinations){
		if (destinations == null || destinations.isBlank()){
			return;
		}
		String cot = getCot();
		for (String destination : destinations.split(",")) {
			String[] addressPortSplit = destination.split(":");
			String addressStr = addressPortSplit[0].trim();
			String portStr = addressPortSplit[1].trim();
			try {
				InetAddress address = InetAddress.getByName(addressStr);
				int port = Integer.parseInt(portStr);
				UdpNetworkingUtils.sendUdpCot(cot, address, port);
			}catch (NumberFormatException e){
				logger.error("Failed to parse CoT port from " + destinations  + " : ", e);
			} catch (UnknownHostException e) {
				logger.error("Failed to parse CoT host " + addressStr + " from " + destinations + " : ", e);
			}
		}
	}

	public String getCot(){
		long currentTime = System.currentTimeMillis();
		String timeStr = DATE_FORMAT.format(currentTime);
		String staleStr = DATE_FORMAT.format(currentTime + STALE_TIMEOUT_MS);
		GpsLocation loc = getLocation();
		String toSend = "";
		toSend += "<?xml version='1.0' encoding='UTF-8'?>";
		toSend += "<event version='2.0' uid='" + uid + "' type='" + "a-f-A" + "' time='" + timeStr + "' start='"
				+ timeStr + "' stale='" + staleStr + "' how='m-g'>";
		toSend += "  <point lat='" + loc.getLatitude() + "' lon='" + loc.getLongitude() + "' hae='" + loc.getAltitude() + "' ce='100.0' le='100.0'/>";
		toSend += "</event>";
		return toSend;
	}


	/*
	 * Public method used to publish the configurations set by the command station to the associated payload hardware
	 */
	public void publishPayloadConfigs(String topic, QualityOfService serviceLevel) {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();

		// Populate the message
		jsonMessage.addProperty("uid", this.getUid());
		jsonMessage.add("intent", this.getIntentToJsonArray());
		
		// Convert the message to a string
		String stringMessage = jsonMessage.toString();

		// Create a new MQTT message
		MqttMessage message = new MqttMessage();

		// Set the MQTT message payload to the JSON object
		message.setPayload(stringMessage.getBytes());

		// Set the quality of service level for the message
		message.setQos(serviceLevel.qos);

		// Publish the message
		try {
			this.client.publish(topic, message);
		} catch (MqttException e) {
			this.logger.error("The payload experienced an expection while publishing a message: ", e);
		}
	}
}
