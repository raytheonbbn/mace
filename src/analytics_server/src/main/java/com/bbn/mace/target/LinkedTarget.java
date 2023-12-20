//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.target;

import com.bbn.mace.utils.MaceMessageTransport;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.QualityOfService;


/*
 * Name: LinkedTarget
 * Description: Class used to represent a Linked target and its respective data
 */
public class LinkedTarget extends Target {
	// Class Variables
	protected double acceptableDetectionRange;	// The detection range that a payload must be within to be counted as in range
	protected int payloadsInRange;				// The current number of payloads in range of the target
	protected ConcurrentHashMap<String, Boolean> networksCaptured;			// The current networks capture state
	protected boolean individualCaptured;		// The current individual capture state
	protected ConcurrentHashMap<String, JsonArray> networks;			// Array of all targets and networks this target is part of
	protected boolean suppression;
	
	public LinkedTarget(String uid, String host, JsonArray targets, String networkName, MaceMessageTransport client, String callback) throws MqttException {
		super(uid, "LINK", host, client, callback);
		
		// Initialize the class variables
		this.acceptableDetectionRange = 0.0;
		this.payloadsInRange = 0;
		this.networksCaptured = new ConcurrentHashMap<>();
		this.networksCaptured.put(networkName, false);
		this.individualCaptured = false;
		this.networks = new ConcurrentHashMap<>();
		this.networks.put(networkName, targets);
		this.suppression = false;
	}
	
	/*
	 * Setter used to set the acceptable detection range of the target
	 */
	private void setAcceptableDetectionRange(double range) {
		this.acceptableDetectionRange = range;
	}
	
	/*
	 * Getter used to retrieve the acceptable detection range for the target
	 */
	public double getAcceptableDetectionRange() {
		return this.acceptableDetectionRange;
	}
	
	/*
	 * Setter used to set the number of payloads within range of the target
	 */
	private void setPayloadsInRange(int payloads) {
		this.payloadsInRange = payloads;
	}
	
	/*
	 * Getter used to retrieve the number of payloads within the target's detection range
	 */
	public int getPayloadsInRange() {
		return this.payloadsInRange;
	}
	
	/*
	 * Setter used to set the capture state of the target
	 */
	public void setNetworkCaptured(String networkName, boolean captured) {
		// once the network is captured, it stays captured
		//if (!suppression && networksCaptured.containsKey(networkName) && Boolean.TRUE.equals(networksCaptured.get(networkName)))
		//	return;

		networksCaptured.remove(networkName);
		networksCaptured.put(networkName, captured);
	}
	
	/*
	 * Getter used to retrieve the capture state of the target
	 */
	public boolean getNetworkCaptured(String networkName) {
		return this.networksCaptured.get(networkName);
	}

	public JsonObject getAllNetworksCaptured() {
		JsonObject jsonCaptured = new JsonObject();

		for (Map.Entry<String, Boolean> entry : networksCaptured.entrySet()) {
			jsonCaptured.addProperty(entry.getKey(), entry.getValue());
		}

		return jsonCaptured;
	}
	
	/*
	 * Setter used to set the capture state of the target
	 */
	private void setIndividualCaptured(boolean captured) {
		// if the network is captured, then the individual target stays captured
		this.individualCaptured = captured;
	}
	
	/*
	 * Getter used to retrieve the capture state of the target
	 */
	public boolean getIndividualCaptured() {
		return this.individualCaptured;
	}

	private void setSuppression(boolean suppression) {
		this.suppression = suppression;
	}

	public boolean getSuppression() {
		return this.suppression;
	}
	
	/*
	 * Getter used to retrieve the array of targets that exist in the network
	 * which the current target belongs to
	 */
	public JsonArray getNetworkTargets(String networkName) {
		return networks.get(networkName);
	}
	
	/*
	 * Setter used to set the array of targets that exist in the network which
	 * which the current target belongs to
	 */
	private void setNetworkTargets(String networkName, JsonArray targets) {
		networks.remove(networkName);
		networks.put(networkName, targets);
	}
	
	/*
	 * Responsible for updating the target state
	 * This method is used to store the state of the associated hardware target
	 */
	public void setTargetState(GpsLocation location, int payloads, boolean captured) {
		this.setLocation(location);
		this.setPayloadsInRange(payloads);
		this.setIndividualCaptured(captured);
	}
	
	/*
	 * Set the configurations of the linked target
	 * This method is used by the command station to set the target configurations
	 */
	public void setTargetConfigs(double range, boolean updateCaptured, boolean netCaptured, JsonArray targets, String networkName, boolean suppression) {
		this.setAcceptableDetectionRange(range);
		this.setNetworkTargets(networkName, targets);
		this.setSuppression(suppression);
		
		// Require that the network capture state have intention to be modified
		if (updateCaptured) {
			this.setNetworkCaptured(networkName, netCaptured);
		}
	}

	public JsonObject getNetwork() {
		JsonObject jsonNetwork = new JsonObject();

		for (Map.Entry<String, JsonArray> entry : networks.entrySet()) {
			jsonNetwork.add(entry.getKey(), entry.getValue());
		}

		return jsonNetwork;
	}
	
	/*
	 * Publish the state of the target to the command station
	 * This method is used to inform the command stations of the target state
	 */
	@Override
	public void publishTargetState(String topic, QualityOfService serviceLevel, boolean discovered) {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", this.getUid());
		jsonMessage.addProperty("callsign", this.getCallsign());
		jsonMessage.addProperty("ipAddress", this.getIpAddress());
		jsonMessage.addProperty("type", this.getTargetType());
		jsonMessage.addProperty("latitude", this.getLocation().getLatitude());
		jsonMessage.addProperty("longitude", this.getLocation().getLongitude());
		jsonMessage.addProperty("altitude", this.getLocation().getAltitude());
		jsonMessage.addProperty("payloads", this.getPayloadsInRange());
		jsonMessage.addProperty("captured", this.getIndividualCaptured());
		jsonMessage.add("networks_captured", this.getAllNetworksCaptured());
		jsonMessage.addProperty("discovered", discovered);
		jsonMessage.add("networks", this.getNetwork());
		jsonMessage.addProperty("suppression", this.getSuppression());

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
			this.logger.error("The Linked target experienced an exception while publishing a message: ", e);
		}
	}
	
	/*
	 * Publish the target settings to the hardware target
	 * This method is used to set configurations on the associated target hardware
	 */
	public void publishTargetConfigs(String topic, QualityOfService serviceLevel) {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", this.getUid());
		jsonMessage.addProperty("type", this.getTargetType());
		jsonMessage.addProperty("detection_range", this.getAcceptableDetectionRange());
		jsonMessage.add("networks_captured", this.getAllNetworksCaptured());
		jsonMessage.add("networks", this.getNetwork());
		jsonMessage.addProperty("suppression", this.getSuppression());
				
		// Convert the message to a string
		String stringMessage = jsonMessage.toString();
		
		// Create a new MQTT message
		MqttMessage message = new MqttMessage();
		
		// Set the MQTT message payload to the JSON object
		message.setPayload(stringMessage.getBytes());
		
		// Set the message quality of service
		message.setQos(serviceLevel.qos);
		
		// Publish the message
		try {
			this.client.publish(topic, message);
		} catch (MqttException e) {
			this.logger.error("The Linked target experienced an exception while publishing a message: ", e);
		}
	}
}
