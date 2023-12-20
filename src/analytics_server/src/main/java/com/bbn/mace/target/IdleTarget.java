//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.target;

import com.bbn.mace.utils.MaceMessageTransport;
import com.google.gson.JsonObject;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.QualityOfService;

import javax.crypto.Mac;


/*
 * Name: IdleTarget
 * Description: Class used to represent an Idle target and its respective data
 */
public class IdleTarget extends Target {

	public IdleTarget(String uid, String host, MaceMessageTransport client, String callsign) throws MqttException {
		super(uid, "IDLE", host, client, callsign);
	}

	
	/*
	 * Publish the idle target state to the desired control station
	 * This method is used to update the command stations
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
		jsonMessage.addProperty("discovered", discovered);
		
		// Convert the message to a string
		String stringMessage = jsonMessage.toString();
		
		// Create a new MQTT message
		MqttMessage message = new MqttMessage();
		
		// Set the MQTT message payload to the JSON object
		message.setPayload(stringMessage.getBytes());
		
		// Set the MQTT quality of service
		message.setQos(serviceLevel.qos);
		
		// Publish the message
		try {
			this.client.publish(topic, message);
		} catch (MqttException e) {
			this.logger.error("The Idle target experienced an expection while publishing a message: ", e);
		}
	}
	
	
	/*
	 * Responsible for updating the target state
	 * This method is used to store the current state of the associated hardware payload
	 */
	public void setTargetState(GpsLocation location) {
		this.setLocation(location);
	}
	
	
	/*
	 * Publish the idle target configurations. The sole configuration that this will be passing
	 * is the configuration type itself. No additional configurations exist
	 */
	public void publishTargetConfigs(String topic, QualityOfService serviceLevel) {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", this.getUid());
		jsonMessage.addProperty("type", this.getTargetType());
		
		// Convert the message to a string
		String stringMessage = jsonMessage.toString();
		
		// Create a new MQTT message
		MqttMessage message = new MqttMessage();
		
		// Set the MQTT message payload to the JSON object
		message.setPayload(stringMessage.getBytes());
		
		// Set the MQTT quality of service
		message.setQos(serviceLevel.qos);
		
		// Publish the message
		try {
			this.client.publish(topic, message);
		} catch (MqttException e) {
			this.logger.error("The Idle target experienced an expection while publishing a message: ", e);
		}
	}
}
