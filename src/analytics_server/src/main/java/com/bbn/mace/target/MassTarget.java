//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.target;

import com.bbn.mace.utils.MaceMessageTransport;
import com.google.gson.JsonObject;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.QualityOfService;


/*
 * Name: MassTarget
 * Description: Class used to represent a Mass target and its respective data
 */
public class MassTarget extends Target {
	
	// Class Variables
	protected double acceptableDetectionRange;	// The range that a payload must be within to be considered in range
	protected int payloadsInRange;				// The current number of payloads in range of the target
	protected int requiredPayloadsForCapture;	// The required number of payloads that are required to be in range of the target for capture
	protected boolean captured;					// The current capture status of the target
	protected boolean setCapturedState;			// Flag used to denote whether the capture state should be reset on the next update
	protected boolean suppression;
	
	public MassTarget(String uid, String host, MaceMessageTransport client, String callback) throws MqttException {
		super(uid, "MASS", host, client, callback);
		
		// Initialize the class variables
		this.acceptableDetectionRange = 0.0;
		this.payloadsInRange = 0;
		this.requiredPayloadsForCapture = 0;
		this.captured = false;
		this.setCapturedState = false;
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
	 * Setter used to set the number of payloads that are required to be in range for a payload to be considered captured
	 */
	private void setRequiredPayloadsForCapture(int payloads) {
		this.requiredPayloadsForCapture = payloads;
	}
	
	
	/*
	 * Getter used to retrieve the number of required payloads for capture
	 */
	public int getRequiredPayloadsForCapture() {
		return this.requiredPayloadsForCapture;
	}
	
	
	/*
	 * Setter used to set the capture state of the target
	 */
	private void setCaptured(boolean captured) {
		this.captured = captured;
	}
	
	
	/*
	 * Getter used to retrieve the capture state of the target
	 */
	public boolean getCaptured() {
		return this.captured;
	}

	private void setSuppression(boolean suppression) {
		this.suppression = suppression;
	}

	public boolean getSuppression() {
		return this.suppression;
	}
	
	
	/*
	 * Setter used to set the setCaptureState flag
	 */
	private void setCapturedStateFlag(boolean reset) {
		this.setCapturedState = reset;
	}
	
	
	/*
	 * Getter used to retrieve the flag denoting whether to reset the setCapturedState
	 */
	public boolean getSetCapturedStateFlag() {
		return this.setCapturedState;
	}
	
	
	/*
	 * Responsible for updating the target state
	 * This method is used by the associated hardware to set the stored state of the target for later
	 * publish to the command station
	 */
	public void setTargetState(GpsLocation location, int payloads, boolean captured) {
		this.setLocation(location);
		this.setPayloadsInRange(payloads);
		this.setCaptured(captured);
	}
	
	
	/*
	 * Set the configurations of the mass target
	 * This method is used by the command stations to set the configurations of the target. Note that the setCaptured
	 * flag is included to require that the command station have a specific intent to set the captured state.
	 */
	public void setTargetConfigs(double range, int requiredPayloads, boolean setCaptured, boolean captured, boolean suppression) {
		this.setAcceptableDetectionRange(range);
		this.setRequiredPayloadsForCapture(requiredPayloads);
		this.setCapturedStateFlag(setCaptured);
		this.setSuppression(suppression);
		
		// Only update the capture state if specifically requested
		if (setCaptured) {
			this.setCaptured(captured);
		}
	}
	
	
	/*
	 * Publish the state of the target to the command station
	 * This method is used to update the command stations on the current state of the associated hardware targets
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
		jsonMessage.addProperty("captured", this.getCaptured());
		jsonMessage.addProperty("discovered", discovered);
		jsonMessage.addProperty("required_payloads", this.getRequiredPayloadsForCapture());
		jsonMessage.addProperty("interaction_range", this.getAcceptableDetectionRange());
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
			this.logger.error("The Mass target experienced an expection while publishing a message: ", e);
		}
	}
	
	
	/*
	 * Publish the target settings to the hardware target
	 * This method is used to set the desired configurations on the associated hardware targets
	 */
	public void publishTargetConfigs(String topic, QualityOfService serviceLevel) {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", this.getUid());
		jsonMessage.addProperty("type", this.getTargetType());
		jsonMessage.addProperty("detection_range", this.getAcceptableDetectionRange());
		jsonMessage.addProperty("required_payloads", this.getRequiredPayloadsForCapture());
		jsonMessage.addProperty("modify_capture_state", this.getSetCapturedStateFlag());
		jsonMessage.addProperty("captured", this.getCaptured());
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

			// Reset the captured flag
			this.setCapturedStateFlag(false);
		} catch (MqttException e) {
			this.logger.error("The Mass target experienced an expection while publishing a message: ", e);
		}
	}
}
