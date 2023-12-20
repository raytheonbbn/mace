//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.target;

import com.bbn.mace.utils.MaceMessageTransport;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.QualityOfService;
import com.google.gson.JsonObject;

/*
 * Name: PeriodicTarget
 * Description: Class used to represent a Periodic target and its respective data
 */
public class PeriodicTarget extends Target {
	// Class Variables
	protected double acceptableDetectionRange;		// The required range that a payload must be within to be considered in range
	protected int payloadsInRange;					// The current number of payloads in range of the target
	protected int requiredPayloadsForCapture;		// The required number of payloads for target capture
	protected double countdownRequiredForCapture;	// The amount of time that the blue force has to capture the target within after first payload entrance
	protected double currentCaptureDuration;		// The current amount of time that the payloads have been within range of the target
	protected boolean captured;						// The current capture state of the target
	protected boolean setCapturedState;				// Flag used to denote whether the capture state should be reset on the next update
	private boolean suppression;

	public PeriodicTarget(String uid, String host, MaceMessageTransport client, String callback) throws MqttException {
		super(uid, "PERI", host, client, callback);
		
		// Initialize the class variables
		this.acceptableDetectionRange = 0.0;
		this.payloadsInRange = 0;
		this.requiredPayloadsForCapture = 0;
		this.countdownRequiredForCapture = 0.0;
		this.currentCaptureDuration = 0.0;
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
	 * Setter used to set the countdown that a team has to capture the payload
	 */
	private void setCountdownRequiredForCapture(double countdown) {
		this.countdownRequiredForCapture = countdown;
	}
	
	/*
	 * Getter used to retrieve the duration that a team has to capture the payload
	 */
	public double getCountdownRequiredForCapture() {
		return this.countdownRequiredForCapture;
	}
	
	/*
	 * Setter used to set the current duration that a payload has been within the detection range
	 */
	private void setCurrentCaptureDuration(double duration) {
		this.currentCaptureDuration = duration;
	}
	
	/*
	 * Getter used to retrieve the current duration that a payload has been within the detection range
	 */
	public double getCurrentCaptureDuration() {
		return this.currentCaptureDuration;
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

	public void setSuppression(boolean suppression) {
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
	private boolean getSetCapturedStateFlag() {
		return this.setCapturedState;
	}

	/*
	 * Responsible for updating the target state
	 * This method is used to update the stored state of the target according to the current state of the 
	 * associated hardware
	 */
	public void setTargetState(GpsLocation location, int payloads, double duration, boolean captured) {
		this.setLocation(location);
		this.setPayloadsInRange(payloads);
		this.setCurrentCaptureDuration(duration);		
		this.setCaptured(captured);
	}

	/*
	 * Update the configurations of the mass target
	 * This method is used to set the configurations provided by the command station for later publish to the associated hardware
	 */
	public void setTargetConfigs(double range, int requiredPayloads, double countdown, boolean setCaptured, boolean captured, boolean suppression) {
		this.setAcceptableDetectionRange(range);
		this.setRequiredPayloadsForCapture(requiredPayloads);
		this.setCountdownRequiredForCapture(countdown);
		this.setCapturedStateFlag(setCaptured);
		this.setSuppression(suppression);
		
		// Only update the capture state if specifically requested
		if (setCaptured) {
			this.setCaptured(captured);
		}
	}

	/*
	 * Publish the state of the target to the command station
	 * This method is used to update the command stations on the current state of the associated hardware
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
		jsonMessage.addProperty("required_payloads", this.getRequiredPayloadsForCapture());
		jsonMessage.addProperty("current_duration", this.getCurrentCaptureDuration());
		jsonMessage.addProperty("captured", this.getCaptured());
		jsonMessage.addProperty("discovered", discovered);
		jsonMessage.addProperty("interaction_range", this.getAcceptableDetectionRange());
		jsonMessage.addProperty("countdown", this.getCountdownRequiredForCapture());
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
			this.logger.error("The Periodic target experienced an expection while publishing a message: ", e);
		}
	}

	/*
	 * Publish the target settings to the hardware target
	 * This method is used to set the configurations applied by the command station on the associated hardware device
	 */
	public void publishTargetConfigs(String topic, QualityOfService serviceLevel) {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", this.getUid());
		jsonMessage.addProperty("type", this.getTargetType());
		jsonMessage.addProperty("detection_range", this.getAcceptableDetectionRange());
		jsonMessage.addProperty("required_payloads", this.getRequiredPayloadsForCapture());
		jsonMessage.addProperty("capture_countdown", this.getCountdownRequiredForCapture());
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
		} catch (MqttException e) {
			this.logger.error("The Periodic target experienced an expection while publishing a message: ", e);
		}
	}
}
