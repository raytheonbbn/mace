//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.callback;

import java.lang.reflect.Type;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

import com.bbn.mace.payload.PayloadIntent;
import com.bbn.mace.server.Server;
import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import com.bbn.mace.payload.Payload;
import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.MaceMessageTransport;
import com.bbn.mace.utils.QualityOfService;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;


/*
 * MQTT callback used to handle payload configuration and state changes
 */
public class PayloadCallback extends HardwareCallback<Payload> {

	// Class Variables
	private String host;								// The host that the hardware should publish their respective data to
	private String outputConfigTopic;					// The topic that the analytics server should publish the configurations to the hardware over
	private String outputStateTopic;					// The state topic that the analytics server should publish the state of the hardware to
	private QualityOfService outputConfigQos;			// The qos level that the configs should be published at
	private QualityOfService outputStateQos;			// The qos level that the state data should be published at

	/*
	 * NOTE: The reason that the input topics and input service levels are passed as an array are because the client subscription for multiple 
	 * topics requires that an array be passed with all topics that should be subscribed to. This is not the case for the output topics. Therefore, the output
	 * topics are left as individual parameters
	 */
	public PayloadCallback(String host, String outputConfigTopic, QualityOfService outputConfigQos,
						   String outputStateTopic, QualityOfService outputStateQos, MaceMessageTransport client) throws MqttException {
		super(client);
		
		// Initialize the class variables
		this.host = host;
		this.outputConfigTopic = outputConfigTopic;
		this.outputStateTopic = outputStateTopic;
		this.outputConfigQos = outputConfigQos;
		this.outputStateQos = outputStateQos;
	}
	
	
	/*
	 * Getter used to retrieve the configuration topic that should be published to
	 */
	private String getOutputConfigTopic() {
		return this.outputConfigTopic;
	}
	
	
	/*
	 * Getter used to retrieve the white force state topic that should be published to
	 */
	private String getOutputStateTopic() {
		return this.outputStateTopic;
	}
	
	
	/*
	 * Getter used to retrieve the qos level that should be used when publishing to the configuration topic
	 */
	private QualityOfService getOutputConfigQos() {
		return this.outputConfigQos;
	}
	
	
	/*
	 * Getter used to retrieve the qos level that should be used when publishing to the state topics
	 */
	private QualityOfService getOutputStateQos() {
		return this.outputStateQos;
	}




	/*
	 * The Handler class is responsible for processing the newly received message and updating the associated data.
	 * Furthermore, this class is responsible for publishing the data as necessary after receiving updates
	 */
	private class Handler implements Runnable {
		
		// Sub-class class variables
		private String topic;
		private MqttMessage message;
		private String stateTopic;
		private String configTopic;
		private QualityOfService configQos;
		private QualityOfService stateQos;
		
		public Handler(String topic, MqttMessage message, String configTopic, String stateTopic,  QualityOfService configQos, QualityOfService stateQos) {
			
			// Initialize sub-class class variables
			this.topic = topic;
			this.message = message;
			this.stateTopic = stateTopic;
			this.configTopic = configTopic;
			this.configQos = configQos;
			this.stateQos = stateQos;
		}
		
		
		/*
		 * Sub-class method
		 * Method used to retrieve the topic over which the message was sent
		 */
		private String getInputTopic() {
			return this.topic;
		}
		
		
		/*
		 * Sub-class method
		 * Getter used to retrieve the blue force topic that the state data should be published to
		 */
		private String getStateTopic() {
			return this.stateTopic;
		}
		
		
		/*
		 * Sub-class method
		 * Getter used to retrieve the topic that configurations should be published to
		 */
		private String getConfigTopic() {
			return this.configTopic;
		}
		
		
		/*
		 * Sub-class method
		 * Getter used to retrieve the qos level that the state data should be published at
		 */
		private QualityOfService getStateQos() {
			return this.stateQos;
		}
		
		
		/*
		 * Sub-class method
		 * Getter used to retrieve the qos level that the config data should be published at
		 */
		private QualityOfService getConfigQos() {
			return this.configQos;
		}
		
		
		/*
		 * Sub-class method
		 * Parse the configurations message provided by the command station and send the configurations to the hardware units
		 */
		private void parseConfigs(HashMap<String, JsonElement> message, String topic, QualityOfService qos) {
			// Get the UID/callsign of the payload who's message is being handled
			String uid = message.get("uid").getAsString();
			
			// Determine if the command station is attempting to modify payload that doesn't exist
			if (!hardware.containsKey(uid)) {
				logger.error("No such payload with UID: " + uid + "exists in the map of payloads");
				
				// An error occurred, exit
				return;
			}

			// Check if this message is telling us to change the callsign
			if (message.containsKey("set_callsign")) {
				String callsign = message.get("callsign").getAsString();
				// Check that UID in the message exists
				if (hardware.containsKey(uid)) {
					// Update the callsign on the payload
					logger.info("Received New callsign: " + callsign + " for Payload UID: " + uid);

					Payload payload = hardware.get(uid);
					payload.setCallsign(callsign);
					return;
				}
				
				logger.error("Error assigning callsign: " + callsign + " for Payload UID: " + uid);
				// An error occurred, exit
				return;
			}

			
			// Get the desired intent configuration
			Gson gson = new Gson();
			Type type = new TypeToken<ArrayList<PayloadIntent>>(){}.getType();
			ArrayList<PayloadIntent> desiredIntent = gson.fromJson(message.get("intent").getAsJsonArray(), type);
			
			// Set the payload configs
			hardware.get(uid).setIntent(desiredIntent);
			
			// Publish the configurations after parsing them
			hardware.get(uid).publishPayloadConfigs(topic, qos);
		}
		
		
		/*
		 * Sub-class method
		 * Parse a state message provided by the hardware payload
		 */
		private void parseState(HashMap<String, JsonElement> message, String topic, QualityOfService qos) {
			// Get the UID/callsign of the payload who's message is being handled
			String uid = message.get("uid").getAsString();
			String callsign = message.get("callsign").getAsString();
			
			// Generate a new payload if the map does not already include this payload
			if (!hardware.containsKey(uid)) {
				
				try {
					// Create a new payload with the given uid
					Payload payload = new Payload(uid, host, client, callsign);
					
					// Add the payload to the map of payloads
					hardware.put(uid, payload);
				} catch (MqttException e) {
					logger.error("An error occurred while attempting to generate a new payload: ", e);
					
					// An error occurred, exit and try again on the next message published by the payload
					return;
				}
			}

			Payload payload = hardware.get(uid);
			
			// Read the GPS location
			GpsLocation location = new GpsLocation(message.get("latitude").getAsDouble(), message.get("longitude").getAsDouble(), 
					message.get("altitude").getAsDouble());

			// Set the payload state information
			payload.setPayloadState(location, message.get("in_range").getAsBoolean(), message.get("type_in_range").getAsString(), 
					message.get("modify_intent").getAsBoolean(), message.get("intent").getAsJsonArray(), message.get("isSimPlatform").getAsBoolean(),
					message.get("batteryLevel").getAsDouble(), message.get("ipAddress").getAsString());

			// add discovered targets to set
			message.get("discovered_targets").getAsJsonArray().forEach(x -> {

				if (Payload.discoverTarget(x.getAsString()) == null) {
					// We have discovered the target
					logger.info(String.format("target %s discovered by %s", x.getAsString(), uid));
				}

				// Acknowledge that we have found it
				String ackTopic = String.format("%s/payload/agent/acknowledge_discovered", uid);
				payload.publishPayloadState(ackTopic, qos, x.getAsString());
			});
			
			// Publish the payload state information to the command stations
			payload.publishPayloadState(topic, qos);

			payload.publishPayloadStateCot(Server.getCotDestinations());
		}

		
		/*
		 * Sub-class method
		 * This method is the method invoked by the new thread when a callback function is called in the main class
		 * The method is responsible for converting the message into a HashMap and passing it of to the correct parsing method
		 */
		@Override
		public void run() {			
			// Create a new parser to parse the message
			JsonParser parser = new JsonParser();
			
			// Get the message as a JSON object
			JsonObject jsonMessage = parser.parse(new String(message.getPayload())).getAsJsonObject();
			
			// Get the entry set from the Object
			Set<Map.Entry<String, JsonElement>> entries = jsonMessage.entrySet();
			
			// Create a new HashMap to store the entries in
			HashMap<String, JsonElement> jsonMap = new HashMap<String, JsonElement>();
			
			// Put all of the entries into the JSON HashMap
			for(Map.Entry<String, JsonElement> entry : entries)
		    {
		        jsonMap.put(entry.getKey(), entry.getValue());
		    }
			
			// Determine whether the message is a configuration or state message
			// Process the message according to its type
			if (this.getInputTopic().contains("configurations")) {
				this.parseConfigs(jsonMap, this.getConfigTopic(), this.getConfigQos());
			} else if (this.getInputTopic().contains("state")) {
				this.parseState(jsonMap, this.getStateTopic(), this.getStateQos());
			}
		}
	}


	/*
	 * Callback function used to receive a message and distribute the message to a new thread handler
	 */
	@Override
	public void messageArrived(String topic, MqttMessage message) throws Exception {
		this.pool.execute(new Handler(topic, message, this.getOutputConfigTopic(), this.getOutputStateTopic(),
				this.getOutputConfigQos(), this.getOutputStateQos()));
	}


}
