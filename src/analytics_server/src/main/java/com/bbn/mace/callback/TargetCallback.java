//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.callback;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import com.bbn.mace.server.Server;
import com.bbn.mace.utils.MaceMessageTransport;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import com.bbn.mace.target.Target;
import com.bbn.mace.payload.Payload;
import com.bbn.mace.target.IdleTarget;
import com.bbn.mace.target.LinkedTarget;
import com.bbn.mace.target.LinkedNetwork;
import com.bbn.mace.target.MassTarget;
import com.bbn.mace.target.PeriodicTarget;
import com.bbn.mace.utils.GpsLocation;
import com.bbn.mace.utils.QualityOfService;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;


/*
 * MQTT callback to handle target configuration changes and state changes
 */
public class TargetCallback extends HardwareCallback<Target> {

	// Class Variables
	private String host;								// The host that the hardware should publish their respective data to
	public ConcurrentHashMap<String, LinkedNetwork> networks;		// The set of linked networks that are being managed
	private String outputConfigTopic;					// The topic that the analytics server should publish the configurations to the hardware over
	private String outputWhiteForceStateTopic;			// The white force state topic that the analytics server should publish the state of the hardware to
	private String outputBlueForceStateTopic;			// The blue force state topic that the analytics server should publish the state of the hardware to
	private QualityOfService outputConfigQos;			// The qos level that the configs should be published at
	private QualityOfService outputStateQos;			// The qos level that the state data should be published at
	private String networkTopic;
	private QualityOfService networkQos;
	
	
	/*
	 * NOTE: The reason that the input topics and input service levels are passed as an array are because the client subscription for multiple 
	 * topics requires that an array be passed with all topics that should be subscribed to. This is not the case for the output topics. Therefore, the output
	 * topics are left as individual parameters
	 */
	public TargetCallback(String host, String outputConfigTopic, QualityOfService outputConfigQos,
						  String outputWhiteForceStateTopic, String outputBlueForceStateTopic, QualityOfService outputStateQos,
						  MaceMessageTransport client) throws MqttException {
		super(client);
		
		// Initialize the class variables
		this.host = host;
		this.networks = new ConcurrentHashMap<String, LinkedNetwork>();
		this.outputConfigTopic = outputConfigTopic;
		this.outputWhiteForceStateTopic = outputWhiteForceStateTopic;
		this.outputBlueForceStateTopic = outputBlueForceStateTopic;
		this.networkTopic = "hardware/server/target/configurations";
		this.outputConfigQos = outputConfigQos;
		this.outputStateQos = outputStateQos;
		this.networkQos = outputConfigQos;
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
	private String getOutputWhiteForceStateTopic() {
		return this.outputWhiteForceStateTopic;
	}
	
	
	/*
	 * Getter used to retrieve the blue force state topic that should be published to
	 */
	private String getOutputBlueForceStateTopic() {
		return this.outputBlueForceStateTopic;
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
	 * Publish the target configurations provided by the command station to the associated hardware component
	 */
	private void publishTargetConfigs(String uid, String topic, QualityOfService serviceLevel) {
		this.hardware.get(uid).publishTargetConfigs(topic, serviceLevel);
	}
	
	
	/*
	 * Publish the network configurations provided by the command station to the associated linked network
	 */
	private void publishNetworkConfigs(String uid, String topic, QualityOfService serviceLevel) {
		this.networks.get(uid).publishNetworkConfigs(topic, serviceLevel);
	}
	
	
	/*
	 * Publish the hardware state provided by the associated hardware to the command station
	 */
	private void publishTargetState(String uid, String topic, QualityOfService serviceLevel, boolean discovered) {
		this.hardware.get(uid).publishTargetState(topic, serviceLevel, discovered);
	}

	/*
	 * Publish the hardware state provided by the associated hardware as CoT
	 */
	private void publishTargetStateCot(String uid, String destinations, boolean discovered) {
		this.hardware.get(uid).publishTargetStateCot(destinations, discovered);
	}


	/*
	 * The Handler class is responsible for processing the newly received message and updating the associated data.
	 * Furthermore, this class is responsible for publishing the data as necessary after receiving updates
	 */
	private class Handler implements Runnable {
		
		// Sub-class class variables
		private String topic;
		private MqttMessage message;
		private String whiteForceStateTopic;
		private String blueForceStateTopic;
		private String configTopic;
		private QualityOfService configQos;
		private QualityOfService stateQos;
		
		public Handler(String topic, MqttMessage message, String configTopic, String whiteForceStateTopic, 
				String blueForceStateTopic, QualityOfService configQos, QualityOfService stateQos) {
			
			// Initialize sub-class class variables
			this.topic = topic;
			this.message = message;
			this.whiteForceStateTopic = whiteForceStateTopic;
			this.blueForceStateTopic = blueForceStateTopic;
			this.configTopic = configTopic;
			this.configQos = configQos;
			this.stateQos = stateQos;
		}
		
		
		private String getInputTopic() {
			return this.topic;
		}
		
		
		/*
		 * Sub-class method
		 * Getter used to retrieve the white force topic that the state data should be published to
		 */
		private String getWhiteForceStateTopic() {
			return this.whiteForceStateTopic;
		}
		
		
		/*
		 * Sub-class method
		 * Getter used to retrieve the blue force topic that the state data should be published to
		 */
		private String getBlueForceStateTopic() {
			return this.blueForceStateTopic;
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

			// First, check if this is a linked network config message
			// If it is, handle this in the handleLinkConfigs helper method
			if (message.containsKey("network")){
				this.handleLinkConfigs(message, topic, qos);
				return;
			}

			// Get the UID of the target who's configurations will be updated
			String uid = message.get("uid").getAsString();

			// Determine if the command station is attempting to modify a target that doesn't exist
			if (!hardware.containsKey(uid)) {
				logger.error("No such target with UID: " + uid + " exists in the map of targets");
				// An error occurred, exit
				return;
			}

			Target target    = hardware.get(uid);
			String ipAddress = target.getIpAddress();
			String callsign  = target.getCallsign();

			// Check if this message is telling us to change the callsign
			if (message.containsKey("set_callsign")) {
				// Check that old UID exists
				String newCallsign = message.get("callsign").getAsString();
				// Update the uid on the target
				target.setCallsign(newCallsign);
				return;
			}
			
			// Get the desired target configuration for the target
			String desiredTargetType = message.get("type").getAsString();
			
			// Ensure that the target configuration is valid
			if (!desiredTargetType.equals("IDLE") && !desiredTargetType.equals("MASS") && !desiredTargetType.equals("LINK") && !desiredTargetType.equals("PERI")) {
				logger.error("An invalid target configuration type was provided. Rejecting the configuration changes.");
				
				return;
			}

			/* 
			* Check if the newly configured type is equal to the current type
			* If not, update the type of target in the dictionary and set its configurations
			* If equal, just update the configurations
			*/
			if (!desiredTargetType.equals(target.getTargetType())) {
				// If we are switching a linked target that is part of a network to a different type
				// Remove it from the network 
				if (target.getTargetType().equals("LINK")) {
					for (LinkedNetwork entry : networks.values()) {
						if (entry.containsTarget(uid)) {
							entry.removeTarget(uid);
						}
					}
				}
				
				// Change the target object by creating a new one and overwriting the old target in the hardware map
				try {
					if (desiredTargetType.equals("IDLE")) {
						// Create a new idle target
						IdleTarget newTarget = new IdleTarget(uid, host, client, callsign);
						newTarget.setIpAddress(ipAddress);
						// Replace the target in the map
						hardware.put(uid, newTarget);
					} else if (desiredTargetType.equals("MASS")) {
						// Create a new mass target
						MassTarget newTarget = new MassTarget(uid, host, client, callsign);
						newTarget.setIpAddress(ipAddress);
						// Replace the target in the map
						hardware.put(uid, newTarget);
					} else if (desiredTargetType.equals("PERI")) {
						// Create a new periodic target
						PeriodicTarget newTarget = new PeriodicTarget(uid, host, client, callsign);
						newTarget.setIpAddress(ipAddress);
						// Replace the target in the map
						hardware.put(uid, newTarget);
					}
					// Update "target" as it has been replaced in the hardware map
					target = hardware.get(uid);
				} catch (MqttException e) {
					logger.error("An error occurred while attempting to replace a target (" + uid + "): ", e);
					return;
				}					
			}
			
			// Update the additional configurations associated with the mass and periodic target types
			if (desiredTargetType.equals("MASS")) {
				((MassTarget)target).setTargetConfigs(message.get("detection_range").getAsDouble(), 
						message.get("required_payloads").getAsInt(), message.get("modify_capture_state").getAsBoolean(), message.get("captured").getAsBoolean(), message.get("suppression").getAsBoolean());
			} else if (desiredTargetType.equals("PERI")) {
				((PeriodicTarget)target).setTargetConfigs(message.get("detection_range").getAsDouble(), 
						message.get("required_payloads").getAsInt(), message.get("capture_countdown").getAsDouble(), message.get("modify_capture_state").getAsBoolean(),
						message.get("captured").getAsBoolean(), message.get("suppression").getAsBoolean());
			}
			
			// Publish the target configurations
			publishTargetConfigs(uid, topic, qos);
			

		}

		private void handleLinkConfigs(HashMap<String, JsonElement> message, String topic, QualityOfService qos) {
					
			/*
			 * Update the state of the linked networks that are being persisted.
			 * This updates the networks, creates new networks, and deletes the networks.
			 */

			 String networkUid = message.get("uid").getAsString();
				
			synchronized (this) {
				// Determine whether to delete the network or not
				// If we are deleting the network, delete the network and return; no new network should be created
				if (message.get("delete").getAsBoolean()) {
					// Remove the network
					networks.remove(networkUid);
					
					// Exit
					return;
				}
				
				// Change the name of the existing network, if necessary
				if (message.get("change_uid").getAsBoolean()) {
					
					// Get name of the network that is being renamed
					String oldName = message.get("old_uid").getAsString();
					
					// Make sure that the old name is in the set of networks
					if (networks.containsKey(oldName)) {
						
						// Get the old network to rename
						LinkedNetwork networkToRename = networks.remove(oldName);
						
						// Make sure that the network associated with the name is not null
						if (networkToRename.equals(null)) {
							logger.error("The network associated with " + oldName + " is null. The network will not be renamed, and a new network will be created instead.");
						} else {
							// Replace the network 
							networks.put(networkUid, networkToRename);
						}
					} else {
						logger.error("There is no such network associated with the name " + oldName + ". A new network will be created instead.");
					}
				}
				
				/*
				*  The network does not yet exist; create a new network.
				*  Note that the change name block above will now include the new identifier in the network
				*  So this block will be skipped and the system will proceed to update the network with any 
				*  other configuration changes
				*/
				if (!networks.containsKey(networkUid)) {
					// Create a new network
					LinkedNetwork newNetwork = new LinkedNetwork(networkUid);
					
					// Add the network to the set of networks
					networks.put(networkUid, newNetwork);
				}
				
				// Get the network as a JSON array
				JsonArray networkTargets = message.get("network").getAsJsonArray();
				
				// Clear the existing network to account for removed targets
				networks.get(networkUid).clearNetwork();
				
				// Iterate through all of the targets and add them to the network
				for (var networkTarget : networkTargets) {
					
					// Get the UID of the target and the Target object
					String currentTargetUid = networkTarget.getAsString();
					Target currentTarget    = hardware.get(currentTargetUid);
					
					// Make sure that the hardware map contains the network
					if (hardware.containsKey(currentTargetUid)) {
						
						// Check if the target is already a linked target
						if (currentTarget.getTargetType().equals("LINK")) {
							// Add the target to the network
							networks.get(networkUid).addTarget((LinkedTarget)currentTarget);
							
						// The target is a different type of target
						} else {
							try {
								// Create a new linked target
								LinkedTarget targetToAdd = new LinkedTarget(currentTargetUid, host, networkTargets, networkUid, client, currentTarget.getCallsign());
								targetToAdd.setIpAddress(currentTarget.getIpAddress());
								
								// Replace the existing device as a linked target
								hardware.put(currentTargetUid, targetToAdd);
								
								// Add the target to the network
								networks.get(networkUid).addTarget(targetToAdd);
							} catch (MqttException e) {
								logger.error("An error occurred while attempting to replace a target (" + networkUid + "). Skipping the target: ", e);
							}		
						}
					} else {
						logger.error("The command station attempted to insert a target into the network that does not exist. Skipping this target.");
					}
				}
				
				// Update the configurations of the network
				networks.get(networkUid).setNetworkConfigurations(message.get("detection_range").getAsDouble(), message.get("modify_capture_state").getAsBoolean(), 
						message.get("captured").getAsBoolean(), networkTargets, networkUid, message.get("suppression").getAsBoolean());
				
				// Publish the network configurations
				publishNetworkConfigs(networkUid, topic, qos);
				networkTopic = topic;
				networkQos = qos;
			}	
			
		}
		
		
		/*
		 * Sub-class method
		 * Parse a state message provided by the hardware target
		 */
		private void parseState(HashMap<String, JsonElement> message, String configTopic, String whiteForceTopic, String blueForceTopic, 
				QualityOfService configQos, QualityOfService stateQos) {
			
			// Get the UID of the target who's message is being handled
			String uid = message.get("uid").getAsString();

			String callsign = message.get("callsign").getAsString();

			String ipAddress = message.get("ipAddress").getAsString();


			// Flag used to denote whether the 
			boolean publishToBothCommandStations = false;
			
			// Check if the UID already exists in the map of targets
			// If not, create a new default idle target and store it
			if (!hardware.containsKey(uid)) {
				try {
					// Create a new default idle target
					IdleTarget newTarget = new IdleTarget(uid, host, client, callsign);
					newTarget.setIpAddress(ipAddress);
					
					// Add the default target to the map of targets
					hardware.put(uid, newTarget);
				} catch (MqttException e) {
					logger.error("An error occurred while attempting to generate a new target: ", e);
					
					// An error occurred, exit and try again on the next message published by the target
					return;
				}
			}

			Target target = hardware.get(uid);
				
			// Get the hardware's current configuration type
			// Note that that targets are initialized as idle targets by default
			String targetType = message.get("type").getAsString();
			
			// Determine if there is a discrepancy between the type stored internally and the hardware type
			if (!targetType.equals(target.getTargetType())) {
				logger.info("There is a type mismatch between the target type stored on the analytics server (" + target.getTargetType()
						+ ") and the target type stored on the associated hardware device (" + targetType + ").");
				logger.info("Attempting to resend the target configurations stored locally to the target hardware device");
				
				// Publish the stored configurations
				publishTargetConfigs(uid, configTopic, configQos);
				
				return;
			}
			
			// Read the GPS location
			GpsLocation location = new GpsLocation(message.get("latitude").getAsDouble(), message.get("longitude").getAsDouble(), 
					message.get("altitude").getAsDouble());

			// Flag denoting whether the target has been captured
			boolean captured = false;
			
			// Update the target state according to the provided state update
			if (targetType.equals("IDLE")) {
				// Update the idle target state
				((IdleTarget)target).setTargetState(location);
			
			} else if (targetType.equals("MASS")) {
				captured = message.get("captured").getAsBoolean();
				publishToBothCommandStations = captured;
				
				// Update the mass target state
				((MassTarget)target).setTargetState(location, message.get("payloads").getAsInt(), captured);
			
			} else if (targetType.equals("PERI")) {
				captured = message.get("captured").getAsBoolean();
				publishToBothCommandStations = captured;
				
				// Update the periodic target state
				((PeriodicTarget)target).setTargetState(location, message.get("payloads").getAsInt(), 
						message.get("current_duration").getAsDouble(), captured);
			
			} else if (targetType.equals("LINK")) {
				captured = message.get("captured").getAsBoolean();
				publishToBothCommandStations = captured;
				
				// Update the linked target
				((LinkedTarget)target).setTargetState(location, message.get("payloads").getAsInt(), captured);
				
				// Update the capture status of the respective network
				for (LinkedNetwork network : networks.values()) {
					if (network.containsTarget(uid)) {
						network.determineNetworkCaptureStatus();
						publishNetworkConfigs(network.getName(), networkTopic, networkQos);
					}
				}
			}
			
			// Publish the target state to the command station(s)
			var discovered = Payload.targetDiscovered(uid);
			// if (publishToBothCommandStations || discovered) {
			// 	// Publish to white force and blue force stations
			// 	publishTargetState(uid, blueForceTopic, stateQos, discovered);
			// }

			// For capstone 2022, public target state to blue force as well. Hide target parameters on stomp.
			publishTargetState(uid, blueForceTopic, stateQos, discovered);
			
			// Publish to the white force station
			publishTargetState(uid, whiteForceTopic, stateQos, discovered);

			//
			publishTargetStateCot(uid, Server.getCotDestinations(), discovered);
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
			for (Map.Entry<String, JsonElement> entry : entries)
		    {
		        jsonMap.put(entry.getKey(), entry.getValue());
		    }
			
			// Determine if the message is a configuration message from the command station
			// or a state message from a target
			if (this.getInputTopic().contains("configurations")) {
				this.parseConfigs(jsonMap, this.getConfigTopic(), this.getConfigQos());
			} else if (this.getInputTopic().contains("state")) {
				this.parseState(jsonMap, this.getConfigTopic(), this.getWhiteForceStateTopic(), this.getBlueForceStateTopic(), this.getConfigQos(), this.getStateQos());
			}
		}
	}

	/*
	 * Callback function used to receive a message and distribute the message to a new thread handler
	 */
	@Override
	public void messageArrived(String topic, MqttMessage message) throws Exception {
		this.pool.execute(new Handler(topic, message, this.getOutputConfigTopic(), this.getOutputWhiteForceStateTopic(), 
				this.getOutputBlueForceStateTopic(), this.getOutputConfigQos(), this.getOutputStateQos()));
	}

}
