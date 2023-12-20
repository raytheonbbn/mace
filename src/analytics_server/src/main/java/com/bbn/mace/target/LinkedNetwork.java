//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.target;

import java.util.Collection;
import java.util.HashMap;

import com.bbn.mace.payload.Payload;
import com.bbn.mace.utils.QualityOfService;
import com.google.gson.JsonArray;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;


/*
 * Name: LinkedNetwork
 * Description: Class used to represent a Linked target network and its respective behavior
 */
public class LinkedNetwork {
	
	// Class Variables
    private HashMap<String, LinkedTarget> network;	// The set of linked targets in the network
    private boolean networkCaptured;				// The capture state of the network
	private String name;
    protected final Logger logger;					// Logger used to log class events and exceptions
    
    
	public LinkedNetwork(String name) {
		// Initialize the class variables
		this.network = new HashMap<String, LinkedTarget>();
		this.networkCaptured = false;
		this.name = name;
		
		// Initialize the class logger
		this.logger = LogManager.getLogger(Target.class.getName());
		
		this.logger.info("Successfully created a new linked target network");
		
	}
	
	/*
	 * Shut down all targets in the network
	 */
	public void shutdown() {
		this.logger.info("Shutting down all targets in the linked target network");
		
		for (LinkedTarget target: this.network.values()) {
			target.shutdown();
		}
	}

	public String getName() {
		return name;
	}
	
	/*
	 * Get the set of targets managed by the network
	 */
	public Collection<LinkedTarget> getNetworkTargets() {
		return this.network.values();
	}
	
	/*
	 * Setter used to set the network capture state
	 */
	private void setNetworkCaptured(boolean captured) {
		this.networkCaptured = captured;
	}
	
	/*
	 * Getter used to retrieve the network capture state
	 */
	public boolean getNetworkCaptured() {
		return this.networkCaptured;
	}
	
	/*
	 * Add a linked target to the network by passing a new UID
	 */
	public void addTarget(LinkedTarget target) {
		var oldTarget = this.network.put(target.uid, target);
		if (oldTarget != null) {
			logger.warn(String.format("old target %s being replaced", target.uid));
		}
	}
	
	/*
	 * Determine whether the network contains the target
	 */
	public boolean containsTarget(String targetUid) {
		return this.network.containsKey(targetUid);
	}
	
	/*
	 * Clear the linked target network
	 */
	public void clearNetwork() {
		this.network.clear();
	}
	
	/*
	 * Remove a target from the network
	 */
	public void removeTarget(String targetUid) {
		this.network.remove(targetUid);
	}
	
	/*
	 * Setter used by the command station to set the configurations of the targets within the network
	 * This method is used to by the command station to set the configurations of all targets in the network.
	 * Note that the modifyCaptured flag is included to require specific intent by the command station to update the
	 * network capture state.
	 */
	public void setNetworkConfigurations(double detectionRange, boolean modifyCaptured, boolean captured, JsonArray targets, String networkName, boolean suppression) {
		// Update each target in the network
		for(LinkedTarget target : this.network.values()) {
			// Update the configurations
		    target.setTargetConfigs(detectionRange, modifyCaptured, captured, targets, networkName, suppression);
		}
	}
	
	/*
	 * Private method used to set the network capture flag of all targets in the network
	 */
	private void setTargetNetworkCaptureStatus(boolean captured) {
		// Update each of the individual targets
		for(LinkedTarget target : this.network.values()) {
			// Set the target's network captured flag
			target.setNetworkCaptured(name, captured);
		}
	}
	
	/*
	 * Determine whether the network has been captured yet, and update the network captured flag accordingly
	 * This method is called after receiving an individual target update to determine whether the network
	 * is now captured or not and to update the network and its respective targets accordingly
	 */
	public void determineNetworkCaptureStatus() {
		// Determine if the network has been captured
		for(LinkedTarget target: this.network.values()) {
			// Determine if a target in the network has not been captured yet
			if (!target.getIndividualCaptured()) {
				// The network has not been captured
				this.setNetworkCaptured(false);
				// Set the network capture state flag for all targets to false
				this.setTargetNetworkCaptureStatus(this.getNetworkCaptured());

				logger.debug(String.format("target %s is not captured so network %s is not captured", target.uid, name));
				// Exit
				return;
			}
		}
		
		// The network has been captured
		this.setNetworkCaptured(true);
		// Set the network captured state flag to true for all targets in the network
		this.setTargetNetworkCaptureStatus(this.getNetworkCaptured());		
		logger.debug("all targets captured for network");
	}
	
	/*
	 * Publish each of the target's configurations to its associated hardware
	 * This method is used by the command stations to set the configurations of the network on the hardware
	 */
	public void publishNetworkConfigs(String topic, QualityOfService serviceLevel) {
		var targets = this.network.values().toArray(new LinkedTarget[this.network.size()]);
		// Each target in the network will publish to its respective hardware
		for(LinkedTarget target : targets) {
			// publish the configurations using the async client (this will be non-blocking)
			target.publishTargetConfigs(topic, serviceLevel);
		}
	}
	
	/*
	 * Publish the network state to each of the targets in the network
	 * This method is used to update the command stations on the current state of the network
	 */
	public void publishNetworkState(String topic, QualityOfService serviceLevel) {
		// Each target in the network will publish to its respective hardware
		for(LinkedTarget target : this.network.values()) {
			// publish the state using the async client (this will be non-blocking)
			target.publishTargetState(topic, serviceLevel, Payload.targetDiscovered(target.uid));
		}
	}
}
