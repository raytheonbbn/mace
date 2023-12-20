//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import com.bbn.mace.target.LinkedTarget;
import com.google.gson.JsonArray;
import com.bbn.mace.target.LinkedNetwork;

import static org.junit.jupiter.api.Assertions.*;


public class TestLinkedNetwork {
	private static LinkedNetwork network;
	private static final String host = "tcp://test.mosquitto.org:1883";
	
	@BeforeAll
	static void setUp() throws InterruptedException {
		network = new LinkedNetwork();
	}
	
	
	@Test
	@DisplayName("Setting Network Configurations Test")
	void testSetNetworkConfigurations() throws InterruptedException {
		// Create new targets to add to the network
		try {
			// Clear the network prior to tests
			network.clearNetwork();
			
			// Add new targets
			LinkedTarget target1 = new LinkedTarget("TARGET-1", host);
			LinkedTarget target2 = new LinkedTarget("TARGET-2", host);
			LinkedTarget target3 = new LinkedTarget("TARGET-3", host);
			
			// Add the targets to the network
			network.addTarget(target1);
			network.addTarget(target2);
			network.addTarget(target3);
			
			// Configs to set
			double detectionRange = 0.4;
			boolean modifyCaptured = true;
			boolean captured = false;
			
			// Create a new targets array that represent a network
			JsonArray targets = new JsonArray();
			
			// Add the targets to the array
			targets.add(target1.getUid());
			targets.add(target2.getUid());
			targets.add(target3.getUid());
			
			// Set the network configurations
			network.setNetworkConfigurations(detectionRange, modifyCaptured, captured, targets, "test-network");
			
			// Ensure that all targets have the correct configurations
			for (LinkedTarget target : network.getNetworkTargets()) {
				assertEquals(target.getAcceptableDetectionRange(), detectionRange);
				assertEquals(target.getNetworkCaptured(), captured);
				assertEquals(target.getNetworkTargets(), targets);
			}			
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to generate new targets");
			e.printStackTrace();
		}
	}
	
	
	@Test
	@DisplayName("Determine and Update the Network Capture Status, Captured Test")
	void testDetermineNetworkCaptureStatusCaptured() throws InterruptedException {
		// Create new targets to add to the network
		try {
			// Clear the network prior to testing
			network.clearNetwork();
			
			// Targets
			LinkedTarget target1 = new LinkedTarget("TARGET-1", host);
			LinkedTarget target2 = new LinkedTarget("TARGET-2", host);
			LinkedTarget target3 = new LinkedTarget("TARGET-3", host);
			
			// Add the targets to the network
			network.addTarget(target1);
			network.addTarget(target2);
			network.addTarget(target3);
			
			boolean networkCaptured = true;
			
			// Set all of the targets to captured
			target1.setTargetState(null, 0, true);
			target2.setTargetState(null, 0, true);
			target3.setTargetState(null, 0, true);
			
			// Update that capture status
			network.determineNetworkCaptureStatus();
			
			// Ensure that all targets have been properly updated
			for (LinkedTarget target : network.getNetworkTargets()) {
				assertEquals(target.getNetworkCaptured(), networkCaptured);
			}
			
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to generate new targets");
			e.printStackTrace();
		}
	}
	
	
	@Test
	@DisplayName("Determine and Update the Network Capture Status, Uncaptured Test")
	void testDetermineNetworkCaptureStatusUncaptured() throws InterruptedException {
		// Create new targets to add to the network
		try {
			network.clearNetwork();
			
			// Targets
			LinkedTarget target1 = new LinkedTarget("TARGET-1", host);
			LinkedTarget target2 = new LinkedTarget("TARGET-2", host);
			LinkedTarget target3 = new LinkedTarget("TARGET-3", host);
			
			// Add the targets to the network
			network.addTarget(target1);
			network.addTarget(target2);
			network.addTarget(target3);
			
			boolean networkCaptured = false;
			
			// Set all of the targets to captured
			target1.setTargetState(null, 0, true);
			target2.setTargetState(null, 0, false);
			target3.setTargetState(null, 0, true);
			
			// Update the capture status
			network.determineNetworkCaptureStatus();
			
			// Ensure that all targets were properly updated
			for (LinkedTarget target : network.getNetworkTargets()) {
				assertEquals(target.getNetworkCaptured(), networkCaptured);
			}
			
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to generate new targets");
			e.printStackTrace();
		}
	}
}
