//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.bbn.mace.target.LinkedTarget;
import com.bbn.mace.utils.GpsLocation;
import com.google.gson.JsonArray;

import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;


public class TestLinkedTarget {
	private static LinkedTarget target;
	private static final String uid = "TARGET-123456";
	private static final String host = "tcp://test.mosquitto.org:1883";
	
	@BeforeAll
	static void setUp() throws InterruptedException {
		try {
			target = new LinkedTarget(uid, host);
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to generate a new target: ");
			e.printStackTrace();
		}
	}
	
	
	@Test
	@DisplayName("Setting Linked Target State Test")
	void testSetTargetState() throws InterruptedException {
		// Test location
		double latitude = 65.4;
		double longitude = 55.4;
		double altitude = 10.56;
		
		// Create a new test location
		GpsLocation testLocation = new GpsLocation(latitude, longitude, altitude);
		
		// Test payloads
		int payloadsInRange = 4;
		
		// Test captured flag
		boolean captured = false;
		
		// Set the state
		target.setTargetState(testLocation, payloadsInRange, captured);

		// Assert that the target properties were set properly
		assertEquals(target.getLocation().getLatitude(), testLocation.getLatitude());
		assertEquals(target.getLocation().getLongitude(), testLocation.getLongitude());
		assertEquals(target.getLocation().getAltitude(), testLocation.getAltitude());
		assertEquals(target.getPayloadsInRange(), payloadsInRange);
		assertEquals(target.getIndividualCaptured(), captured);
	}
	
	
	@Test
	@DisplayName("Setting Mass Target Configurations Test")
	void testSetTargetConfigsWithCapture() throws InterruptedException {
		// Test configuration values
		double range = 1.5;
		boolean setCaptured = true;
		boolean captured = true; // this should be set to true if working properly
		JsonArray targets = new JsonArray();
		
		targets.add(target.getUid());
		
		// Set the test configurations
		target.setTargetConfigs(range, setCaptured, captured, targets, "test-network");
		
		// Assert that the configurations were set properly
		assertEquals(target.getAcceptableDetectionRange(), range);
		assertEquals(target.getNetworkCaptured(), captured);
		assertEquals(target.getNetworkTargets(), targets);
	}	
}
