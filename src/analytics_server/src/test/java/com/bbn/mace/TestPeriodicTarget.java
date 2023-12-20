//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.bbn.mace.target.PeriodicTarget;
import com.bbn.mace.utils.GpsLocation;
import org.junit.jupiter.api.DisplayName;
import static org.junit.jupiter.api.Assertions.*;


public class TestPeriodicTarget {
	private static PeriodicTarget target;
	private static final String uid = "TARGET-123456";
	private static final String host = "tcp://test.mosquitto.org:1883";
	
	@BeforeAll
	static void setUp() throws InterruptedException {
		try {
			target = new PeriodicTarget(uid, host);
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to generate a new target: ");
			e.printStackTrace();
		}
	}
	
	
	@Test
	@DisplayName("Setting Mass Target State Test")
	void testSetTargetState() throws InterruptedException {
		// Test parameters
		double latitude = 65.4;
		double longitude = 55.4;
		double altitude = 10.56;
		int payloadsInRange = 4;
		boolean captured = false;
		double duration = 65.0;
		
		// Create a new test location
		GpsLocation testLocation = new GpsLocation(latitude, longitude, altitude);
		
		// Set the state
		target.setTargetState(testLocation, payloadsInRange, duration, captured);

		// Assert that the target properties were set properly
		assertEquals(target.getLocation().getLatitude(), testLocation.getLatitude());
		assertEquals(target.getLocation().getLongitude(), testLocation.getLongitude());
		assertEquals(target.getLocation().getAltitude(), testLocation.getAltitude());
		assertEquals(target.getPayloadsInRange(), payloadsInRange);
		assertEquals(target.getCaptured(), captured);
		assertEquals(target.getCurrentCaptureDuration(), duration);
	}
	
	
	@Test
	@DisplayName("Setting Periodic Target Configurations Test")
	void testSetTargetConfigs() throws InterruptedException {
		// Test configuration values
		double range = 1.5;
		int requiredPayloads = 5;
		boolean setCaptured = true;
		boolean captured = true; // this should not be set to true if working properly
		double countdown = 70.44;
		
		// Set the test configurations
		target.setTargetConfigs(range, requiredPayloads, countdown, setCaptured, captured, false);
		
		assertEquals(target.getRequiredPayloadsForCapture(), requiredPayloads);
		assertEquals(target.getAcceptableDetectionRange(), range);
		assertEquals(target.getCaptured(), captured);
	}	
}
