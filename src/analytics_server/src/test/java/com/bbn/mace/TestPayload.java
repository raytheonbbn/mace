//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace;

import com.bbn.mace.payload.PayloadIntent;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import com.bbn.mace.payload.Payload;
import com.bbn.mace.utils.GpsLocation;
import org.junit.jupiter.api.DisplayName;

import java.util.ArrayList;
import java.util.Collection;

import static org.junit.jupiter.api.Assertions.*;


public class TestPayload {
	private static Payload payload;
	private static final String uid = "PAYLOAD-123456";
	private static final String host = "tcp://test.mosquitto.org:1883";
	
	@BeforeAll
	static void setUp() throws InterruptedException {
		try {
			payload = new Payload(uid, host);
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to generate a new target: ");
			e.printStackTrace();
		}
	}
	
	
	@Test
	@DisplayName("Setting Payload State Test")
	void testSetPayloadState() throws InterruptedException {
		// Test Parameters
		double latitude = 65.4;
		double longitude = 55.4;
		double altitude = 10.56;
		boolean inRange = false;
		boolean modifyIntent = false;
		Collection<PayloadIntent> intent = new ArrayList<PayloadIntent>();
		intent.add(PayloadIntent.IDLE);
		JsonArray gsonIntent = new Gson().toJsonTree(intent).getAsJsonArray();
		String typeInRange = "IDLE";
		
		// Create a new test location
		GpsLocation testLocation = new GpsLocation(latitude, longitude, altitude);
		
		payload.setPayloadState(testLocation, inRange, typeInRange, modifyIntent, gsonIntent, true, 100);
		
		// Assert that the target properties were set properly
		assertEquals(payload.getLocation().getLatitude(), testLocation.getLatitude());
		assertEquals(payload.getLocation().getLongitude(), testLocation.getLongitude());
		assertEquals(payload.getLocation().getAltitude(), testLocation.getAltitude());
		assertEquals(payload.getInRange(), inRange);
		assertEquals(payload.getTypeInRange(), typeInRange);
		assertNotEquals(payload.getIntent(), intent);
	}
	
	
	@Test
	@DisplayName("Setting Valid Payload Configurations (Intent) Test")
	void testSetPayloadIntent() throws InterruptedException {
		Collection<PayloadIntent> intent = new ArrayList<PayloadIntent>();
		intent.add(PayloadIntent.MASS);
		
		payload.setIntent(intent);
		
		assertEquals(payload.getIntent(), intent);
	}
}
