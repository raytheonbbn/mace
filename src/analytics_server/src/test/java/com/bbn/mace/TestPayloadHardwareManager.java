//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace;

import com.bbn.mace.utils.MaceMessageTransport;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttAsyncClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import com.bbn.mace.callback.PayloadCallback;
import com.bbn.mace.manager.HardwareManager;
import com.bbn.mace.payload.Payload;
import com.bbn.mace.utils.QualityOfService;
import com.google.gson.JsonObject;


public class TestPayloadHardwareManager {
	
	// MQTT test client
	private static MqttAsyncClient testClient;
	
	// Payload Manager
	private static HardwareManager manager;
	private static PayloadCallback callback;
	
	// Payload Manager parameters
	private static final String host = "tcp://127.0.0.1:1883";
	private static final String configTopic = "payload/configurations";
	private static final String stateTopic = "payload/state";
	private static final String inputTopics[] = {configTopic, stateTopic};
	private static final int inputQos[] = {QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue(), QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue()};
	private static final String outputConfigTopic = "command_station/payload/configurations";
	private static final String outputStateTopic = "hardware/payload/state";
	private static final QualityOfService configQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
	private static final QualityOfService stateQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
	private static MaceMessageTransport client;


	@BeforeAll
	static void setUp() throws InterruptedException {
		try {
			client = new MaceMessageTransport(host, "PayloadTest");


			// Create a new hardware manager
			manager = new HardwareManager(client);


			// Create a new callback for the messages
			callback = new PayloadCallback(host, outputConfigTopic, configQos, outputStateTopic, stateQos, client);

			// Subscribe to the desired topics and set the callback for the messages
			manager.subscribe(inputTopics, inputQos, callback);
			
			// Create a new test client to publish the data
			testClient = new MqttAsyncClient(host, MqttAsyncClient.generateClientId());
			
			// Attempt to connect to the broker
			IMqttToken token = testClient.connect();
			
			// Wait for the client to connect
			token.waitForCompletion();
		} catch (MqttException e) {
			System.out.println("Unable to generate a new payload manager");
			e.printStackTrace();
		}
	}
	
	
	@AfterAll
	static void tearDown() {
		try {
			testClient.disconnect();
			testClient.close();
		} catch (MqttException e) {
			System.out.println("Unable to shut down the test MQTT client");
			e.printStackTrace();
		}
	}
	
	
	/*
	 * Helper method used to send a json object message
	 */
	static void sendJsonMessage(JsonObject json, String topic) throws InterruptedException {
		
		// Convert the message to a string
		String stringMessage = json.toString();
		
		// Create a new MQTT message
		MqttMessage message = new MqttMessage();
		
		// Set the MQTT message target to the JSON object
		message.setPayload(stringMessage.getBytes());
		
		// Set the quality of service level for the message
		message.setQos(QualityOfService.RECEIVE_EXACTLY_ONCE.getValue());
		
		// Publish the message
		try {
			// Send only one message
			IMqttDeliveryToken token = testClient.publish(topic, message);

			token.waitForCompletion();
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to publish a test payload state message");
		}
		
		// Wait for the message to be processed by the client
        Thread.sleep(3000);
	}
	
	
	@Test
	@DisplayName("Parse a Single Payload State Test")
	void testParseOneState() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		String testUid = "PAYLOAD-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		boolean testModifyIntent = false;
		String testIntent = "IDLE";
		boolean testInRange = false;
		String testTypeInRange = "IDLE";
		
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", testUid);
		jsonMessage.addProperty("latitude", testLatitude);
		jsonMessage.addProperty("longitude", testLongitude);
		jsonMessage.addProperty("altitude", testAltitude);
		jsonMessage.addProperty("modify_intent", testModifyIntent);
		jsonMessage.addProperty("intent", testIntent);
		jsonMessage.addProperty("in_range", testInRange);
		jsonMessage.addProperty("type_in_range", testTypeInRange);
		
		sendJsonMessage(jsonMessage, stateTopic);

		// Ensure that only one payload exists in the set
		assertEquals(callback.hardware.size(), 1);
		
		// Get the payload sent
		Payload payload = callback.hardware.get(testUid);
		
		// Assert that all of the data was transmitted properly
    	assertEquals(payload.getLocation().getLatitude(), testLatitude);
    	assertEquals(payload.getLocation().getLongitude(), testLongitude);
    	assertEquals(payload.getLocation().getAltitude(), testAltitude);
    	assertEquals(payload.getIntent(), testIntent);
    	assertEquals(payload.getInRange(), testInRange);
    	assertEquals(payload.getTypeInRange(), testTypeInRange);    	
	}
	
	
	@Test
	@DisplayName("Prevent Duplicate Payloads Test")
	void testParseStateNoDuplicates() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		String testUid = "PAYLOAD-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		boolean testModifyIntent = false;
		String testIntent = "IDLE";
		boolean testInRange = false;
		String testTypeInRange = "IDLE";
		
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", testUid);
		jsonMessage.addProperty("latitude", testLatitude);
		jsonMessage.addProperty("longitude", testLongitude);
		jsonMessage.addProperty("altitude", testAltitude);
		jsonMessage.addProperty("modify_intent", testModifyIntent);
		jsonMessage.addProperty("intent", testIntent);
		jsonMessage.addProperty("in_range", testInRange);
		jsonMessage.addProperty("type_in_range", testTypeInRange);
		
		// Send the same message twice
		sendJsonMessage(jsonMessage, stateTopic);
		sendJsonMessage(jsonMessage, stateTopic);

		// Ensure that only one payload exists in the set
		assertEquals(callback.hardware.size(), 1);
	}
	
	
	@Test
	@DisplayName("Parse Multiple Payload States Test")
	void testParseMultipleStates() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		// Test data
		String testUid1 = "PAYLOAD-1";
		String testUid2 = "PAYLOAD-2";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		boolean testModifyIntent = false;
		String testIntent = "IDLE";
		boolean testInRange = false;
		String testTypeInRange = "IDLE";
		
		// Create new JSON objects to send the message contents within
		JsonObject jsonMessage1 = new JsonObject();
		JsonObject jsonMessage2 = new JsonObject();
		
		// Populate the first message
		jsonMessage1.addProperty("uid", testUid1);
		jsonMessage1.addProperty("latitude", testLatitude);
		jsonMessage1.addProperty("longitude", testLongitude);
		jsonMessage1.addProperty("altitude", testAltitude);
		jsonMessage1.addProperty("modify_intent", testModifyIntent);
		jsonMessage1.addProperty("intent", testIntent);
		jsonMessage1.addProperty("in_range", testInRange);
		jsonMessage1.addProperty("type_in_range", testTypeInRange);
		
		// Populate the second message
		jsonMessage2.addProperty("uid", testUid2);
		jsonMessage2.addProperty("latitude", testLatitude);
		jsonMessage2.addProperty("longitude", testLongitude);
		jsonMessage2.addProperty("altitude", testAltitude);
		jsonMessage2.addProperty("modify_intent", testModifyIntent);
		jsonMessage2.addProperty("intent", testIntent);
		jsonMessage2.addProperty("in_range", testInRange);
		jsonMessage2.addProperty("type_in_range", testTypeInRange);
		
		sendJsonMessage(jsonMessage1, stateTopic);
		sendJsonMessage(jsonMessage2, stateTopic);

		// Ensure that only both payloads exist in the set
		assertEquals(callback.hardware.size(), 2);
		
		// Ensure that both payloads were generated
		assertTrue(callback.hardware.containsKey(testUid1));
		assertTrue(callback.hardware.containsKey(testUid2));
		
		for (Payload payload : callback.hardware.values()) {
			// Assert that all of the data was transmitted properly
	    	assertEquals(payload.getLocation().getLatitude(), testLatitude);
	    	assertEquals(payload.getLocation().getLongitude(), testLongitude);
	    	assertEquals(payload.getLocation().getAltitude(), testAltitude);
	    	assertEquals(payload.getIntent(), testIntent);
	    	assertEquals(payload.getInRange(), testInRange);
	    	assertEquals(payload.getTypeInRange(), testTypeInRange);
		}
	}
	
	
	@Test
	@DisplayName("Parse Payload Configurations Test")
	void testParseConfigs() throws InterruptedException {
		
		// Clear the data
		callback.hardware.clear();
		
		/*
		 * It is first necessary to send a payload state package to generate 
		 * a new payload in the set of payloads
		 */
		
		// Payload parameters to generate
		String testUid = "PAYLOAD-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		boolean testModifyIntent = false;
		String testIntent = "IDLE";
		boolean testInRange = false;
		String testTypeInRange = "IDLE";
		
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", testUid);
		jsonMessage.addProperty("latitude", testLatitude);
		jsonMessage.addProperty("longitude", testLongitude);
		jsonMessage.addProperty("altitude", testAltitude);
		jsonMessage.addProperty("modify_intent", testModifyIntent);
		jsonMessage.addProperty("intent", testIntent);
		jsonMessage.addProperty("in_range", testInRange);
		jsonMessage.addProperty("type_in_range", testTypeInRange);
		
		sendJsonMessage(jsonMessage, stateTopic);
        
		// Ensure that only one payload exists in the set
		assertEquals(1, callback.hardware.size());
		
		// Get the payload sent
		Payload payload = callback.hardware.get(testUid);
		
		// Assert that all of the data was transmitted properly
    	assertEquals(payload.getLocation().getLatitude(), testLatitude);
    	assertEquals(payload.getLocation().getLongitude(), testLongitude);
    	assertEquals(payload.getLocation().getAltitude(), testAltitude);
    	assertEquals(payload.getIntent(), testIntent);
    	assertEquals(payload.getInRange(), testInRange);
    	assertEquals(payload.getTypeInRange(), testTypeInRange);
    	
    	/*
    	 * Now that a payload has been generated, reset its configurations
    	 */
    	
    	String configIntent = "MASS";

		// Create a new JSON object to send the message contents within
		JsonObject jsonConfigMessage = new JsonObject();
		
		jsonConfigMessage.addProperty("uid", testUid);
		jsonConfigMessage.addProperty("intent", configIntent);
		
		sendJsonMessage(jsonConfigMessage, configTopic);
        
        // Ensure that the configuration was properly set
        assertEquals(callback.hardware.get(testUid).getIntent(), configIntent);		
	}
}
