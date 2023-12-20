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
import com.bbn.mace.callback.TargetCallback;
import com.bbn.mace.manager.HardwareManager;
import com.bbn.mace.target.IdleTarget;
import com.bbn.mace.target.LinkedTarget;
import com.bbn.mace.target.LinkedNetwork;
import com.bbn.mace.target.MassTarget;
import com.bbn.mace.target.PeriodicTarget;
import com.bbn.mace.utils.QualityOfService;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;


public class TestTargetHardwareManager {
	
	// MQTT test client
	private static MqttAsyncClient testClient;
	
	// Payload Manager
	private static HardwareManager manager;
	private static TargetCallback callback;
	
	// Payload Manager parameters
	private static final String host = "tcp://127.0.0.1:1883";
	private static final String configTopic = "target/configurations";
	private static final String stateTopic = "target/state";
	private static final String inputTopics[] = {configTopic, stateTopic};
	private static final int inputQos[] = {QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue(), QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue()};
	private static final String outputConfigTopic = "command_station/target/configurations";
	private static final String outputWhiteForceStateTopic = "hardware/white_force/target/state";
	private static final String outputBlueForceStateTopic = "hardware/blue_force/target/state";
	private static final QualityOfService configQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
	private static final QualityOfService stateQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
	private static MaceMessageTransport client;


	@BeforeAll
	static void setUp() throws InterruptedException {
		try {
			client = new MaceMessageTransport(host, "TestTarget", "white_force", "maceRef!");

			// Create a new hardware manager
			manager = new HardwareManager(client);
			
			// Create a new callback for the messages
			callback = new TargetCallback(host, outputConfigTopic, configQos, outputWhiteForceStateTopic, outputBlueForceStateTopic, stateQos, client);
			
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
	 * Helper method used to create a default idle target
	 */
	static void createDefaultTarget(String uid, double latitude, double longitude, double altitude) throws InterruptedException {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();
		
		// Populate the message
		jsonMessage.addProperty("uid", uid);
		jsonMessage.addProperty("type", "IDLE");
		jsonMessage.addProperty("latitude", latitude);
		jsonMessage.addProperty("longitude", longitude);
		jsonMessage.addProperty("altitude", altitude);
		
		// Convert the message to a string
		String stringMessage = jsonMessage.toString();
		
		// Create a new MQTT message
		MqttMessage message = new MqttMessage();
		
		// Set the MQTT message target to the JSON object
		message.setPayload(stringMessage.getBytes());
		
		// Set the quality of service level for the message
		message.setQos(QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue());
		
		// Publish the message
		try {
			// Send only one message
			IMqttDeliveryToken token = testClient.publish(stateTopic, message);

			token.waitForCompletion();
		} catch (MqttException e) {
			System.out.println("An error occurred while attempting to publish a test payload state message");
		}
		
		// Wait for the message to be processed by the client
        Thread.sleep(3000);
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
	@DisplayName("Parse an Idle Target State Test")
	void testParseIdleState() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		// Register a new target 
		String testUid = "TARGET-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		// Create a default idle target
		createDefaultTarget(testUid, testLatitude, testLongitude, testAltitude);

		// Ensure that only one target exists in the set
		assertTrue(callback.hardware.containsKey(testUid));
		
		// Get the payload sent
		IdleTarget target = (IdleTarget)callback.hardware.get(testUid);
		
		// Assert that all of the data was transmitted properly
		assertEquals(target.getUid(), testUid);
    	assertEquals(target.getLocation().getLatitude(), testLatitude);
    	assertEquals(target.getLocation().getLongitude(), testLongitude);
    	assertEquals(target.getLocation().getAltitude(), testAltitude);
	}
	
	
	@Test
	@DisplayName("Parse a Mass Target Configs Test")
	void testParseMassConfigs() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		/*
		 * The first step is to register a new target that the configs can
		 * modify
		 */
		
		// Default values
		String testUid = "TARGET-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		createDefaultTarget(testUid, testLatitude, testLongitude, testAltitude);

		// Ensure that only one target exists in the set
		assertEquals(callback.hardware.size(), 1);
		
		/*
		 * Next Send the Mass target configurations to reset the stored target
		 */
		
		String modifiedType = "MASS";
		double detectionRange = 1.5;
		int requiredPayloads = 4;
		boolean modifyCaptureState = false;
		boolean captured = false;
	        
		// Create a new JSON object to send the message contents within
		JsonObject jsonConfigMessage = new JsonObject();
	    
		// Set the configurations
		jsonConfigMessage.addProperty("uid", testUid);
		jsonConfigMessage.addProperty("type", modifiedType);
		jsonConfigMessage.addProperty("detection_range", detectionRange);
		jsonConfigMessage.addProperty("required_payloads", requiredPayloads);
		jsonConfigMessage.addProperty("modify_capture_state", modifyCaptureState);
		jsonConfigMessage.addProperty("captured", captured);
	    
		// Send the message
		sendJsonMessage(jsonConfigMessage, configTopic);

		// Ensure that only one target exists in the set
		assertTrue(callback.hardware.containsKey(testUid));
		
        // Get the target
        MassTarget target = (MassTarget)callback.hardware.get(testUid);
        
        // Ensure that the configs were properly set
        assertEquals(target.getTargetType(), modifiedType);
        assertEquals(target.getAcceptableDetectionRange(), detectionRange);
        assertEquals(target.getRequiredPayloadsForCapture(), requiredPayloads);
	}
	
	
	@Test
	@DisplayName("Parse a Mass Target State Test")
	void testParseMassState() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		/*
		 * The first step is to register a new target that the configs can
		 * modify
		 */
		
		// Default values
		String testUid = "TARGET-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		createDefaultTarget(testUid, testLatitude, testLongitude, testAltitude);

		// Ensure that only one target exists in the set
		assertEquals(callback.hardware.size(), 1);
		
		/*
		 * Next Send the Mass target configurations to reset the stored target
		 */
		
		String modifiedType = "MASS";
		double detectionRange = 1.5;
		int requiredPayloads = 4;
		boolean modifyCaptureState = false;
		boolean captured = false;
	        
		// Create a new JSON object to send the message contents within
		JsonObject jsonConfigMessage = new JsonObject();
	    
		// Set the configurations
		jsonConfigMessage.addProperty("uid", testUid);
		jsonConfigMessage.addProperty("type", modifiedType);
		jsonConfigMessage.addProperty("detection_range", detectionRange);
		jsonConfigMessage.addProperty("required_payloads", requiredPayloads);
		jsonConfigMessage.addProperty("modify_capture_state", modifyCaptureState);
		jsonConfigMessage.addProperty("captured", captured);
	    
		// Send the message
		sendJsonMessage(jsonConfigMessage, configTopic);
		
		// Ensure that only one target exists in the set
		assertEquals(callback.hardware.size(), 1);
		
		/*
		 * Modify the new mass target state
		 */
		
        JsonObject jsonStateMessage = new JsonObject();
        
        // Modify the parameters
		testLatitude = 119.3;
		testLongitude = 54.6;
		testAltitude = 22.5;
		double testPayloads = 4;
		double testRequiredPayloads = 8;
		boolean testCaptured = true;
        
		// Populate the message
        jsonStateMessage.addProperty("uid", testUid);
        jsonStateMessage.addProperty("type", modifiedType);
        jsonStateMessage.addProperty("latitude", testLatitude);
        jsonStateMessage.addProperty("longitude", testLongitude);
        jsonStateMessage.addProperty("altitude", testAltitude);
        jsonStateMessage.addProperty("payloads", testPayloads);
        jsonStateMessage.addProperty("required_payloads", testRequiredPayloads);
        jsonStateMessage.addProperty("captured", testCaptured);
        
        // Send the new state message
        sendJsonMessage(jsonStateMessage, stateTopic);

		// Ensure that only one target exists in the set
		assertTrue(callback.hardware.containsKey(testUid));
		
        MassTarget target = (MassTarget)callback.hardware.get(testUid);
        
        assertEquals(testLatitude, target.getLocation().getLatitude());
        assertEquals(testLongitude, target.getLocation().getLongitude());
        assertEquals(testAltitude, target.getLocation().getAltitude());
        assertEquals(testPayloads, target.getPayloadsInRange());
        assertEquals(testCaptured, target.getCaptured());
	}
	
	
	@Test
	@DisplayName("Parse Periodic Target Configs Test")
	void testParsePeriodicConfigs() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		/*
		 * The first step is to register a new target that the configs can
		 * modify
		 */
		
		// Default values
		String testUid = "TARGET-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		createDefaultTarget(testUid, testLatitude, testLongitude, testAltitude);

		// Ensure that only one target exists in the set
		assertEquals(callback.hardware.size(), 1);
		
		/*
		 * Next Send the periodic target configurations to reset the stored target
		 */
		
		String modifiedType = "PERI";
		double detectionRange = 1.5;
		int requiredPayloads = 4;
		double captureCountdown = 55.43;
		boolean modifyCaptureState = false;
		boolean captured = false;
	        
		// Create a new JSON object to send the message contents within
		JsonObject jsonConfigMessage = new JsonObject();
	    
		// Set the configurations
		jsonConfigMessage.addProperty("uid", testUid);
		jsonConfigMessage.addProperty("type", modifiedType);
		jsonConfigMessage.addProperty("detection_range", detectionRange);
		jsonConfigMessage.addProperty("required_payloads", requiredPayloads);
		jsonConfigMessage.addProperty("capture_countdown", captureCountdown);
		jsonConfigMessage.addProperty("modify_capture_state", modifyCaptureState);
		jsonConfigMessage.addProperty("captured", captured);
	    
		// Send the message
		sendJsonMessage(jsonConfigMessage, configTopic);	
		
		// Ensure that only one target exists in the set
		assertTrue(callback.hardware.containsKey(testUid));
		
        // Get the target
        PeriodicTarget target = (PeriodicTarget)callback.hardware.get(testUid);
        
        // Ensure that the configs were properly set
        assertEquals(target.getTargetType(), modifiedType);
        assertEquals(target.getAcceptableDetectionRange(), detectionRange);
        assertEquals(target.getRequiredPayloadsForCapture(), requiredPayloads);
        assertEquals(target.getCountdownRequiredForCapture(), captureCountdown);
	}
	
	
	@Test
	@DisplayName("Parse a Periodic Target State Test")
	void testParsePeriodicState() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		
		/*
		 * The first step is to register a new target that the configs can modify
		 */
		
		// Default values
		String testUid = "TARGET-1";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		createDefaultTarget(testUid, testLatitude, testLongitude, testAltitude);

		// Ensure that only one target exists in the set
		assertTrue(callback.hardware.containsKey(testUid));
		
		/*
		 * Next Send the Mass target configurations to reset the stored target
		 */
		
		String modifiedType = "PERI";
		double detectionRange = 1.5;
		int requiredPayloads = 4;
		double captureCountdown = 55.43;
		boolean modifyCaptureState = false;
		boolean captured = false;
	        
		// Create a new JSON object to send the message contents within
		JsonObject jsonConfigMessage = new JsonObject();
	    
		// Set the configurations
		jsonConfigMessage.addProperty("uid", testUid);
		jsonConfigMessage.addProperty("type", modifiedType);
		jsonConfigMessage.addProperty("detection_range", detectionRange);
		jsonConfigMessage.addProperty("required_payloads", requiredPayloads);
		jsonConfigMessage.addProperty("capture_countdown", captureCountdown);
		jsonConfigMessage.addProperty("modify_capture_state", modifyCaptureState);
		jsonConfigMessage.addProperty("captured", captured);
	    
		// Send the message
		sendJsonMessage(jsonConfigMessage, configTopic);
		
		/*
		 * Modify the new mass target state
		 */
		
        JsonObject jsonStateMessage = new JsonObject();
        
        // Modify the parameters
		testLatitude = 119.3;
		testLongitude = 54.6;
		testAltitude = 22.5;
		double testPayloads = 4;
		double testRequiredPayloads = 8;
		boolean testCaptured = true;
		double testCountdown = 60.5;
		double testCurrentDuration = 24.5;
        
		// Populate the message
        jsonStateMessage.addProperty("uid", testUid);
        jsonStateMessage.addProperty("type", modifiedType);
        jsonStateMessage.addProperty("latitude", testLatitude);
        jsonStateMessage.addProperty("longitude", testLongitude);
        jsonStateMessage.addProperty("altitude", testAltitude);
        jsonStateMessage.addProperty("payloads", testPayloads);
        jsonStateMessage.addProperty("countdown", testCountdown);
        jsonStateMessage.addProperty("current_duration", testCurrentDuration);
        jsonStateMessage.addProperty("required_payloads", testRequiredPayloads);
        jsonStateMessage.addProperty("captured", testCaptured);
        
        // Send the new state message
        sendJsonMessage(jsonStateMessage, stateTopic);
        
		// Ensure that only one target exists in the set
		assertTrue(callback.hardware.containsKey(testUid));
		
        PeriodicTarget target = (PeriodicTarget)callback.hardware.get(testUid);
        
        assertEquals(testLatitude, target.getLocation().getLatitude());
        assertEquals(testLongitude, target.getLocation().getLongitude());
        assertEquals(testAltitude, target.getLocation().getAltitude());
        assertEquals(testPayloads, target.getPayloadsInRange());
        assertEquals(testCaptured, target.getCaptured());
        assertEquals(testCurrentDuration, target.getCurrentCaptureDuration());
	}
	
	
	@Test
	@DisplayName("Parse a Linked Target Network Configs Test")
	void testParseLinkedNetworkConfigs() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		callback.networks.clear();
		
		/*
		 * The first step is to generate a set of new targets that will become a network
		 */
		
		// Default values
		String testUid1 = "TARGET-1";
		String testUid2 = "TARGET-2";
		String testUid3 = "TARGET-3";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		// Generate 3 new targets
		createDefaultTarget(testUid1, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid2, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid3, testLatitude, testLongitude, testAltitude);

		// Ensure that all 3 target exists in the set
		assertEquals(callback.hardware.size(), 3);
		
		JsonObject networkConfigMessage = new JsonObject();
		
		// Initialize network configs
		String networkUid = "LINK-NETWORK-1";
		String type = "LINK";
		boolean delete = false;
		boolean changeUid = false;
		String oldUid = networkUid;
		double detectionRange = 0.5;
		boolean modifyCapture = false;
		boolean captured = false;
		
		// Create the network
		JsonArray targetNetwork = new JsonArray();
		targetNetwork.add(testUid1);
		targetNetwork.add(testUid2);
		targetNetwork.add(testUid3);
		
		// Populate the message
		networkConfigMessage.addProperty("uid", networkUid);
		networkConfigMessage.addProperty("type", type);
		networkConfigMessage.addProperty("delete", delete);
		networkConfigMessage.addProperty("change_uid", changeUid);
		networkConfigMessage.addProperty("old_uid", oldUid);
		networkConfigMessage.add("network", targetNetwork);
		networkConfigMessage.addProperty("detection_range", detectionRange);
		networkConfigMessage.addProperty("modify_capture_state", modifyCapture);
		networkConfigMessage.addProperty("captured", captured);
		
		sendJsonMessage(networkConfigMessage, configTopic);
		
		assertTrue(callback.networks.containsKey(networkUid));
		
		LinkedNetwork generatedNetwork = callback.networks.get(networkUid);
		
		assertEquals(targetNetwork.size(), generatedNetwork.getNetworkTargets().size());
		
		for (LinkedTarget target : generatedNetwork.getNetworkTargets()) {
			assertEquals(detectionRange, target.getAcceptableDetectionRange());
		}
	}
	
	
	@Test
	@DisplayName("Add a Target to a Linked Network Test")
	void testAddTargetToLinkedNetwork() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		callback.networks.clear();
		
		/*
		 * The first step is to generate a set of new targets that will become a network
		 */
		
		// Default values
		String testUid1 = "TARGET-1";
		String testUid2 = "TARGET-2";
		String testUid3 = "TARGET-3";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		// Generate 3 new targets
		createDefaultTarget(testUid1, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid2, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid3, testLatitude, testLongitude, testAltitude);

		// Ensure that all 3 target exists in the set
		assertEquals(callback.hardware.size(), 3);
		
		JsonObject networkConfigMessage = new JsonObject();
		
		// Initialize network configs
		String networkUid = "LINK-NETWORK-1";
		String type = "LINK";
		boolean delete = false;
		boolean changeUid = false;
		String oldUid = networkUid;
		double detectionRange = 0.5;
		boolean modifyCapture = false;
		boolean captured = false;
		
		// Create the network
		JsonArray targetNetwork = new JsonArray();
		targetNetwork.add(testUid1);
		targetNetwork.add(testUid2);
		
		// Populate the message
		networkConfigMessage.addProperty("uid", networkUid);
		networkConfigMessage.addProperty("type", type);
		networkConfigMessage.addProperty("delete", delete);
		networkConfigMessage.addProperty("change_uid", changeUid);
		networkConfigMessage.addProperty("old_uid", oldUid);
		networkConfigMessage.add("network", targetNetwork);
		networkConfigMessage.addProperty("detection_range", detectionRange);
		networkConfigMessage.addProperty("modify_capture_state", modifyCapture);
		networkConfigMessage.addProperty("captured", captured);
		
		sendJsonMessage(networkConfigMessage, configTopic);
		
		assertTrue(callback.networks.containsKey(networkUid));
		
		// Add a new target to the network
		targetNetwork.add(testUid3);
		
		// Update the message
		networkConfigMessage.add("network", targetNetwork);
		
		sendJsonMessage(networkConfigMessage, configTopic);
		
		LinkedNetwork generatedNetwork = callback.networks.get(networkUid);
		
		assertEquals(targetNetwork.size(), generatedNetwork.getNetworkTargets().size());
	}
	
	
	@Test
	@DisplayName("Remove a Target from a Linked Network Test")
	void testRemoveTargetFromLinkedNetwork() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		callback.networks.clear();
		
		/*
		 * The first step is to generate a set of new targets that will become a network
		 */
		
		// Default values
		String testUid1 = "TARGET-1";
		String testUid2 = "TARGET-2";
		String testUid3 = "TARGET-3";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		// Generate 3 new targets
		createDefaultTarget(testUid1, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid2, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid3, testLatitude, testLongitude, testAltitude);

		// Ensure that all 3 target exists in the set
		assertEquals(callback.hardware.size(), 3);
		
		JsonObject networkConfigMessage = new JsonObject();
		
		// Initialize network configs
		String networkUid = "LINK-NETWORK-1";
		String type = "LINK";
		boolean delete = false;
		boolean changeUid = false;
		String oldUid = networkUid;
		double detectionRange = 0.5;
		boolean modifyCapture = false;
		boolean captured = false;
		
		// Create the network
		JsonArray targetNetwork = new JsonArray();
		targetNetwork.add(testUid1);
		targetNetwork.add(testUid2);
		
		// Populate the message
		networkConfigMessage.addProperty("uid", networkUid);
		networkConfigMessage.addProperty("type", type);
		networkConfigMessage.addProperty("delete", delete);
		networkConfigMessage.addProperty("change_uid", changeUid);
		networkConfigMessage.addProperty("old_uid", oldUid);
		networkConfigMessage.add("network", targetNetwork);
		networkConfigMessage.addProperty("detection_range", detectionRange);
		networkConfigMessage.addProperty("modify_capture_state", modifyCapture);
		networkConfigMessage.addProperty("captured", captured);
		
		sendJsonMessage(networkConfigMessage, configTopic);
		
		assertTrue(callback.networks.containsKey(networkUid));
		
		// Add a new target to the network
		targetNetwork.add(testUid3);
		
		// Update the message
		networkConfigMessage.add("network", targetNetwork);
		
		sendJsonMessage(networkConfigMessage, configTopic);
		
		LinkedNetwork generatedNetwork = callback.networks.get(networkUid);
		
		assertEquals(targetNetwork.size(), generatedNetwork.getNetworkTargets().size());
	}
	
	
	@Test
	@DisplayName("Delete a Linked Network Test")
	void testDeleteLinkedNetwork() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		callback.networks.clear();
		
		/*
		 * The first step is to generate a set of new targets that will become a network
		 */
		
		// Default values
		String testUid1 = "TARGET-1";
		String testUid2 = "TARGET-2";
		String testUid3 = "TARGET-3";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		// Generate 3 new targets
		createDefaultTarget(testUid1, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid2, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid3, testLatitude, testLongitude, testAltitude);

		// Ensure that all 3 target exists in the set
		assertEquals(callback.hardware.size(), 3);
		
		JsonObject networkConfigMessage = new JsonObject();
		
		// Initialize network configs
		String networkUid = "LINK-NETWORK-1";
		String type = "LINK";
		boolean delete = false;
		boolean changeUid = false;
		String oldUid = networkUid;
		double detectionRange = 0.5;
		boolean modifyCapture = false;
		boolean captured = false;
		
		// Create the network
		JsonArray targetNetwork = new JsonArray();
		targetNetwork.add(testUid1);
		targetNetwork.add(testUid2);
		targetNetwork.add(testUid3);
		
		// Populate the message
		networkConfigMessage.addProperty("uid", networkUid);
		networkConfigMessage.addProperty("type", type);
		networkConfigMessage.addProperty("delete", delete);
		networkConfigMessage.addProperty("change_uid", changeUid);
		networkConfigMessage.addProperty("old_uid", oldUid);
		networkConfigMessage.add("network", targetNetwork);
		networkConfigMessage.addProperty("detection_range", detectionRange);
		networkConfigMessage.addProperty("modify_capture_state", modifyCapture);
		networkConfigMessage.addProperty("captured", captured);
		
		sendJsonMessage(networkConfigMessage, configTopic);
		
		assertTrue(callback.networks.containsKey(networkUid));
		
		// Set the delete config to true
		delete = true;

		// Update the message
		networkConfigMessage.addProperty("delete", delete);
		
		// Reset the configs
		sendJsonMessage(networkConfigMessage, configTopic);
		
		assertFalse(callback.networks.containsKey(networkUid));
	}
	
	
	@Test
	@DisplayName("Rename a Linked Network Test")
	void testRenameLinkedNetwork() throws InterruptedException {
		// Clear the data
		callback.hardware.clear();
		callback.networks.clear();
		
		/*
		 * The first step is to generate a set of new targets that will become a network
		 */
		
		// Default values
		String testUid1 = "TARGET-1";
		String testUid2 = "TARGET-2";
		String testUid3 = "TARGET-3";
		double testLatitude = 98.4;
		double testLongitude = 34.6;
		double testAltitude = 336.5;
		
		// Generate 3 new targets
		createDefaultTarget(testUid1, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid2, testLatitude, testLongitude, testAltitude);
		createDefaultTarget(testUid3, testLatitude, testLongitude, testAltitude);

		// Ensure that all 3 target exists in the set
		assertEquals(callback.hardware.size(), 3);
		
		JsonObject networkConfigMessage = new JsonObject();
		
		// Initialize network configs
		String networkUid = "LINK-NETWORK-1";
		String type = "LINK";
		boolean delete = false;
		boolean changeUid = false;
		String oldUid = networkUid;
		double detectionRange = 0.5;
		boolean modifyCapture = false;
		boolean captured = false;
		
		// Create the network
		JsonArray targetNetwork = new JsonArray();
		targetNetwork.add(testUid1);
		targetNetwork.add(testUid2);
		targetNetwork.add(testUid3);
		
		// Populate the message
		networkConfigMessage.addProperty("uid", networkUid);
		networkConfigMessage.addProperty("type", type);
		networkConfigMessage.addProperty("delete", delete);
		networkConfigMessage.addProperty("change_uid", changeUid);
		networkConfigMessage.addProperty("old_uid", oldUid);
		networkConfigMessage.add("network", targetNetwork);
		networkConfigMessage.addProperty("detection_range", detectionRange);
		networkConfigMessage.addProperty("modify_capture_state", modifyCapture);
		networkConfigMessage.addProperty("captured", captured);
		
		sendJsonMessage(networkConfigMessage, configTopic);
		
		assertTrue(callback.networks.containsKey(networkUid));
		
		// Rename the network
		changeUid = true;
		networkUid = "LINK-NETWORK-RENAMED";

		// Update the message
		networkConfigMessage.addProperty("uid", networkUid);
		networkConfigMessage.addProperty("change_uid", changeUid);
		
		// Reset the configs
		sendJsonMessage(networkConfigMessage, configTopic);
		
		assertTrue(callback.networks.containsKey(networkUid));
	}
}
