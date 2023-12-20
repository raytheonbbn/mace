//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.server;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;

import com.bbn.mace.callback.*;
import com.bbn.mace.utils.MaceMessageTransport;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttException;
import com.bbn.mace.manager.HardwareManager;
import com.bbn.mace.utils.Config;
import com.bbn.mace.utils.MaceLogger;
import com.bbn.mace.utils.QualityOfService;
import org.eclipse.paho.client.mqttv3.MqttMessage;

/*
 * Main class used to execute all analytics server functionality and behavior
 */
public class Server {

	// Class Variables
	private HardwareManager payloadManager;			// Hardware manager used to handle payload messages
	private HardwareManager targetManager;		    // Hardware manager used to handle target messages
	private MaceLogger logger;						// System logger used to log all topic messages
	private final Logger consoleLogger;				// Logger used to display error messages to users
	private String host;							// MQTT host
	private MaceMessageTransport client; 			// MQTT client
	private static final String COMMAND_TOPIC = "command/server/command";
	private CommandCallback commandCallback;

	private static String cotDestinations;


	public Server(String host, String mqttUser, String mqttPassword) {

		// Initialize the class logger
		this.consoleLogger = LogManager.getLogger(Server.class.getName());
		
		this.consoleLogger.info("Initializing a new analytics server");
		
		// Initialize the class variables
		this.host = host;

		this.client = new MaceMessageTransport(host, "Analytics Server", mqttUser, mqttPassword);

		// Allow everything else to start up. This prevents getting several warnings about the inability to connect.
		while (this.client.isConnected() == false)
		{
			try
			{
				Thread.sleep(1000);
				this.consoleLogger.info("Waiting for MQTT connection.");
			}
			catch(InterruptedException ex)
			{
				Thread.currentThread().interrupt();
			}
		}
		
		// Initialize the new hardware managers
		try {
			this.consoleLogger.info("Creating a new payload manager and a new hardware manager");
			payloadManager = new HardwareManager(client);
			targetManager = new HardwareManager(client);
		} catch (MqttException e) {
			this.consoleLogger.error("The system was unable to generate the required hardware managers. Exiting. ", e);
			return;
		}
		
		try {
			this.consoleLogger.info("Creating a new MACE file logger");
			this.logger = new MaceLogger(client);
		} catch (MqttException e) {
			this.consoleLogger.info("The system was unable to generate the required logger. System processing will resume without the logger.", e);
		}
	}

	/*
	 * Getter used to retrieve the MQTT host string
	 */
	private String getHost() {
		return this.host;
	}

	/*
	 * Sends a start_new_run command which is picked up by STOMP and that loads in targets, and sets their discovery status to undiscovered
	 */
    public void sendPrepareNewRunCommand() {
		// Create a new JSON object to send the message contents within
		JsonObject jsonMessage = new JsonObject();

		// Populate the message
		jsonMessage.addProperty("command", "prepare_new_run");

		// Convert the message to a string
		String stringMessage = jsonMessage.toString();

		// Create a new MQTT message
		MqttMessage message = new MqttMessage();

		// Set the MQTT message payload to the JSON object
		message.setPayload(stringMessage.getBytes());

        try {
            this.client.publish(COMMAND_TOPIC, message);
        } catch (MqttException e) {
            this.consoleLogger.warn("Failed to send start_new_run command MQTT:", e);
        }
    }

	/*
	 * Method responsible for starting the MACE logger. Note that this is distinct from the base "start" method because this is not required
	 * functionality. Furthermore, order of the topics, service levels, and file names is critical in this method. Specifically, the index of
	 * each topic should correlate to the index of the desired service level and desired file name. (e.g., topics[0] will be assigned the 
	 * service level at serviceLevels[0] and the file name at fileNames[0]).
	 * Returns the tag name for the log file
	 */
	public String startMaceLogger(String topics[], int serviceLevels[], String fileNames[], ArrayList<ArrayList<String>> headers) throws InterruptedException{
		
		// Declare the return variable
		String tag = "";

		// Ensure that the proper configurations were provided
		if (topics.length != serviceLevels.length || topics.length != fileNames.length || serviceLevels.length != fileNames.length) {
			this.consoleLogger.error("An invalid configuration was provided to start the MACE file logger. An equivalent number of topics, service levels, "
					+ "and file names should be provided. Topics provided: " + topics.length + " Service Levels provided: " + serviceLevels.length 
					+ " File Names provided: " + fileNames.length);
			
			return tag;
		}
	
		this.consoleLogger.info("Attempting to start the MACE logger");
		
		// Create a new array of callbacks to subscribe to
		MaceLoggerCallback callbacks[] = new MaceLoggerCallback[fileNames.length];
		
		try {
			// Populate the array of callbacks
			for (int i = 0; i < fileNames.length; ++i) {
				MaceLoggerCallback callbackToAdd;
				callbackToAdd = new MaceLoggerCallback(fileNames[i], headers.get(i));
				callbacks[i] = callbackToAdd;
			}
			
		} catch (IOException e) {
			this.consoleLogger.error("The system was unable to generate new callback functions to assign to the MACE logger. The logger will not be started.");
			return tag;
		} catch (MqttException e) {
			this.consoleLogger.error("Failed to create MQTT callback: ", e);
		}

		// Connect the MACE logger
		// Subscribe to the desired topics
		this.logger.subscribe(topics, serviceLevels, callbacks);

		// The server is up and running. Prepare for a new run
		try
		{
			Thread.sleep(2000);
			this.consoleLogger.info("Waiting for MQTT connection.");
		}
		catch(InterruptedException ex)
		{
			Thread.currentThread().interrupt();
		}
		sendPrepareNewRunCommand();
		
		this.consoleLogger.info("MACE logger successfully started");

		try {
			//Keep alive until interrupted
			while (commandCallback.getResetStatus() == false){
				// Sleep and check the reset button again
				Thread.sleep(100);
			}
			// Get the tag name
			tag = commandCallback.getTag();
			// Reset is complete
			commandCallback.resetCompleted();
		} catch (InterruptedException e) {
			throw new InterruptedException();
		}

		// Return the tag name
		return tag;
	}

	/*
	 * Responsible for starting the payload hardware manager by connecting it to the MQTT client and subscribing it to the correct 
	 * topics with its respective callback class
	 */
	private boolean startPayloadManager(String host, String payloadConfigTopic, QualityOfService payloadConfigQos, String payloadStateTopic, QualityOfService payloadStateQos,
			String inputTopics[], int inputQos[]) {
		
		this.consoleLogger.info("Attempting to create a new payload hardware manager");
		
		// Start the payload manager
		try {
			// Create a new payload callback class to handle the messages and maintain the hardware state
			PayloadCallback payloadCallback = new PayloadCallback(host, payloadConfigTopic,
					payloadConfigQos, payloadStateTopic, payloadStateQos, client);


			// Subscribe to the desired input topics and set the respective callback method
			this.payloadManager.subscribe(inputTopics, inputQos, payloadCallback);
		} catch (MqttException e) {
			consoleLogger.error("The system could not generate a new payload manager. Received error: ", e);
			return false;
		}
		
		this.consoleLogger.info("Successfully created a new payload hardware manager");
		
		return true;
	}
	
	
	/*
	 * Responsible for starting the target hardware manager by connecting it to the MQTT client and subscribing it to the correct 
	 * topics with its respective callback class
	 */
	private boolean startTargetManager(String host, String targetConfigTopic, QualityOfService targetConfigQos, String targetWhiteForceStateTopic, 
			String targetBlueForceStateTopic, QualityOfService targetStateQos, String inputTopics[], int inputQos[]) {

		this.consoleLogger.info("Attempting to create a new target hardware manager");

		try {
			// Create a new target callback class to handle the target messages and the hardware state
			TargetCallback targetCallback = new TargetCallback(host, targetConfigTopic, targetConfigQos, 
					targetWhiteForceStateTopic, targetBlueForceStateTopic, targetStateQos, client);

			// Subscribe to the desired input topics and set the respective callback method
			this.targetManager.subscribe(inputTopics, inputQos, targetCallback);
		} catch (MqttException e) {
			consoleLogger.error("The system could not generate a new target manager. Received error: ", e);
			return false;
		}
		
		this.consoleLogger.info("Successfully created a new target hardware manager");
		
		return true;		
	}
	
	
	/*
	 * Start the server
	 */
	public void startServer(String payloadOutputConfigTopic, QualityOfService payloadOutputConfigQos, String payloadOutputStateTopic, QualityOfService payloadOutputStateQos,
			String payloadInputTopics[], int payloadInputQos[], String targetOutputConfigTopic, QualityOfService targetOutputConfigQos, 
			String targetOutputWhiteForceStateTopic, String targetOutputBlueForceStateTopic, QualityOfService targetOutputStateQos,
			String targetInputTopics[], int targetInputQos[]) {
		
		this.consoleLogger.info("Starting the analytics server");
				
		// Start the payload manager
		boolean payloadFailed = this.startPayloadManager(this.getHost(), payloadOutputConfigTopic, payloadOutputConfigQos, payloadOutputStateTopic, payloadOutputStateQos, 
				payloadInputTopics, payloadInputQos);
		
		// If an error occurred, exit
		if (!payloadFailed) {
			return;
		}
		
		// Start the target manager
		boolean targetFailed = this.startTargetManager(this.getHost(), targetOutputConfigTopic, targetOutputConfigQos, targetOutputWhiteForceStateTopic, 
				targetOutputBlueForceStateTopic, targetOutputStateQos, targetInputTopics, targetInputQos);
		
		// If an error occurred, exit
		if (!targetFailed) {
			return;
		}

		commandCallback = new CommandCallback(client);
		client.subscribe(COMMAND_TOPIC, QualityOfService.RECEIVE_EXACTLY_ONCE.getValue(), commandCallback, false);
		
		this.consoleLogger.info("Successfully started the analytics server");
		
	}
	
	
	/*
	 * Shut down the server
	 */
	public void stopServer() {
		this.consoleLogger.info("Stopping the analytics server");
		this.client.disconnectAndClose();
	}
	
	
	public static void main(String[] args) {		
		// Create a new configuration manager
		Config config;
		
		try {
			// Initialize the configuration manager
			config = new Config("MaceConfigs.properties");
		} catch (IOException e) {
			return;
		}

		// CoT Config
		Server.cotDestinations = config.getPropertyValue("cotDestinations");

		// MQTT Broker
		final String host = config.getPropertyValue("serverHost");
		final String mqttUser = config.getPropertyValue("mqttUser");
		final String mqttPassword = config.getPropertyValue("mqttPassword");


		System.out.println(host);
		
		// Target Parameters
		final String targetInputConfigTopic = config.getPropertyValue("targetInputConfigTopic");
		final String targetInputStateTopic = config.getPropertyValue("targetInputStateTopic");
		final String targetInputTopics[] = {targetInputConfigTopic, targetInputStateTopic};
		final int targetInputQos[] = {QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue(), QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue()};
		final String targetOutputConfigTopic = config.getPropertyValue("targetOutputConfigTopic");
		final String targetWhiteForceOutputStateTopic = config.getPropertyValue("targetWhiteForceOutputStateTopic");
		final String targetBlueForceOutputStateTopic = config.getPropertyValue("targetBlueForceOutputStateTopic");
		final QualityOfService targetOutputConfigQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
		final QualityOfService targetOutputStateQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
		
		// Payload Parameters
		final String payloadInputConfigTopic = config.getPropertyValue("payloadInputConfigTopic");
		final String payloadInputStateTopic = config.getPropertyValue("payloadInputStateTopic");
		final String payloadInputTopics[] = {payloadInputConfigTopic, payloadInputStateTopic};
		final int payloadInputQos[] = {QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue(), QualityOfService.RECEIVE_AT_LEAST_ONCE.getValue()};
		final String payloadOutputConfigTopic = config.getPropertyValue("payloadOutputConfigTopic");
		final String payloadOutputStateTopic = config.getPropertyValue("payloadOutputStateTopic");
		final QualityOfService payloadOutputConfigQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
		final QualityOfService payloadOutputStateQos = QualityOfService.RECEIVE_AT_LEAST_ONCE;
		
		// Create the log file
		try {
			//Keep alive until interrupted
			while (true) {

				// Create a new server
				Server server = new Server(host, mqttUser, mqttPassword);
				
				// Start the server
				server.startServer(
						payloadOutputConfigTopic, 
						payloadOutputConfigQos, 
						payloadOutputStateTopic, 
						payloadOutputStateQos, 
						payloadInputTopics, 
						payloadInputQos, 
						targetOutputConfigTopic, 
						targetOutputConfigQos, 
						targetWhiteForceOutputStateTopic, 
						targetBlueForceOutputStateTopic, 
						targetOutputStateQos, 
						targetInputTopics, 
						targetInputQos
				);
				
				// Create the set of new topics that should be logged
				final String loggedTopics[] = {
						targetInputConfigTopic, 
						targetInputStateTopic, 
						targetOutputConfigTopic, 
						targetWhiteForceOutputStateTopic,
						targetBlueForceOutputStateTopic, 
						payloadInputConfigTopic, 
						payloadInputStateTopic, 
						payloadOutputConfigTopic, 
						payloadOutputStateTopic
				};
				
				// Create an array of service levels for the topic logging reliability
				final int logServiceLevels[] = new int[loggedTopics.length];
				
				// Set the service level to unreliable for each topic
				for (int i = 0; i < logServiceLevels.length; ++i) {
					logServiceLevels[i] = QualityOfService.UNRELIABLE.getValue();
				}
		
				// Get the current data and time to ensure that new files g
				String currentDateTime = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date()) + "/";

				// Specify the log directory
				String logDir = "../../" + config.getPropertyValue("maceLogDirectory") + currentDateTime;

				// Create the new log directory if it doesn't already exist
				File dir = new File(logDir);
				if (!dir.exists()){
					dir.mkdirs();
				}

				// Create an array of file names corresponding to the logged topics
				final String logFiles[] = {
						logDir + config.getPropertyValue("targetInputConfigFile"),
						logDir + config.getPropertyValue("targetInputStateFile"),
						logDir + config.getPropertyValue("targetOutputConfigFile"),
						logDir + config.getPropertyValue("targetWhiteForceOutputStateFile"),
						logDir + config.getPropertyValue("targetBlueForceOutputStateFile"),
						logDir + config.getPropertyValue("payloadInputConfigFile"),
						logDir + config.getPropertyValue("payloadInputStateFile"),
						logDir + config.getPropertyValue("payloadOutputConfigFile"),
						logDir + config.getPropertyValue("payloadOutputStateFile")				
				};

				// Read the headers to an ArrayList
				ArrayList<String> targetStateHeaders = new ArrayList<String>(Arrays.asList(config.getPropertyValue("targetStateHeaders").split(",")));
				ArrayList<String> targetConfigHeaders = new ArrayList<String>(Arrays.asList(config.getPropertyValue("targetConfigHeaders").split(",")));
				ArrayList<String> payloadStateHeaders = new ArrayList<String>(Arrays.asList(config.getPropertyValue("payloadStateHeaders").split(",")));
				ArrayList<String> payloadConfigHeaders = new ArrayList<String>(Arrays.asList(config.getPropertyValue("payloadConfigHeaders").split(",")));

				// Create an array of the callbacks that should be used for each respective topic
				final ArrayList<ArrayList<String>> headers = new ArrayList<>();

				// Add the headers in order of the topics that they are associated with
				headers.add(targetConfigHeaders);
				headers.add(targetStateHeaders);
				headers.add(targetConfigHeaders);
				headers.add(targetStateHeaders);
				headers.add(targetStateHeaders);
				headers.add(payloadConfigHeaders);
				headers.add(payloadStateHeaders);
				headers.add(payloadConfigHeaders);
				headers.add(payloadStateHeaders);

				// Start the logger and get the tag
				String tag = server.startMaceLogger(loggedTopics, logServiceLevels, logFiles, headers);

				// Disconnect
				server.stopServer();

				// If we have a tag
				if (tag != null && tag.length() > 0){
					// Get the original LogDir
					Path originalLogDir = Paths.get(logDir);
					
					// Insert the tag
					String fileName = "/" + tag + "_" + originalLogDir.getFileName().toString();
					String fileParent = originalLogDir.getParent().toString();

					// Create the new logDir
					Path newLogDir = Paths.get(fileParent + fileName);

					// Rename the directory
					File sourceFile = new File(originalLogDir.toString());
					File destFile = new File(newLogDir.toString());
					if (sourceFile.renameTo(destFile)) {
						System.out.println("Directory renamed successfully");
					} else {
						System.out.println("Failed to rename directory");
					}
				}

			}
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
    }

	public static String getCotDestinations(){
		return cotDestinations;
	}
}
