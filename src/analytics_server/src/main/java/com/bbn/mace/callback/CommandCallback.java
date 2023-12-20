//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.callback;

import com.bbn.mace.payload.Payload;
import com.bbn.mace.utils.MaceMessageTransport;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


/*
 * Abstract class used to define common variables and functionality between the callback methods
 */
public class CommandCallback implements IMqttMessageListener {

	private static final String SET_DISCOVERY_COMMAND	= "set_discovered";
	private static final String RESET_COMMAND 			= "reset";
	protected final Logger logger;					// Logger used to log class events and exceptions
	protected MaceMessageTransport client;
	private boolean resetRequested						= false;
	private String	resetTag							= "log";


	public boolean getResetStatus() {
		return this.resetRequested;
	}

	public String getTag() {
		return this.resetTag;
	}

	public void resetCompleted() {
		this.resetRequested = false;
		this.resetTag = "log";
	}

    public CommandCallback(MaceMessageTransport client) {
		// Initialize the class logger
		this.logger = LogManager.getLogger(CommandCallback.class.getName());

		this.client = client;
	}

	@Override
	public void messageArrived(String topic, MqttMessage message) throws Exception {
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
		try {
			String cmd = jsonMap.get("command").getAsString();
			switch (cmd){
				case SET_DISCOVERY_COMMAND:
					handleSetDiscovery(jsonMap);
					break;
				case RESET_COMMAND:
					handleResetCommand(jsonMap);
					break;
			}
		}catch (Exception e){
			logger.error("Failed to parse command: " + jsonMessage);
		}
	}

	private void handleSetDiscovery(Map<String, JsonElement> jsonMap) {
		JsonArray targetIdList = jsonMap.get("targets").getAsJsonArray();
		boolean isDiscovered = jsonMap.get("discovered").getAsBoolean();
		logger.info("Executing set discovery: " + targetIdList + ": "  + isDiscovered);
		for (JsonElement jsonElement : targetIdList){
			String targetId = jsonElement.getAsString();
			if (isDiscovered) {
				Payload.discoverTarget(targetId);
			}else{
				Payload.undiscoverTarget(targetId);
			}
		}
	}

	private void handleResetCommand(Map<String, JsonElement> jsonMap) {
		String tag = jsonMap.get("tag").getAsString();

		// Set requested to true
		resetRequested = true;

		// If a tag name was given, use that
		if (tag != null && tag.length() > 0){
			resetTag = tag;
		}

		logger.info("Executing reset");
	}
}
