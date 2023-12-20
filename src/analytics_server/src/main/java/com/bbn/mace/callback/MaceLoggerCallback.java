//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.callback;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.*;
import com.opencsv.CSVWriter;


/*
 * Message Listener class used to handle incoming messages on a desired topic and log those to a file
 */
public class MaceLoggerCallback implements IMqttMessageListener {
	
	// Class variables
	private final Logger logger;			// Logger used to log class events and exceptions
    private CSVWriter writer;				// CSV writer used to log the desired data
	private FileWriter file;				// File writer object that will be used by the CSVWriter to create the log file
	private ArrayList<String> columns; 		// ArrayList used to store the column names

	
	public MaceLoggerCallback(String outputFile, ArrayList<String> columns) throws IOException, MqttException {	
		
		// Initialize class variables
		this.columns = columns;
		
		// Initialize the class debug logger
		this.logger = LogManager.getLogger(MaceLoggerCallback.class.getName());	
		
		// Initialize the file writer and CSV writer
		try {
			this.file = new FileWriter(outputFile);
			this.writer = new CSVWriter(this.file);
		} catch (IOException e) {
			logger.error("An error occurred while attempting to create a new file writer: ", e);
			
			// Re-throw the exception so that it must be handled
			throw e;
		}
		
		// Convert the ArrayList to an array to enable writing the columns
		String[] columnsArray = columns.toArray(String[]::new);
		
		// Write the column names to the file
		this.writer.writeNext(columnsArray, false);
		this.writer.flush();
	}
	
	
	/*
	 * Write the JSON values to the CSV file
	 */
	private void logData(Set<Map.Entry<String, JsonElement>> entries, ArrayList<String> columns) {

		// Create a new array to store the column names in
		String values[] = new String[columns.size()];
		
		// Add each column to the array
		for(Map.Entry<String, JsonElement> entry: entries) {
			
			// Get the current timestamp
			values[columns.indexOf("timestamp")] = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());
			
			// If the entry key is in the list of columns, set the column value to be that message value
			if (columns.contains(entry.getKey())) {
				if (entry.getValue().isJsonArray()) {
					var sb = new StringBuilder();
					for (var s : entry.getValue().getAsJsonArray()) {
						sb.append(s.getAsString());
						sb.append(',');
					}
					sb.deleteCharAt(sb.length() - 1);
					values[columns.indexOf(entry.getKey())] = sb.toString();

				} else if (entry.getValue().isJsonObject()) {
					// Handle JSONObjects (for example networks_captured)
					Gson gson = new GsonBuilder().create();
					values[columns.indexOf(entry.getKey())] = gson.fromJson(entry.getValue().toString(), ConcurrentHashMap.class).toString().replace("{", "").replace("}", "");
				} else {
					values[columns.indexOf(entry.getKey())] = entry.getValue().getAsString();
				}
			}
		}
		
		// Write the values to the file
		this.writer.writeNext(values, false);
		
		// Flush the stream to write contents to the file
		try {
			this.writer.flush();
		} catch (IOException e) {
			this.logger.error("The MACE logger was unable to flush the data stream");
		}
	}


	/*
	 * Callback method called on a new message reception; used to log the messages to a file
	 */
	@Override
	public void messageArrived(String topic, MqttMessage message) {	

		try {
			// Create a new parser to parse the message
			JsonParser parser = new JsonParser();
			
			// Get the message as a JSON object
			JsonObject jsonMessage = parser.parse(new String(message.getPayload())).getAsJsonObject();
			
			// Get the entry set from the Object
			Set<Map.Entry<String, JsonElement>> entries = jsonMessage.entrySet();
			
			// Log the message
			this.logData(entries, this.columns);
		} catch (Exception e) {
			logger.error("MACE logger encountered an exception logging: " + topic + ": " + message.toString(), e);
		}
	}
}
