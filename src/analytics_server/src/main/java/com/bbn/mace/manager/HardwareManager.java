//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.manager;

import com.bbn.mace.utils.MaceMessageTransport;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttException;


/*
 * Responsible for managing the MQTT client and connecting callback functions
 */
public class HardwareManager {

	// Class Variables
	private final MaceMessageTransport client;
	private final Logger logger;		// Logger used to log class events and exceptions


	public HardwareManager(MaceMessageTransport client) throws MqttException {
		// Initialize the class logger
		this.logger = LogManager.getLogger(HardwareManager.class.getName());
		this.client = client;

	}

	public void subscribe(String[] topics, int[] serviceLevels, IMqttMessageListener listener) {
		// Subscribe to the desired topics
		this.logger.info("The Hardware Manager is attempting to subscribe to the following topics: ");

		for (int i = 0; i < topics.length; ++i) {
			this.logger.info(" - " + topics[i]);
			this.client.subscribe(topics[i], serviceLevels[i], listener, false);
		}
		this.logger.info("The Hardware Manager successfully subscribed to all topics and set the desired callback function");
	}

	public void subscribe(String[] topics, int[] serviceLevels, IMqttMessageListener[] listeners) {
		// Subscribe to the desired topics
		this.logger.info("The Hardware Manager is attempting to subscribe to the following topics: ");

		for (int i = 0; i < topics.length; ++i) {
			this.logger.info(" - " + topics[i]);
			this.client.subscribe(topics[i], serviceLevels[i], listeners[i], false);
		}
		this.logger.info("The Hardware Manager successfully subscribed to all topics and set the desired callback function");
	}

}
