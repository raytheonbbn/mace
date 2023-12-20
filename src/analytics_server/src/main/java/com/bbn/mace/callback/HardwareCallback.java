//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.callback;

import com.bbn.mace.utils.MaceMessageTransport;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


/*
 * Abstract class used to define common variables and functionality between the callback methods
 */
public abstract class HardwareCallback<T> implements IMqttMessageListener {

	public ConcurrentHashMap<String, T> hardware;	// HashMap used to store UID, hardware pairs
	protected final Logger logger;					// Logger used to log class events and exceptions
	protected ExecutorService pool;					// Thread pool used by the manager to execute map updates
	protected MaceMessageTransport client;

    public HardwareCallback(MaceMessageTransport client) {

		// Initialize the thread pool
		this.pool = Executors.newCachedThreadPool();
		
		// Initialize the class logger
		this.logger = LogManager.getLogger(HardwareCallback.class.getName());
		
		// Initialize the hardware map
		this.hardware = new ConcurrentHashMap<String, T>();

		this.client = client;
	}
}
