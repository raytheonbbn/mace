//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.utils;

import java.io.IOException;
import java.util.Properties;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.bbn.mace.payload.Payload;


/*
 * Helper class used to read properties from a configuration file
 */
public class Config {
	
	// Class variables
	Properties properties;			// Properties object used to retrieve properties from a config file
	private final Logger logger;	// Logger used to log class events and exceptions
 

	public Config(String filename) throws IOException {
		
		// Initialize the class variables
		this.properties = new Properties();
		this.logger = LogManager.getLogger(Payload.class.getName());
		
		// Load the properties file
		try {
			properties.load(this.getClass().getClassLoader().getResourceAsStream(filename));
		} catch(IOException e){
			this.logger.error("The system was unable to retrieve the desired configurations: ", e);
			throw e;
		}
	}
	
	public String getPropertyValue(String key) {
		String value = this.properties.getProperty(key);
		
		return value;
	}
}
