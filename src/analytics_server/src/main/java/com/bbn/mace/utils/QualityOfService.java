//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.mace.utils;

// Quality of service ENUM to use for an MQTT message
public enum QualityOfService {
    UNRELIABLE(0),
    RECEIVE_AT_LEAST_ONCE(1),
    RECEIVE_EXACTLY_ONCE(2);
	
    public final int qos;
    
    QualityOfService(int qos){
        this.qos = qos;
    }
    
    public int getValue() {
    	return this.qos;
    }
}
