//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import javax.json.Json;
import javax.json.JsonObject;

class RosMessageHeader {

	static volatile int dummySequence = 0;
	
	static final JsonObject build(String frameId) {
		
		long time = System.currentTimeMillis();
		int timeSeconds = (int) (time / 1000);
		int timeMilliFraction = (int) (time % 1000);
		int timeNanoFraction = (int) (timeMilliFraction / 1e6);

		JsonObject stamp = Json.createObjectBuilder().add("sec", timeSeconds).add("nsec", timeNanoFraction).build();

		// "frame_id" is probably not used, but let's just call it base_link
		JsonObject header = Json.createObjectBuilder().add("seq", dummySequence).add("stamp", stamp)
				.add("frame_id", frameId).build();
		
		dummySequence++;
		
		return header;
	}
}
