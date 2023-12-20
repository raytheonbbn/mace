//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.config;

import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

public class CapabilitySets {

	public HashMap<String, List<String>> capabilitySets = new HashMap<>();

	public CapabilitySets() {
	}

	public void setCapabilitySets(HashMap<String, List<String>> capabilitySets) {
		this.capabilitySets = capabilitySets;
	}

	public List<String> getCapabilitySets(String key) {
		return capabilitySets.get(key);
	}

	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer();
		for (Entry<String, List<String>> entrySet : capabilitySets.entrySet()) {
			sb.append("key: " + entrySet.getKey() + "\n");
			sb.append("capabilities: " + entrySet.getValue() + "\n");
		}
		return sb.toString();
	}
}
