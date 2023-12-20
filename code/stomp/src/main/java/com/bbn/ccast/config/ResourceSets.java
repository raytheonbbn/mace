//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.config;

import java.util.HashMap;
import java.util.Map;

public class ResourceSets {

    //public HashMap<String, List<String>> resourceSets = new HashMap<>();

    String robotType;
    Map<String, Object> resources = new HashMap<>();

    public ResourceSets() { }

    public String getRobotType() {return robotType; }
    public Map<String, Object> getResources(){return resources; }


/*    @Override
    public String toString() {
        StringBuffer sb = new StringBuffer();
        for (Entry<String, List<String>> entrySet : resourceSets.entrySet()) {
            sb.append("key: " + entrySet.getKey() + "\n");
            sb.append("resources: " + entrySet.getValue() + "\n");
        }
        return sb.toString();
    }*/
}
