//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.ros;

import com.google.gson.JsonObject;

public interface RosTopicPublisher {

    public boolean isOpen();

    public void close();

    public void publish(String topic, String type, JsonObject msg);
}
