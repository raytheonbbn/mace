//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.ros;

import com.google.gson.JsonElement;

import java.util.function.Consumer;

public interface RosServiceInterface {


    /**
     * Calls a service on the ROS system with possibly arguments for the request
     * message. The message will be called with the message upon receipt of the
     * message service response
     *
     * @param service   The name of the service to call
     * @param args      The arguments of a request, if null (or empty) then an empty
     *                  request is sent
     * @param messageHandler   The callback method for this service request
     */
    public void callService(String service, JsonElement args, Consumer<JsonElement> messageHandler);

    public boolean isOpen();

}
