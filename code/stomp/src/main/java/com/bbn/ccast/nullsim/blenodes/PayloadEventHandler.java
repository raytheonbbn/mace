//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.SimVehicle.Intent;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import org.apache.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.Collection;

public class PayloadEventHandler {
    private static final Logger logger = Logger.getLogger(PayloadEventHandler.class.getName());
    private static final String PAYLOAD_CONFIG_TOPIC = "server/command/payload/configurations";
    private static final String HARDWARE_PAYLOAD_CONFIG_TOPIC = "hardware/server/payload/configurations";
    

    protected final MaceMessageTransport transport;

    public PayloadEventHandler(MaceMessageTransport transport){
        this.transport = transport;
    }

    public void sendPayloadConfig(String payloadId, Collection<Intent> intent){
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("uid", payloadId);
        jsonObj.add("intent", SimVehicle.getIntentToJsonArray(intent));
        String msgPayload = jsonObj.toString();
        try {
            logger.info("Sending config information: " + msgPayload);
            transport.publish(PAYLOAD_CONFIG_TOPIC, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send payload configuration over MQTT:", e);
        }
    }

    public void sendPayloadCallsign(String payloadId, String callsign) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("set_callsign", true);
        jsonObj.addProperty("uid", payloadId);
        jsonObj.addProperty("callsign", callsign);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(PAYLOAD_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send network config over MQTT:", e);
        }
        try {
            transport.publish(HARDWARE_PAYLOAD_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send network config over MQTT:", e);
        }
    }


}
