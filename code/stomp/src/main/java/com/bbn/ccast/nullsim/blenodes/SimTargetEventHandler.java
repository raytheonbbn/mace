//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

import com.bbn.ccast.Utils;
import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.bbn.ccast.nullsim.SimSolo;
import com.bbn.ccast.nullsim.SimVehicle;
import com.bbn.ccast.nullsim.SimVehicle.Intent;
import com.google.gson.JsonObject;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import org.apache.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.Collection;
import java.util.concurrent.locks.ReentrantLock;

public class SimTargetEventHandler extends TargetEventHandler{
    private static final Logger logger = Logger.getLogger(SimTargetEventHandler.class.getName());
    private static final String INTERACTION_TOPIC_PREFIX = "sim/hardware/ble/";
    private static final String POSITION_TOPIC_PREFIX = "sim/hardware/position/";
    private static final String TARGET_STOP_PREFIX = "sim/target/stop/";
    private static final String TARGET_START_TOPIC = "sim/target/start";
    private static final long POSITION_UPDATE_TIME_MS = 5000;


    private final String targetId;

    private final String interactionTopic;
    private final String positionTopic;
    private final String targetStopTopic;

    private final ReentrantLock positionLock = new ReentrantLock();
    private Position position;
    private Thread positionUpdateThread;

    public SimTargetEventHandler(String targetId, MaceMessageTransport transport){
        super(transport);
        this.targetId = targetId;
        this.interactionTopic = INTERACTION_TOPIC_PREFIX + targetId;
        this.positionTopic = POSITION_TOPIC_PREFIX + targetId;
        this.targetStopTopic = TARGET_STOP_PREFIX + targetId;
        startPeriodicallySendingPosition();
    }

    public void payloadStartedInteraction(String payloadId, Collection<Intent> intent, SimTargetType config) {

        logger.debug(String.format("Payload %s is BLE interacting with target %s on %s", payloadId, this.targetId, INTERACTION_TOPIC_PREFIX + payloadId));
        boolean intentMatch = intent.contains(Intent.valueOf(config.name())) && config != SimTargetType.IDLE;
        logger.debug(String.format("intent: %s, config: %s", intent, config));

        {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("payload_id", payloadId);
        jsonObj.addProperty("interacting", intentMatch);
        jsonObj.addProperty("in_range", true);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(interactionTopic, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send interaction event over MQTT:", e);
        }        
        }

        // send the payload an interaction message no matter what. It may need to discover target
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("target_id", this.targetId);
        jsonObj.addProperty("in_range", true);
        jsonObj.addProperty("interacting", intentMatch);
        jsonObj.add("intent", SimVehicle.getIntentToJsonArray(intent));
        jsonObj.addProperty("config", config.name());
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(INTERACTION_TOPIC_PREFIX + payloadId, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send interaction event over MQTT:", e);
        }
    }

    public void payloadInDiscoveryRangeOfTarget(String payloadId, Collection<Intent> intent, SimTargetType config) {

        // logger.debug(String.format("Payload %s is BLE discovering target %s on %s", payloadId, this.targetId, INTERACTION_TOPIC_PREFIX + payloadId));

        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("target_id", this.targetId);
        jsonObj.addProperty("in_range", true);
        jsonObj.addProperty("interacting", false);
        jsonObj.add("intent", SimVehicle.getIntentToJsonArray(intent));
        jsonObj.addProperty("config", config.name());
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(INTERACTION_TOPIC_PREFIX + payloadId, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send interaction event over MQTT:", e);
        }
    }

    public void payloadStoppedInteraction(String payloadId, SimTargetType config) {
        logger.debug(String.format("payload %s is stopping BLE interaction with target %s", payloadId, this.targetId));
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("payload_id", payloadId);
        jsonObj.addProperty("interacting", false);
        jsonObj.addProperty("in_range", false);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(interactionTopic, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send interaction event over MQTT:", e);
        }

        // send the payload an interaction message too
        jsonObj = new JsonObject();
        jsonObj.addProperty("target_id", this.targetId);
        jsonObj.addProperty("in_range", false);
        jsonObj.addProperty("config", config.name());
        msgPayload = jsonObj.toString();
        try {
            transport.publish(INTERACTION_TOPIC_PREFIX + payloadId, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send interaction event over MQTT:", e);
        }
    }

    private void stopTarget(){
        try {
            String msgPayload = targetId;
            logger.info("Sending message to " + targetStopTopic);
            transport.publish(targetStopTopic, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send interaction event over MQTT:", e);
        }
    }

    public void handlePositionChange(Position position) {
        positionLock.lock();
        try {
            this.position = position;
        }finally {
            positionLock.unlock();
        }
        sendPositionMessage();
    }

    public void stop() {
        positionLock.lock();
        try {
            if (positionUpdateThread != null) {
                positionUpdateThread.interrupt();
                positionUpdateThread = null;
            }
        }finally {
            positionLock.unlock();
        }
        stopTarget();
    }

    private void startPeriodicallySendingPosition(){
        positionUpdateThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted()){
                    sendPositionMessage();
                    try {
                        Thread.sleep(POSITION_UPDATE_TIME_MS);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }, targetId + "_position_update");
        positionUpdateThread.start();
    }

    private void sendPositionMessage(){
        Position pos = null;
        positionLock.lock();
        try {
            pos = this.position;
        }finally {
            positionLock.unlock();
        }
        if (pos == null){
            logger.info("Target " + targetId + " does not have a position yet.");
            return;
        }

        double lat = this.position.getLatitude().getDegrees();
        double lon = this.position.getLongitude().getDegrees();
        double elev = Utils.EARTH.getElevation(Angle.fromDegreesLatitude(lat),
                Angle.fromDegreesLongitude(lon));
        double alt = this.position.getAltitude() + elev;
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("latitude", lat);
        jsonObj.addProperty("longitude", lon);
        jsonObj.addProperty("altitude", alt);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(positionTopic, msgPayload, MaceMessageTransport.QualityOfService.UNRELIABLE);
        } catch (MqttException e) {
            logger.warn("Failed to send gps position over MQTT:", e);
        }
    }

}
