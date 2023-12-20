//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

import java.util.List;

import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import org.apache.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttException;

public class TargetEventHandler {
    private static final Logger logger = Logger.getLogger(TargetEventHandler.class.getName());
    private static final String TARGET_CONFIG_TOPIC = "server/white_force/target/configurations";
    private static final String HARDWARE_TARGET_CONFIG_TOPIC = "hardware/server/target/configurations";
    private static final String COMMAND_TOPIC = "command/server/command";

    protected final MaceMessageTransport transport;

    public TargetEventHandler(MaceMessageTransport transport) {
        this.transport = transport;
    }

    public void sendMassTargetConfig(String targetId, double range, double payloadsRequired,
                                     boolean suppression) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("uid", targetId);
        jsonObj.addProperty("type", SimTargetType.MASS.name());
        jsonObj.addProperty("detection_range", range);
        jsonObj.addProperty("required_payloads", payloadsRequired);
        jsonObj.addProperty("modify_capture_state", true);
        jsonObj.addProperty("captured", false);
        jsonObj.addProperty("suppression", suppression);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(TARGET_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to target config position over MQTT:", e);
        }
    }

    public void sendIdleTargetConfig(String targetId) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("uid", targetId);
        jsonObj.addProperty("type", SimTargetType.IDLE.name());
        jsonObj.addProperty("modify_capture_state", true);
        jsonObj.addProperty("captured", false);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(TARGET_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to target config position over MQTT:", e);
        }
    }

    public void sendPeriTargetConfig(String targetId, double range, double payloadsRequired,
                                     double captureCoutdown, boolean suppression) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("uid", targetId);
        jsonObj.addProperty("type", SimTargetType.PERI.name());
        jsonObj.addProperty("detection_range", range);
        jsonObj.addProperty("required_payloads", payloadsRequired);
        jsonObj.addProperty("capture_countdown", captureCoutdown);
        jsonObj.addProperty("modify_capture_state", true);
        jsonObj.addProperty("captured", false);
        jsonObj.addProperty("suppression", suppression);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(TARGET_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send target config over MQTT:", e);
        }
    }

    public void sendNetworkConfig(String networkName, double range, List<String> targets, boolean suppression) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("uid", networkName);
        jsonObj.addProperty("type", "LINK");
        jsonObj.addProperty("delete", false);
        jsonObj.addProperty("change_uid", false);
        var jTargets = new JsonArray();
        targets.forEach(x -> jTargets.add(x));
        jsonObj.add("network", jTargets);
        jsonObj.addProperty("detection_range", range);
        jsonObj.addProperty("modify_capture_state", true);
        jsonObj.addProperty("captured", false);
        jsonObj.addProperty("suppression", suppression);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(TARGET_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send network config over MQTT:", e);
        }
    }

    public void sendSetDiscoverCommand(boolean isDiscovered, String[] targets) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("command", "set_discovered");
        jsonObj.addProperty("discovered", isDiscovered);
        JsonArray jTargets = new JsonArray();
        for (String target : targets) {
            jTargets.add(target);
        }
        jsonObj.add("targets", jTargets);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(COMMAND_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send set_discovered command over MQTT:", e);
        }
    }

    public void sendResetCommand(String tag) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("command", "reset");
        jsonObj.addProperty("tag", tag);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(COMMAND_TOPIC, msgPayload, MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send reset command MQTT:", e);
        }
    }

    public void sendTargetCallsign(String targetId, String callsign) {
        JsonObject jsonObj = new JsonObject();
        jsonObj.addProperty("set_callsign", true);
        jsonObj.addProperty("uid", targetId);
        jsonObj.addProperty("callsign", callsign);
        String msgPayload = jsonObj.toString();
        try {
            transport.publish(TARGET_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send network config over MQTT:", e);
        }
        // Also send the new callsign to the hardware topic so the PIs pick it up
        try {
            transport.publish(HARDWARE_TARGET_CONFIG_TOPIC, msgPayload,
                    MaceMessageTransport.QualityOfService.RECEIVE_EXACTLY_ONCE);
        } catch (MqttException e) {
            logger.warn("Failed to send network config over MQTT:", e);
        }
    }
    
}


