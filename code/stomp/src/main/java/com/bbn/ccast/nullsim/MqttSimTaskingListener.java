//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim;

import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.bbn.ccast.util.LatLonAlt;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.apache.log4j.Logger;
import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.Map;
import java.util.function.Consumer;

public class MqttSimTaskingListener {
    private static final Logger logger = Logger.getLogger(NullSim.class.getName());

    private static final String MQTT_SIM_TASKING_PREFIX = "sim/agent/tasking/";
    private static final String MQTT_GOTO_TOPIC = MQTT_SIM_TASKING_PREFIX + "goto";

    private final MaceMessageTransport transport;
    private final NullSim nullSim;

    public MqttSimTaskingListener(NullSim nullSim, MaceMessageTransport transport){
        this.transport = transport;
        this.nullSim = nullSim;
        transport.subscribe(MQTT_GOTO_TOPIC, new Consumer<String>() {
            @Override
            public void accept(String msgStr) {
                logger.info("Received GOTO request over MQTT: " + msgStr);
                JsonElement json = new JsonParser().parse(msgStr);
                JsonObject msg = json.getAsJsonObject();
                String agent = msg.get("agent").getAsString();
                Map<String, SimVehicle> vehicleMap = nullSim.getAllVehicles();
                SimVehicle vehicle = vehicleMap.get(agent);
                if (vehicle == null){
                    logger.warn("Couldn't find agent " + agent);
                    return;
                }

                JsonElement element = msg.get("latitude");
                if (element == null){
                    logger.warn("No latitude specified.");
                    return;
                }
                double lat = element.getAsDouble();

                element = msg.get("longitude");
                if (element == null){
                    logger.warn("No longitude specified.");
                    return;
                }
                double lon = element.getAsDouble();

                element = msg.get("altitude");
                if (element == null){
                    logger.warn("No altitude specified.");
                    return;
                }
                double alt = element.getAsDouble();

                LatLonAlt lla = new LatLonAlt(lat, lon, alt);
                logger.info("Setting lla target of " + agent + " to " + lla);
                vehicle.setTarget(lla);
            }
        });
    }
}
