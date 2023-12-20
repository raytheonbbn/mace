//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

import com.bbn.ccast.Utils;
import com.bbn.ccast.mqtt.MaceMessageTransport;
import com.bbn.ccast.nullsim.SimVehicle.Intent;
import com.google.gson.*;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import org.apache.log4j.Logger;

import java.io.*;
import java.lang.reflect.Type;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class SimTargetManager {
    private static final Logger logger = Logger.getLogger(SimTargetManager.class.getName());

    public static SimTargetManager instance;

    private final ConcurrentHashMap<String, SimTarget> targetIdMap;
    private final MaceMessageTransport transport;
    private final ConcurrentHashMap<String, List<SimTarget>> interactionMap = new ConcurrentHashMap<>();
    private short nextTargetId = 0;


    public SimTargetManager(MaceMessageTransport transport) {
        this.targetIdMap = new ConcurrentHashMap<>();
        this.transport = transport;

        instance = this;
    }

    public void interactWithTargetsInRange(Position payloadPos, String payloadId,
                                           Collection<Intent> intent) {
        List<SimTarget> targetsInCaptureRange = getAllTargetsInCaptureRange(payloadPos);
        List<SimTarget> oldInteractions = interactionMap.get(payloadId);
        List<SimTarget> newInteractions;
        if (oldInteractions != null) {
            //This payload was previously interacting with some targets
            newInteractions = new ArrayList<>();
            for (SimTarget tgt : targetsInCaptureRange) {
                if (oldInteractions.contains(tgt)) {
                    //Remove any still active interactions from oldInteractions
                    oldInteractions.remove(tgt);
                } else {
                    //Add any new interactions to newInteractions
                    newInteractions.add(tgt);
                }
            }
            for (SimTarget oldTgt : oldInteractions) {
                //Remove any interactions that are no longer active
                oldTgt.payloadStoppedInteraction(payloadId);
            }
        } else {
            //All interactions are new interactions
            newInteractions = targetsInCaptureRange;
        }

        for (SimTarget tgt : newInteractions) {
            tgt.payloadStartedInteraction(payloadId, intent);
        }
        interactionMap.put(payloadId, targetsInCaptureRange);

    }

    public void discoverTargetsInRange(Position payloadPos, String payloadId, Collection<Intent> intent) {
        List<SimTarget> targetsInDiscoverRange = getAllTargetsInDiscoverRange(payloadPos);
        for (SimTarget tgt : targetsInDiscoverRange) {
            tgt.payloadInDiscoveryRangeOfTarget(payloadId, intent);
        }

    }

    public void stopInteractions(String payloadId) {
        List<SimTarget> oldInteractions = interactionMap.remove(payloadId);
        if (oldInteractions != null) {
            for (SimTarget tgt : oldInteractions) {
                tgt.payloadStoppedInteraction(payloadId);
            }
        }
    }


    public List<SimTarget> getAllTargetsInDiscoverRange(Position payloadPos) {
        List<SimTarget> result = new ArrayList<>();
        for (SimTarget tgt : targetIdMap.values()) {
            double dist = computeTargetDistance(payloadPos, tgt);
            if (dist <= tgt.getDiscoverDistance()) {
                result.add(tgt);
            }
        }
        return result;
    }

    public List<SimTarget> getAllTargetsInCaptureRange(Position payloadPos) {
        List<SimTarget> result = new ArrayList<>();
        for (SimTarget tgt : targetIdMap.values()) {
            double dist = computeTargetDistance(payloadPos, tgt);
            if (dist <= tgt.getCaptureRange()) {
                result.add(tgt);
            }
        }
        return result;
    }

    protected double computeTargetDistance(Position payloadPos, SimTarget target) {
        Position targetPos = target.getPosition();
        double horizontalDistance = Utils.distanceInMetersFromLlaToLla(payloadPos, targetPos);
        double verticalDistance = targetPos.getAltitude() - payloadPos.getAltitude();
        return Math.sqrt(horizontalDistance * horizontalDistance + verticalDistance * verticalDistance);
    }

    public void createSimTarget(Position position) {
        while (targetIdMap.containsKey(nextTargetId) && nextTargetId > 0) {
            nextTargetId++;
            if (nextTargetId < 0 && targetIdMap.size() < Short.MAX_VALUE + 1) {
                nextTargetId = 0;
            }
        }
        if (nextTargetId < 0) {
            logger.warn("Too many sim targets!");
        }
        createOrUpdateSimTarget("" + nextTargetId++, position);
    }

    public void createOrUpdateSimTarget(SimTargetConfig config) {
        Position targetPos = new Position(
                Angle.fromDegrees(config.latitude), Angle.fromDegrees(config.longitude), config.altitude);
        createOrUpdateSimTarget(config.targetId, targetPos);
        SimTargetType type = SimTargetType.valueOf(config.type);
        switch (type) {
            default:
            case IDLE:
                configureIdleTarget(config.targetId, config.discoverDistance);
                break;
            case MASS:
                SimMassTargetConfig massConfig = (SimMassTargetConfig) config;
                configureMassTarget(config.targetId, massConfig.captureRange, massConfig.payloadsRequired,
                        massConfig.suppression, config.discoverDistance);
                break;
            case PERI:
                SimPeriTargetConfig periConfig = (SimPeriTargetConfig) config;
                configurePeriTarget(config.targetId, periConfig.captureRange, periConfig.payloadsRequired,
                        periConfig.captureCountdown, periConfig.suppression, config.discoverDistance);
                break;
            case LINK:
                // taken care of separately.
                break;
        }
    }

    public void createOrUpdateSimTarget(String targetId, Position position, double discoverDistance) {
        SimTargetEventHandler eventHandler = new SimTargetEventHandler(targetId, this.transport);
        SimTarget simTarget = targetIdMap.get(targetId);
        if (simTarget == null) {
            simTarget = new SimTarget(targetId, position, discoverDistance, eventHandler);
        } else {
            simTarget.setPosition(position);
        }
        targetIdMap.put(targetId, simTarget);
        logger.debug(String.format("Sim target created with uid %s", targetId));
    }

    public void createOrUpdateSimTarget(String targetId, Position position) {
        SimTargetEventHandler eventHandler = new SimTargetEventHandler(targetId, this.transport);
        SimTarget simTarget = targetIdMap.get(targetId);
        if (simTarget == null) {
            simTarget = new SimTarget(targetId, position, 5.0, eventHandler);
        } else {
            simTarget.setPosition(position);       
        }
        targetIdMap.put(targetId, simTarget);
        logger.debug(String.format("Sim target created with uid %s", targetId));
    }


    public void removeSimTarget(String targetId) {
        SimTarget simTarget = targetIdMap.remove(targetId);
        if (simTarget != null) {
            simTarget.stop();
        }
    }

    private static class SimTargetConfigAdapter implements JsonDeserializer<SimTargetConfig> {
        @Override
        public SimTargetConfig deserialize(JsonElement json, Type typeOfT,
                                           JsonDeserializationContext context) throws JsonParseException {
            JsonObject jsonObject = json.getAsJsonObject();
            JsonPrimitive prim = (JsonPrimitive) jsonObject.get("type");
            String typeString = prim.getAsString();
            SimTargetType type = SimTargetType.valueOf(typeString);
            Class<? extends SimTargetConfig> klass;
            switch (type) {
                case IDLE:
                    klass = SimIdleTargetConfig.class;
                    break;
                case MASS:
                    klass = SimMassTargetConfig.class;
                    break;
                case PERI:
                    klass = SimPeriTargetConfig.class;
                    break;
                case LINK:
                    klass = SimLinkTargetConfig.class;
                    break;
                default:
                    throw new IllegalArgumentException("Unknown type: " + typeString);
            }
            return context.deserialize(jsonObject, klass);
        }
    }

    public String[] getSimTargetIDs(){
        // Get all target ID's
        String[] targetIDs = targetIdMap.keySet().toArray(new String[0]);
        return targetIDs;
    }

    public SimTargetConfig getSimTargetConfigFromID(String targetID){
        // A way to get a target configuration from its ID
        return targetIdMap.get(targetID).getConfig();
    }

    public void loadSimTargetConfigsFromFile(File targetPosFile){
        try (BufferedReader bufferedReader = new BufferedReader(new FileReader(targetPosFile))) {
            Gson gson = new GsonBuilder()
                    .registerTypeAdapter(SimTargetConfig.class, new SimTargetConfigAdapter())
                    .create();
            JsonArray configs = gson.fromJson(bufferedReader, JsonArray.class);
            List<List<String>> linkedTargetIDs = new ArrayList<List<String>>();
            List<String> linkedNetworkNames = new ArrayList<>();
            List<Double> linkedNetworkRanges = new ArrayList<>();
            List<Boolean> linkedSuppression = new ArrayList<>();
            List<Double> linkedDiscoverDistances = new ArrayList<>();

            for (JsonElement element : configs) {
                SimTargetConfig config = gson.fromJson(element, SimTargetConfig.class);
                SimTargetType type = SimTargetType.valueOf(config.type);
                if (type == SimTargetType.LINK) {
                    SimLinkTargetConfig lt = (SimLinkTargetConfig) config;
                    int ii = 0;
                    for (String network : lt.networkName) {
                        if (linkedNetworkNames.contains(network)) {
                            linkedTargetIDs.get(linkedNetworkNames.indexOf(network)).add(config.targetId);
                        } else {
                            linkedNetworkNames.add(network);
                            linkedNetworkRanges.add(lt.captureRange);
                            linkedTargetIDs.add(new ArrayList<>());
                            linkedTargetIDs.get(linkedTargetIDs.size() - 1).add(config.targetId);
                            linkedSuppression.add(lt.suppression);
                            linkedDiscoverDistances.add(lt.discoverDistance);
                        }
                        ii++;
                    }
                }
                createOrUpdateSimTarget(config);
            }

            int ii = 0;
            for (List<String> targets : linkedTargetIDs) {
                configureLinkedTargets(linkedNetworkRanges.get(ii), linkedNetworkNames.get(ii), targets, linkedSuppression.get(ii), linkedDiscoverDistances.get(ii));
                ii++;
            }
        } catch (IOException e) {
            logger.error("Failed to load sim target configuration JSON file '" + targetPosFile + "'.  " +
                    "Aborting!", e);
        }
    }

    public void saveSimTargetConfigsToFile(File targetPosFile) {
        try (BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(targetPosFile))) {
            Gson gson = new Gson();
            SimTargetConfig[] simTargetConfigs = new SimTargetConfig[targetIdMap.size()];
            int i = 0;
            for (SimTarget simTarget : targetIdMap.values()) {
                SimTargetConfig config = simTarget.getConfig();
                simTargetConfigs[i++] = config;
            }
            gson.toJson(simTargetConfigs, bufferedWriter);
        } catch (IOException e) {
            logger.error("Failed to save sim target configuration to JSON file '" + targetPosFile + "'.  " +
                    "Aborting!", e);
        }
    }

    public void configureIdleTarget(String targetId, double discoverDistance) {
        SimTarget target = targetIdMap.getOrDefault(targetId, null);
        if (target == null) {
            this.createOrUpdateSimTarget(targetId, new Position(Angle.fromDegrees(15000),
                    Angle.fromDegrees(15000), 15000), discoverDistance);
            target = targetIdMap.getOrDefault(targetId, null);
        }
        target.configureAsIdleTarget();
        target.setDiscoverDistance(discoverDistance);
    }

    public void configureMassTarget(String targetId, double captureRange, int payloadsRequired,
                                    boolean suppression, double discoverDistance) {
        SimTarget target = targetIdMap.getOrDefault(targetId, null);
        if (target == null) {
            this.createOrUpdateSimTarget(targetId, new Position(Angle.fromDegrees(15000),
                    Angle.fromDegrees(15000), 15000), discoverDistance);
            target = targetIdMap.getOrDefault(targetId, null);
        }
        target.configureAsMassTarget(captureRange, payloadsRequired, suppression);
        target.setDiscoverDistance(discoverDistance);
    }

    public void resetInteractions(String payloadId) {
        interactionMap.remove(payloadId);
    }

    public void configurePeriTarget(String targetId, double captureRange, int payloadsRequired,
                                    double captureCoutdown, boolean suppression, double discoverDistance) {
        SimTarget target = targetIdMap.getOrDefault(targetId, null);
        if (target == null) {
            this.createOrUpdateSimTarget(targetId, new Position(Angle.fromDegrees(15000),
                    Angle.fromDegrees(15000), 15000), discoverDistance);
            target = targetIdMap.getOrDefault(targetId, null);
        }
        target.configureAsPeriTarget(captureRange, payloadsRequired, captureCoutdown, suppression);
        target.setDiscoverDistance(discoverDistance);
    }

    public void configureLinkedTargets(double captureRange, String networkName, List<String> targetList, boolean suppression, double discoverDistance) {
        List<String> updatedTargets = new ArrayList<>(targetList);
        for (SimTarget targ : targetIdMap.values()) {
            if (targ.getType() == SimTargetType.LINK) {
                if (!targetList.contains(targ.getTargetId()) && ((SimLinkTargetConfig) targ.getConfig()).networkName.contains(networkName)) {
                    updatedTargets.add(targ.getTargetId());
                }
            }
        }
        for (var tid : targetList) {
            var target = targetIdMap.getOrDefault(tid, null);
            if (target == null) {
                this.createOrUpdateSimTarget(tid, new Position(Angle.fromDegrees(15000),
                        Angle.fromDegrees(15000), 15000), discoverDistance);
                target = targetIdMap.getOrDefault(tid, null);
            }
            target.configureAsNetworkTarget(networkName, captureRange, updatedTargets, suppression);
            target.setDiscoverDistance(discoverDistance);
        }
    }
}
