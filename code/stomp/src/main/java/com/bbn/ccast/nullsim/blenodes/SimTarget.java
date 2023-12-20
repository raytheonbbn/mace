//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

import gov.nasa.worldwind.geom.Position;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.bbn.ccast.nullsim.SimVehicle.Intent;

public class SimTarget {

    private final String targetId;
    private Position position;
    /** A map of payload IDs to a structure with information on ongoing interactions between that payload and target**/
    private final Map<String, SimTargetInteractionState> payloadInteractionMap;
    public SimTargetType configurationType = SimTargetType.IDLE;
    private SimTargetConfig config;
    private double discoverDistance = 5.0; // The maximum distance at which a target can be discovered by a payload
    private double captureRange  = -1;   // The maximum distance at which a target can be discovered by a payload

    /** A component to handle target events such as interactions **/
    private final SimTargetEventHandler eventHandler;

    public SimTarget(String targetId, Position position, double discoverDistance, SimTargetEventHandler eventHandler){
        this.targetId = targetId;
        this.eventHandler = eventHandler;
        payloadInteractionMap = new HashMap<>();
        config = new SimIdleTargetConfig(targetId,
                position.getLatitude().getDegrees(), position.getLongitude().getDegrees(), position.getAltitude(), discoverDistance);
        setPosition(position);
    }

    public void configureAsMassTarget(double captureRange, int payloadsRequired, boolean suppression){
        config = new SimMassTargetConfig(config, captureRange, payloadsRequired, suppression);
        eventHandler.sendMassTargetConfig(getTargetId(), captureRange, payloadsRequired, suppression);
        this.configurationType = SimTargetType.MASS;
        this.captureRange = captureRange;
    }

    public void configureAsIdleTarget(){
        config = new SimIdleTargetConfig(config);
        eventHandler.sendIdleTargetConfig(getTargetId());
        this.configurationType = SimTargetType.IDLE;
        this.captureRange = -1;
    }

    public void configureAsPeriTarget(double captureRange, int payloadsRequired, double captureCountdown, boolean suppression) {
        config = new SimPeriTargetConfig(config, captureRange, payloadsRequired, captureCountdown, suppression);
        eventHandler.sendPeriTargetConfig(getTargetId(), captureRange, payloadsRequired, captureCountdown, suppression);
        this.configurationType = SimTargetType.PERI;
        this.captureRange = captureRange;
    }

    public void configureAsNetworkTarget(String networkName, double captureRange, List<String> targetNetworks, boolean suppression) {
        if (getType() == SimTargetType.LINK) {
            if (!((SimLinkTargetConfig) config).networkName.contains(networkName)) {
                ((SimLinkTargetConfig) config).addNetwork(networkName);
                ((SimLinkTargetConfig) config).updateRange(captureRange);
            }
        } else {
            config = new SimLinkTargetConfig(config, captureRange, networkName, suppression);
        }
        eventHandler.sendNetworkConfig(networkName, captureRange, targetNetworks, suppression);
        this.configurationType = SimTargetType.LINK;
        this.captureRange = captureRange;
    }

    public void setPosition(Position position){
        this.position = position;
        this.config.latitude = position.getLatitude().getDegrees();
        this.config.longitude = position.getLongitude().getDegrees();
        this.config.altitude = position.getAltitude();
        eventHandler.handlePositionChange(position);
        //TODO: check if any payloads are in or out of captureRange after move?
    }

    public void payloadStartedInteraction(String payload, Collection<Intent> intent){
        eventHandler.payloadStartedInteraction(payload, intent, this.configurationType);
        if (intent.contains(Intent.valueOf(this.configurationType.name())) && this.configurationType != SimTargetType.IDLE) {
            SimTargetInteractionState state = payloadInteractionMap.computeIfAbsent(payload, SimTargetInteractionState::new);
            state.payloadStartedInteracitonTime = System.currentTimeMillis();
        }
    }

    public void payloadStoppedInteraction(String payload){
        SimTargetInteractionState state = payloadInteractionMap.computeIfAbsent(payload, SimTargetInteractionState::new);
        state.payloadStartedInteracitonTime = 0;
        eventHandler.payloadStoppedInteraction(payload, this.configurationType);
    }

    public void payloadInDiscoveryRangeOfTarget(String payload, Collection<Intent> intent){
        eventHandler.payloadInDiscoveryRangeOfTarget(payload, intent, this.configurationType);
    }

    public Position getPosition() {
        return position;
    }

    public String getTargetId() {
        return targetId;
    }

    public void stop() {
        eventHandler.stop();
    }

    public SimTargetType getType(){
        return configurationType;
    }

    public SimTargetConfig getConfig() {
        return config;
    }

    public double getDiscoverDistance() {
        return discoverDistance;
    }

    public void setDiscoverDistance(double discoverDistance) {
        this.discoverDistance = discoverDistance;
    }

    public double getCaptureRange() {
        return captureRange;
    }

    public void setCaptureRange(double captureRange) {
        this.captureRange = captureRange;
    }


}
