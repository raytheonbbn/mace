//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast;

import java.util.Date;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

public class TargetTelemPackage {
    private String uid;
    private String type;
    private double latitude;
    private double longitude;
    private double altitude;
    private Boolean captured;
    private ConcurrentHashMap<String, Boolean> networks_captured;
    private ConcurrentHashMap<String, List<String>> networks;
    private Double required_payloads;
    private Double payloads;
    private Double countdown;
    private Double current_duration;
    private Date timeReceived;
    private Boolean discovered;
    private Double interaction_range;
    private Boolean suppression;
    private String callsign;
    private String ipAddress;


    public static TargetTelemPackage fromJson(String json) {
        Gson gson = new GsonBuilder().create();
        return gson.fromJson(json, TargetTelemPackage.class);
    }

    public String getUid(){
        return uid;
    }

    public String getType(){
        return type;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public double getAltitude(){
        return altitude;
    }

    public boolean isCaptured(){
        return captured != null && captured;
    }

    public void setTimeReceived(long time){
        this.timeReceived = new Date(time);
    }

    public Date getTimeReceived() {
        return timeReceived;
    }

    public boolean getDiscovered() {
        return discovered != null && discovered;
    }

    public String getNetworkNames() {
        if (networks != null) {
            return String.join(", ", networks.keySet());
        }

        return "";
    }

    public ConcurrentHashMap<String, List<String>> getNetworks() {
        return this.networks == null ? new ConcurrentHashMap<>() : this.networks;
    }

    public ConcurrentHashMap<String, Boolean> getNetworksCaptured() {
        return networks_captured == null ? new ConcurrentHashMap<>() : networks_captured;
    }

    public double getCurrentDuration() {
        return this.current_duration;
    }

    public int getRequiredPayloads() {
        if (this.type.equals("LINK"))
            return 1;

        return this.required_payloads == null ? 0 : (int)(double)this.required_payloads;
    }

    public int getPayloads() {
        return this.payloads == null ? 0 : (int)(double)this.payloads;
    }

    public double getInteractionRange() {
        return this.interaction_range == null ? 0 : (double)this.interaction_range;
    }

    public double getCountdown() {
        return this.countdown == null ? 0 : (double)this.countdown;
    }

    public boolean getSuppression() {
        return this.suppression != null && this.suppression;
    }

    public String getCallsign(){
        return this.callsign;
    }

    public String getIpAddress() { return this.ipAddress; }
}
