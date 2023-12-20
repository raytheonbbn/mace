//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast;

import java.util.HashSet;
import java.util.Set;

public class LinkedNetwork {
    private String uid;
    private Set<String> targets = new HashSet<String>();
    private double detectionRange;
    private boolean captured = false;

    public LinkedNetwork(String uid, double detectionRange, Set<String> targets) {
        this.uid = uid;
        this.detectionRange = detectionRange;
        this.targets = targets;
    }

    public void addTarget(String targetUid) {
        targets.add(targetUid);
    }

    public void removeTarget(String targetUid) {

    }

    public void deleteNetwork() {

    }

    public void renameNetwork(String new_uid) {
        this.uid = new_uid;
    }

    public void setCaptured(boolean captured) {
        this.captured = captured;
    }
}
