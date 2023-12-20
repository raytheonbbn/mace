//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.nullsim.blenodes;

import java.util.ArrayList;
import java.util.Collection;

public class SimLinkTargetConfig extends SimTargetConfig {
    public double captureRange;
    protected Collection<String> networkName;

    public SimLinkTargetConfig(SimTargetConfig oldConfig, double captureRange, String networkName, boolean suppression) {
        super(oldConfig, SimTargetType.LINK.name());
        this.captureRange = captureRange;
        this.networkName = new ArrayList<>();
        this.networkName.add(networkName);
        this.suppression = suppression;
    }

    public void addNetwork(String networkName) {
        this.networkName.add(networkName);
    }

    public void updateRange(double captureRange) {
        this.captureRange = captureRange;
    }
}
